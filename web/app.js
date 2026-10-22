/* Browser telemetry UI for the drone-show service.
 * Three.js stage with Canvas fallback. Actual positions come from state.positions only.
 */
(function () {
  'use strict';

  var Core = window.DroneShowCore;
  if (!Core) {
    document.body.innerHTML = '<div style="padding:24px;color:#ff9dab">Telemetry core failed to load.</div>';
    return;
  }

  var WS_URL = Core.DEFAULT_WS_URL;
  try {
    var params = new URLSearchParams(window.location.search);
    if (params.get('ws')) {
      WS_URL = params.get('ws');
    } else if (window.location.protocol === 'http:' || window.location.protocol === 'https:') {
      // Use the same hostname the page was opened with (localhost vs 127.0.0.1, proxies, etc.).
      WS_URL = (window.location.protocol === 'https:' ? 'wss://' : 'ws://') + window.location.host + '/ws';
    }
  } catch (err) { /* ignore malformed query */ }

  var SERVICE_HTTP_BASE = (window.location.protocol === 'http:' || window.location.protocol === 'https:')
    ? window.location.origin
    : WS_URL.replace(/^ws:/, 'http:').replace(/^wss:/, 'https:').replace(/\/ws(\?.*)?$/, '');

  var TRAIL_LENGTH = 30;
  var TARGET_TRAIL_LENGTH = 40;
  var MAX_EVENTS = 8;
  var GLOW_BLUR = 10;
  var STALE_MS = 3000;
  var LIVE_POLL_FAST_MS = 200;
  var LIVE_POLL_SLOW_MS = 1000;
  var REPLAY_HZ = 15;

  var dom = {
    connection: document.getElementById('connection'),
    connText: document.getElementById('conn-text'),
    retryText: document.getElementById('retry-text'),
    disconnectedOverlay: document.getElementById('disconnected-overlay'),
    overlayTitle: document.getElementById('overlay-title'),
    overlayDetail: document.getElementById('overlay-detail'),
    overlaySub: document.getElementById('overlay-sub'),
    referenceLabel: document.getElementById('reference-label'),
    referenceLegend: document.getElementById('reference-legend'),
    canvas: document.getElementById('scene'),
    currentSegment: document.getElementById('current-segment'),
    currentSegmentId: document.getElementById('current-segment-id'),
    pendingSegment: document.getElementById('pending-segment'),
    pendingSegmentId: document.getElementById('pending-segment-id'),
    simTime: document.getElementById('sim-time'),
    dronesCount: document.getElementById('drones-count'),
    revisions: document.getElementById('revisions'),
    maxError: document.getElementById('max-error'),
    rmsError: document.getElementById('rms-error'),
    lastMessage: document.getElementById('last-message'),
    lastMessageAge: document.getElementById('last-message-age'),
    progressText: document.getElementById('progress-text'),
    progressFill: document.getElementById('progress-fill'),
    errorText: document.getElementById('error-text'),
    warningText: document.getElementById('warning-text'),
    statusChip: document.getElementById('status-chip'),
    hudDrones: document.getElementById('hud-drones'),
    hudSegment: document.getElementById('hud-segment'),
    btnReset: document.getElementById('btn-reset'),
    btnTargets: document.getElementById('btn-targets'),
    chkFollow: document.getElementById('chk-follow'),
    eventList: document.getElementById('event-list'),
    colorSource: document.getElementById('color-source'),
    colorSwatches: document.getElementById('color-swatches'),
    colorNote: document.getElementById('color-note'),
    modeBadge: document.getElementById('mode-badge'),
    readyBanner: document.getElementById('ready-banner'),
    liveState: document.getElementById('live-state'),
    btnReplay: document.getElementById('btn-replay'),
    replayStatus: document.getElementById('replay-status')
  };

  var stage = window.DroneStage && window.DroneStage.create(dom.canvas);
  if (!stage && !dom.canvas.getContext('2d')) {
    var fallback = dom.canvas.cloneNode();
    dom.canvas.replaceWith(fallback); dom.canvas = fallback;
  }
  var ctx = stage ? null : dom.canvas.getContext('2d');
  var verification = document.getElementById('verification');
  var verifiedRuns = {};
  var requestStatus = null;
  var view = Core.createView({});
  var state = Core.normalizeState({});
  var trails = [];
  var targetTrails = [];
  var autoFollow = true;
  var closeupDroneIndex = null;
  var showTargets = false;
  var connectionStatus = 'connecting';
  var connectionDetail = '';
  var protocolError = null;
  var serviceError = null;
  var lastMessageAt = null;
  var reconnectDelay = 1000;
  var reconnectTimer = null;
  var retryAt = 0;
  var ws = null;
  var events = [];
  var lastRenderedAt = 0;
  var dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  var displayMode = 'live';
  var latestRunId = null;
  var lastWsMessageAt = null;
  var pendingPingAt = null;
  var lastPollAt = null;
  var pollTimer = null;
  var pollInFlight = false;
  var forceReconnectArmed = false;
  var replayActive = false;
  var replayFrames = [];
  var replayIndex = 0;
  var replayStartedAt = 0;
  var replayPaused = false;
  var replayElapsed = 0;
  var replayTransport = document.getElementById('replay-transport');
  var replaySeek = document.getElementById('replay-seek');
  var replayClock = document.getElementById('replay-clock');
  var replayPause = document.getElementById('btn-replay-pause');
  var replayPreview = false;
  var replayOffline = false;
  var replayLiveRunAtStart = null;
  var replayTimer = null;
  var previousPositions = [];
  var receivedAt = 0;
  var frameInterval = 50;

  if (new URLSearchParams(window.location.search).get('targets') === '1') {
    showTargets = true;
  }

  function setText(el, value) {
    if (el) el.textContent = value === null || value === undefined ? '—' : String(value);
  }

  function eventTime(value) {
    return Core.isFiniteNumber(value) ? Core.formatTime(value) : '--';
  }

  function detailSummary(detail) {
    if (detail === null || detail === undefined) return '';
    if (typeof detail === 'string') return detail;
    try {
      var text = JSON.stringify(detail);
      return text.length > 180 ? text.slice(0, 177) + '...' : text;
    } catch (err) {
      return String(detail);
    }
  }

  function fetchJson(url) {
    return window.fetch(url, { cache: 'no-store' }).then(function (response) {
      if (response.ok) return response.json();
      return response.text().then(function (text) {
        throw new Error('HTTP ' + response.status + ' ' + text.slice(0, 120));
      });
    });
  }

  function pollHealthy() {
    return lastPollAt !== null && (Date.now() - lastPollAt) < 2500;
  }

  function liveDataFresh() {
    var wsFresh = connectionStatus === 'connected' && lastMessageAt !== null && (Date.now() - lastMessageAt) < STALE_MS;
    return wsFresh || pollHealthy();
  }

  function updateConnectionDisplay() {
    var fresh = liveDataFresh();
    var statusName = connectionStatus;
    if (statusName === 'connected' && !fresh) statusName = 'syncing';
    if (statusName !== 'connected' && pollHealthy()) statusName = 'polling';
    if (dom.connection) dom.connection.setAttribute('data-state', statusName);
    setText(dom.connText, statusName === 'connected' ? '已连接' : (statusName === 'polling' ? '已连接' : statusName === 'syncing' ? '正在同步' : '正在连接'));
    var overlayVisible = !fresh;
    dom.disconnectedOverlay.hidden = !overlayVisible;
    if (overlayVisible) {
      var titles = { connecting: 'CONNECTING', disconnected: 'DISCONNECTED', stalled: 'TELEMETRY STALLED' };
      dom.overlayTitle.textContent = titles[connectionStatus] || 'DISCONNECTED';
      var detailText = connectionDetail || ('Endpoint: ' + WS_URL);
      if (pollHealthy()) detailText += ' · polling ' + SERVICE_HTTP_BASE + '/api/state';
      dom.overlayDetail.textContent = detailText;
      dom.overlaySub.textContent = 'Polling fallback is active while the WebSocket recovers. Last frame stays dimmed. Reference targets are never treated as actual positions.';
    }
  }

  function pollState() {
    pollTimer = null;
    if (pollInFlight) {
      schedulePoll();
      return;
    }
    pollInFlight = true;
    fetchJson(SERVICE_HTTP_BASE + '/api/state').then(function (body) {
      pollInFlight = false;
      lastPollAt = Date.now();
      lastMessageAt = Date.now();
      handleIncomingState(body, 'poll');
      schedulePoll();
      updateConnectionDisplay();
    }).catch(function () {
      pollInFlight = false;
      schedulePoll();
      updateConnectionDisplay();
    });
  }

  function schedulePoll() {
    if (pollTimer) window.clearTimeout(pollTimer);
    var delay = liveDataFresh() ? LIVE_POLL_SLOW_MS : LIVE_POLL_FAST_MS;
    pollTimer = window.setTimeout(pollState, delay);
  }

  function forceReconnect() {
    if (forceReconnectArmed) return;
    forceReconnectArmed = true;
    if (ws) {
      try { ws.close(); } catch (err) { /* ignored */ }
    }
    if (reconnectTimer === null) {
      scheduleReconnect('No fresh WebSocket state for ' + Core.formatNumber(STALE_MS / 1000, 1) + 's; forcing reconnect');
    }
  }

  function visibilityRefresh() {
    if (document.visibilityState === 'visible') {
      pendingPingAt = null;
      if (pollTimer) {
        window.clearTimeout(pollTimer);
        pollTimer = null;
      }
      pollState();
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        if (reconnectTimer === null) {
          if (ws) {
            try { ws.close(); } catch (err) { /* ignored */ }
          }
          scheduleReconnect('Page became visible; refreshing state connection');
        }
      }
      refreshLatestRun();
    }
  }

  function setDisplayMode(mode) {
    displayMode = mode;
    if (dom.modeBadge) {
      dom.modeBadge.textContent = mode === 'replay' ? (replayPreview ? '规模预演回放 · 非物理飞行' : replayOffline ? '离线物理飞行回放' : '实际飞行回放') : (mode === 'ended' ? '回放结束' : '物理仿真');
      dom.modeBadge.setAttribute('data-mode', mode);
    }
  }

  function rememberLatestRun(raw) {
    if (raw && typeof raw.latest_run_id === 'string' && raw.latest_run_id) latestRunId = raw.latest_run_id;
  }

  function refreshLatestRun() {
    if (latestRunId) {
      updateLiveState();
      return;
    }
    fetchJson(SERVICE_HTTP_BASE + '/api/runs/latest').then(function (body) {
      if (body && typeof body.run_id === 'string' && body.run_id) latestRunId = body.run_id;
      updateLiveState();
    }).catch(function () { updateLiveState(); });
  }

  function updateReplayButton() {
    var available = latestRunId !== null && displayMode !== 'replay' && state.activeSegment === null && (state.status === 'idle' || state.status === 'completed');
    if (dom.btnReplay) {
      dom.btnReplay.hidden = !available;
      dom.btnReplay.disabled = !available;
      dom.btnReplay.textContent = displayMode === 'ended' ? '再次回放' : '回放上一场';
    }
    if (dom.replayStatus) dom.replayStatus.textContent = '';
  }

  function phaseLabel(segment) {
    if (!segment) return '准备就绪';
    var name = segment.name || '';
    if (name.indexOf('takeoff') === 0) return '起飞';
    if (name.indexOf('landing') === 0) return '降落';
    if (/scene:|return:/.test(name)) return name.split(':')[1];
    if (name === 'return') return '返回停机位';
    if (name === 'clearance') return '安全过渡';
    if (name.indexOf('assemble:') === 0 || name === 'transition') return '编队变换';
    if (name.indexOf('rotate') >= 0) return '旋转表演';
    if (name.indexOf('wave') >= 0) return '波浪动画';
    if (name.indexOf('scale') >= 0) return '舒展编队';
    return '编队展示';
  }

  function updateLiveState() {
    var active = state.activeSegment !== null;
    var fresh = liveDataFresh();
    var ready = displayMode === 'live' && !active && (state.status === 'idle' || state.status === 'completed') && fresh;
    if (dom.readyBanner) dom.readyBanner.hidden = !ready;
    if (dom.liveState) {
      var modeName = active ? 'running' : (ready ? 'ready' : 'offline');
      var text = fresh ? (active ? phaseLabel(state.activeSegment) + ' · 自动播放中' : '准备就绪 · 等待下一场表演') : '正在连接表演服务';
      if (displayMode === 'replay') { modeName = 'replay'; text = '回放 · ' + phaseLabel(state.activeSegment); }
      if (displayMode === 'ended') { modeName = 'replay'; text = '回放结束'; }
      dom.liveState.textContent = text;
      dom.liveState.setAttribute('data-state', modeName);
    }
    updateReplayButton();
    updateVerification();
  }

  function updateVerification() {
    if (requestStatus && state.activeSegment && displayMode === 'live') {
      verification.hidden = false; verification.textContent = requestStatus.text;
      return;
    }
    if (displayMode !== 'live' || state.activeSegment || state.raw.kinematic_preview) {
      if (verification && !state.pendingSegment) verification.hidden = true;
      return;
    }
    if (!latestRunId) return;
    if (verifiedRuns[latestRunId] && verifiedRuns[latestRunId] !== 'checking') {
      showVerification(verifiedRuns[latestRunId]);
      return;
    }
    if (verifiedRuns[latestRunId] === 'checking') return;
    var runId = latestRunId;
    verifiedRuns[runId] = 'checking';
    verification.hidden = false; verification.textContent = '正在检查实际飞行记录';
    fetch(SERVICE_HTTP_BASE + '/api/evaluate', { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ run_id: runId }) }).then(function (response) {
      if (!response.ok) throw new Error('检查暂不可用');
      return response.json();
    }).then(function (result) {
      verifiedRuns[runId] = result.verdict;
      if (latestRunId !== runId || state.activeSegment || displayMode !== 'live') return;
      showVerification(result.verdict);
    }).catch(function () {
      verifiedRuns[runId] = 'unavailable';
      if (latestRunId === runId && !state.activeSegment && displayMode === 'live') showVerification('unavailable');
    });
  }

  function showVerification(verdict) {
    verification.hidden = false;
    verification.dataset.verdict = verdict;
    verification.textContent = verdict === 'pass' ? '实际飞行检查通过 · 可回放'
      : verdict === 'fail' ? '实际飞行检查未通过 · 查看运行详情'
      : verdict === 'preview' ? '规模预演完成 · 未进行物理飞行'
      : verdict === 'unavailable' ? '实际飞行检查暂不可用' : '记录尚未完整结束';
  }

  function applyReplayFrame(index) {
    var frame = replayFrames[index];
    if (frame === null || frame === undefined) return;
    replayIndex = index;
    replayElapsed = frame.t;
    var total = replayFrames[replayFrames.length - 1].t;
    replaySeek.value = total > 0 ? Math.round(frame.t / total * 1000) : 0;
    replayClock.textContent = Core.formatNumber(frame.t, 1) + ' / ' + Core.formatNumber(total, 1) + ' s';
    var replayError = null;
    if (!replayPreview && frame.target_positions && frame.target_positions.length === frame.positions.length) {
      var squared = frame.positions.map(function (point, i) {
        return point.reduce(function (sum, value, axis) { return sum + Math.pow(value - frame.target_positions[i][axis], 2); }, 0);
      });
      replayError = { max: Math.sqrt(Math.max.apply(null, squared)), rms: Math.sqrt(squared.reduce(function (a, b) { return a + b; }, 0) / squared.length) };
    }
    var raw = {
      type: 'state',
      data: {
        simulation_time: frame.t,
        show_elapsed_seconds: frame.t,
        num_drones: (frame.positions || []).length,
        status: 'replay',
        kinematic_preview: replayPreview,
        offline_execution: replayOffline,
        active_segment: frame.active_segment || null,
        pending_segment: null,
        positions: frame.positions || [],
        velocities: [],
        target_positions: frame.target_positions || [],
        actual_error: replayError,
        last_error: null,
        show_revision: state.showRevision,
        request_revision: state.requestRevision
      }
    };
    applyNormalizedState(Core.normalizeState(raw));
  }

  function replayStep(now) {
    if (replayActive === false) return;
    if (replayPaused) {
      replayTimer = window.requestAnimationFrame(replayStep);
      return;
    }
    var elapsed = (now - replayStartedAt) / 1000;
    var index = replayIndex;
    while (index + 1 < replayFrames.length && replayFrames[index + 1].t <= elapsed) index++;
    if (elapsed <= replayFrames[replayFrames.length - 1].t) {
      if (index !== replayIndex) applyReplayFrame(index);
      replayTimer = window.requestAnimationFrame(replayStep);
    } else {
      applyReplayFrame(replayFrames.length - 1);
      exitReplay('replay finished', true);
    }
  }

  function startReplay(requestedRunId) {
    var runId = typeof requestedRunId === 'string' ? requestedRunId : latestRunId;
    if (runId === null || !/^r_[a-f0-9]{12}$/.test(runId)) return;
    replayLiveRunAtStart = state.raw.active_run_id || state.raw.latest_run_id || null;
    fetchJson(SERVICE_HTTP_BASE + '/api/run/' + encodeURIComponent(runId) + '/replay?hz=' + REPLAY_HZ).then(function (body) {
      var frames = body && Array.isArray(body.frames) ? body.frames : [];
      if (frames.length === 0) {
        serviceError = 'Replay returned no frames for ' + runId;
        renderErrors();
        return;
      }
      replayFrames = frames;
      replayPreview = !!body.kinematic_preview;
      replayOffline = !!body.offline_execution;
      replayActive = true;
      replayPaused = false;
      replayPause.textContent = '暂停';
      replayTransport.hidden = false;
      replayIndex = 0;
      trails = [];
      targetTrails = [];
      replayStartedAt = window.performance.now();
      setDisplayMode('replay');
      applyReplayFrame(0);
      addEvent('replay_started', null, 'run=' + runId + ' frames=' + frames.length);
      updateLiveState();
      if (replayTimer) window.cancelAnimationFrame(replayTimer);
      replayTimer = window.requestAnimationFrame(replayStep);
    }).catch(function (err) {
      serviceError = 'Replay failed: ' + err.message;
      renderErrors();
    });
  }

  function exitReplay(reason, keepFrame) {
    replayTransport.hidden = true;
    var wasReplay = replayActive;
    replayActive = false;
    if (replayTimer) window.cancelAnimationFrame(replayTimer);
    replayTimer = null;
    if (keepFrame) {
      setDisplayMode('ended');
    } else {
      trails = [];
      targetTrails = [];
      setDisplayMode('live');
    }
    if (wasReplay) addEvent(keepFrame ? 'replay_finished' : 'replay_cancelled', null, reason || '');
    updateLiveState();
    updateConnectionDisplay();
    renderErrors();
  }

  function handleIncomingState(raw, source) {
    var next = Core.normalizeState(raw);
    rememberLatestRun(next.raw);
    if (displayMode !== 'live') {
      var liveStarting = next.status === 'running' || next.activeSegment !== null;
      if (displayMode === 'replay' || displayMode === 'ended') {
        var liveRunId = next.raw.active_run_id || next.raw.latest_run_id || null;
        if (replayLiveRunAtStart === null) replayLiveRunAtStart = liveRunId;
        liveStarting = liveStarting && (!liveRunId || liveRunId !== replayLiveRunAtStart);
      }
      if (liveStarting) {
        if (displayMode === 'replay') exitReplay('new live run started', false);
        else {
          setDisplayMode('live');
          trails = [];
          targetTrails = [];
        }
      } else if (displayMode === 'replay' || displayMode === 'ended') {
        updateLiveState();
        return;
      }
    }
    applyNormalizedState(next);
    if (source === 'poll') lastPollAt = Date.now();
  }

  function addEvent(kind, at, detail) {
    events.unshift({ kind: kind, at: at, detail: detail, received: new Date() });
    if (events.length > MAX_EVENTS) events.length = MAX_EVENTS;
    renderEvents();
  }

  function renderEvents() {
    if (!dom.eventList) return;
    if (events.length === 0) {
      dom.eventList.innerHTML = '<li class="muted">Waiting for telemetry…</li>';
      return;
    }
    var html = '';
    for (var i = 0; i < events.length; i++) {
      var item = events[i];
      var summary = detailSummary(item.detail);
      html += '<li><span class="event-time">' + esc(eventTime(item.at)) + '</span> ' + esc(item.kind);
      if (summary) html += ' — ' + esc(summary);
      html += '</li>';
    }
    dom.eventList.innerHTML = html;
  }

  function esc(value) {
    return String(value)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;');
  }

  function renderErrors() {
    var parts = [];
    if (serviceError) parts.push('Service error: ' + serviceError);
    if (protocolError) parts.push('WebSocket / protocol: ' + protocolError);
    if (connectionStatus !== 'connected' && connectionDetail) parts.push('Connection: ' + connectionDetail);
    dom.errorText.textContent = parts.length
      ? parts.join('  |  ')
      : '运行正常';
    dom.errorText.style.color = parts.length ? '#ffd4d9' : '#c8d5ef';

    var warnings = [];
    if (state.missingActualWarning) warnings.push(state.missingActualWarning);
    if (showTargets && state.targetPositions.length === 0) {
      warnings.push('Reference target overlay is on, but state.target_positions is empty.');
    }
    if (warnings.length) {
      dom.warningText.textContent = warnings.join('  |  ');
      dom.warningText.hidden = false;
    } else {
      dom.warningText.textContent = '';
      dom.warningText.hidden = true;
    }
  }

  function renderPanel() {
    var current = state.activeSegment;
    var pending = state.pendingSegment;
    setText(dom.currentSegment, phaseLabel(current));
    setText(dom.currentSegmentId, current
      ? (Core.isFiniteNumber(current.duration) ? 'duration ' + Core.formatNumber(current.duration, 1) + 's' : '')
      : '等待指令');
    setText(dom.pendingSegment, pending ? Core.segmentLabel(pending) : 'none');
    setText(dom.pendingSegmentId, pending && pending.id ? 'queued id ' + pending.id : 'no queued request');
    var elapsed = state.raw.show_elapsed_seconds;
    setText(dom.simTime, Core.isFiniteNumber(elapsed) ? elapsed.toFixed(1) + ' 秒' : '—');
    setText(dom.dronesCount, String(state.numDrones) + ' 架无人机');
    setText(dom.revisions, Core.formatRevision(state.showRevision) + ' / ' + Core.formatRevision(state.requestRevision));
    setText(dom.maxError, Core.isFiniteNumber(state.actualMaxError) ? Core.formatNumber(state.actualMaxError, 3) + ' m' : '—');
    setText(dom.rmsError, Core.isFiniteNumber(state.actualRmsError) ? 'RMS ' + Core.formatNumber(state.actualRmsError, 3) + ' m' : 'RMS —');
    setText(dom.progressText, Core.formatNumber(Core.progressPercent(current), 0) + '%');
    dom.progressFill.style.width = Core.clamp(Core.progressPercent(current), 0, 100).toFixed(1) + '%';
    dom.statusChip.textContent = state.status === 'running' ? '表演中' : state.status === 'replay' ? '回放中'
      : state.status === 'idle' || state.status === 'completed' ? '待命' : state.status === 'error' ? '运行异常' : '连接中';
    dom.statusChip.setAttribute('data-status', state.status);
    renderColorIndicator();
    updateLiveState();
    renderErrors();
  }

  function renderColorIndicator() {
    var segment = state.activeSegment;
    var displayColors = actualColors();
    var declaredCount = state.positions.length;
    if (declaredCount === 0 && segment) {
      var declared = segment.colors && segment.colors.length ? segment.colors.length : 1;
      displayColors = [];
      var d = 0;
      for (; declared > d; d++) {
        displayColors.push(Core.colorForDrone(segment, d, declared));
      }
      declaredCount = declared;
    }
    dom.colorSwatches.textContent = '';
    var shown = Math.min(displayColors.length, 24);
    var i = 0;
    for (; shown > i; i++) {
      var chip = document.createElement('span');
      chip.className = 'color-chip';
      chip.style.background = displayColors[i];
      chip.title = 'actual drone ' + i + ': ' + displayColors[i];
      dom.colorSwatches.appendChild(chip);
    }
    if (displayColors.length > shown) {
      var more = document.createElement('span');
      more.className = 'color-chip more';
      more.textContent = '+' + (displayColors.length - shown);
      dom.colorSwatches.appendChild(more);
    }
    var source = 'generated fallback palette';
    if (segment) {
      if (segment.colors && segment.colors.length) source = 'active_segment.colors[' + segment.colors.length + ']';
      else if (segment.color) source = 'active_segment.color ' + segment.color;
      else source = 'active segment without color metadata';
    } else if (declaredCount > 0) {
      source = 'fallback palette';
    }
    dom.colorSource.textContent = source;
    dom.colorNote.textContent = declaredCount + ' actual drone color' + (declaredCount === 1 ? '' : 's')
      + ' · points and trails update from each state message. Reference targets stay amber dashed and labeled.';
  }

  function renderHud() {
    var count = state.positions.length;
    setText(dom.hudDrones, String(count) + ' 架无人机');
    var current = state.activeSegment;
    setText(dom.hudSegment, current
      ? phaseLabel(current) + ' · ' + Core.formatNumber(Core.progressPercent(current), 0) + '%'
      : '等待指令');
  }

  function setConnection(nextStatus, detail) {
    connectionStatus = nextStatus;
    connectionDetail = detail || '';
    updateConnectionDisplay();
    renderErrors();
  }

  function updateRetryCountdown() {
    if (!retryAt) {
      setText(dom.retryText, '');
      return;
    }
    var remaining = Math.max(0, Math.ceil((retryAt - Date.now()) / 1000));
    setText(dom.retryText, 'retry in ' + remaining + 's');
  }

  function scheduleReconnect(reason) {
    if (reconnectTimer) return;
    if (retryAt && Date.now() < retryAt && connectionStatus === 'disconnected') return;
    var delay = reconnectDelay;
    reconnectDelay = Math.min(reconnectDelay * 1.8, 10000);
    retryAt = Date.now() + delay;
    updateRetryCountdown();
    setConnection('disconnected', (reason ? reason + '. ' : '') + 'Retrying in ' + Math.round(delay / 1000) + 's');
    reconnectTimer = window.setTimeout(function () {
      reconnectTimer = null;
      retryAt = 0;
      connect();
    }, delay);
  }

  function handleRejected(detail) {
    var summary = detailSummary(detail);
    serviceError = summary ? 'plan rejected: ' + summary : 'plan rejected';
    renderErrors();
  }

  function handleServiceEvent(data) {
    var kind = data && data.kind ? data.kind : 'event';
    addEvent(kind, data ? data.at : null, data ? data.detail : null);
    if (kind === 'amendment_planning' || kind === 'amendment_waiting' || kind === 'amended') {
      verification.hidden = false; verification.dataset.verdict = '';
      verification.textContent = kind === 'amendment_planning' ? '正在编排新场景 · 原表演继续'
        : kind === 'amendment_waiting' ? '安全检查通过 · 等待稳定阶段接入' : '已接入新编排';
      requestStatus = { text: verification.textContent,
        runId: kind === 'amendment_waiting' ? data.detail.run_id : requestStatus && requestStatus.runId };
    }
    if (kind === 'segment_started' && requestStatus && data.detail.run_id === requestStatus.runId) requestStatus = null;
    if (kind === 'plan_rejected') {
      requestStatus = { text: '新编排未接入 · 查看运行详情', runId: null };
      verification.hidden = false; verification.textContent = requestStatus.text; verification.dataset.verdict = 'fail';
    }
    if (kind === 'plan_rejected') handleRejected(data.detail);
    if (kind === 'error') serviceError = detailSummary(data.detail) || 'service reported an error';
    if (kind === 'segment_started' || kind === 'segment_finished' || kind === 'completed') {
      serviceError = null;
    }
    renderErrors();
  }

  function applyState(rawState) {
    applyNormalizedState(Core.normalizeState(rawState));
  }

  function applyNormalizedState(next) {
    previousPositions = state.positions;
    var incomingAt = window.performance.now();
    frameInterval = Math.min(200, Math.max(25, incomingAt - receivedAt));
    receivedAt = incomingAt;
    state = next;
    if (dom.modeBadge && displayMode === 'live') {
      dom.modeBadge.textContent = state.raw.kinematic_preview ? '规模预演 · 非物理飞行' : state.raw.offline_execution ? '离线物理仿真' : '物理仿真';
    }
    var phase = state.activeSegment ? state.activeSegment.name : '';
    var label = phase.indexOf(':') >= 0 ? phase.split(':')[1].split('_')[0] : '';
    var phaseTitle = '等待下一场表演';
    if (phase === 'takeoff') phaseTitle = '起飞';
    else if (phase.indexOf('assemble:') === 0 || phase === 'transition') phaseTitle = '正在编队';
    else if (phase === 'return') phaseTitle = '归位';
    else if (phase === 'clearance') phaseTitle = '安全过渡';
    else if (phase === 'landing') phaseTitle = '降落';
    else if (phase.indexOf('dwell:') === 0) phaseTitle = '稳定悬停';
    else if (phase.indexOf('scene') === 0 || phase === 'hold') phaseTitle = phaseLabel(state.activeSegment);
    if (/^(takeoff|landing|return):/.test(phase)) phaseTitle = phase.split(':')[1];
    if (document.getElementById('show-phase')) document.getElementById('show-phase').textContent = phaseTitle;
    var titles = { heart: '爱心', ring: '圆环', star: '星星', grid: '方阵', helix: '双螺旋', atom: '原子轨道', bird: '振翅飞鸟' };
    if (document.getElementById('show-content')) document.getElementById('show-content').textContent = titles[label] || label;
    trails = Core.pushTrails(trails, state.positions, TRAIL_LENGTH);
    targetTrails = Core.pushTrails(targetTrails, state.targetPositions, TARGET_TRAIL_LENGTH);

    if (autoFollow && state.positions.length > 0) {
      var name = state.activeSegment ? state.activeSegment.name : '';
      var overhead = name && !/^(takeoff|landing|return)(:|$)/.test(name);
      var visiblePoints = state.positions.filter(function (_, i) {
        return !overhead || !state.activeSegment.colors.length || state.activeSegment.colors[i] !== '#26334b';
      });
      var fit = Core.viewTargetFor(visiblePoints.length ? visiblePoints : state.positions, view.target);
      var textScene = name.indexOf('helix') < 0 && name.indexOf('atom') < 0 && name.indexOf('bird') < 0
        && name.indexOf('heart') < 0 && name.indexOf('ring') < 0 && name.indexOf('star') < 0 && name.indexOf('grid') < 0;
      var dynamicScene = /helix|atom|bird|双螺旋|星河绽放|千机光浪/.test(name);
      var pitch = overhead ? (dynamicScene ? .48 : textScene ? 1.047 : 1.49) : .72;
      if (name.indexOf('AGNES') >= 0 && (state.raw.kinematic_preview || state.raw.offline_execution)) pitch = .12;
      if (/A诞生|金色A/.test(name)) pitch = 1.134;
      view.pitch += (pitch - view.pitch) * .08;
      var cameraYaw = dynamicScene ? Math.sin((state.raw.show_elapsed_seconds || 0) * .12) * .45 : 0;
      view.yaw += (cameraYaw - view.yaw) * .08;
      for (var axis = 0; axis < 3; axis++) view.target[axis] += (fit.target[axis] - view.target[axis]) * .12;
      if (displayMode === 'replay' && replayPaused) {
        // A seek supplies one frame, so camera convergence cannot depend on
        // receiving subsequent flight samples while playback is paused.
        view.pitch = pitch; view.yaw = cameraYaw; view.target = fit.target.slice();
      }
      var desired = Core.clamp(fit.span * (dynamicScene ? 1.35 : overhead ? 1.05 : 1.1) + 1.4, 3, 70);
      if (stage) desired = Core.clamp(Core.perspectiveFitDistance(
        visiblePoints.length ? visiblePoints : state.positions, view,
        dom.canvas.clientWidth / Math.max(1, dom.canvas.clientHeight)), 2.5, 70);
      view.distance = Math.max(desired, view.distance + (desired - view.distance) * .08);
      if (displayMode === 'replay' && replayPaused) view.distance = desired;
      if (/^takeoff:/.test(name) && state.raw.show_elapsed_seconds < 2.5) {
        view.target = state.positions[0].slice();
        view.pitch = .35; view.yaw = -.2; view.distance = 1.4;
      }
    }

    if (state.lastError) {
      serviceError = state.lastError;
    } else {
      serviceError = null;
    }
    protocolError = null;
    renderPanel();
    renderHud();
    updateConnectionDisplay();
  }

  function handleMessage(event) {
    lastWsMessageAt = Date.now();
    pendingPingAt = null;
    var payload;
    try {
      payload = JSON.parse(event.data);
    } catch (err) {
      protocolError = 'received non-JSON WebSocket message';
      renderErrors();
      return;
    }

    if (payload && payload.type === 'pong') return;

    if (payload && payload.type === 'hello') {
      addEvent('hello', null, 'service=' + (payload.service || '?') + ' protocol=' + (payload.protocol || '?'));
      return;
    }

    if (payload && payload.type === 'event') {
      handleServiceEvent(payload.data || payload);
      return;
    }

    if (payload && payload.type === 'state') {
      lastMessageAt = Date.now();
      handleIncomingState(payload.data || payload, 'ws');
      return;
    }

    if (payload && (Array.isArray(payload.positions) || Array.isArray(payload.target_positions))) {
      lastMessageAt = Date.now();
      handleIncomingState(payload, 'ws');
      return;
    }

    protocolError = 'unrecognized WebSocket message: ' + String(payload && payload.type ? payload.type : typeof payload);
    renderErrors();
  }

  function connect() {
    if (ws) {
      try { ws.close(); } catch (err) { /* replaced below */ }
    }
    window.clearTimeout(reconnectTimer);
    reconnectTimer = null;
    retryAt = 0;
    setConnection('connecting', 'Opening ' + WS_URL);
    setText(dom.retryText, '');

    var socket;
    try {
      socket = new WebSocket(WS_URL);
    } catch (err) {
      scheduleReconnect('WebSocket constructor failed: ' + err.message);
      return;
    }
    ws = socket;

    socket.onopen = function () {
      pendingPingAt = null;
      reconnectDelay = 1000;
      retryAt = 0;
      protocolError = null;
      lastWsMessageAt = Date.now();
      forceReconnectArmed = false;
      setConnection('connected', 'Connected to ' + WS_URL);
      addEvent('connected', null, WS_URL);
      updateRetryCountdown();
    };

    socket.onmessage = handleMessage;

    socket.onerror = function () {
      setConnection('stalled', 'WebSocket transport error on ' + WS_URL);
    };

    socket.onclose = function (event) {
      ws = null;
      lastWsMessageAt = null;
      if (connectionStatus === 'connected' || connectionStatus === 'stalled' || connectionStatus === 'connecting') {
        var suffix = event && event.code ? ' (close code ' + event.code + ')' : '';
        scheduleReconnect('Connection closed' + suffix);
      }
    };
  }

  function resizeCanvas() {
    var rect = dom.canvas.getBoundingClientRect();
    var width = Math.max(320, Math.floor(rect.width || 320));
    var height = Math.max(240, Math.floor(rect.height || 240));
    dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
    var pixelWidth = Math.floor(width * dpr);
    var pixelHeight = Math.floor(height * dpr);
    if (dom.canvas.width !== pixelWidth || dom.canvas.height !== pixelHeight) {
      dom.canvas.width = pixelWidth;
      dom.canvas.height = pixelHeight;
    }
    return { width: width, height: height };
  }

  function drawGroundGrid(width, height) {
    var extent = 10;
    var lines = [];
    var i;
    for (i = -extent; i <= extent; i += 2) {
      lines.push([[-extent, i, 0], [extent, i, 0]]);
      lines.push([[i, -extent, 0], [i, extent, 0]]);
    }
    ctx.save();
    ctx.lineWidth = 1;
    for (i = 0; i < lines.length; i++) {
      var a = Core.project(lines[i][0], view, width, height);
      var b = Core.project(lines[i][1], view, width, height);
      if (!(a.visible && b.visible)) continue;
      ctx.globalAlpha = i % 2 === 0 ? 0.10 : 0.055;
      ctx.strokeStyle = '#9fc3ff';
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawTrails(trailList, colorList, dashed, alphaBase) {
    var width = dom.canvas.clientWidth || 1;
    var height = dom.canvas.clientHeight || 1;
    ctx.save();
    ctx.lineWidth = dashed ? 1.4 : 1.05;
    if (dashed) ctx.setLineDash([5, 5]);
    for (var i = 0; i < trailList.length; i++) {
      var trail = trailList[i];
      if (!trail || trail.length < 2) continue;
      var color = colorList[i % colorList.length] || '#5ac8fa';
      for (var j = 1; j < trail.length; j++) {
        var a = Core.project(trail[j - 1], view, width, height);
        var b = Core.project(trail[j], view, width, height);
        if (!(a.visible && b.visible)) continue;
        var age = j / trail.length;
        ctx.globalAlpha = alphaBase * (0.18 + age * 0.62);
        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
    }
    ctx.setLineDash([]);
    ctx.restore();
  }

  function actualColors() {
    var colors = [];
    var count = state.positions.length;
    for (var i = 0; i < count; i++) {
      colors.push(Core.colorForDrone(state.activeSegment, i, count));
    }
    return colors.length ? colors : [Core.colorFromIndex(0)];
  }

  function drawActual(width, height) {
    var colors = actualColors();
    drawTrails(trails, colors, false, 0.18);
    ctx.save();
    for (var i = 0; i < state.positions.length; i++) {
      var point = state.positions[i];
      if (previousPositions.length === state.positions.length) {
        var u = Core.clamp((window.performance.now() - receivedAt) / frameInterval, 0, 1);
        point = point.map(function (v, k) { return previousPositions[i][k] + (v - previousPositions[i][k]) * u; });
      }
      var p = Core.project(point, view, width, height);
      if (p.visible === false) continue;
      var color = colors[i % colors.length];
      var radius = Core.clamp(p.scale * 0.065, 1.5, 3.8);
      var arm = Core.clamp(p.scale * .075, 1.8, 11);
      if (arm > 3.5) {
        ctx.save();
        ctx.translate(p.x, p.y);
        ctx.strokeStyle = '#667994';
        ctx.lineWidth = Math.max(1, arm * .16);
        ctx.globalAlpha = color === '#26334b' ? .12 : (view.pitch > 1.2 && view.distance > 5 ? .36 : .85);
        ctx.beginPath();
        ctx.moveTo(-arm, -arm * .6); ctx.lineTo(arm, arm * .6);
        ctx.moveTo(-arm, arm * .6); ctx.lineTo(arm, -arm * .6);
        ctx.stroke();
        for (var motor = 0; motor < 4; motor++) {
          var mx = (motor < 2 ? -1 : 1) * arm;
          var my = (motor % 2 ? -1 : 1) * arm * .6;
          ctx.strokeStyle = '#9dafc6'; ctx.lineWidth = .8;
          ctx.beginPath(); ctx.ellipse(mx, my, arm * .38, arm * .22, 0, 0, Math.PI * 2); ctx.stroke();
          if (point[2] > .3) {
            var theta = window.performance.now() * .025 + motor;
            ctx.beginPath();
            ctx.moveTo(mx - Math.cos(theta) * arm * .3, my - Math.sin(theta) * arm * .16);
            ctx.lineTo(mx + Math.cos(theta) * arm * .3, my + Math.sin(theta) * arm * .16);
            ctx.stroke();
          }
        }
        ctx.fillStyle = '#a5b8d1'; ctx.fillRect(-arm * .22, -arm * .18, arm * .44, arm * .36);
        ctx.restore();
      }
      var glowRadius = radius * 2.1;
      var gradient = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, glowRadius);
      gradient.addColorStop(0, color);
      gradient.addColorStop(0.45, color);
      gradient.addColorStop(1, 'rgba(0, 0, 0, 0)');
      ctx.globalCompositeOperation = 'lighter';
      ctx.globalAlpha = color === '#26334b' ? .08 : .18;
      ctx.fillStyle = gradient;
      ctx.beginPath();
      ctx.arc(p.x, p.y, glowRadius, 0, Math.PI * 2);
      ctx.fill();
      ctx.globalCompositeOperation = 'source-over';
      ctx.globalAlpha = 0.88;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  function drawTargets(width, height) {
    if (!showTargets) return;
    if (state.targetPositions.length === 0) return;
    var targetColor = ['#ffb038'];
    drawTrails(targetTrails, targetColor, true, 0.92);
    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = '#ffb038';
    ctx.lineWidth = 1.5;
    for (var i = 0; i < state.targetPositions.length; i++) {
      var p = Core.project(state.targetPositions[i], view, width, height);
      if (!p.visible) continue;
      var r = Core.clamp(p.scale * 0.11, 3.5, 9);
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(p.x - r * 1.6, p.y);
      ctx.lineTo(p.x + r * 1.6, p.y);
      ctx.moveTo(p.x, p.y - r * 1.6);
      ctx.lineTo(p.x, p.y + r * 1.6);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.restore();
  }

  function renderFrame(now) {
    window.requestAnimationFrame(renderFrame);
    // The physical feed is 20 Hz. Interpolate at 30 fps for smooth recording
    // without competing with the simulator at the display's 60/120 Hz rate.
    if (document.hidden || now - lastRenderedAt < 1000 / 30 - 1) return;
    lastRenderedAt = now;
    if (stage) {
      var u = Core.clamp((window.performance.now() - receivedAt) / frameInterval, 0, 1);
      var interpolated = state.positions.map(function (point, i) {
        return previousPositions.length === state.positions.length
          ? point.map(function (v, k) { return previousPositions[i][k] + (v - previousPositions[i][k]) * u; }) : point;
      });
      if (closeupDroneIndex !== null && interpolated[closeupDroneIndex]) view.target = interpolated[closeupDroneIndex].slice();
      stage.render(state, interpolated, view, trails, showTargets, displayMode === 'replay' || displayMode === 'ended' || liveDataFresh(), now);
      lastRenderedAt = now;
      return;
    }
    var size = resizeCanvas();
    var width = size.width;
    var height = size.height;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    drawGroundGrid(width, height);

    var dim = connectionStatus === 'connected' ? 1 : 0.30;
    ctx.save();
    ctx.globalAlpha = dim;
    drawTargets(width, height);
    drawActual(width, height);
    ctx.restore();

    if (state.positions.length === 0 && connectionStatus === 'connected') {
      ctx.save();
      ctx.fillStyle = 'rgba(143, 160, 191, 0.9)';
      ctx.font = '13px ui-monospace, Menlo, monospace';
      ctx.textAlign = 'center';
      ctx.fillText('Connected, waiting for actual positions in state.positions…', width / 2, height / 2);
      ctx.restore();
    }

    lastRenderedAt = now || Date.now();
  }

  function resetView() {
    closeupDroneIndex = null;
    autoFollow = true; dom.chkFollow.checked = true;
    var fit = Core.viewTargetFor(state.positions, [0, 0, 1.2]);
    view = Core.createView({
      yaw: 0,
      pitch: 1.49,
      distance: fit && fit.span ? Core.clamp(fit.span * .8 + 1.4, 2.5, 70) : 9,
      target: fit && fit.target ? fit.target : [0, 0, 1.2]
    });
  }

  function setShowTargets(value) {
    showTargets = !!value;
    dom.btnTargets.setAttribute('aria-pressed', showTargets ? 'true' : 'false');
    dom.referenceLabel.hidden = !showTargets;
    dom.referenceLegend.hidden = !showTargets;
    renderErrors();
  }

  function bindControls() {
    var dragging = null;
    dom.canvas.addEventListener('pointerdown', function (event) {
      if (event.button !== 0 && event.pointerType === 'mouse') return;
      dragging = { id: event.pointerId, x: event.clientX, y: event.clientY };
      closeupDroneIndex = null;
      autoFollow = false;
      dom.chkFollow.checked = false;
      dom.canvas.classList.add('dragging');
      if (dom.canvas.setPointerCapture) dom.canvas.setPointerCapture(event.pointerId);
    });

    dom.canvas.addEventListener('pointermove', function (event) {
      if (!dragging || dragging.id !== event.pointerId) return;
      var dx = event.clientX - dragging.x;
      var dy = event.clientY - dragging.y;
      dragging.x = event.clientX;
      dragging.y = event.clientY;
      view.yaw -= dx * 0.006;
      view.pitch = Core.clamp(view.pitch + dy * 0.006, -1.42, 1.42);
    });

    function endDrag(event) {
      if (!dragging || (event && dragging.id !== event.pointerId)) return;
      dragging = null;
      dom.canvas.classList.remove('dragging');
    }
    dom.canvas.addEventListener('pointerup', endDrag);
    dom.canvas.addEventListener('pointercancel', endDrag);
    dom.canvas.addEventListener('wheel', function (event) {
      event.preventDefault();
      autoFollow = false;
      closeupDroneIndex = null;
      dom.chkFollow.checked = false;
      var factor = Math.exp(event.deltaY * 0.0012);
      view.distance = Core.clamp(view.distance * factor, 1.5, 80);
    }, { passive: false });
    dom.canvas.addEventListener('dblclick', resetView);

    dom.btnReset.addEventListener('click', resetView);
    document.getElementById('btn-fullscreen').addEventListener('click', function () {
      if (document.fullscreenElement) document.exitFullscreen();
      else document.getElementById('viewport').requestFullscreen();
    });
    var captureDialog = document.getElementById('capture-preview');
    document.getElementById('capture-close').addEventListener('click', function () { captureDialog.close(); });
    captureDialog.addEventListener('close', function () {
      document.getElementById('capture-image').removeAttribute('src');
    });
    document.getElementById('btn-export').addEventListener('click', function () {
      if (!stage) return;
      stage.capture().then(function (blob) {
        if (!blob) return;
        return fetch(SERVICE_HTTP_BASE + '/api/frame-captures', { method: 'POST',
          headers: { 'Content-Type': 'image/png' }, body: blob }).then(function (response) {
          if (!response.ok) throw new Error('画面暂时无法导出');
          return response.json();
        });
      }).then(function (result) {
        if (!result) return;
        document.getElementById('capture-image').src = result.url;
        var link = document.getElementById('capture-save');
        link.href = result.url; link.download = 'drone-show-' + Date.now() + '.png';
        captureDialog.showModal();
      }).catch(function () {
        dom.warningText.hidden = false; dom.warningText.textContent = '画面暂时无法导出，请稍后再试';
      });
    });
    document.getElementById('btn-closeup').addEventListener('click', function () {
      if (!state.positions.length) return;
      autoFollow = false; dom.chkFollow.checked = false;
      var index = state.positions.findIndex(function (p) { return p[2] > .5; });
      closeupDroneIndex = index < 0 ? 0 : index;
      view = Core.createView({ yaw: -.4, pitch: .45, distance: .9,
        target: state.positions[index < 0 ? 0 : index].slice() });
    });
    dom.btnTargets.addEventListener('click', function () {
      setShowTargets(!showTargets);
    });
    if (dom.btnReplay) dom.btnReplay.addEventListener('click', startReplay);
    replayPause.addEventListener('click', function () {
      replayPaused = !replayPaused;
      replayPause.textContent = replayPaused ? '继续' : '暂停';
      if (!replayPaused) replayStartedAt = window.performance.now() - replayElapsed * 1000;
    });
    replaySeek.addEventListener('input', function () {
      if (!replayActive || !replayFrames.length) return;
      var requestedProgress = Number(replaySeek.value);
      var time = requestedProgress / 1000 * replayFrames[replayFrames.length - 1].t;
      var index = 0;
      while (index + 1 < replayFrames.length && replayFrames[index + 1].t <= time) index++;
      replayPaused = true;
      replayPause.textContent = '继续';
      trails = []; targetTrails = [];
      applyReplayFrame(index);
      replaySeek.value = requestedProgress;
    });
    dom.chkFollow.addEventListener('change', function () {
      autoFollow = dom.chkFollow.checked;
      if (autoFollow) {
        var fit = Core.viewTargetFor(state.positions, view.target);
        if (fit && fit.target) view.target = fit.target;
      }
    });
  }

  function heartbeat() {
    updateRetryCountdown();
    if (lastMessageAt) {
      var age = Date.now() - lastMessageAt;
      setText(dom.lastMessageAge, 'age ' + Core.formatNumber(age / 1000, 1) + 's');
    } else {
      setText(dom.lastMessageAge, 'no data yet');
    }
    // Missing telemetry can mean busy simulation or a backgrounded page. Probe
    // transport liveness before replacing a healthy socket; pong is NOT a frame.
    if (!document.hidden && connectionStatus === 'connected' && ws && ws.readyState === WebSocket.OPEN
        && lastWsMessageAt !== null && (Date.now() - lastWsMessageAt) > 1000) {
      if (pendingPingAt === null) {
        pendingPingAt = Date.now();
        try { ws.send(JSON.stringify({type: 'ping'})); } catch (err) { forceReconnect(); }
      } else if (Date.now() - pendingPingAt > STALE_MS) {
        setConnection('stalled', 'WebSocket liveness probe timed out');
        forceReconnect();
      }
    }
    updateConnectionDisplay();
    updateLiveState();
  }

  function boot() {
    var videoButton = document.getElementById('btn-record');
    var videoRecorder = null;
    var recordingTimer = null;
    if (videoButton) videoButton.addEventListener('click', function () {
      if (videoRecorder && videoRecorder.state === 'recording') { videoRecorder.stop(); return; }
      if (!replayActive && replayFrames.length && displayMode === 'ended') {
        replayActive = true; setDisplayMode('replay'); replayTransport.hidden = false;
        replayTimer = window.requestAnimationFrame(replayStep);
      }
      if (!replayActive || !replayFrames.length) {
        dom.warningText.hidden = false;
        dom.warningText.textContent = '请先打开本场回放，再录制整场';
        return;
      }
      replayPaused = false; replayPause.textContent = '暂停';
      replayIndex = 0; trails = []; targetTrails = []; autoFollow = true;
      applyReplayFrame(0); previousPositions = state.positions;
      replayStartedAt = window.performance.now();
      if (stage) stage.render(state, state.positions, view, trails, showTargets, true, replayStartedAt);
      var stream = dom.canvas.captureStream(30);
      var mime = 'video/webm;codecs=vp8';
      var chunks = [];
      var save = document.getElementById('video-save');
      if (save.href) URL.revokeObjectURL(save.href);
      save.hidden = true;
      videoRecorder = new MediaRecorder(stream, {mimeType: mime, videoBitsPerSecond: 6000000});
      videoRecorder.ondataavailable = function (event) { if (event.data.size) chunks.push(event.data); };
      videoRecorder.onstop = function () {
        window.clearTimeout(recordingTimer);
        stream.getTracks().forEach(function (track) { track.stop(); });
        var blob = new Blob(chunks, {type: mime});
        save.href = URL.createObjectURL(blob);
        save.download = 'agnes-actual-full-replay.webm';
        save.hidden = false; videoButton.textContent = '录制整场';
        fetch(SERVICE_HTTP_BASE + '/api/video-captures', {method: 'POST',
          headers: {'Content-Type': 'video/webm'}, body: blob}).then(function (response) {
          if (!response.ok) throw new Error('保存视频失败');
          return response.json();
        }).then(function (result) {
          URL.revokeObjectURL(save.href); save.href = result.url;
        }).catch(function () {
          dom.warningText.hidden = false;
          dom.warningText.textContent = '服务器保存失败，仍可使用本地视频链接';
        });
      };
      videoRecorder.start(1000); videoButton.textContent = '停止录制';
      recordingTimer = window.setTimeout(function () {
        if (videoRecorder.state === 'recording') videoRecorder.stop();
      }, (replayFrames[replayFrames.length - 1].t + 3) * 1000);
    });
    window.addEventListener('message', function (event) {
      if (event.origin !== window.location.origin || event.source !== window.parent || event.data?.type !== 'agnes-record-start') return;
      if (!replayActive || !replayFrames.length) return;
      replayPaused = false; replayPause.textContent = '暂停';
      replayIndex = 0; applyReplayFrame(0); replayStartedAt = window.performance.now();
    });
    bindControls();
    var detailButton = document.getElementById('btn-details');
    if (detailButton) detailButton.addEventListener('click', function () { document.body.classList.toggle('show-details'); });
    setShowTargets(showTargets);
    dom.chkFollow.checked = autoFollow;
    renderPanel();
    renderHud();
    renderEvents();
    setDisplayMode('live');
    setConnection('connecting', 'Opening ' + WS_URL);
    refreshLatestRun();
    var replayLink = new URLSearchParams(window.location.search).get('replay');
    if (replayLink && /^r_[a-f0-9]{12}$/.test(replayLink)) startReplay(replayLink);
    document.addEventListener('visibilitychange', visibilityRefresh);
    window.requestAnimationFrame(renderFrame);
    window.setInterval(heartbeat, 500);
    pollState();
    connect();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }
})();
