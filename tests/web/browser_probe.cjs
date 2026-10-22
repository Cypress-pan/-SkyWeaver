'use strict';

var childProcess = require('child_process');
var fs = require('fs');
var path = require('path');

var CHROME = process.env.CHROME_BIN || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
var url = process.argv[2] || 'http://127.0.0.1:8765/';
var screenshotPath = process.argv[3] || '';
var waitMs = Number(process.argv[4] || 4000);
var clickSelector = process.argv[5] || '';
var postClickWaitMs = Number(process.argv[6] || waitMs);
var port = Number(process.env.CHROME_CDP_PORT || 9227);
var userData = fs.mkdtempSync(path.join(require('os').tmpdir(), 'drone-show-chrome-'));
var child = null;

function sleep(ms) {
  return new Promise(function (resolve) { setTimeout(resolve, ms); });
}

async function waitJson(endpoint, options) {
  var tries = 0;
  while (tries > -1) {
    try {
      var response = await fetch(endpoint, options);
      if (response.ok) return await response.json();
    } catch (err) { /* retry */ }
    tries += 1;
    if (tries > 80) throw new Error('DevTools endpoint unavailable: ' + endpoint);
    await sleep(100);
  }
}

async function main() {
  child = childProcess.spawn(CHROME, [
    '--headless=new',
    '--disable-gpu',
    '--no-sandbox',
    '--remote-allow-origins=*',
    '--remote-debugging-port=' + port,
    '--user-data-dir=' + userData,
    'about:blank'
  ], { stdio: ['ignore', 'pipe', 'pipe'] });

  await waitJson('http://127.0.0.1:' + port + '/json/version');
  var target = await waitJson('http://127.0.0.1:' + port + '/json/new?' + encodeURIComponent(url), { method: 'PUT' });
  var socket = new WebSocket(target.webSocketDebuggerUrl);

  var seq = 0;
  var pending = new Map();

  socket.onmessage = function (event) {
    var message = JSON.parse(event.data);
    if (message.id && pending.has(message.id)) {
      var entry = pending.get(message.id);
      pending.delete(message.id);
      if (message.error) entry.reject(new Error(JSON.stringify(message.error)));
      else entry.resolve(message.result);
    }
  };

  await new Promise(function (resolve, reject) {
    socket.onopen = resolve;
    socket.onerror = function () { reject(new Error('DevTools WebSocket connection failed')); };
  });

  function send(method, params) {
    return new Promise(function (resolve, reject) {
      var id = ++seq;
      pending.set(id, { resolve: resolve, reject: reject });
      socket.send(JSON.stringify({ id: id, method: method, params: params || {} }));
    });
  }

  await send('Page.enable');
  await send('Runtime.enable');
  await send('Network.enable');
  await send('Page.navigate', { url: url });
  if (clickSelector) {
    await sleep(1200);
    await send('Runtime.evaluate', { expression: 'document.querySelector(' + JSON.stringify(clickSelector) + ').click()', returnByValue: true });
    await sleep(postClickWaitMs);
  } else {
    await sleep(waitMs);
  }

  var expression = "(() => { var t = function (id) { var el = document.getElementById(id); return el ? el.textContent : null; }; var ref = document.getElementById('reference-label'); var conn = document.getElementById('connection'); var list = document.getElementById('event-list'); var sw = document.getElementById('color-swatches'); return JSON.stringify({ ready: document.readyState, connectionText: t('conn-text'), connectionState: conn ? conn.getAttribute('data-state') : null, status: t('status-chip'), currentSegment: t('current-segment'), pendingSegment: t('pending-segment'), simulationTime: t('sim-time'), revisions: t('revisions'), actualMaxError: t('max-error'), actualRmsError: t('rms-error'), errorText: t('error-text'), warningText: t('warning-text'), referenceHidden: ref ? ref.hidden : null, colorSource: t('color-source'), colorNote: t('color-note'), colorChipCount: sw ? sw.childElementCount : -1, colorChipColors: sw ? Array.prototype.slice.call(sw.children, 0, 4).map(function (el) { return getComputedStyle(el).backgroundColor; }) : [], modeBadge: t('mode-badge'), readyBannerHidden: (document.getElementById('ready-banner') || {}).hidden, liveState: t('live-state'), replayStatus: t('replay-status'), btnReplayHidden: (document.getElementById('btn-replay') || {}).hidden, eventText: list ? list.innerText : '' }); })()";

  var evaluation = await send('Runtime.evaluate', {
    expression: expression,
    returnByValue: true,
    awaitPromise: false
  });
  console.log(evaluation.result.value);

  if (screenshotPath) {
    var shot = await send('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(screenshotPath, Buffer.from(shot.data, 'base64'));
    console.log('screenshot=' + screenshotPath);
  }

  socket.close();
}

main().catch(function (err) {
  console.error('PROBE_ERROR: ' + err.message);
  process.exitCode = 1;
}).finally(function () {
  if (child) child.kill('SIGKILL');
  setTimeout(function () {
    try { fs.rmSync(userData, { recursive: true, force: true }); } catch (err) { /* best effort */ }
  }, 300);
});
