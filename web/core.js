/* Drone Show telemetry core utilities.
 * Vanilla JS, no dependencies. Works in the browser and in Node for tests.
 *
 * IMPORTANT: actual drone positions come from state.positions only.
 * target_positions is reference data and is exposed separately. It is never
 * substituted into positions.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.DroneShowCore = factory();
  }
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  var DEFAULT_WS_URL = 'ws://127.0.0.1:8765/ws';
  var SOFT_PALETTE = ['#5ac8fa', '#4f8cff', '#7c6cff', '#4fd1c5', '#6f9dff', '#9a7bff'];

  function isFiniteNumber(value) {
    return typeof value === 'number' && Number.isFinite(value);
  }

  function numberOrNull(value) {
    if (isFiniteNumber(value)) return value;
    if (typeof value === 'string' && value.trim() !== '' && Number.isFinite(Number(value))) {
      return Number(value);
    }
    return null;
  }

  function clamp(value, lo, hi) {
    return Math.min(hi, Math.max(lo, value));
  }

  function point3(raw) {
    if (!Array.isArray(raw) || !(raw.length >= 3)) return null;
    var x = numberOrNull(raw[0]);
    var y = numberOrNull(raw[1]);
    var z = numberOrNull(raw[2]);
    if (x === null || y === null || z === null) return null;
    return [x, y, z];
  }

  function pointList(raw) {
    if (!Array.isArray(raw)) return [];
    var out = [];
    for (var i = 0; i < raw.length; i++) {
      var p = point3(raw[i]);
      if (p) out.push(p);
    }
    return out;
  }

  function errorText(value) {
    if (value === null || value === undefined || value === '') return null;
    if (typeof value === 'string') return value;
    if (typeof value === 'number' || typeof value === 'boolean') return String(value);
    if (value && typeof value === 'object') {
      if (typeof value.message === 'string' && value.message) return value.message;
      if (typeof value.error === 'string' && value.error) return value.error;
      if (typeof value.code === 'string' && value.code) {
        return value.detail ? value.code + ': ' + value.detail : value.code;
      }
      try {
        return JSON.stringify(value);
      } catch (err) {
        return String(value);
      }
    }
    return String(value);
  }

  function normalizeSegment(raw) {
    if (!raw || typeof raw !== 'object') return null;
    var colors = [];
    var candidateColors = raw.colors || raw.drone_colors || raw.droneColors || raw.palette;
    if (Array.isArray(candidateColors)) colors = candidateColors.slice();
    var color = typeof raw.color === 'string' ? raw.color : null;
    var name = typeof raw.name === 'string' && raw.name ? raw.name : 'segment';
    var id = typeof raw.id === 'string' && raw.id ? raw.id : null;
    var progress = numberOrNull(raw.progress);
    if (progress === null && isFiniteNumber(raw.p)) progress = raw.p;
    return {
      id: id,
      name: name,
      progress: progress === null ? 0 : clamp(progress, 0, 1),
      duration: numberOrNull(raw.duration),
      color: color,
      colors: colors,
      raw: raw
    };
  }

  function segmentLabel(segment) {
    if (!segment) return '—';
    var label = segment.name || 'segment';
    if (segment.id && segment.id !== segment.name) label += ' (' + segment.id + ')';
    return label;
  }

  function normalizeState(message) {
    var envelope = message && typeof message === 'object' ? message : {};
    var src = envelope;
    if (envelope.type === 'state' && envelope.data && typeof envelope.data === 'object') {
      src = envelope.data;
    }

    var positions = pointList(src.positions);
    var targetPositions = pointList(src.target_positions);
    var actualError = src.actual_error && typeof src.actual_error === 'object' ? src.actual_error : {};
    var numDrones = numberOrNull(src.num_drones);
    if (numDrones === null) numDrones = positions.length;

    return {
      simulationTime: numberOrNull(src.simulation_time),
      showRevision: numberOrNull(src.show_revision),
      requestRevision: numberOrNull(src.request_revision),
      numDrones: numDrones,
      status: typeof src.status === 'string' ? src.status : 'unknown',
      activeSegment: normalizeSegment(src.active_segment),
      pendingSegment: normalizeSegment(src.pending_segment),
      positions: positions,
      targetPositions: targetPositions,
      velocities: pointList(src.velocities),
      orientations: pointList(src.orientations),
      actualMaxError: numberOrNull(actualError.max),
      actualRmsError: numberOrNull(actualError.rms),
      lastError: errorText(src.last_error),
      missingActualWarning: positions.length === 0
        ? (targetPositions.length > 0
          ? 'positions[] missing: reference targets are NOT used as actual telemetry'
          : 'positions[] missing in state message')
        : null,
      raw: src
    };
  }

  function hsl(h, s, l) {
    return 'hsl(' + Math.round(((h % 360) + 360) % 360) + ', ' + s + '%, ' + l + '%)';
  }

  function colorFromIndex(index) {
    var count = SOFT_PALETTE.length;
    var i = ((index % count) + count) % count;
    return SOFT_PALETTE[i];
  }

  function isCssColor(value) {
    if (typeof value !== 'string') return false;
    var v = value.trim();
    if (!v) return false;
    if (/^#[0-9a-f]{3,8}$/i.test(v)) return true;
    if (/^rgba?\([^)]*\)$/i.test(v)) return true;
    if (/^hsla?\([^)]*\)$/i.test(v)) return true;
    return false;
  }

  function colorForDrone(segment, index, droneCount) {
    if (segment && typeof segment === 'object') {
      if (Array.isArray(segment.colors) && segment.colors.length) {
        var c = segment.colors[index % segment.colors.length];
        if (isCssColor(c)) return c;
      }
      if (isCssColor(segment.color)) return segment.color;
      if (isFiniteNumber(segment.hue)) return hsl(segment.hue + index * 12, 88, 66);
    }
    if (droneCount && droneCount > 0) {
      return colorFromIndex(index);
    }
    return colorFromIndex(index);
  }

  function createView(options) {
    var opts = options || {};
    return {
      yaw: isFiniteNumber(opts.yaw) ? opts.yaw : 0.65,
      pitch: isFiniteNumber(opts.pitch) ? opts.pitch : 0.38,
      distance: isFiniteNumber(opts.distance) ? opts.distance : 9,
      focal: isFiniteNumber(opts.focal) ? opts.focal : null,
      target: Array.isArray(opts.target) && (opts.target.length >= 3)
        ? [opts.target[0], opts.target[1], opts.target[2]]
        : [0, 0, 1.2]
    };
  }

  function viewTargetFor(positions, fallback) {
    var base = Array.isArray(fallback) && (fallback.length >= 3)
      ? [fallback[0], fallback[1], fallback[2]]
      : [0, 0, 1.2];
    if (!positions || positions.length === 0) return { target: base, span: 1.2 };
    var min = [Infinity, Infinity, Infinity];
    var max = [-Infinity, -Infinity, -Infinity];
    for (var i = 0; i < positions.length; i++) {
      for (var k = 0; k < 3; k++) {
        if (positions[i][k] < min[k]) min[k] = positions[i][k];
        if (positions[i][k] > max[k]) max[k] = positions[i][k];
      }
    }
    var center = [
      (min[0] + max[0]) / 2,
      (min[1] + max[1]) / 2,
      (min[2] + max[2]) / 2
    ];
    var span = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2], 0.8);
    return { target: center, span: span };
  }

  function perspectiveFitDistance(positions, view, aspect) {
    var cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
    var cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
    var vertical = Math.tan(48 * Math.PI / 360);
    var horizontal = vertical * Math.max(.1, aspect);
    var distance = 2.5;
    positions.forEach(function (p) {
      var dx = p[0] - view.target[0], dy = p[1] - view.target[1], dz = p[2] - view.target[2];
      var right = cy * dx + sy * dy;
      var up = -sp * sy * dx + sp * cy * dy + cp * dz;
      var towardCamera = cp * sy * dx - cp * cy * dy + sp * dz;
      distance = Math.max(distance, towardCamera + Math.max(
        Math.abs(right) / (horizontal * .85), Math.abs(up) / (vertical * .72)) + .2);
    });
    return distance;
  }

  function project(point, view, width, height) {
    var p = point3(point);
    if (!p) return { x: 0, y: 0, depth: Infinity, scale: 0, visible: false };
    var v = view;
    var dx = p[0] - v.target[0];
    var dy = p[1] - v.target[1];
    var dz = p[2] - v.target[2];

    var cp = Math.cos(v.pitch);
    var sp = Math.sin(v.pitch);
    var cy = Math.cos(v.yaw);
    var sy = Math.sin(v.yaw);

    // Camera basis: eye = target + distance * (cp*sy, -cp*cy, sp).
    var fx = -cp * sy;
    var fy = cp * cy;
    var fz = -sp;
    var rx = cy;
    var ry = sy;
    var rz = 0;
    var ux = -sp * sy;
    var uy = sp * cy;
    var uz = cp;

    var camRight = dx * rx + dy * ry + dz * rz;
    var camUp = dx * ux + dy * uy + dz * uz;
    var camForward = dx * fx + dy * fy + dz * fz;
    var depth = v.distance + camForward;
    var near = 0.08;
    var visible = depth > near;
    var focal = v.focal || Math.max(220, Math.min(width, height) * 0.96);
    var scale = visible ? focal / depth : 0;

    return {
      x: width / 2 + camRight * scale,
      y: height / 2 - camUp * scale,
      depth: depth,
      scale: scale,
      visible: visible
    };
  }

  function pushTrails(trails, positions, maxLength) {
    var maxLen = Math.max(2, maxLength || 24);
    var next = [];
    var count = positions ? positions.length : 0;
    for (var i = 0; i < count; i++) {
      var previous = Array.isArray(trails) && Array.isArray(trails[i]) ? trails[i] : [];
      var arr = previous.length > maxLen ? previous.slice(previous.length - maxLen) : previous.slice();
      var p = positions[i];
      if (p) {
        arr.push([p[0], p[1], p[2]]);
        if (arr.length > maxLen) arr.splice(0, arr.length - maxLen);
      }
      next.push(arr);
    }
    return next;
  }

  function formatNumber(value, digits) {
    if (!isFiniteNumber(value)) return '—';
    var d = isFiniteNumber(digits) ? digits : 2;
    return value.toFixed(d);
  }

  function formatRevision(value) {
    if (!isFiniteNumber(value)) return '—';
    return String(Math.trunc(value));
  }

  function formatTime(value) {
    if (!isFiniteNumber(value)) return '—';
    var sign = value < 0 ? '-' : '';
    var total = Math.abs(value);
    var minutes = Math.floor(total / 60);
    var seconds = total - minutes * 60;
    if (minutes > 0) return sign + minutes + 'm ' + seconds.toFixed(1).padStart(4, '0') + 's';
    return sign + seconds.toFixed(2) + 's';
  }

  function progressPercent(segment) {
    if (!segment || !isFiniteNumber(segment.progress)) return 0;
    return clamp(segment.progress, 0, 1) * 100;
  }

  return {
    DEFAULT_WS_URL: DEFAULT_WS_URL,
    SOFT_PALETTE: SOFT_PALETTE,
    clamp: clamp,
    point3: point3,
    pointList: pointList,
    errorText: errorText,
    normalizeSegment: normalizeSegment,
    segmentLabel: segmentLabel,
    normalizeState: normalizeState,
    colorFromIndex: colorFromIndex,
    colorForDrone: colorForDrone,
    createView: createView,
    viewTargetFor: viewTargetFor,
    perspectiveFitDistance: perspectiveFitDistance,
    project: project,
    pushTrails: pushTrails,
    formatNumber: formatNumber,
    formatRevision: formatRevision,
    formatTime: formatTime,
    progressPercent: progressPercent,
    isFiniteNumber: isFiniteNumber
  };
});
