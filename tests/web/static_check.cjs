'use strict';

var assert = require('assert');
var fs = require('fs');
var path = require('path');
var core = require('../../web/core.js');

var ROOT = path.resolve(__dirname, '../..');
var LT = String.fromCharCode(60);
var LT_TOKEN = '@' + 'LT' + '@';
var BANG_TOKEN = '@' + 'BANG' + '@';

function read(rel) {
  return fs.readFileSync(path.join(ROOT, rel), 'utf8');
}

function occurrences(text, token) {
  return text.split(token).length - 1;
}

assert.strictEqual(core.DEFAULT_WS_URL, 'ws://127.0.0.1:8765/ws');
assert.strictEqual(typeof core.project, 'function');
assert.strictEqual(typeof core.normalizeState, 'function');
assert.strictEqual(typeof core.isFiniteNumber, 'function');

var state = core.normalizeState({
  type: 'state',
  data: {
    simulation_time: 1.5,
    show_revision: 3,
    request_revision: 4,
    num_drones: 2,
    status: 'running',
    active_segment: { id: 'seg_1', name: 'transition', progress: 0.42, color: '#00ffaa' },
    pending_segment: { id: 'seg_2', name: 'dwell' },
    positions: [[1, 2, 3], [4, 5, 6]],
    orientations: [[0.1, 0.2, 0.3], [0, 0, 0]],
    target_positions: [[9, 9, 9], [8, 8, 8]],
    actual_error: { max: 0.08, rms: 0.03 },
    last_error: null
  }
});

assert.deepStrictEqual(state.positions, [[1, 2, 3], [4, 5, 6]]);
assert.deepStrictEqual(state.orientations, [[0.1, 0.2, 0.3], [0, 0, 0]]);
assert.deepStrictEqual(state.targetPositions, [[9, 9, 9], [8, 8, 8]]);
assert.strictEqual(state.showRevision, 3);
assert.strictEqual(state.requestRevision, 4);
assert.strictEqual(state.actualMaxError, 0.08);
assert.strictEqual(state.actualRmsError, 0.03);
assert.strictEqual(state.activeSegment.name, 'transition');
assert.strictEqual(state.activeSegment.progress, 0.42);
assert.strictEqual(state.pendingSegment.id, 'seg_2');
assert.strictEqual(state.lastError, null);

var noActual = core.normalizeState({
  type: 'state',
  data: { positions: [], target_positions: [[1, 2, 3]], num_drones: 1 }
});
assert.strictEqual(noActual.positions.length, 0);
assert.strictEqual(noActual.targetPositions.length, 1);
assert(noActual.missingActualWarning.indexOf('NOT used') >= 0);
assert.strictEqual(state.missingActualWarning, null);

var view = core.createView({ yaw: 0, pitch: 0.38, distance: 9, target: [0, 0, 1.2] });
var center = core.project([0, 0, 1.2], view, 800, 600);
assert.strictEqual(Math.abs(center.x - 400) < 0.001, true);
assert.strictEqual(Math.abs(center.y - 300) < 0.001, true);
assert.strictEqual(center.visible, true);

var high = core.project([0, 0, 2.2], view, 800, 600);
assert(center.y > high.y);
assert.strictEqual(high.visible, true);

var behind = core.project([0, -20, 0], view, 800, 600);
assert.strictEqual(behind.visible, false);

var trails = core.pushTrails([], [[0, 0, 0]], 2);
trails = core.pushTrails(trails, [[1, 1, 1]], 2);
trails = core.pushTrails(trails, [[2, 2, 2]], 2);
assert.deepStrictEqual(trails[0], [[1, 1, 1], [2, 2, 2]]);

assert.strictEqual(core.colorForDrone({ colors: ['#f00', '#0f0'] }, 1, 2), '#0f0');
assert.strictEqual(core.colorForDrone({ color: '#abc' }, 0, 2), '#abc');
assert.strictEqual(core.colorForDrone(null, 0, 2), core.SOFT_PALETTE[0]);
assert.strictEqual(core.formatTime(65.4), '1m 05.4s');
assert.strictEqual(core.progressPercent({ progress: 0.42 }), 42);

var index = read('web/index.html');
var app = read('web/app.js');
var css = read('web/styles.css');
var coreSource = read('web/core.js');
var all = [index, app, css, coreSource];

assert(index.indexOf('id="scene"') >= 0);
assert(index.indexOf('core.js') >= 0);
assert(index.indexOf('app.js') >= 0);
assert(index.indexOf('styles.css') >= 0);
assert(index.indexOf('REFERENCE TARGETS') >= 0);
assert(index.indexOf('state.positions') >= 0);
assert(index.indexOf('state.target_positions') >= 0);
assert(app.indexOf('new WebSocket(WS_URL)') >= 0);
assert(app.indexOf('scheduleReconnect') >= 0);
assert(app.indexOf('targetPositions') >= 0);
assert(app.indexOf('showTargets') >= 0);
assert(coreSource.indexOf('ws://127.0.0.1:8765/ws') >= 0);
assert(app.indexOf('http://') === -1);
assert(app.indexOf('https://') === -1);
assert(index.indexOf('http://') === -1);
assert(index.indexOf('https://') === -1);
assert(css.indexOf('http://') === -1);
assert(css.indexOf('https://') === -1);

all.forEach(function (text) {
  assert.strictEqual(occurrences(text, LT_TOKEN), 0);
  assert.strictEqual(occurrences(text, BANG_TOKEN), 0);
});
assert.strictEqual(occurrences(index, LT + LT), 0);

['html', 'head', 'body', 'div', 'span', 'section', 'aside', 'main', 'header', 'footer', 'ul', 'li', 'label', 'button', 'canvas'].forEach(function (tag) {
  var opened = occurrences(index, LT + tag + ' ') + occurrences(index, LT + tag + '>');
  var closed = occurrences(index, LT + '/' + tag + '>');
  assert.strictEqual(opened, closed, 'unbalanced tag ' + tag + ': open=' + opened + ' close=' + closed);
});

var colorBoth = core.normalizeState({
  type: 'state',
  data: {
    positions: [[0, 0, 1], [1, 0, 1]],
    target_positions: [],
    active_segment: { id: 'seg_color', name: 'transition', progress: 0.3, color: '#00ffaa', colors: ['#ff0044', '#00aaff'] }
  }
});
assert.strictEqual(colorBoth.activeSegment.color, '#00ffaa');
assert.deepStrictEqual(colorBoth.activeSegment.colors, ['#ff0044', '#00aaff']);
assert.strictEqual(core.colorForDrone(colorBoth.activeSegment, 0, 2), '#ff0044');
assert.strictEqual(core.colorForDrone(colorBoth.activeSegment, 1, 2), '#00aaff');

var colorSolid = core.normalizeState({
  type: 'state',
  data: { positions: [[0, 0, 1]], target_positions: [], active_segment: { color: '#00ffaa' } }
});
assert.strictEqual(core.colorForDrone(colorSolid.activeSegment, 0, 1), '#00ffaa');

assert(index.indexOf('color-swatches') >= 0);
assert(index.indexOf('color-source') >= 0);
assert(index.indexOf('Live segment colors') >= 0);
assert(app.indexOf('renderColorIndicator') >= 0);
assert(app.indexOf('actualColors') >= 0);
assert(css.indexOf('color-chip') >= 0);

assert(index.indexOf('mode-badge') >= 0);
assert(index.indexOf('ready-banner') >= 0);
assert(index.indexOf('btn-replay') >= 0);
assert(index.indexOf('live-state') >= 0);
assert(app.indexOf('pollState') >= 0);
assert(app.indexOf('schedulePoll') >= 0);
assert(app.indexOf('visibilityRefresh') >= 0);
assert(app.indexOf('forceReconnect') >= 0);
assert(app.indexOf('startReplay') >= 0);
assert(app.indexOf('latest_run_id') >= 0);
assert(app.indexOf('mode-badge') >= 0);
assert(coreSource.indexOf('SOFT_PALETTE') >= 0);
assert.strictEqual(core.SOFT_PALETTE.length, 6);
assert.strictEqual(core.SOFT_PALETTE[0], '#5ac8fa');
assert(css.indexOf('mode-badge') >= 0);
assert(css.indexOf('ready-banner') >= 0);
assert(css.indexOf('live-state') >= 0);

console.log('WEB_STATIC_CHECK_OK: core state/projection/trail/color/format tests plus static HTML/CSS/JS checks passed.');
