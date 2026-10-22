'use strict';

var http = require('http');
var fs = require('fs');
var path = require('path');
var crypto = require('crypto');
var URL = require('url').URL;

var port = Number(process.argv[2] || 8799);
var mode = process.argv[3] || 'active';
var webRoot = path.resolve(__dirname, '../../web');
var mime = {
  '.html': 'text/html',
  '.js': 'text/javascript',
  '.css': 'text/css',
  '.png': 'image/png',
  '.json': 'application/json'
};

function sendJson(res, status, body) {
  var text = JSON.stringify(body);
  res.writeHead(status, { 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(text) });
  res.end(text);
}

function serveStatic(req, res) {
  var parsed = new URL(req.url, 'http://127.0.0.1');
  var pathname = parsed.pathname === '/' ? '/index.html' : parsed.pathname;
  var file = path.join(webRoot, pathname);
  if (file.indexOf(webRoot) === 0) {
    fs.readFile(file, function (err, data) {
      if (err) {
        sendJson(res, 404, { error: 'E_NOT_FOUND' });
        return;
      }
      res.writeHead(200, { 'Content-Type': mime[path.extname(file)] || 'application/octet-stream' });
      res.end(data);
    });
  } else {
    sendJson(res, 403, { error: 'E_FORBIDDEN' });
  }
}

function idleState() {
  var positions = [];
  var targets = [];
  var i;
  for (i = 0; 16 > i; i += 1) {
    var row = Math.floor(i / 4);
    var col = i % 4;
    positions.push([Number((-0.9 + col * 0.6).toFixed(4)), Number((-0.9 + row * 0.6).toFixed(4)), 0.4]);
    var angle = (2 * Math.PI * i) / 16;
    targets.push([Number((1.45 * Math.cos(angle)).toFixed(4)), Number((1.45 * Math.sin(angle)).toFixed(4)), 1.2]);
  }
  return {
    type: 'state',
    data: {
      simulation_time: 12.0,
      show_revision: 11,
      request_revision: 12,
      num_drones: 16,
      status: 'completed',
      latest_run_id: 'r_mock_replay',
      active_segment: null,
      pending_segment: null,
      positions: positions,
      velocities: positions.map(function () { return [0, 0, 0]; }),
      target_positions: targets,
      actual_error: { max: 0.012, rms: 0.006 },
      last_error: null
    }
  };
}

function replayFrames() {
  var frames = [];
  var count = 45;
  for (var index = 0; count > index; index += 1) {
    var progress = index / (count - 1);
    var ease = progress * progress * (3 - 2 * progress);
    var positions = [];
    var targets = [];
    var colors = [];
    for (var i = 0; 16 > i; i += 1) {
      var row = Math.floor(i / 4);
      var col = i % 4;
      var gridX = -0.9 + col * 0.6;
      var gridY = -0.9 + row * 0.6;
      var angle = (2 * Math.PI * i) / 16;
      var ringX = 1.45 * Math.cos(angle);
      var ringY = 1.45 * Math.sin(angle);
      positions.push([
        Number((gridX + (ringX - gridX) * ease).toFixed(4)),
        Number((gridY + (ringY - gridY) * ease).toFixed(4)),
        Number((0.4 + 0.8 * ease + 0.08 * Math.sin(index * 0.3 + i)).toFixed(4))
      ]);
      targets.push([Number(ringX.toFixed(4)), Number(ringY.toFixed(4)), 1.2]);
      colors.push(i % 2 === 0 ? '#5ac8fa' : '#8b7cff');
    }
    frames.push({
      t: Number((index / 15).toFixed(3)),
      positions: positions,
      target_positions: targets,
      active_segment: {
        id: 'seg_replay_transition',
        name: 'transition',
        progress: Number(progress.toFixed(4)),
        duration: 3.0,
        color: '#5ac8fa',
        colors: colors
      }
    });
  }
  return frames;
}

function handleApi(req, res, parsed) {
  if (req.method === 'GET' && parsed.pathname === '/api/health') {
    sendJson(res, 200, { ok: true, service: 'drone-show-mock', version: '0.1.0' });
    return;
  }
  if (req.method === 'GET' && parsed.pathname === '/api/state') {
    sendJson(res, 200, idleState().data);
    return;
  }
  if (req.method === 'GET' && parsed.pathname === '/api/runs/latest') {
    sendJson(res, 200, { run_id: 'r_mock_replay', status: 'completed', plan_id: 'p_mock' });
    return;
  }
  if (req.method === 'GET' && parsed.pathname.indexOf('/api/run/') === 0 && parsed.pathname.indexOf('/replay') > 0) {
    sendJson(res, 200, {
      run_id: 'r_mock_replay',
      status: 'completed',
      plan_id: 'p_mock',
      hz: 15,
      actual_positions_source: 'mock frames',
      target_positions_separate: true,
      frames: replayFrames()
    });
    return;
  }
  sendJson(res, 404, { error: 'E_NOT_FOUND' });
}

var server = http.createServer(function (req, res) {
  var parsed = new URL(req.url, 'http://127.0.0.1');
  if (parsed.pathname.indexOf('/api/') === 0) handleApi(req, res, parsed);
  else serveStatic(req, res);
});


function wsFrame(message) {
  var payload = Buffer.from(JSON.stringify(message));
  var length = payload.length;
  var header;
  if (length > 125) {
    header = Buffer.alloc(4);
    header[0] = 0x81;
    header[1] = 126;
    header.writeUInt16BE(length, 2);
  } else {
    header = Buffer.alloc(2);
    header[0] = 0x81;
    header[1] = length;
  }
  return Buffer.concat([header, payload]);
}

function mockState(tick) {
  var positions = [];
  var targets = [];
  var colors = [];
  var i;
  for (i = 0; i < 16; i += 1) {
    var row = Math.floor(i / 4);
    var col = i % 4;
    var x = -0.9 + col * 0.6;
    var y = -0.9 + row * 0.6;
    var z = 0.35 + 0.22 * Math.sin(tick * 0.12 + i * 0.7);
    positions.push([Number(x.toFixed(4)), Number(y.toFixed(4)), Number(z.toFixed(4))]);
    var angle = (2 * Math.PI * i) / 16;
    targets.push([Number((1.45 * Math.cos(angle)).toFixed(4)), Number((1.45 * Math.sin(angle)).toFixed(4)), 1.2]);
    colors.push(i % 2 === 0 ? '#55d6ff' : '#8b7cff');
  }
  var cycle = (tick % 220) / 220;
  return {
    type: 'state',
    data: {
      simulation_time: Number((tick * 0.05).toFixed(3)),
      show_revision: 7,
      request_revision: 8,
      num_drones: 16,
      status: 'running',
      active_segment: {
        id: 'seg_mock_transition',
        name: 'transition',
        progress: Number(cycle.toFixed(4)),
        duration: 11.0,
        color: mode === 'solid' ? '#ff8844' : '#55d6ff',
        colors: mode === 'solid' ? [] : colors
      },
      pending_segment: {
        id: 'seg_mock_hold',
        name: 'hold',
        progress: 0,
        duration: 5.0
      },
      positions: positions,
      velocities: positions.map(function () { return [0, 0, 0]; }),
      target_positions: targets,
      actual_error: {
        max: Number((0.06 + 0.05 * Math.sin(tick * 0.1)).toFixed(4)),
        rms: Number((0.03 + 0.02 * Math.sin(tick * 0.08)).toFixed(4))
      },
      last_error: tick > 260 ? { code: 'E_MOCK_WARN', detail: 'mock demonstration error text' } : null
    }
  };
}

server.on('upgrade', function (req, socket) {
  var parsed = new URL(req.url, 'http://127.0.0.1');
  var key = req.headers['sec-websocket-key'];
  if (parsed.pathname === '/ws' && req.headers.upgrade === 'websocket' && key) {
    var accept = crypto.createHash('sha1')
      .update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11')
      .digest('base64');
    socket.write('HTTP/1.1 101 Switching Protocols\r\n');
    socket.write('Upgrade: websocket\r\n');
    socket.write('Connection: Upgrade\r\n');
    socket.write('Sec-WebSocket-Accept: ' + accept + '\r\n\r\n');
    socket.write(wsFrame({ type: 'hello', service: 'drone-show-mock', protocol: 1 }));
    socket.on('error', function () { /* ignore client resets */ });

    if (mode === 'stall') {
      socket.write(wsFrame(idleState()));
      return;
    }

    if (mode === 'idle') {
      var idleTimer = setInterval(function () { socket.write(wsFrame(idleState())); }, 80);
      var idleStop = setTimeout(function () {
        clearInterval(idleTimer);
        server.close();
      }, 15000);
      socket.on('close', function () {
        clearInterval(idleTimer);
        clearTimeout(idleStop);
      });
      return;
    }

    if (mode === 'replay_then_live') {
      var idleTimer = setInterval(function () { socket.write(wsFrame(idleState())); }, 80);
      var liveTimer = null;
      var switchTimer = setTimeout(function () {
        clearInterval(idleTimer);
        var tick = 0;
        liveTimer = setInterval(function () {
          tick += 1;
          socket.write(wsFrame(mockState(tick)));
        }, 50);
      }, 2400);
      socket.on('close', function () {
        clearInterval(idleTimer);
        clearTimeout(switchTimer);
        if (liveTimer) clearInterval(liveTimer);
      });
      return;
    }

    if (mode === 'drop') {
      var dropTimer = setInterval(function () { socket.write(wsFrame(mockState(1))); }, 80);
      setTimeout(function () {
        clearInterval(dropTimer);
        socket.end();
        server.close();
      }, 600);
      return;
    }

    var tick = 0;
    var stateTimer = setInterval(function () {
      tick += 1;
      socket.write(wsFrame(mockState(tick)));
    }, 50);
    setTimeout(function () {
      clearInterval(stateTimer);
      server.close();
    }, 15000);
    socket.on('close', function () { clearInterval(stateTimer); });
  } else {
    socket.destroy();
  }
});

server.listen(port, '127.0.0.1', function () {
  console.log('mock-http-ws listening on http://127.0.0.1:' + port + ' mode=' + mode);
});
