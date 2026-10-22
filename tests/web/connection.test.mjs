import assert from 'node:assert/strict'
import {readFileSync} from 'node:fs'
import {createRequire} from 'node:module'
import {pathToFileURL} from 'node:url'
import test from 'node:test'
const require = createRequire(new URL('../../tools/stage/package.json', import.meta.url))
const {Window} = await import(pathToFileURL(require.resolve('happy-dom')).href)
const root = new URL('../../', import.meta.url)

test('busy telemetry and background tabs probe a live socket; pong never substitutes for a frame', async () => {
  const window = new Window({url:'http://127.0.0.1:8765/',settings:{disableCSSFileLoading:true,disableJavaScriptFileLoading:true}})
  window.document.write(readFileSync(new URL('web/index.html', root),'utf8').replace(/<script[\s\S]*?<\/script>/g,''))
  let now = 10000, heartbeat, hidden = false, closes = 0
  window.Date = class extends Date { static now() { return now } }
  Object.defineProperty(window.document, 'hidden', {get:()=>hidden})
  Object.defineProperty(window.document, 'visibilityState', {get:()=>hidden?'hidden':'visible'})
  window.requestAnimationFrame = () => 1
  window.setInterval = (callback) => { heartbeat = callback; return 1 }
  window.setTimeout = () => 1
  window.DroneStage = {create:()=>({render(){}})}
  const state = {positions:[[0,0,1]],target_positions:[],num_drones:1,status:'running',simulation_time:1,active_segment:{id:'s',name:'takeoff',progress:.5}}
  window.fetch = async (url) => ({ok:true,json:async()=>url.includes('/api/runs/latest')?{run_id:null}:state})
  let socket
  window.WebSocket = class {
    static OPEN = 1
    readyState = 1
    sent = []
    constructor() { socket = this }
    send(value) { this.sent.push(JSON.parse(value)) }
    close() { this.readyState = 3; closes++ }
  }
  try {
    window.eval(readFileSync(new URL('web/core.js',root),'utf8'))
    window.eval(readFileSync(new URL('web/app.js',root),'utf8'))
    socket.onopen()
    socket.onmessage({data:JSON.stringify({type:'state',data:state})})
    await new Promise(resolve=>setImmediate(resolve))
    now += 4000
    heartbeat()
    assert.equal(closes,0)
    assert.deepEqual(socket.sent,[{type:'ping'}])
    socket.onmessage({data:'{"type":"pong"}'})
    heartbeat()
    assert.equal(window.document.getElementById('conn-text').textContent,'正在同步')
    assert.equal(window.document.getElementById('disconnected-overlay').hidden,false)
    assert.doesNotMatch(window.document.getElementById('error-text').textContent,/unrecognized/)
    hidden = true
    now += 10000
    heartbeat()
    assert.equal(closes,0)
    hidden = false
    window.document.dispatchEvent(new window.Event('visibilitychange'))
    assert.equal(closes,0)
    socket.onmessage({data:JSON.stringify({type:'state',data:state})})
    assert.equal(window.document.getElementById('conn-text').textContent,'已连接')
    now += 4000
    heartbeat()
    now += 3100
    heartbeat()
    assert.equal(closes,1,'a genuinely unresponsive transport still reconnects')
  } finally { await window.happyDOM.abort() }
})
