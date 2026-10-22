import assert from 'node:assert/strict'
import {readFileSync} from 'node:fs'
import {createRequire} from 'node:module'
import {pathToFileURL} from 'node:url'
import test from 'node:test'
const require = createRequire(new URL('../../tools/stage/package.json', import.meta.url))
const {Window} = await import(pathToFileURL(require.resolve('happy-dom')).href)
const root = new URL('../../', import.meta.url)

test('actual replay pauses on seek, resumes there, and yields to new live flight', async () => {
  const window = new Window({url:'http://127.0.0.1:8765/?replay=r_0123456789ab',settings:{disableCSSFileLoading:true,disableJavaScriptFileLoading:true}})
  window.document.write(readFileSync(new URL('web/index.html',root),'utf8').replace(/<script[\s\S]*?<\/script>/g,''))
  let now = 1000, id = 0, socket, rendered
  const callbacks = new Map()
  window.performance.now = () => now
  window.requestAnimationFrame = (callback) => { callbacks.set(++id,callback); return id }
  window.cancelAnimationFrame = (key) => callbacks.delete(key)
  window.setInterval = window.setTimeout = () => 1
  window.DroneStage = {create:()=>({render(state){ rendered = JSON.parse(JSON.stringify(state)) }})}
  const idle = {positions:[[0,0,.1]],num_drones:1,status:'idle',active_segment:null}
  const frames = [0,2,4,6,8,10].map(t=>({t,positions:[[t,0,1]],target_positions:[[100+t,0,1]],active_segment:{id:'s',name:'scene0:AGNES_none',progress:t/10}}))
  window.fetch = async (url) => ({ok:true,json:async()=>url.includes('/replay?')?{frames,kinematic_preview:false}:url.includes('/api/runs/latest')?{run_id:null}:idle})
  window.WebSocket = class {
    static OPEN = 1
    readyState = 1
    constructor() { socket = this }
    send() {}
    close() {}
  }
  const tick = () => { const batch = [...callbacks.values()]; callbacks.clear(); batch.forEach(callback=>callback(now)) }
  const element = (key) => window.document.getElementById(key)
  try {
    window.eval(readFileSync(new URL('web/core.js',root),'utf8'))
    window.eval(readFileSync(new URL('web/app.js',root),'utf8'))
    socket.onopen()
    await new Promise(resolve=>setImmediate(resolve))
    tick()
    assert.equal(element('replay-transport').hidden,false)
    assert.equal(element('mode-badge').textContent,'实际飞行回放')
    socket.onmessage({data:JSON.stringify({type:'state',data:{...idle,status:'running',active_run_id:'r_existing',positions:[[9,1,2]],active_segment:{id:'live',name:'takeoff',progress:.1}}})})
    assert.equal(element('replay-transport').hidden,false,'a flight already active when opening history must not cancel it')
    element('replay-seek').value = '600'
    element('replay-seek').dispatchEvent(new window.Event('input'))
    now = 30000
    tick()
    assert.equal(element('btn-replay-pause').textContent,'继续')
    assert.equal(element('replay-clock').textContent,'6.0 / 10.0 s')
    assert.deepEqual(rendered.positions,[[6,0,1]])
    assert.deepEqual(rendered.targetPositions,[[106,0,1]])
    assert.equal(rendered.actualMaxError,100,'replay error belongs to this recorded frame')
    element('btn-replay-pause').click()
    now += 2000
    tick()
    now += 34
    tick()
    assert.equal(element('btn-replay-pause').textContent,'暂停')
    assert.deepEqual(rendered.positions,[[8,0,1]],'resume continues from the selected recording time')
    now += 3000
    tick()
    socket.onmessage({data:JSON.stringify({type:'state',data:idle})})
    now += 34
    tick()
    assert.deepEqual(rendered.positions,[[10,0,1]],'ended history keeps its final recorded fleet until a new run starts')
    socket.onmessage({data:JSON.stringify({type:'state',data:{...idle,status:'running',active_run_id:'r_new',positions:[[9,1,2]],active_segment:{id:'live',name:'takeoff',progress:.1}}})})
    now += 34
    tick()
    assert.equal(element('replay-transport').hidden,true)
    assert.equal(element('mode-badge').textContent,'物理仿真')
    assert.deepEqual(rendered.positions,[[9,1,2]])
  } finally { await window.happyDOM.abort() }
})
