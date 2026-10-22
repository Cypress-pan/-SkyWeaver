import assert from 'node:assert/strict'
import test from 'node:test'
import { latestSubmittedRun, progressLabel, phaseLabel } from '../../agh-plugin/client/progress.js'
import { progressService } from '../../agh-plugin/src/progress.mjs'

test('card selects only a successful run from this session projection', () => {
  const node = (name, id, status = 'completed') => ({kind: 'tool', name, status, resultPreview: JSON.stringify({run_id: id})})
  const first = node('show_perform', 'r_abcdef123456')
  assert.equal(latestSubmittedRun({nodes: [first, node('shell', 'r_999999999999'), node('show_perform', 'r_111111111111', 'failed')]}), 'r_abcdef123456')
  assert.equal(latestSubmittedRun({nodes: [first, node('show_perform', 'r_222222222222')]}), 'r_222222222222')
  assert.equal(latestSubmittedRun({nodes: [node('pokemon', 'r_abcdef123456')]}), null)
  assert.equal(latestSubmittedRun({nodes: [node('show_perform', '../../secret')]}), null)
})
test('submitted and completed are distinct, preview is never physical success', () => {
  assert.match(progressLabel({status: 'queued'}), /等待安全接入/)
  assert.match(progressLabel({status: 'running', segment: {name: 'takeoff'}}), /起飞/)
  assert.match(progressLabel({status: 'completed', completed: true, evaluation: {verdict: 'pass'}}), /实际检查通过/)
  assert.match(progressLabel({status: 'completed', completed: true, evaluation: {verdict: 'preview'}}), /尚未进行物理仿真/)
  assert.match(progressLabel({status: 'completed', completed: true, evaluation: {verdict: 'fail'}}), /未通过/)
  assert.equal(phaseLabel('scene2:bird_motion'), '振翅飞鸟')
})
test('progress query is bounded and rejects paths before any network access', async () => {
  assert.equal(progressService.kind, 'query')
  assert.equal(progressService.maxResultBytes, 8192)
  await assert.rejects(progressService.handler({run_id: '../secret'}, {}), /Invalid run_id/)
})
