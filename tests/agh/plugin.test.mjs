import assert from 'node:assert/strict'
import { once } from 'node:events'
import http from 'node:http'
import test from 'node:test'

const pluginModule = await import('../../agh-plugin/index.mjs')

const EXPECTED_TOOL_NAMES = [
  'show_perform',
  'show_get_state',
  'show_build_formation',
  'show_plan_transition',
  'show_validate',
  'show_submit',
  'show_get_run',
  'show_cancel_pending',
  'experiment_evaluate',
  'show_text_show',
  'show_timeline',
  'show_fleet_timeline',
]

function captureRegisteredTools() {
  const tools = new Map()
  const extension = {
    registerTool(definition) {
      tools.set(definition.name, definition)
    },
    on() {
      // The plugin currently registers no hooks. Capture-only fixture.
    },
    events: { append() {} },
  }
  pluginModule.droneShowPlugin.apply({
    services: { register() {} },
    extension() {
      return extension
    },
  })
  return tools
}

async function readRequestBody(request) {
  const chunks = []
  for await (const chunk of request) chunks.push(chunk)
  return Buffer.concat(chunks).toString('utf8')
}

async function startMockService(handler) {
  const requests = []
  const server = http.createServer(async (request, response) => {
    const rawBody = await readRequestBody(request)
    let body = null
    if (rawBody.length > 0) body = JSON.parse(rawBody)
    const record = { method: request.method, url: request.url, body }
    requests.push(record)
    let result
    try {
      result = await handler(record)
    } catch (error) {
      result = {
        status: 500,
        body: { error: 'E_MOCK_HANDLER', message: String(error?.message ?? error) },
      }
    }
    response.writeHead(result.status ?? 200, { 'content-type': 'application/json' })
    response.end(JSON.stringify(result.body ?? {}))
  })
  server.listen(0, '127.0.0.1')
  await once(server, 'listening')
  const address = server.address()
  const url = 'http://127.0.0.1:' + address.port
  return {
    requests,
    url,
    async close() {
      server.close()
      await once(server, 'close')
    },
  }
}

async function withServiceUrl(url, run) {
  const previous = process.env.DRONE_SHOW_SERVICE_URL
  process.env.DRONE_SHOW_SERVICE_URL = url
  try {
    return await run()
  } finally {
    if (previous === undefined) delete process.env.DRONE_SHOW_SERVICE_URL
    else process.env.DRONE_SHOW_SERVICE_URL = previous
  }
}

async function callTool(tools, name, args) {
  const definition = tools.get(name)
  assert.ok(definition, 'tool not registered: ' + name)
  return definition.execute(args, {})
}

function assertSuccess(result, name) {
  assert.equal(result.isError, undefined, name + ' should not be an error')
  assert.equal(result.structured.ok, true, name + ' should report ok')
  assert.equal(result.structured.tool, name)
}

function assertFailure(result, name) {
  assert.equal(result.isError, true, name + ' should be an error')
  assert.equal(result.structured.ok, false, name + ' should report failure')
  assert.equal(result.structured.tool, name)
  assert.equal(typeof result.structured.error.code, 'string')
  assert.ok(result.structured.error.code.length > 0)
}

test('module loads and captures the twelve expected AGH tool definitions', () => {
  const tools = captureRegisteredTools()
  assert.deepEqual([...tools.keys()], EXPECTED_TOOL_NAMES)
  const kind = Symbol.for('TypeBox.Kind')
  const expectedMetaKeys = [
    'costHint',
    'deferLoading',
    'isConcurrencySafe',
    'isDestructive',
    'isOpenWorld',
    'isReadOnly',
    'replay',
    'requiresApproval',
  ]
  for (const definition of tools.values()) {
    assert.equal(typeof definition.description, 'string')
    assert.ok(definition.description.length > 0, definition.name + ' needs a description')
    assert.equal(typeof definition.execute, 'function')
    assert.equal(definition.parameters[kind], 'Object')
    assert.equal(typeof definition.parameters.type, 'string')
    assert.equal(definition.parameters.type, 'object')
    assert.deepEqual(Object.keys(definition.meta).sort(), expectedMetaKeys)
    assert.equal(typeof definition.meta.isReadOnly, 'boolean')
    assert.equal(typeof definition.meta.isOpenWorld, 'boolean')
    assert.equal(typeof definition.meta.replay, 'string')
    for (const property of Object.values(definition.parameters.properties)) {
      assert.equal(typeof property[kind], 'string')
    }
  }
  const buildFormation = tools.get('show_build_formation')
  assert.ok(buildFormation.description.includes('"type":"text"'))
  assert.ok(buildFormation.description.includes('HELLO'))
  assert.ok(buildFormation.description.includes('gradient'))
})

test('show_perform sends landing as an ending action rather than a formation', async () => {
  const tools = captureRegisteredTools()
  const mock = await startMockService(() => ({ body: {
    ok: true, run_id: 'r_director', submitted: true, duration: 32,
    num_drones: 128, motion_mode: 'physical', scenes: [], validation: {valid: true},
  } }))
  try {
    await withServiceUrl(mock.url, async () => {
      const scenes = [{type: 'text', text: '卷卷'}, {type: 'heart', color: '#4285ff'}]
      assertSuccess(await callTool(tools, 'show_perform', {scenes, ending: 'land'}), 'show_perform')
      assert.deepEqual(mock.requests[0].body, {scenes, include_landing: true})
      await callTool(tools, 'show_perform', {scenes, ending: 'hover'})
      assert.equal(mock.requests[1].body.include_landing, false)
      assert.equal(mock.requests[0].url, '/api/perform')
    })
  } finally { await mock.close() }
})

test('completed run reads independent actual checks; a failed check stays failed', async () => {
  const tools = captureRegisteredTools()
  const mock = await startMockService(({url}) => ({status: 200, body: url === '/api/evaluate'
    ? {ok: true, run_id: 'r_test', verdict: 'fail', checks: [{name: 'spacing', pass: false}]}
    : {run_id: 'r_test', status: 'completed', completed: true}}))
  try {
    await withServiceUrl(mock.url, async () => {
      const result = await callTool(tools, 'show_get_run', {run_id: 'r_test'})
      assertSuccess(result, 'show_get_run')
      assert.equal(result.structured.summary.actual_evaluation.verdict, 'fail')
      assert.equal(mock.requests[1].method, 'POST')
      assert.equal(mock.requests[1].url, '/api/evaluate')
    })
  } finally { await mock.close() }
})

test('every tool calls the service endpoint and returns a structured success summary', async () => {
  const state = {
    simulation_time: 1.23,
    show_revision: 3,
    request_revision: 4,
    num_drones: 4,
    status: 'idle',
    active_segment: null,
    pending_segment: null,
    positions: [
      [0, 0, 1],
      [1, 0, 1],
    ],
    velocities: [
      [0, 0, 0],
      [0, 0, 0],
    ],
    target_positions: [
      [0, 0, 1],
      [1, 0, 1],
    ],
    actual_error: { max: 0.01, rms: 0.005 },
    last_error: null,
  }
  const mock = await startMockService((record) => {
    if (record.method === 'GET' && record.url === '/api/state') {
      return { status: 200, body: state }
    }
    if (record.method === 'POST' && record.url === '/api/formation') {
      return {
        status: 200,
        body: {
          formation_id: 'f_1',
          points: [
            [0, 0, 1],
            [1, 0, 1],
          ],
          resource_report: { used: 4, idle: 0, min_distance: 0.5 },
        },
      }
    }
    if (record.method === 'POST' && record.url === '/api/plan') {
      return {
        status: 200,
        body: {
          plan_id: 'p_1',
          plan_hash: 'hash_1',
          duration: 8.5,
          segments: [{ id: 'seg_1', name: 'takeoff', duration: 2 }],
        },
      }
    }
    if (record.method === 'POST' && record.url === '/api/validate') {
      return {
        status: 200,
        body: {
          valid: true,
          credential: 'test-credential',
          issues: [],
          metrics: { min_distance: 0.5, max_speed: 1, max_accel: 0.5 },
        },
      }
    }
    if (record.method === 'POST' && record.url === '/api/submit') {
      return { status: 200, body: { accepted: true, run_id: 'r_1', scheduled_for: 'next_dwell' } }
    }
    if (record.method === 'GET' && record.url.startsWith('/api/run/')) {
      return {
        status: 200,
        body: {
          run_id: 'r_1',
          status: 'completed',
          actual_error: { max: 0.04, rms: 0.02 },
          completed: true,
          telemetry_ref: 'telemetry.npz',
        },
      }
    }
    if (record.method === 'POST' && record.url === '/api/cancel_pending') {
      return { status: 200, body: { cancelled: true } }
    }
    if (record.method === 'POST' && record.url === '/api/evaluate') {
      return {
        status: 200,
        body: {
          ok: true,
          run_id: 'r_1',
          verdict: 'pass',
          checks: [{ name: 'completed', pass: true, detail: 'run completed' }],
          metrics: { max_error: 0.04, rms_error: 0.02, min_distance: 0.5 },
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const stateResult = await callTool(tools, 'show_get_state', {})
      assertSuccess(stateResult, 'show_get_state')
      assert.equal(stateResult.structured.result.show_revision, 3)

      const formationResult = await callTool(tools, 'show_build_formation', {
        num_drones: 4,
        spec: { type: 'grid', rows: 2, cols: 2, spacing: 0.6, z: 1.2 },
      })
      assertSuccess(formationResult, 'show_build_formation')
      assert.equal(formationResult.structured.result.formation_id, 'f_1')

      const planResult = await callTool(tools, 'show_plan_transition', {
        state_revision: 3,
        target_formation_id: 'f_1',
        duration: 4,
        hold_seconds: 2,
        include_landing: true,
      })
      assertSuccess(planResult, 'show_plan_transition')
      assert.equal(planResult.structured.result.plan_id, 'p_1')

      const validateResult = await callTool(tools, 'show_validate', { plan_id: 'p_1' })
      assertSuccess(validateResult, 'show_validate')
      assert.equal(validateResult.structured.summary.credential, 'test-credential')

      const submitResult = await callTool(tools, 'show_submit', {
        plan_id: 'p_1',
        credential: 'test-credential',
        request_revision: 4,
        source_state_revision: 3,
      })
      assertSuccess(submitResult, 'show_submit')
      assert.equal(submitResult.structured.result.run_id, 'r_1')

      const runResult = await callTool(tools, 'show_get_run', { run_id: 'r_1' })
      assertSuccess(runResult, 'show_get_run')
      assert.equal(runResult.structured.result.status, 'completed')

      const cancelResult = await callTool(tools, 'show_cancel_pending', { request_revision: 4 })
      assertSuccess(cancelResult, 'show_cancel_pending')
      assert.equal(cancelResult.structured.result.cancelled, true)

      const evaluateResult = await callTool(tools, 'experiment_evaluate', { run_id: 'r_1' })
      assertSuccess(evaluateResult, 'experiment_evaluate')
      assert.equal(evaluateResult.structured.summary.verdict, 'pass')
    })

    const submitRequest = mock.requests.find((request) => request.url === '/api/submit')
    assert.deepEqual(submitRequest.body.insert_after, 'immediate')
    const evaluateRequest = mock.requests.find((request) => request.url === '/api/evaluate')
    assert.deepEqual(evaluateRequest.body, { run_id: 'r_1' })
  } finally {
    await mock.close()
  }
})

test('HTTP and service rejection errors are surfaced as structured tool errors', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'GET' && record.url === '/api/state') {
      return {
        status: 409,
        body: {
          error: 'E_STALE_STATE',
          message: 'state moved to a newer revision',
          state_revision: 5,
          credential: 'placeholder-credential-value',
          api_key: 'placeholder-api-key-value',
        },
      }
    }
    if (record.method === 'POST' && record.url === '/api/validate') {
      return {
        status: 422,
        body: {
          error: 'E_BAD_PLAN',
          message: 'unknown plan',
          details: { plan_id: 'missing' },
        },
      }
    }
    return { status: 500, body: { error: 'E_MOCK', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()

      const stateError = await callTool(tools, 'show_get_state', {})
      assertFailure(stateError, 'show_get_state')
      assert.equal(stateError.structured.error.code, 'E_STALE_STATE')
      assert.equal(stateError.structured.error.status, 409)
      assert.equal(stateError.structured.error.details.credential, '[redacted]')
      assert.equal(stateError.structured.error.details.api_key, '[redacted]')
      assert.equal(JSON.stringify(stateError).includes('placeholder-credential-value'), false)
      assert.equal(JSON.stringify(stateError).includes('placeholder-api-key-value'), false)

      const validateError = await callTool(tools, 'show_validate', { plan_id: 'p_missing' })
      assertFailure(validateError, 'show_validate')
      assert.equal(validateError.structured.error.code, 'E_BAD_PLAN')
      assert.equal(validateError.structured.error.status, 422)
      assert.deepEqual(validateError.structured.error.details.details, { plan_id: 'missing' })
    })
  } finally {
    await mock.close()
  }
})

test('unreachable service and invalid configuration become structured errors', async () => {
  const mock = await startMockService(() => ({ status: 200, body: { ok: true } }))
  const closedUrl = mock.url
  await mock.close()

  const tools = captureRegisteredTools()
  await withServiceUrl(closedUrl, async () => {
    const unreachable = await callTool(tools, 'show_get_state', {})
    assertFailure(unreachable, 'show_get_state')
    assert.equal(unreachable.structured.error.code, 'E_SERVICE_UNREACHABLE')
  })

  await withServiceUrl('not-a-valid-url', async () => {
    const invalidConfig = await callTool(tools, 'show_get_state', {})
    assertFailure(invalidConfig, 'show_get_state')
    assert.equal(invalidConfig.structured.error.code, 'E_CONFIG')
  })
  const originalFetch = globalThis.fetch
  let outsideRequests = 0
  globalThis.fetch = async () => { outsideRequests++; throw new Error('unexpected network call') }
  try {
    for (const url of ['https://example.com', 'http://192.168.1.2:8765', 'http://user:secret@127.0.0.1:8765']) {
      await withServiceUrl(url, async () => {
        const refused = await callTool(tools, 'show_get_state', {})
        assertFailure(refused, 'show_get_state')
        assert.equal(refused.structured.error.code, 'E_CONFIG')
      })
    }
    assert.equal(outsideRequests, 0)
  } finally { globalThis.fetch = originalFetch }
  const redirect = await startMockService(() => ({ status: 302, body: {} }))
  try {
    await withServiceUrl(redirect.url, async () => {
      const refused = await callTool(tools, 'show_get_state', {})
      assertFailure(refused, 'show_get_state')
      assert.equal(refused.structured.error.code, 'E_SERVICE_UNREACHABLE')
    })
  } finally { await redirect.close() }
})

test('stale submit is surfaced as structured E_STALE_STATE', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/submit') {
      return { status: 409, body: { error: 'E_STALE_STATE', state_revision: 5 } }
    }
    return { status: 500, body: { error: 'E_MOCK', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_submit', {
        plan_id: 'p_1',
        credential: 'test-credential',
        request_revision: 4,
        source_state_revision: 3,
      })
      assertFailure(result, 'show_submit')
      assert.equal(result.structured.error.code, 'E_STALE_STATE')
      assert.equal(result.structured.error.status, 409)
      assert.equal(result.structured.error.details.state_revision, 5)
    })
    assert.equal(mock.requests.length, 1)
  } finally {
    await mock.close()
  }
})

test('uses the documented default base URL when the environment variable is unset', async () => {
  const previous = process.env.DRONE_SHOW_SERVICE_URL
  const originalFetch = globalThis.fetch
  const state = {
    simulation_time: 0,
    show_revision: 1,
    request_revision: 1,
    num_drones: 1,
    status: 'idle',
    active_segment: null,
    pending_segment: null,
    positions: [[0, 0, 1]],
    velocities: [[0, 0, 0]],
    target_positions: [[0, 0, 1]],
    actual_error: { max: 0, rms: 0 },
    last_error: null,
  }
  let calledUrl = null

  delete process.env.DRONE_SHOW_SERVICE_URL
  globalThis.fetch = async (url) => {
    calledUrl = String(url)
    return new Response(JSON.stringify(state), {
      status: 200,
      headers: { 'content-type': 'application/json' },
    })
  }

  try {
    const tools = captureRegisteredTools()
    const result = await callTool(tools, 'show_get_state', {})
    assertSuccess(result, 'show_get_state')
    assert.equal(calledUrl, 'http://127.0.0.1:8765/api/state')
  } finally {
    globalThis.fetch = originalFetch
    if (previous === undefined) delete process.env.DRONE_SHOW_SERVICE_URL
    else process.env.DRONE_SHOW_SERVICE_URL = previous
  }
})

test('show_get_run keeps a rejected run 200 response as a successful domain result', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'GET' && record.url === '/api/run/r_rejected') {
      return {
        status: 200,
        body: {
          run_id: 'r_rejected',
          status: 'rejected',
          completed: true,
          actual_error: null,
          telemetry_ref: null,
          error: 'E_CANCELLED',
        },
      }
    }
    return { status: 500, body: { error: 'E_MOCK', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_get_run', { run_id: 'r_rejected' })
      assertSuccess(result, 'show_get_run')
      assert.equal(result.structured.summary.status, 'rejected')
      assert.equal(result.structured.summary.completed, true)
      assert.equal(result.structured.summary.error, 'E_CANCELLED')
    })
  } finally {
    await mock.close()
  }
})

test('show_build_formation normalizes JSON-string text specs and surfaces color fields', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/formation') {
      if (record.body.spec.type === 'text') {
        return {
          status: 200,
          body: {
            formation_id: 'f_hello',
            points: [[0, 0, 1.4]],
            color: '#00ffaa',
            colors: null,
                        gradient: null,
            required_points: 40,
            warnings: [{ code: 'E_SIZE_CLAMPED', detail: 'size was clamped' }],
resource_report: { used: 64, idle: 0, min_distance: 0.3 },
          },
        }
      }
      return {
        status: 200,
        body: {
          formation_id: 'f_grid_gradient',
          points: [[0, 0, 1.2]],
          color: null,
          colors: ['#ff0044', '#00aaff'],
                      gradient: ['#ff0044', '#00aaff'],
            warnings: [],
resource_report: { used: 16, idle: 0, min_distance: 0.6 },
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const textSpec = JSON.stringify({
        type: 'text',
        text: 'HELLO',
        size: 1.2,
        z: 1.4,
        color: '#00ffaa',
      })
      const textResult = await callTool(tools, 'show_build_formation', {
        num_drones: 64,
        spec: textSpec,
      })
      assertSuccess(textResult, 'show_build_formation')
      assert.equal(textResult.structured.summary.formation_id, 'f_hello')
      assert.equal(textResult.structured.summary.color, '#00ffaa')
      assert.equal(textResult.structured.summary.point_count, 1)
      assert.equal(textResult.structured.summary.required_points, 40)
      assert.equal(textResult.structured.summary.warnings[0].code, 'E_SIZE_CLAMPED')

      const textRequest = mock.requests.find((request) => request.url === '/api/formation')
      assert.deepEqual(textRequest.body.spec, {
        type: 'text',
        text: 'HELLO',
        size: 1.2,
        z: 1.4,
        color: '#00ffaa',
      })

      const gradientResult = await callTool(tools, 'show_build_formation', {
        num_drones: 16,
        spec: {
          type: 'grid',
          rows: 4,
          cols: 4,
          spacing: 0.6,
          z: 1.2,
          gradient: ['#ff0044', '#00aaff'],
        },
      })
      assertSuccess(gradientResult, 'show_build_formation')
      assert.equal(gradientResult.structured.summary.formation_id, 'f_grid_gradient')
      assert.deepEqual(gradientResult.structured.summary.colors, ['#ff0044', '#00aaff'])
      assert.deepEqual(gradientResult.structured.summary.gradient, ['#ff0044', '#00aaff'])
    })
  } finally {
    await mock.close()
  }
})

test('show_build_formation rejects invalid JSON-string specs without calling the service', async () => {
  const mock = await startMockService(() => ({ status: 200, body: {} }))

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_build_formation', {
        num_drones: 16,
        spec: 'not-json',
      })
      assertFailure(result, 'show_build_formation')
      assert.equal(result.structured.error.code, 'E_BAD_ARGUMENT')
    })
    assert.equal(mock.requests.length, 0)
  } finally {
    await mock.close()
  }
})

test('show_get_state and show_plan_transition surface segment color and colors', async () => {
  const state = {
    simulation_time: 1.23,
    show_revision: 3,
    request_revision: 4,
    num_drones: 16,
    status: 'running',
    active_segment: {
      id: 'seg_active',
      name: 'transition',
      progress: 0.42,
      color: '#00ffaa',
      colors: null,
    },
    pending_segment: {
      id: 'seg_pending',
      name: 'hold',
      progress: 0.0,
      color: null,
      colors: ['#ff0044', '#00aaff'],
    },
    positions: [[0, 0, 1]],
    velocities: [[0, 0, 0]],
    target_positions: [[0, 0, 1]],
    actual_error: { max: 0.01, rms: 0.005 },
    last_error: null,
  }
  const mock = await startMockService((record) => {
    if (record.method === 'GET' && record.url === '/api/state') {
      return { status: 200, body: state }
    }
    if (record.method === 'POST' && record.url === '/api/plan') {
      return {
        status: 200,
        body: {
          plan_id: 'p_color',
          plan_hash: 'hash_color',
          duration: 2,
          segments: [
            {
              id: 'seg_1',
              name: 'takeoff',
              duration: 1,
              color: '#00ffaa',
              colors: ['#ff0044', '#00aaff'],
            },
          ],
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const stateResult = await callTool(tools, 'show_get_state', {})
      assertSuccess(stateResult, 'show_get_state')
      assert.equal(stateResult.structured.summary.active_segment.color, '#00ffaa')
      assert.deepEqual(stateResult.structured.summary.pending_segment.colors, ['#ff0044', '#00aaff'])

      const planResult = await callTool(tools, 'show_plan_transition', {
        state_revision: 3,
        target_formation_id: 'f_1',
        duration: 1,
        hold_seconds: 0,
        include_landing: false,
      })
      assertSuccess(planResult, 'show_plan_transition')
      assert.equal(planResult.structured.summary.segments[0].color, '#00ffaa')
      assert.deepEqual(planResult.structured.summary.segments[0].colors, ['#ff0044', '#00aaff'])
    })
  } finally {
    await mock.close()
  }
})

test('show_text_show calls POST /api/text_show and shapes pages, allocation, validation, and warnings', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/text_show') {
      const submitted = record.body.submit === true
      return {
        status: 200,
        body: {
          ok: true,
          text: record.body.text,
          num_drones: record.body.num_drones,
          pages: [
            {
              page: 0,
              text: '卷',
              formation_id: 'f_page1',
              points: 32,
              used: 32,
              idle: 0,
              min_distance: 0.3,
            },
          ],
          allocation_report: [{ char: '卷', complexity: 100, points: 32 }],
          plan_id: 'p_text',
          plan_hash: 'hash_text',
          validation: { valid: true, issues: [], metrics: { min_distance: 0.3 } },
          submitted,
          run_id: submitted ? 'r_text' : null,
          scheduled_for: submitted ? 'immediate' : null,
          warnings: [{ code: 'E_SIZE_CLAMPED', detail: 'size was clamped' }],
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_text_show', {
        text: '卷卷',
        num_drones: 64,
        auto_allocate: true,
        min_points_per_char: 16,
        hold_seconds: 3,
        transition_seconds: 4,
        include_landing: true,
        gradient: ['#55d6ff', '#8b7cff'],
      })
      assertSuccess(result, 'show_text_show')
      assert.equal(result.structured.summary.text, '卷卷')
      assert.equal(result.structured.summary.num_drones, 64)
      assert.equal(result.structured.summary.pages[0].formation_id, 'f_page1')
      assert.equal(result.structured.summary.allocation_report[0].points, 32)
      assert.equal(result.structured.summary.plan_id, 'p_text')
      assert.equal(result.structured.summary.plan_hash, 'hash_text')
      assert.equal(result.structured.summary.validation.valid, true)
      assert.equal(result.structured.summary.submitted, true)
      assert.equal(result.structured.summary.run_id, 'r_text')
      assert.equal(result.structured.summary.scheduled_for, 'immediate')
      assert.equal(result.structured.summary.warnings[0].code, 'E_SIZE_CLAMPED')

      const submitRequest = mock.requests.find((request) => request.url === '/api/text_show')
      assert.deepEqual(submitRequest.body, {
        text: '卷卷',
        num_drones: 64,
        submit: true,
        auto_allocate: true,
        min_points_per_char: 16,
        hold_seconds: 3,
        transition_seconds: 4,
        include_landing: true,
        gradient: ['#55d6ff', '#8b7cff'],
        points_per_char: 16,
      })

      const noSubmit = await callTool(tools, 'show_text_show', {
        text: '卷卷',
        num_drones: 64,
        submit: false,
      })
      assertSuccess(noSubmit, 'show_text_show')
      assert.equal(noSubmit.structured.summary.submitted, false)
      assert.equal(noSubmit.structured.summary.run_id, null)
      const requests = mock.requests.filter((request) => request.url === '/api/text_show')
      assert.equal(requests[requests.length - 1].body.submit, false)
    })
  } finally {
    await mock.close()
  }
})

test('show_text_show surfaces service rejection codes as structured tool errors', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/text_show') {
      return {
        status: 400,
        body: {
          error: 'E_POINT_BUDGET',
          message: 'text needs at least 40 points',
          required_points: 40,
        },
      }
    }
    return { status: 500, body: { error: 'E_MOCK', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_text_show', {
        text: 'HELLO',
        num_drones: 16,
        submit: false,
      })
      assertFailure(result, 'show_text_show')
      assert.equal(result.structured.error.code, 'E_POINT_BUDGET')
      assert.equal(result.structured.error.status, 400)
      assert.equal(result.structured.error.details.required_points, 40)
    })
  } finally {
    await mock.close()
  }
})

test('show_text_show normalizes nested allocation, pagination, plan, and submission shapes', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/text_show') {
      return {
        status: 200,
        body: {
          ok: true,
          text: '卷卷',
          allocation: {
            num_drones: 64,
            points_per_char: 32,
            pages: [
              {
                page_index: 0,
                characters: ['卷', '卷'],
                allocations: [32, 32],
                points: 64,
                idle: 0,
              },
            ],
          },
          pagination: {
            strategy: 'single_page',
            pages: [{ page_index: 0, characters: ['卷', '卷'], allocations: [32, 32], points: 64, idle: 0 }],
          },
          plan: { plan_id: 'p_nested', plan_hash: 'hash_nested' },
          validation: { valid: true, issues: [], metrics: { min_distance: 0.3 } },
          submission: { accepted: true, run_id: 'r_nested', scheduled_for: 'immediate' },
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_text_show', {
        text: '卷卷',
        num_drones: 64,
        min_points_per_char: 32,
      })
      assertSuccess(result, 'show_text_show')
      assert.equal(result.structured.summary.pages[0].page, 0)
      assert.equal(result.structured.summary.pages[0].text, '卷卷')
      assert.equal(result.structured.summary.pages[0].points, 64)
      assert.deepEqual(result.structured.summary.allocation_report, [
        { char: '卷', complexity: null, points: 32 },
        { char: '卷', complexity: null, points: 32 },
      ])
      assert.equal(result.structured.summary.plan_id, 'p_nested')
      assert.equal(result.structured.summary.plan_hash, 'hash_nested')
      assert.equal(result.structured.summary.submitted, true)
      assert.equal(result.structured.summary.run_id, 'r_nested')
      assert.equal(result.structured.summary.scheduled_for, 'immediate')
    })
  } finally {
    await mock.close()
  }
})

test('show_timeline calls POST /api/timeline and shapes scenes, validation, and submission', async () => {
  const scenes = [
    {
      type: 'text',
      duration_sec: 4,
      transition_seconds: 1,
      text: 'HELLO',
      color: '#55d6ff',
    },
    {
      type: 'ring',
      duration_sec: 4,
      transition_seconds: 1,
      effect: { type: 'rotate', turns: 1 },
      gradient: ['#55d6ff', '#8b7cff'],
      include_landing: true,
    },
  ]
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/timeline') {
      const submitted = record.body.submit === true
      return {
        status: 200,
        body: {
          ok: true,
          plan_id: 'p_timeline',
          plan_hash: 'hash_timeline',
          duration: record.body.duration_sec,
          scenes: [
            {
              scene_index: 0,
              type: 'text',
              duration_sec: 4,
              formation_id: 'f_scene1',
              points: 64,
              used: 64,
              idle: 0,
              min_distance: 0.3,
            },
          ],
          validation: { valid: true, issues: [], metrics: { min_distance: 0.3 } },
          submitted,
          run_id: submitted ? 'r_timeline' : null,
          scheduled_for: submitted ? 'immediate' : null,
          warnings: [{ code: 'E_SIZE_CLAMPED', detail: 'test warning' }],
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_timeline', {
        num_drones: 64,
        duration_sec: 8,
        scenes,
      })
      assertSuccess(result, 'show_timeline')
      assert.equal(result.structured.summary.plan_id, 'p_timeline')
      assert.equal(result.structured.summary.duration, 8)
      assert.equal(result.structured.summary.scenes[0].formation_id, 'f_scene1')
      assert.equal(result.structured.summary.validation.valid, true)
      assert.equal(result.structured.summary.submitted, true)
      assert.equal(result.structured.summary.run_id, 'r_timeline')
      assert.equal(result.structured.summary.scheduled_for, 'immediate')
      assert.equal(result.structured.summary.warnings[0].code, 'E_SIZE_CLAMPED')

      const timelineRequest = mock.requests.find((request) => request.url === '/api/timeline')
      assert.deepEqual(timelineRequest.body, {
        num_drones: 64,
        duration_sec: 8,
        scenes,
        submit: true,
      })

      const noSubmit = await callTool(tools, 'show_timeline', {
        num_drones: 64,
        duration_sec: 8,
        scenes,
        submit: false,
      })
      assertSuccess(noSubmit, 'show_timeline')
      assert.equal(noSubmit.structured.summary.submitted, false)
      assert.equal(noSubmit.structured.summary.run_id, null)
      const requests = mock.requests.filter((request) => request.url === '/api/timeline')
      assert.equal(requests[requests.length - 1].body.submit, false)
    })
  } finally {
    await mock.close()
  }
})

test('show_timeline normalizes nested plan, scene_summaries, and submission shapes', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/timeline') {
      return {
        status: 200,
        body: {
          ok: true,
          plan: { plan_id: 'p_nested_timeline', plan_hash: 'hash_nested', duration: 12 },
          scene_summaries: [{ scene_index: 0, type: 'heart', duration_sec: 3, formation_id: 'f_heart' }],
          validation: { valid: true, issues: [] },
          submission: { accepted: true, run_id: 'r_nested_timeline', scheduled_for: 'next_dwell' },
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_timeline', {
        num_drones: 64,
        duration_sec: 12,
        scenes: [{ type: 'heart', duration_sec: 12 }],
      })
      assertSuccess(result, 'show_timeline')
      assert.equal(result.structured.summary.plan_id, 'p_nested_timeline')
      assert.equal(result.structured.summary.plan_hash, 'hash_nested')
      assert.equal(result.structured.summary.duration, 12)
      assert.equal(result.structured.summary.scenes[0].formation_id, 'f_heart')
      assert.equal(result.structured.summary.submitted, true)
      assert.equal(result.structured.summary.run_id, 'r_nested_timeline')
      assert.equal(result.structured.summary.scheduled_for, 'next_dwell')
    })
  } finally {
    await mock.close()
  }
})

test('show_timeline surfaces E_DURATION_MISMATCH as a structured tool error', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/timeline') {
      return {
        status: 400,
        body: {
          error: 'E_DURATION_MISMATCH',
          message: 'scene durations do not match duration_sec',
          expected: 60,
          actual: 50,
        },
      }
    }
    return { status: 500, body: { error: 'E_MOCK', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_timeline', {
        num_drones: 64,
        duration_sec: 60,
        scenes: [{ type: 'ring', duration_sec: 50 }],
        submit: false,
      })
      assertFailure(result, 'show_timeline')
      assert.equal(result.structured.error.code, 'E_DURATION_MISMATCH')
      assert.equal(result.structured.error.status, 400)
      assert.equal(result.structured.error.details.expected, 60)
      assert.equal(result.structured.error.details.actual, 50)
    })
  } finally {
    await mock.close()
  }
})

test('show_fleet_timeline calls POST /api/fleet_timeline and shapes allocations and motion mode', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/fleet_timeline') {
      const submitted = record.body.submit === true
      return {
        status: 200,
        body: {
          ok: true,
          total_drones: record.body.total_drones,
          duration: record.body.duration_sec,
          motion_mode: record.body.motion_mode ?? 'kinematic',
          kinematic_preview: (record.body.motion_mode ?? 'kinematic') === 'kinematic',
          scene_allocations: [
            {
              scene_index: 0,
              type: 'text',
              drone_budget: 'auto',
              active: 300,
              idle: 700,
              points_per_char: 100,
              min_distance: 0.3,
            },
            {
              scene_index: 1,
              type: 'ring',
              drone_budget: 510,
              active: 510,
              idle: 490,
              points_per_char: 0,
              min_distance: 0.35,
            },
          ],
          plan_id: 'p_fleet',
          plan_hash: 'hash_fleet',
          validation: { valid: true, issues: [], metrics: { min_distance: 0.3 } },
          submitted,
          run_id: submitted ? 'r_fleet' : null,
          scheduled_for: submitted ? 'immediate' : null,
          warnings: [{ code: 'KINEMATIC_PREVIEW', detail: 'large fleet preview' }],
        },
      }
    }
    return { status: 404, body: { error: 'E_NOT_FOUND', message: 'unexpected mock request' } }
  })

  const scenes = [
    {
      type: 'text',
      drone_budget: 'auto',
      min_points_per_char: 80,
      duration_sec: 15,
      transition_seconds: 5,
      text: 'HELLO',
    },
    {
      type: 'ring',
      drone_budget: 510,
      duration_sec: 45,
      transition_seconds: 5,
      include_landing: true,
    },
  ]

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const preview = await callTool(tools, 'show_fleet_timeline', {
        total_drones: 1000,
        duration_sec: 60,
        motion_mode: 'kinematic',
        scenes,
      })
      assertSuccess(preview, 'show_fleet_timeline')
      assert.equal(preview.structured.summary.total_drones, 1000)
      assert.equal(preview.structured.summary.motion_mode, 'kinematic')
      assert.equal(preview.structured.summary.kinematic_preview, true)
      assert.equal(preview.structured.summary.scene_allocations[0].active, 300)
      assert.equal(preview.structured.summary.scene_allocations[1].active, 510)
      assert.equal(preview.structured.summary.plan_id, 'p_fleet')
      assert.equal(preview.structured.summary.validation.valid, true)
      assert.equal(preview.structured.summary.submitted, false)
      assert.equal(preview.structured.summary.run_id, null)
      assert.equal(preview.structured.summary.warnings[0].code, 'KINEMATIC_PREVIEW')

      const previewRequest = mock.requests.find((request) => request.url === '/api/fleet_timeline')
      assert.deepEqual(previewRequest.body, {
        total_drones: 1000,
        duration_sec: 60,
        motion_mode: 'kinematic',
        scenes,
        submit: false,
      })

      const submitted = await callTool(tools, 'show_fleet_timeline', {
        total_drones: 1000,
        duration_sec: 60,
        motion_mode: 'kinematic',
        scenes,
        submit: true,
      })
      assertSuccess(submitted, 'show_fleet_timeline')
      assert.equal(submitted.structured.summary.submitted, true)
      assert.equal(submitted.structured.summary.run_id, 'r_fleet')
      assert.equal(submitted.structured.summary.scheduled_for, 'immediate')
      const requests = mock.requests.filter((request) => request.url === '/api/fleet_timeline')
      assert.equal(requests[requests.length - 1].body.submit, true)
    })
  } finally {
    await mock.close()
  }
})

test('show_fleet_timeline surfaces E_PHYSICAL_LIMIT as a structured tool error', async () => {
  const mock = await startMockService((record) => {
    if (record.method === 'POST' && record.url === '/api/fleet_timeline') {
      return {
        status: 400,
        body: {
          error: 'E_PHYSICAL_LIMIT',
          message: 'physical mode is unavailable above the measured fleet limit',
          total_drones: 1000,
          physical_limit: 256,
        },
      }
    }
    return { status: 500, body: { error: 'E_MOCK', message: 'unexpected mock request' } }
  })

  try {
    await withServiceUrl(mock.url, async () => {
      const tools = captureRegisteredTools()
      const result = await callTool(tools, 'show_fleet_timeline', {
        total_drones: 1000,
        duration_sec: 15,
        motion_mode: 'physical',
        scenes: [{ type: 'grid', drone_budget: 300, duration_sec: 15 }],
        submit: false,
      })
      assertFailure(result, 'show_fleet_timeline')
      assert.equal(result.structured.error.code, 'E_PHYSICAL_LIMIT')
      assert.equal(result.structured.error.status, 400)
      assert.equal(result.structured.error.details.physical_limit, 256)
    })
  } finally {
    await mock.close()
  }
})
