import { requestJson } from './client.mjs'

export const progressService = {
  name: 'drone.show.progress',
  kind: 'query',
  inputSchema: {
    type: 'object', additionalProperties: false,
    properties: { run_id: { type: 'string', pattern: '^r_[a-f0-9]{12}$' } },
    required: ['run_id'],
  },
  outputSchema: { type: 'object', properties: {
    run_id: { type: 'string' }, status: { type: 'string' }, completed: { type: 'boolean' },
    num_drones: { type: 'integer' }, elapsed_seconds: { type: ['number', 'null'] },
    duration_seconds: { type: ['number', 'null'] }, segment: { type: ['object', 'null'] },
    evaluation: { type: ['object', 'null'] }, error: { type: ['string', 'null'] },
    kinematic_preview: { type: 'boolean' },
  }, required: ['run_id', 'status', 'completed', 'num_drones', 'evaluation'], additionalProperties: false },
  timeoutMs: 5000,
  maxResultBytes: 8192,
  async handler(input, ctx) {
    // Defensive check also protects direct calls outside the Host schema gate.
    if (!/^r_[a-f0-9]{12}$/.test(input?.run_id ?? '')) throw Error('Invalid run_id')
    return requestJson('drone.show.progress', 'GET', '/api/run/' + input.run_id + '/progress', undefined, ctx)
  },
}
