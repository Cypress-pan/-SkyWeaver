// HTTP transport and structured error shaping for the Drone Show AGH plugin.
// There is no logging in this module; credentials are never written to source or logs.

const DEFAULT_SERVICE_URL = 'http://127.0.0.1:8765'
const DEFAULT_TIMEOUT_MS = 15_000
const SENSITIVE_KEY = /(?:pass(?:word)?|secret|token|credential|authorization|api[_-]?key|cookie)/iu

export class DroneShowServiceError extends Error {
  constructor(message, options = {}) {
    super(message)
    this.name = 'DroneShowServiceError'
    this.code = options.code ?? 'E_SERVICE'
    if (options.status !== undefined) this.status = options.status
    if (options.method !== undefined) this.method = options.method
    if (options.path !== undefined) this.path = options.path
    if (options.details !== undefined) this.details = options.details
    if (options.cause !== undefined) this.cause = options.cause
  }
}

export function redactSecrets(value, depth = 0) {
  if (depth > 8 || value === null || typeof value !== 'object') return value
  if (Array.isArray(value)) return value.map((item) => redactSecrets(item, depth + 1))
  const output = {}
  for (const [key, item] of Object.entries(value)) {
    output[key] = SENSITIVE_KEY.test(key) ? '[redacted]' : redactSecrets(item, depth + 1)
  }
  return output
}

export function jsonSafe(value) {
  return JSON.parse(JSON.stringify(value))
}

function serviceBaseUrl() {
  const configured = process.env.DRONE_SHOW_SERVICE_URL
  const value =
    configured === undefined || configured === null || String(configured).trim() === ''
      ? DEFAULT_SERVICE_URL
      : String(configured).trim()
  let url
  try {
    url = new URL(value)
  } catch {
    throw new DroneShowServiceError(
      `Invalid DRONE_SHOW_SERVICE_URL: expected an absolute http(s) URL, received ${JSON.stringify(value)}`,
      { code: 'E_CONFIG' },
    )
  }
  if (url.protocol !== 'http:' && url.protocol !== 'https:') {
    throw new DroneShowServiceError('Invalid DRONE_SHOW_SERVICE_URL: only http and https are supported', {
      code: 'E_CONFIG',
    })
  }
  // This plugin is the local simulator bridge, never a generic network/hardware controller.
  // That closed endpoint contract is also what permits trusted structured telemetry metadata.
  if (!['127.0.0.1', 'localhost', '[::1]'].includes(url.hostname)
      || url.username || url.password) {
    throw new DroneShowServiceError('Drone Show only accepts a loopback simulator endpoint', {
      code: 'E_CONFIG',
    })
  }
  if (url.search !== '' || url.hash !== '') {
    throw new DroneShowServiceError(
      'Invalid DRONE_SHOW_SERVICE_URL: query strings and fragments are not supported',
      { code: 'E_CONFIG' },
    )
  }
  return value.replace(/\/+$/u, '')
}

function timeoutMs() {
  const configured = Number(process.env.DRONE_SHOW_TIMEOUT_MS)
  return Number.isFinite(configured) && configured > 0 ? configured : DEFAULT_TIMEOUT_MS
}

export async function requestJson(toolName, method, path, body, ctx) {
  const baseUrl = serviceBaseUrl()
  const url = baseUrl + (path.charAt(0) === '/' ? path : '/' + path)
  const requestTimeout = toolName === 'show_perform' ? Math.max(timeoutMs(), 90_000) : timeoutMs()
  const timeoutSignal = AbortSignal.timeout(requestTimeout)
  const requestSignal = ctx?.signal ? AbortSignal.any([ctx.signal, timeoutSignal]) : timeoutSignal
  const headers = { accept: 'application/json' }
  const init = { method, headers, signal: requestSignal, redirect: 'error' }
  if (body !== undefined) {
    headers['content-type'] = 'application/json'
    init.body = JSON.stringify(body)
  }

  let response
  try {
    response = await fetch(url, init)
  } catch (error) {
    if (ctx?.signal?.aborted === true && timeoutSignal.aborted === false) {
      throw new DroneShowServiceError(toolName + ': request was cancelled', {
        code: 'E_CANCELLED',
        method,
        path,
        cause: error,
      })
    }
    if (timeoutSignal.aborted === true) {
      throw new DroneShowServiceError(toolName + ': request timed out after ' + requestTimeout + 'ms', {
        code: 'E_TIMEOUT',
        method,
        path,
        cause: error,
      })
    }
    throw new DroneShowServiceError(toolName + ': cannot reach Drone Show service at ' + baseUrl, {
      code: 'E_SERVICE_UNREACHABLE',
      method,
      path,
      cause: error,
    })
  }

  const raw = await response.text()
  let data = null
  if (raw.length > 0) {
    try {
      data = JSON.parse(raw)
    } catch (error) {
      throw new DroneShowServiceError(toolName + ': Drone Show service returned invalid JSON', {
        code: 'E_INVALID_RESPONSE',
        status: response.status,
        method,
        path,
        details: raw.slice(0, 500),
        cause: error,
      })
    }
  }

  if (response.ok === false) {
    let code = 'E_HTTP_' + response.status
    if (typeof data?.error === 'string') code = data.error
    else if (typeof data?.code === 'string') code = data.code
    let message = toolName + ': Drone Show service ' + method + ' ' + path + ' failed with HTTP ' + response.status
    if (typeof data?.message === 'string' && data.message.length > 0) message = data.message
    throw new DroneShowServiceError(message, {
      code,
      status: response.status,
      method,
      path,
      details: redactSecrets(data),
    })
  }

  if (data === null) {
    throw new DroneShowServiceError(toolName + ': Drone Show service returned an empty JSON body', {
      code: 'E_INVALID_RESPONSE',
      status: response.status,
      method,
      path,
    })
  }
  if (typeof data === 'object') {
    if (Array.isArray(data) === true) {
      throw new DroneShowServiceError(toolName + ': Drone Show service returned a JSON array, expected an object', {
        code: 'E_INVALID_RESPONSE',
        status: response.status,
        method,
        path,
      })
    }
  } else {
    throw new DroneShowServiceError(toolName + ': Drone Show service returned a non-object JSON value', {
      code: 'E_INVALID_RESPONSE',
      status: response.status,
      method,
      path,
    })
  }

  if (
    typeof data.error === 'string' &&
    data.run_id === undefined &&
    data.status === undefined &&
    (data.ok === true) === false
  ) {
    let message = data.error
    if (typeof data.message === 'string') message = data.message
    throw new DroneShowServiceError(message, {
      code: data.error,
      status: response.status,
      method,
      path,
      details: redactSecrets(data),
    })
  }

  return data
}

export function errorPayload(error) {
  let serviceError = error
  if (error instanceof DroneShowServiceError) {
    serviceError = error
  } else {
    const code = typeof error?.code === 'string' && error.code.length > 0 ? error.code : 'E_PLUGIN_TOOL'
    serviceError = new DroneShowServiceError(String(error?.message ?? error), { code })
  }
  const payload = { code: serviceError.code, message: serviceError.message }
  if (serviceError.status === undefined) payload.status = undefined
  else payload.status = serviceError.status
  if (serviceError.method === undefined) payload.method = undefined
  else payload.method = serviceError.method
  if (serviceError.path === undefined) payload.path = undefined
  else payload.path = serviceError.path
  if (serviceError.details === undefined) payload.details = undefined
  else payload.details = redactSecrets(serviceError.details)
  return payload
}
