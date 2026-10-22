import {
  booleanSchema,
  enumerationSchema,
  integerSchema,
  literalSchema,
  numberSchema,
  objectSchema,
  optionalSchema,
  arraySchema,
  stringSchema,
  unionSchema,
  unknownSchema,
} from './schema.mjs'
import { DroneShowServiceError, errorPayload, jsonSafe, requestJson } from './client.mjs'
import { progressService } from './progress.mjs'

function success(toolName, summary, result) {
  const structured = { ok: true, tool: toolName, summary }
  if (result === undefined) {
    // no result payload to attach
  } else {
    structured.result = result
  }
  return {
    content: [{ type: 'text', text: JSON.stringify(summary) }],
    structured: jsonSafe(structured),
  }
}

function failure(toolName, error) {
  const errorInfo = errorPayload(error)
  const structured = { ok: false, tool: toolName, error: errorInfo }
  return {
    content: [{ type: 'text', text: JSON.stringify(structured) }],
    isError: true,
    structured: jsonSafe(structured),
  }
}

function guarded(toolName, execute) {
  return async (args, ctx) => {
    try {
      return await execute(args ?? {}, ctx)
    } catch (error) {
      return failure(toolName, error)
    }
  }
}

function compactSegment(segment) {
  if (segment === null || segment === undefined) return null
  if (typeof segment === 'object') {
    if (Array.isArray(segment) === true) return segment
    const output = {}
    if (segment.id === undefined) {
      // omit undefined
    } else {
      output.id = segment.id
    }
    if (segment.name === undefined) {
      // omit undefined
    } else {
      output.name = segment.name
    }
    if (segment.progress === undefined) {
      // omit undefined
    } else {
      output.progress = segment.progress
    }
    if (segment.color === undefined) {
      // omit undefined
    } else {
      output.color = segment.color
    }
    if (Array.isArray(segment.colors)) {
      output.colors = segment.colors
    }
    return output
  }
  return segment
}

function get(toolName, path, ctx) {
  return requestJson(toolName, 'GET', path, undefined, ctx)
}

function post(toolName, path, body, ctx) {
  return requestJson(toolName, 'POST', path, body, ctx)
}

const readMeta = () => ({
  isReadOnly: true,
  isDestructive: false,
  isConcurrencySafe: true,
  isOpenWorld: false,
  replay: 'safe',
  costHint: {},
  deferLoading: false,
  requiresApproval: 'never',
})

const writeMeta = () => ({
  isReadOnly: false,
  isDestructive: false,
  isConcurrencySafe: false,
  isOpenWorld: false,
  replay: 'never',
  costHint: {},
  deferLoading: false,
  requiresApproval: 'never',
})

const validateMeta = () => ({
  isReadOnly: false,
  isDestructive: false,
  isConcurrencySafe: true,
  isOpenWorld: false,
  replay: 'idempotent',
  costHint: {},
  deferLoading: false,
  requiresApproval: 'never',
})

const emptyObject = objectSchema({})

const formationSpecSchema = unionSchema([
  stringSchema({ minLength: 1, maxLength: 16384 }),
  objectSchema({}, { additionalProperties: true }),
])

const colorSchema = stringSchema({
  minLength: 4,
  maxLength: 9,
  pattern: '^#[0-9a-fA-F]{3}(?:[0-9a-fA-F]{3})?$',
})

const sceneEffectSchema = objectSchema(
  {
    type: optionalSchema(enumerationSchema('none', 'rotate', 'scale', 'wave')),
    turns: optionalSchema(numberSchema({ minimum: 0, maximum: 100 })),
    from: optionalSchema(numberSchema({ minimum: 0.01, maximum: 1000 })),
    to: optionalSchema(numberSchema({ minimum: 0.01, maximum: 1000 })),
    amplitude: optionalSchema(numberSchema({ minimum: 0, maximum: 1000 })),
    cycles: optionalSchema(numberSchema({ minimum: 0, maximum: 1000 })),
  },
  { additionalProperties: true },
)

const sceneSchema = objectSchema(
  {
    type: enumerationSchema('text', 'grid', 'ring', 'heart', 'star'),
    duration_sec: numberSchema({ exclusiveMinimum: 0, maximum: 3600 }),
    transition_seconds: optionalSchema(numberSchema({ exclusiveMinimum: 0, maximum: 3600 })),
    text: optionalSchema(stringSchema({ minLength: 1, maxLength: 256 })),
    color: optionalSchema(colorSchema),
    gradient: optionalSchema(arraySchema(colorSchema, { minItems: 2, maxItems: 2 })),
    effect: optionalSchema(sceneEffectSchema),
    include_landing: optionalSchema(booleanSchema()),
  },
  { additionalProperties: true },
)

const droneBudgetSchema = unionSchema([
  literalSchema('auto'),
  integerSchema({ minimum: 1, maximum: 100000 }),
])

const fleetSceneSchema = objectSchema(
  {
    type: enumerationSchema('text', 'grid', 'ring', 'heart', 'star'),
    drone_budget: optionalSchema(droneBudgetSchema),
    min_points_per_char: optionalSchema(integerSchema({ minimum: 1, maximum: 100000 })),
    duration_sec: numberSchema({ exclusiveMinimum: 0, maximum: 7200 }),
    transition_seconds: optionalSchema(numberSchema({ exclusiveMinimum: 0, maximum: 3600 })),
    text: optionalSchema(stringSchema({ minLength: 1, maxLength: 256 })),
    color: optionalSchema(colorSchema),
    gradient: optionalSchema(arraySchema(colorSchema, { minItems: 2, maxItems: 2 })),
    effect: optionalSchema(sceneEffectSchema),
    include_landing: optionalSchema(booleanSchema()),
  },
  { additionalProperties: true },
)

const showGetState = {
  name: 'show_get_state',
  description:
    'Read the current Drone Show state from the local service. Returns actual positions (field positions), reference targets (field target_positions), revisions, status, active/pending segments, and actual tracking error.',
  parameters: emptyObject,
  meta: readMeta(),
  execute: guarded('show_get_state', async (_args, ctx) => {
    const result = await get('show_get_state', '/api/state', ctx)
    const summary = {
      status: result.status,
      simulation_time: result.simulation_time,
      show_revision: result.show_revision,
      request_revision: result.request_revision,
      num_drones: result.num_drones,
      active_segment: compactSegment(result.active_segment),
      pending_segment: compactSegment(result.pending_segment),
      actual_error: result.actual_error ?? null,
      last_error: result.last_error ?? null,
      positions: Array.isArray(result.positions) ? result.positions : [],
      target_positions: Array.isArray(result.target_positions) ? result.target_positions : [],
    }
    return success('show_get_state', summary, result)
  }),
}

function normalizeFormationSpec(value) {
  let spec = value
  if (typeof spec === 'string') {
    const text = spec.trim()
    if (text.length === 0) {
      throw new DroneShowServiceError('spec must not be an empty string', { code: 'E_BAD_ARGUMENT' })
    }
    try {
      spec = JSON.parse(text)
    } catch (error) {
      throw new DroneShowServiceError(
        'spec must be an object or a JSON-encoded object string',
        { code: 'E_BAD_ARGUMENT' },
      )
    }
  }
  if (spec === null || typeof spec !== 'object' || Array.isArray(spec)) {
    throw new DroneShowServiceError(
      'spec must be an object or a JSON-encoded object string such as {"type":"text","text":"HELLO","size":1.2,"z":1.4,"color":"#00ffaa"}',
      { code: 'E_BAD_ARGUMENT' },
    )
  }
  return spec
}

const showBuildFormation = {
  name: 'show_build_formation',
  description:
    'Build a target formation for the Drone Show service. Supported first types include grid, ring, heart, star, and text. `spec` may be an object or a JSON-encoded object string. Text and color example: {"type":"text","text":"HELLO","size":1.2,"z":1.4,"color":"#00ffaa"}. Gradient color example: {"type":"grid","rows":4,"cols":4,"spacing":0.6,"z":1.2,"gradient":["#ff0044","#00aaff"]}. The service returns formation_id, points, color/colors when supplied, and a resource report.',
  parameters: objectSchema({
    num_drones: integerSchema({ minimum: 1, maximum: 1024 }),
    spec: formationSpecSchema,
  }),
  meta: writeMeta(),
  execute: guarded('show_build_formation', async (args, ctx) => {
    const spec = normalizeFormationSpec(args.spec)
    const result = await post(
      'show_build_formation',
      '/api/formation',
      { num_drones: args.num_drones, spec },
      ctx,
    )
    const points = Array.isArray(result.points) ? result.points : []
    const summary = {
      formation_id: result.formation_id,
      num_drones: args.num_drones,
      point_count: points.length,
      points_preview: points.slice(0, 8),
      color: result.color ?? null,
      colors: Array.isArray(result.colors) ? result.colors : null,
      gradient: result.gradient ?? null,
      required_points: result.required_points ?? null,
      warnings: Array.isArray(result.warnings) ? result.warnings : [],
      resource_report: result.resource_report ?? null,
    }
    return success('show_build_formation', summary, result)
  }),
}

const showPlanTransition = {
  name: 'show_plan_transition',
  description:
    'Plan a transition from a known Drone Show state revision to a target formation. Returns plan_id, plan_hash, total duration, and ordered segments.',
  parameters: objectSchema({
    state_revision: integerSchema({ minimum: 0 }),
    target_formation_id: stringSchema({ minLength: 1, maxLength: 256 }),
    duration: optionalSchema(numberSchema({ exclusiveMinimum: 0, maximum: 3600 })),
    hold_seconds: optionalSchema(numberSchema({ minimum: 0, maximum: 3600 })),
    include_landing: optionalSchema(booleanSchema()),
  }),
  meta: writeMeta(),
  execute: guarded('show_plan_transition', async (args, ctx) => {
    const body = {
      state_revision: args.state_revision,
      target_formation_id: args.target_formation_id,
    }
    if (args.duration === undefined) {
      // service default
    } else {
      body.duration = args.duration
    }
    if (args.hold_seconds === undefined) {
      // service default
    } else {
      body.hold_seconds = args.hold_seconds
    }
    if (args.include_landing === undefined) {
      // service default
    } else {
      body.include_landing = args.include_landing
    }
    const result = await post('show_plan_transition', '/api/plan', body, ctx)
    const segments = Array.isArray(result.segments)
      ? result.segments.map((segment) => compactSegment(segment))
      : []
    const summary = {
      plan_id: result.plan_id,
      plan_hash: result.plan_hash,
      duration: result.duration,
      segment_count: segments.length,
      segments,
    }
    return success('show_plan_transition', summary, result)
  }),
}

const showValidate = {
  name: 'show_validate',
  description:
    'Validate a Drone Show plan. Returns valid, credential, issues, and metrics. The credential is required by show_submit and must be used before the state revision advances.',
  parameters: objectSchema({
    plan_id: stringSchema({ minLength: 1, maxLength: 256 }),
  }),
  meta: validateMeta(),
  execute: guarded('show_validate', async (args, ctx) => {
    const result = await post('show_validate', '/api/validate', { plan_id: args.plan_id }, ctx)
    const issues = Array.isArray(result.issues) ? result.issues : []
    const summary = {
      plan_id: args.plan_id,
      valid: result.valid,
      credential: result.credential,
      issue_count: issues.length,
      issues,
      metrics: result.metrics ?? null,
    }
    return success('show_validate', summary, result)
  }),
}

const showSubmit = {
  name: 'show_submit',
  description:
    'Submit a validated Drone Show plan with its credential. Returns accepted, run_id, and scheduled_for. A stale request is surfaced as structured error E_STALE_STATE.',
  parameters: objectSchema({
    plan_id: stringSchema({ minLength: 1, maxLength: 256 }),
    credential: stringSchema({ minLength: 1, maxLength: 512 }),
    request_revision: integerSchema({ minimum: 0 }),
    source_state_revision: integerSchema({ minimum: 0 }),
    insert_after: optionalSchema(enumerationSchema('current_segment', 'dwell', 'immediate')),
  }),
  meta: writeMeta(),
  execute: guarded('show_submit', async (args, ctx) => {
    const result = await post(
      'show_submit',
      '/api/submit',
      {
        plan_id: args.plan_id,
        credential: args.credential,
        request_revision: args.request_revision,
        source_state_revision: args.source_state_revision,
        insert_after: args.insert_after ?? 'immediate',
      },
      ctx,
    )
    const summary = {
      accepted: result.accepted,
      run_id: result.run_id,
      scheduled_for: result.scheduled_for,
    }
    return success('show_submit', summary, result)
  }),
}

const showGetRun = {
  name: 'show_get_run',
  description:
    'Read status, actual tracking error, completion, and telemetry reference for a submitted Drone Show run. Completed runs also return actual_evaluation from the recorded telemetry. Claim complete only when status=completed and completed=true; claim actual checks passed only when actual_evaluation.verdict=pass. Preview is not physical flight.',
  parameters: objectSchema({
    run_id: stringSchema({ minLength: 1, maxLength: 256 }),
  }),
  meta: readMeta(),
  execute: guarded('show_get_run', async (args, ctx) => {
    const path = '/api/run/' + encodeURIComponent(args.run_id)
    const result = await get('show_get_run', path, ctx)
    const summary = {
      run_id: result.run_id ?? args.run_id,
      status: result.status,
      completed: result.completed,
      actual_error: result.actual_error ?? null,
      telemetry_ref: result.telemetry_ref ?? null,
    }
    if (result.status === 'completed' && result.completed === true) {
      summary.actual_evaluation = await post('show_get_run', '/api/evaluate', { run_id: args.run_id }, ctx)
    }
    if (result.last_error === undefined || result.last_error === null) {
      // omit null last_error
    } else {
      summary.last_error = result.last_error
    }
    if (result.error === undefined || result.error === null) {
      // omit null run error
    } else {
      summary.error = result.error
    }
    return success('show_get_run', summary, result)
  }),
}

const showCancelPending = {
  name: 'show_cancel_pending',
  description: 'Cancel the currently pending submitted Drone Show segment for a request revision.',
  parameters: objectSchema({
    request_revision: integerSchema({ minimum: 0 }),
  }),
  meta: writeMeta(),
  execute: guarded('show_cancel_pending', async (args, ctx) => {
    const result = await post(
      'show_cancel_pending',
      '/api/cancel_pending',
      { request_revision: args.request_revision },
      ctx,
    )
    return success('show_cancel_pending', { cancelled: result.cancelled }, result)
  }),
}

const experimentEvaluate = {
  name: 'experiment_evaluate',
  description:
    'Evaluate one completed Drone Show run with the service-side fixed experiment endpoint. Returns verdict, per-check pass/fail details, and actual metrics. Thresholds are owned by the service and cannot be overridden by this tool.',
  parameters: objectSchema({
    run_id: stringSchema({ minLength: 1, maxLength: 256 }),
  }),
  meta: readMeta(),
  execute: guarded('experiment_evaluate', async (args, ctx) => {
    const result = await post('experiment_evaluate', '/api/evaluate', { run_id: args.run_id }, ctx)
    const summary = {
      run_id: result.run_id ?? args.run_id,
      verdict: result.verdict,
      checks: Array.isArray(result.checks) ? result.checks : [],
      metrics: result.metrics ?? null,
    }
    return success('experiment_evaluate', summary, result)
  }),
}

const showTextShow = {
  name: 'show_text_show',
  description:
    'Create and optionally submit a complete drone show from text. Users can say: 用 64 架显示AGNES，颜色渐变，停 3 秒. For denser and larger characters, use num_drones=128 and base_height=4.0 (for two Chinese characters this yields 64 points per character in one page). This high-level tool handles CJK/text glyphs, drone allocation, pagination, collision repair, the service validator, and atomic submission. It calls POST /api/text_show. Prefer it over manually chaining show_build_formation, show_plan_transition, show_validate, and show_submit. `submit` defaults to true. The service response is returned as pages, allocation_report, plan_id, validation, submitted, run_id, scheduled_for, and warnings.',
  parameters: objectSchema({
    text: stringSchema({ minLength: 1, maxLength: 256 }),
    num_drones: integerSchema({ minimum: 1, maximum: 256 }),
    auto_allocate: optionalSchema(booleanSchema()),
    min_points_per_char: optionalSchema(integerSchema({ minimum: 1, maximum: 4096 })),
    hold_seconds: optionalSchema(numberSchema({ minimum: 0, maximum: 3600 })),
    transition_seconds: optionalSchema(numberSchema({ exclusiveMinimum: 0, maximum: 3600 })),
    include_landing: optionalSchema(booleanSchema()),
    base_height: optionalSchema(numberSchema({ exclusiveMinimum: 0.5, maximum: 20 })),
    letter_spacing: optionalSchema(numberSchema({ minimum: 0, maximum: 5 })),
    z: optionalSchema(numberSchema({ minimum: 0.05, maximum: 2.9 })),
    color: optionalSchema(colorSchema),
    gradient: optionalSchema(arraySchema(colorSchema, { minItems: 2, maxItems: 2 })),
    submit: optionalSchema(booleanSchema()),
  }),
  meta: writeMeta(),
  execute: guarded('show_text_show', async (args, ctx) => {
    const body = {
      text: args.text,
      num_drones: args.num_drones,
      submit: args.submit ?? true,
    }
    for (const key of [
      'auto_allocate',
      'min_points_per_char',
      'hold_seconds',
      'transition_seconds',
      'include_landing',
      'base_height',
      'letter_spacing',
      'z',
      'color',
      'gradient',
    ]) {
      if (args[key] === undefined) {
        // service default
      } else {
        body[key] = args[key]
      }
    }
    if (args.min_points_per_char === undefined) {
      // service default
    } else {
      // The service may expose either min_points_per_char or points_per_char.
      body.points_per_char = args.min_points_per_char
    }
    const result = await post('show_text_show', '/api/text_show', body, ctx)
    const rawPages = Array.isArray(result.pages)
      ? result.pages
      : Array.isArray(result.allocation?.pages)
        ? result.allocation.pages
        : Array.isArray(result.pagination?.pages)
          ? result.pagination.pages
          : []
    const pages = rawPages.map((page, index) => {
      const characters = Array.isArray(page.characters) ? page.characters : []
      const pageText =
        page.text === undefined || page.text === null
          ? characters.length > 0
            ? characters.join('')
            : null
          : page.text
      return {
        page: page.page ?? page.page_index ?? index,
        text: pageText,
        formation_id: page.formation_id ?? null,
        points: page.points ?? null,
        used: page.used ?? page.points ?? null,
        idle: page.idle ?? null,
        min_distance: page.min_distance ?? null,
      }
    })
    let allocationReport = Array.isArray(result.allocation_report) ? result.allocation_report : []
    if (allocationReport.length === 0 && rawPages.length > 0) {
      allocationReport = rawPages.flatMap((page) => {
        const characters = Array.isArray(page.characters) ? page.characters : []
        const allocations = Array.isArray(page.allocations) ? page.allocations : []
        return characters.map((char, index) => ({
          char,
          complexity: null,
          points: allocations[index] ?? null,
        }))
      })
    }
    const planId = result.plan_id ?? result.plan?.plan_id ?? null
    const planHash = result.plan_hash ?? result.plan?.plan_hash ?? null
    const validation =
      result.validation && typeof result.validation === 'object'
        ? result.validation
        : { valid: null, issues: [] }
    const submission = result.submission && typeof result.submission === 'object' ? result.submission : null
    const summary = {
      text: result.text ?? args.text,
      num_drones: result.num_drones ?? result.allocation?.num_drones ?? args.num_drones,
      pages,
      allocation_report: allocationReport,
      plan_id: planId,
      plan_hash: planHash,
      validation,
      submitted: result.submitted === true ? true : submission?.accepted === true,
      run_id: result.run_id ?? submission?.run_id ?? null,
      scheduled_for: result.scheduled_for ?? submission?.scheduled_for ?? null,
      warnings: Array.isArray(result.warnings) ? result.warnings : [],
    }
    return success('show_text_show', summary, result)
  }),
}

const showTimeline = {
  name: 'show_timeline',
  description:
    'Create and optionally submit a complete timeline animation. Users can say: 做一个60秒动画：先 HELLO，再变成旋转圆环，再变心形，最后显示AGNES并降落. Each scene has type (text/grid/ring/heart/star), duration_sec, transition_seconds, text, color, gradient, effect (none/rotate/scale/wave), and other scene fields. This tool calls POST /api/timeline, which plans, validates, optionally repairs collisions, and optionally atomically submits the whole show. `submit` defaults to true. Returns plan_id, duration, scenes, validation, submitted, run_id, scheduled_for, and warnings.',
  parameters: objectSchema({
    num_drones: integerSchema({ minimum: 1, maximum: 256 }),
    duration_sec: numberSchema({ exclusiveMinimum: 0, maximum: 3600 }),
    scenes: arraySchema(sceneSchema, { minItems: 1, maxItems: 64 }),
    submit: optionalSchema(booleanSchema()),
  }),
  meta: writeMeta(),
  execute: guarded('show_timeline', async (args, ctx) => {
    const body = {
      num_drones: args.num_drones,
      duration_sec: args.duration_sec,
      scenes: args.scenes,
      submit: args.submit ?? true,
    }
    const result = await post('show_timeline', '/api/timeline', body, ctx)
    const scenes = Array.isArray(result.scenes)
      ? result.scenes
      : Array.isArray(result.scene_summaries)
        ? result.scene_summaries
        : []
    const validation =
      result.validation && typeof result.validation === 'object'
        ? result.validation
        : { valid: null, issues: [] }
    const submission = result.submission && typeof result.submission === 'object' ? result.submission : null
    const summary = {
      plan_id: result.plan_id ?? result.plan?.plan_id ?? null,
      plan_hash: result.plan_hash ?? result.plan?.plan_hash ?? null,
      duration: result.duration ?? result.plan?.duration ?? null,
      scenes,
      validation,
      submitted: result.submitted === true ? true : submission?.accepted === true,
      run_id: result.run_id ?? submission?.run_id ?? null,
      scheduled_for: result.scheduled_for ?? submission?.scheduled_for ?? null,
      warnings: Array.isArray(result.warnings) ? result.warnings : [],
    }
    return success('show_timeline', summary, result)
  }),
}

const showFleetTimeline = {
  name: 'show_fleet_timeline',
  description:
    'Create and optionally submit a large-scale fleet timeline. Users can say: 总共1000架，第一段3个字自动选300架，第二段选510架做圆环，最后降落. Each scene has type, drone_budget (auto or an integer), min_points_per_char, duration_sec, transition_seconds, text/color/gradient/effect, and optional planner fields. This tool calls POST /api/fleet_timeline, which chooses active drones per scene, parks idle drones safely, builds and validates the plan, and optionally submits atomically. `submit` defaults to false for large previews. Returns scene_allocations, active/idle/points_per_char, motion_mode, plan_id, validation, submitted, run_id, scheduled_for, and warnings.',
  parameters: objectSchema({
    total_drones: integerSchema({ minimum: 1, maximum: 100000 }),
    duration_sec: numberSchema({ exclusiveMinimum: 0, maximum: 7200 }),
    motion_mode: optionalSchema(enumerationSchema('kinematic', 'physical')),
    scenes: arraySchema(fleetSceneSchema, { minItems: 1, maxItems: 128 }),
    submit: optionalSchema(booleanSchema()),
  }),
  meta: writeMeta(),
  execute: guarded('show_fleet_timeline', async (args, ctx) => {
    const body = {
      total_drones: args.total_drones,
      duration_sec: args.duration_sec,
      scenes: args.scenes,
      submit: args.submit ?? false,
    }
    if (args.motion_mode === undefined) {
      // service default
    } else {
      body.motion_mode = args.motion_mode
    }
    const result = await post('show_fleet_timeline', '/api/fleet_timeline', body, ctx)
    const sceneAllocations = Array.isArray(result.scene_allocations)
      ? result.scene_allocations
      : Array.isArray(result.allocation?.scene_allocations)
        ? result.allocation.scene_allocations
        : Array.isArray(result.allocation?.scenes)
          ? result.allocation.scenes
          : Array.isArray(result.scenes)
            ? result.scenes
            : []
    const validation =
      result.validation && typeof result.validation === 'object'
        ? result.validation
        : { valid: null, issues: [] }
    const submission = result.submission && typeof result.submission === 'object' ? result.submission : null
    const summary = {
      total_drones: result.total_drones ?? args.total_drones,
      duration: result.duration ?? args.duration_sec,
      motion_mode:
        result.motion_mode ??
        args.motion_mode ??
        (result.kinematic_preview === true ? 'kinematic' : null),
      kinematic_preview: result.kinematic_preview === true,
      scene_allocations: sceneAllocations,
      plan_id: result.plan_id ?? result.plan?.plan_id ?? null,
      plan_hash: result.plan_hash ?? result.plan?.plan_hash ?? null,
      validation,
      submitted: result.submitted === true ? true : submission?.accepted === true,
      run_id: result.run_id ?? submission?.run_id ?? null,
      scheduled_for: result.scheduled_for ?? submission?.scheduled_for ?? null,
      warnings: Array.isArray(result.warnings) ? result.warnings : [],
    }
    return success('show_fleet_timeline', summary, result)
  }),
}

const showPerform = {
  name: 'show_perform',
  description: '在本地 PyBullet 仿真中自动编排并播放无人机秀，无真实硬件。普通表演和表演中的改编都优先调用此工具，用户只说一句话，提取文字、图形、颜色和动画，不询问点数、间距、高度、时长或 JSON。文字 text、爱心 heart、圆环 ring、星 star、方阵 grid；三维动态编队双螺旋 helix、原子轨道 atom、振翅飞鸟 bird 自动运动，默认紧凑半圈，完整旋转可明确指定 1 圈；effect.cycles 可指定 .25 到 3。用户要求 AGNES 千架主题秀、一念成光或六幕表演时，用 theme="agnes-light", num_drones=1000, ending="land"，省略 scenes。这个六幕主题包含地面分批起降、A与AGNES、双螺旋、星河、千机光浪和金色回归，运行完整真实离线物理仿真，可能约需20分钟，页面按仿真时间显示进度。普通动态场景仍支持16到128架实时物理，其他千架自定义场景保持明确标注的规模预演。未指定数量时省略 num_drones。起飞、归位、降落自动添加，降落仅用 ending="land"，悬停 ending="hover"，不要额外添加 grid。表演中调用会在稳定阶段安全接入新场景，保留当前数量；先 show_get_state 查看 request_revision 可避免过期改编。waiting_checkpoint 仅表示等待接入，started 仅表示已提交，只有 show_get_run.status="completed" 且 completed=true 才表示完整结束，superseded 表示旧表演被改编。先读状态，再提交 show_perform，提交后查询一次 show_get_run，随后用一两句话告知已提交或等待接入。页面会自动显示进度、检查与结束状态；不要调用 shell、sleep 或循环查询来等待演出结束，不要输出长篇参数解释。',

  parameters: objectSchema({
    theme: optionalSchema(enumerationSchema('agnes-light')),
    scenes: optionalSchema(arraySchema(objectSchema({
      type: enumerationSchema('text', 'heart', 'ring', 'star', 'grid', 'helix', 'atom', 'bird'),
      text: optionalSchema(stringSchema()),
      color: optionalSchema(stringSchema()),
      gradient: optionalSchema(arraySchema(stringSchema())),
      hold_seconds: optionalSchema(numberSchema({ minimum: 2, maximum: 120 })),
      effect: optionalSchema(objectSchema({
        type: enumerationSchema('none', 'rotate', 'scale', 'wave'),
        turns: optionalSchema(numberSchema()),
        amplitude: optionalSchema(numberSchema()),
        cycles: optionalSchema(numberSchema()),
        from: optionalSchema(numberSchema()),
        to: optionalSchema(numberSchema()),
      })),
      drone_budget: optionalSchema(unionSchema([literalSchema('auto'), integerSchema()])),
    }))),
    num_drones: optionalSchema(integerSchema()),
    request_revision: optionalSchema(integerSchema()),
    ending: { ...enumerationSchema('land', 'hover'), description: '结束动作：land 自动归位并降落；hover 保持末尾编队。降落不要添加 grid 场景。默认选 land。' },
    submit: optionalSchema(booleanSchema()),
  }),
  meta: writeMeta(),
  execute: guarded('show_perform', async (args, ctx) => {
    const { ending, ...show } = args
    const result = await post('show_perform', '/api/perform', {
      ...show, include_landing: ending !== 'hover',
    }, ctx)
    return success('show_perform', {
      run_id: result.run_id,
      status: !result.submitted ? 'planned' : result.scheduled_for === 'safe_dwell' ? 'waiting_checkpoint' : 'started',
      source_run_id: result.source_run_id ?? null,
      duration: result.duration,
      num_drones: result.num_drones ?? result.total_drones,
      motion_mode: result.motion_mode,
      offline_execution: !!result.offline_execution,
      scenes: result.scenes ?? result.scene_allocations,
      validation: result.validation,
      page: 'http://127.0.0.1:8765/',
    }, result)
  }),
}

export const tools = Object.freeze([
  showPerform,
  showGetState,
  showBuildFormation,
  showPlanTransition,
  showValidate,
  showSubmit,
  showGetRun,
  showCancelPending,
  experimentEvaluate,
  showTextShow,
  showTimeline,
  showFleetTimeline,
])

export const droneShowPlugin = {
  inject: ['extension', 'services'],
  apply(ctx) {
    const agnes = ctx.extension()
    for (const tool of tools) agnes.registerTool(tool)
    ctx.services.register(progressService)
  },
}
