import { createElement as h, useEffect, useState, useSyncExternalStore } from 'react'
import { latestSubmittedRun, progressLabel } from './progress.js'

export function apply(ctx) {
  function FlightCard() {
    const sessionId = useSyncExternalStore(ctx.session.subscribe, ctx.session.getSnapshot)
    const [runId, setRunId] = useState(null)
    const [view, setView] = useState(null)
    useEffect(() => {
      let disposed = false, timer, reading = false, dirty = false
      setRunId(null)
      setView(null)
      const refresh = async () => {
        if (disposed) return
        if (reading) { dirty = true; return }
        reading = true
        try {
          const result = await ctx.session.projection.read()
          if (!disposed && result.status === 'available' && result.value.sessionId === sessionId) {
            setRunId(latestSubmittedRun(result.value))
          }
        } finally {
          reading = false
          if (dirty && !disposed) { dirty = false; schedule() }
        }
      }
      const schedule = () => { clearTimeout(timer); timer = setTimeout(() => void refresh().catch(() => {}), 180) }
      const off = ctx.session.projection.subscribe(schedule)
      void refresh().catch(() => {})
      return () => { disposed = true; clearTimeout(timer); off() }
    }, [sessionId])
    useEffect(() => {
      if (!runId) return
      let disposed = false, timer
      setView({ runId, loading: true })
      const poll = async () => {
        try {
          const data = await ctx.agnes.services.call('drone.show.progress', { run_id: runId })
          if (disposed) return
          if (data.run_id !== runId) throw Error('Mismatched run')
          setView({ runId, data })
          if (['completed', 'superseded', 'error', 'rejected', 'cancelled'].includes(data.status)) return
        } catch {
          if (disposed) return
          setView({ runId, offline: true })
        }
        timer = setTimeout(poll, 1000)
      }
      void poll()
      return () => { disposed = true; clearTimeout(timer) }
    }, [runId, sessionId])
    if (!runId || view?.runId !== runId) return null
    const data = view.data
    const label = view.offline ? '连接中断 · 正在重试' : view.loading ? '读取飞行状态…' : progressLabel(data)
    const percent = data?.elapsed_seconds != null && data?.duration_seconds > 0
      ? Math.max(0, Math.min(100, data.elapsed_seconds / data.duration_seconds * 100)) : null
    return h('div', { className: 'agnes-flight-card', 'data-drone-run': runId, 'data-flight-status': data?.status ?? 'connecting', role: 'status' },
      h('div', { className: 'agnes-flight-title' }, h('strong', null, 'AGNES · 无人机秀'),
        h('a', { href: 'http://127.0.0.1:8765/' + (data?.completed ? '?replay=' + runId : ''), target: '_blank', rel: 'noopener noreferrer' }, data?.completed ? '看回放 ↗' : '看表演 ↗')),
      h('div', { className: 'agnes-flight-phase' }, label),
      data && h('div', { className: 'agnes-flight-meta' }, `${data.num_drones} 架${data.kinematic_preview ? ' · 规模预演' : ' · 物理仿真'}`,
        percent != null ? ` · ${Math.round(percent)}%` : ''),
      percent != null && h('progress', { value: percent, max: 100, 'aria-label': '本场实际飞行进度' }),
      data?.evaluation?.metrics?.min_distance != null && h('div', { className: 'agnes-flight-meta' },
        `实际最小间距 ${data.evaluation.metrics.min_distance.toFixed(3)} m · ${data.evaluation.metrics.samples} 帧检查`))
  }
  ctx.slots.register({ name: 'conversation.session.header.corner', id: 'agnes-flight-progress', priority: 10 }, FlightCard)
}
