// Public UI projection only. A card belongs to the current session's latest
// successful show submission, never the simulator's unrelated global latest run.
export function latestSubmittedRun(timeline) {
  const nodes = timeline?.nodes ?? []
  for (let i = nodes.length - 1; i >= 0; i--) {
    const node = nodes[i]
    if (node.kind !== 'tool' || node.name !== 'show_perform' || node.status !== 'completed') continue
    const match = /"run_id"\s*:\s*"(r_[a-f0-9]{12})"/.exec(node.resultPreview ?? '')
    if (match) return match[1]
  }
  return null
}

export function phaseLabel(name = '') {
  if (name.startsWith('takeoff')) return '起飞'
  if (name === 'return' || name === 'clearance') return '安全过渡'
  if (name.startsWith('landing')) return '降落'
  const labels = { helix: '双螺旋', atom: '原子轨道', bird: '振翅飞鸟', heart: '爱心', ring: '圆环', star: '星星', grid: '方阵' }
  const kind = name.replace(/^assemble:|^scene\d*:|^dwell:|^return:|^landing:|^takeoff:/, '').replace(/_motion$/, '')
  return (name.startsWith('assemble:') ? '编队 · ' : '') + ((labels[kind] ?? kind) || '飞行中')
}

export function progressLabel(data) {
  if (data?.status === 'completed' && data.completed) {
    if (data.evaluation?.verdict === 'pass') return '表演完成 · 实际检查通过'
    if (data.evaluation?.verdict === 'preview') return '预演完成 · 尚未进行物理仿真'
    return data.evaluation?.verdict === 'fail' ? '表演结束 · 实际检查未通过' : '飞行结束 · 等待检查'
  }
  if (data?.status === 'superseded') return '已接入新的改编'
  if (['error', 'rejected', 'cancelled'].includes(data?.status)) return '运行中断 · ' + data.status
  return data?.segment ? phaseLabel(data.segment.name) : '已提交 · 等待安全接入'
}
