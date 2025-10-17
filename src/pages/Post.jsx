import { useParams } from 'react-router-dom'
import ReactMarkdown from 'react-markdown'

// ✅ Load markdown as RAW text
const mdModules = import.meta.glob('../content/posts/*.md', { eager: true, as: 'raw' })

function parseFrontMatter(raw) {
  const m = raw.match(/^---\n([\s\S]*?)\n---\n?([\s\S]*)$/)
  if (!m) return { data: {}, body: raw }
  const yaml = m[1], body = m[2]
  const data = {}
  yaml.split('\n').forEach(line => {
    const i = line.indexOf(':')
    if (i !== -1) {
      const k = line.slice(0, i).trim()
      const v = line.slice(i + 1).trim().replace(/^"|"$|^'|'$/g, '')
      data[k] = v
    }
  })
  return { data, body }
}

export default function Post() {
  const { slug } = useParams()
  const key = Object.keys(mdModules).find(k => k.endsWith(`${slug}.md`))
  const raw = key ? mdModules[key] : null

  if (!raw) return <p style={{ color: '#64748b' }}>Post not found.</p>

  const { data, body } = parseFrontMatter(raw)

  return (
    <article>
      <h1 style={{ fontSize: 28, margin: '0 0 6px' }}>{data.title || slug}</h1>
      {data.date && <div style={{ fontSize: 12, color: '#64748b' }}>{data.date}</div>}
      <ReactMarkdown>{body}</ReactMarkdown>
    </article>
  )
}
