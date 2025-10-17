import { Link } from 'react-router-dom'
import styled from 'styled-components'

const ListWrap = styled.ul` list-style:none; padding:0; margin:0; display:grid; gap:12px; `
const Item = styled.li` border:1px solid #e2e8f0; border-radius:16px; padding:16px; `
const Title = styled.div` font-size:16px; font-weight:600; `
const DateTxt = styled.div` font-size:12px; color:#64748b; `
const Summary = styled.p` font-size:14px; margin:8px 0 0; `

// ✅ Load markdown as RAW text, not as a module
const mdModules = import.meta.glob('../content/posts/*.md', { eager: true, as: 'raw' })

function parseFrontMatter(raw) {
  // tiny YAML front-matter parser: ---\n...\n---\n<body>
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

const posts = Object.entries(mdModules)
  .map(([path, raw]) => {
    const slug = path.split('/').pop().replace(/\.md$/, '')
    const { data } = parseFrontMatter(raw)
    return {
      slug,
      title: data.title || slug,
      date: data.date || 'Unknown date',
      summary: data.summary || ''
    }
  })
  .sort((a, b) => new Date(b.date) - new Date(a.date))

export default function Blog() {
  return (
    <section>
      <h1 style={{ fontSize: 24, margin: '0 0 12px' }}>Tech Blog</h1>
      <ListWrap>
        {posts.map(p => (
          <Item key={p.slug}>
            <Link to={`/blog/${p.slug}`}>
              <Title>{p.title}</Title>
              <DateTxt>{p.date}</DateTxt>
              {p.summary && <Summary>{p.summary}</Summary>}
            </Link>
          </Item>
        ))}
        {posts.length === 0 && (
          <li style={{ color: '#64748b', fontSize: 14 }}>
            No posts yet. Add Markdown files to <code>src/content/posts</code>.
          </li>
        )}
      </ListWrap>
    </section>
  )
}
