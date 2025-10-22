import { Link } from "react-router-dom";
import styled from "styled-components";

const ListWrap = styled.ul`
  list-style: none;
  padding: 0;
  margin: 0;
  display: grid;
  gap: 20px;
  max-width: 900px;
`;

const Item = styled.li`
  border: 1px solid #e2e8f0;
  border-radius: 12px;
  padding: 24px;
  background: #fff;
  transition: all 0.3s ease;
  box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.05);

  &:hover {
    transform: translateY(-2px);
    box-shadow: 0 8px 16px -4px rgba(0, 0, 0, 0.1);
    border-color: #cbd5e1;
  }

  a {
    text-decoration: none;
    color: inherit;
    display: block;
  }
`;

const Title = styled.div`
  font-size: 20px;
  font-weight: 700;
  color: #0f172a;
  margin-bottom: 12px;
  line-height: 1.3;
  letter-spacing: -0.01em;
  transition: color 0.2s ease;

  ${Item}:hover & {
    color: #2563eb;
  }
`;

const MetaRow = styled.div`
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  align-items: center;
  margin-bottom: 12px;
`;

const MetaBadge = styled.span`
  display: inline-flex;
  align-items: center;
  gap: 4px;
  padding: 4px 10px;
  background: #f1f5f9;
  border-radius: 6px;
  font-size: 12px;
  font-weight: 500;
  color: #64748b;
`;

const KeywordTag = styled.span`
  display: inline-flex;
  align-items: center;
  padding: 4px 10px;
  background: #dbeafe;
  color: #1e40af;
  border-radius: 6px;
  font-size: 12px;
  font-weight: 500;
`;

const Summary = styled.p`
  font-size: 15px;
  line-height: 1.6;
  color: #475569;
  margin: 0;
`;

// ✅ Load markdown as RAW text, not as a module
const mdModules = import.meta.glob("../content/docs/*.md", {
  query: "?raw",
  import: "default",
  eager: true,
});

function parseFrontMatter(raw) {
  // tiny YAML front-matter parser: ---\n...\n---\n<body>
  const m = raw.match(/^---\n([\s\S]*?)\n---\n?([\s\S]*)$/);
  if (!m) return { data: {}, body: raw };
  const yaml = m[1],
    body = m[2];
  const data = {};
  yaml.split("\n").forEach(line => {
    const i = line.indexOf(":");
    if (i !== -1) {
      const k = line.slice(0, i).trim();
      const v = line
        .slice(i + 1)
        .trim()
        .replace(/^"|"$|^'|'$/g, "");
      data[k] = v;
    }
  });
  return { data, body };
}

const posts = Object.entries(mdModules)
  .map(([path, raw]) => {
    const slug = path.split("/").pop().replace(/\.md$/, "");
    const { data, body } = parseFrontMatter(raw);
    // Parse keywords (comma-separated string to array)
    const keywords = data.keywords
      ? data.keywords
          .split(",")
          .map(k => k.trim())
          .filter(Boolean)
      : [];
    return {
      slug,
      title: data.title || slug,
      date: data.date || "Unknown date",
      summary: data.summary || "",
      writer: data.writer || "",
      keywords,
    };
  })
  .sort((a, b) => new Date(b.date) - new Date(a.date));

const PageHeader = styled.header`
  margin-bottom: 32px;
`;

const PageTitle = styled.h1`
  font-size: clamp(28px, 5vw, 36px);
  font-weight: 800;
  color: #0f172a;
  margin: 0 0 8px;
  letter-spacing: -0.02em;
`;

const PageSubtitle = styled.p`
  font-size: 16px;
  color: #64748b;
  margin: 0;
`;

export default function Documents() {
  return (
    <section style={{ paddingTop: "40px" }}>
      <PageHeader>
        <PageTitle>Documents</PageTitle>
        <PageSubtitle>
          Technical documentation and guides for the rover project
        </PageSubtitle>
      </PageHeader>

      <ListWrap>
        {posts.map(p => (
          <Item key={p.slug}>
            <Link to={`/documents/${p.slug}`}>
              <Title>{p.title}</Title>
              <MetaRow>
                {p.date && <MetaBadge>📅 {p.date}</MetaBadge>}
                {p.writer && <MetaBadge>✍️ {p.writer}</MetaBadge>}
                {p.keywords.map((keyword, idx) => (
                  <KeywordTag key={idx}>{keyword}</KeywordTag>
                ))}
              </MetaRow>
              {p.summary && <Summary>{p.summary}</Summary>}
            </Link>
          </Item>
        ))}
        {posts.length === 0 && (
          <li style={{ color: "#64748b", fontSize: 14 }}>
            No posts yet. Add Markdown files to <code>src/content/docs</code>.
          </li>
        )}
      </ListWrap>
    </section>
  );
}
