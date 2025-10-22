import { useParams, Link } from "react-router-dom";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import styled, { css } from "styled-components";

// ✅ Load markdown as RAW text
const mdModules = import.meta.glob("../content/docs/*.md", {
  query: "?raw",
  import: "default",
  eager: true,
});

function parseFrontMatter(raw) {
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

/* ---------- UI ---------- */
const PostBody = styled.article`
  padding-top: 40px;
  max-width: 800px;
  margin: 0 auto;
`;

const BackBtn = styled(Link)`
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  border-radius: 8px;
  font-weight: 600;
  font-size: 14px;
  color: #475569;
  text-decoration: none;
  border: 1px solid #e2e8f0;
  background: #fff;
  transition: all 0.2s ease;

  &:hover {
    background: #f8fafc;
    border-color: #cbd5e1;
    transform: translateX(-2px);
  }

  &:before {
    content: "←";
    font-size: 16px;
    line-height: 1;
  }
`;

const PostHeader = styled.header`
  margin: 32px 0 40px;
  padding-bottom: 32px;
  border-bottom: 2px solid #f1f5f9;
`;

const PostTitle = styled.h1`
  font-size: clamp(28px, 5vw, 42px);
  font-weight: 800;
  line-height: 1.2;
  letter-spacing: -0.02em;
  color: #0f172a;
  margin: 0 0 16px;
`;

const PostSummary = styled.p`
  font-size: 18px;
  line-height: 1.6;
  color: #475569;
  margin: 0 0 20px;
`;

const MetaBar = styled.div`
  display: flex;
  flex-wrap: wrap;
  gap: 16px;
  align-items: center;
  margin-top: 16px;
`;

const MetaItem = styled.div`
  display: inline-flex;
  align-items: center;
  gap: 6px;
  font-size: 14px;
  color: #64748b;

  &:before {
    content: "${props => props.icon || ""}";
    font-size: 16px;
  }
`;

const MetaBadge = styled.span`
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 6px 12px;
  background: #f1f5f9;
  border-radius: 6px;
  font-size: 13px;
  font-weight: 500;
  color: #475569;
`;

const KeywordTag = styled.span`
  display: inline-flex;
  align-items: center;
  padding: 6px 12px;
  background: #dbeafe;
  color: #1e40af;
  border-radius: 6px;
  font-size: 13px;
  font-weight: 500;
`;

const ContentWrapper = styled.div`
  line-height: 1.8;
  color: #1e293b;
  font-size: 16px;

  h1,
  h2,
  h3 {
    line-height: 1.3;
    margin: 32px 0 16px;
    font-weight: 700;
    letter-spacing: -0.01em;
    color: #0f172a;
  }

  h2 {
    font-size: clamp(24px, 4vw, 28px);
    padding-bottom: 8px;
    border-bottom: 1px solid #f1f5f9;
  }

  h3 {
    font-size: clamp(20px, 3.5vw, 24px);
  }

  p {
    margin: 16px 0;
  }

  /* First paragraph as lead */
  > p:first-child {
    font-size: 18px;
    line-height: 1.7;
    color: #334155;
  }

  /* Images: responsive, centered, with shadow */
  img {
    display: block;
    width: auto;
    max-width: 500px;
    height: auto;
    margin: 24px auto;
    border-radius: 12px;
    box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1),
      0 2px 4px -1px rgba(0, 0, 0, 0.06);
  }

  @media (max-width: 768px) {
    img {
      max-width: 100%;
    }
  }

  ul,
  ol {
    margin: 16px 0;
    padding-left: 24px;
  }

  li {
    margin: 8px 0;
  }

  table {
    border-collapse: collapse;
    width: 100%;
    overflow: hidden;
    border-radius: 8px;
    box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.1);
  }

  thead {
    background: #f8fafc;
  }

  th,
  td {
    border: 1px solid #e2e8f0;
    padding: 12px 16px;
    text-align: left;
  }

  th {
    font-weight: 600;
    color: #0f172a;
  }

  .table-wrap {
    overflow-x: auto;
    margin: 24px 0;
  }

  code {
    background: #f1f5f9;
    border: 1px solid #e2e8f0;
    border-radius: 4px;
    padding: 3px 8px;
    font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, monospace;
    font-size: 0.9em;
    color: #e11d48;
  }

  pre {
    background: #1e293b;
    border-radius: 8px;
    padding: 20px;
    overflow-x: auto;
    margin: 24px 0;
  }

  pre code {
    display: block;
    padding: 0;
    background: transparent;
    border: none;
    white-space: pre;
    color: #e2e8f0;
    font-size: 14px;
    line-height: 1.6;
  }

  a {
    color: #2563eb;
    text-decoration: none;
    border-bottom: 1px solid #93c5fd;
    transition: all 0.2s ease;

    &:hover {
      color: #1d4ed8;
      border-bottom-color: #2563eb;
    }
  }

  blockquote {
    border-left: 4px solid #e2e8f0;
    padding-left: 20px;
    margin: 24px 0;
    color: #64748b;
    font-style: italic;
  }
`;

export default function Post() {
  const { slug } = useParams();
  const key = Object.keys(mdModules).find(k => k.endsWith(`${slug}.md`));
  const raw = key ? mdModules[key] : null;

  if (!raw) return <p style={{ color: "#64748b" }}>Post not found.</p>;

  const { data, body } = parseFrontMatter(raw);

  // Parse keywords (comma-separated string to array)
  const keywords = data.keywords
    ? data.keywords
        .split(",")
        .map(k => k.trim())
        .filter(Boolean)
    : [];

  return (
    <PostBody>
      <BackBtn to="/documents" aria-label="Back to all documents">
        Back to Documents
      </BackBtn>

      <PostHeader>
        <PostTitle>{data.title || slug}</PostTitle>

        {data.summary && <PostSummary>{data.summary}</PostSummary>}

        <MetaBar>
          {data.date && <MetaBadge>📅 {data.date}</MetaBadge>}
          {data.writer && <MetaBadge>✍️ {data.writer}</MetaBadge>}
          {keywords.map((keyword, idx) => (
            <KeywordTag key={idx}>{keyword}</KeywordTag>
          ))}
        </MetaBar>
      </PostHeader>

      <ContentWrapper>
        <ReactMarkdown
          remarkPlugins={[remarkGfm]}
          components={{
            img({ node, ...props }) {
              return (
                <img
                  {...props}
                  loading="lazy"
                  decoding="async"
                  sizes="(max-width: 768px) 100vw, 720px"
                  alt={props.alt ?? ""}
                />
              );
            },
            a({ node, ...props }) {
              const href = props.href || "";
              const external = /^https?:\/\//i.test(href);
              return (
                <a
                  {...props}
                  target={external ? "_blank" : undefined}
                  rel={external ? "noopener noreferrer" : undefined}
                />
              );
            },
            table({ node, ...props }) {
              return (
                <div className="table-wrap">
                  <table {...props} />
                </div>
              );
            },
          }}
        >
          {body}
        </ReactMarkdown>
      </ContentWrapper>
    </PostBody>
  );
}
