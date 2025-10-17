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
  line-height: 1.7;
  color: #0f172a;

  h1,
  h2,
  h3 {
    line-height: 1.25;
    margin: 24px 0 12px;
    font-weight: 800;
    letter-spacing: -0.01em;
  }
  h1 {
    font-size: clamp(24px, 4.2vw, 32px);
  }
  h2 {
    font-size: clamp(20px, 3.6vw, 24px);
  }
  h3 {
    font-size: clamp(18px, 3.2vw, 20px);
  }

  p {
    margin: 12px 0;
  }

  /* Images: responsive, centered, not huge */
  img {
    display: block;
    width: min(100%, 720px);
    height: auto;
    margin: 16px auto;
    border-radius: 12px;
  }

  ul,
  ol {
    margin: 12px 0 12px 22px;
  }

  table {
    border-collapse: collapse;
    width: 100%;
    overflow: hidden;
    border-radius: 8px;
  }
  thead {
    background: #f1f5f9;
  }
  th,
  td {
    border: 1px solid #e2e8f0;
    padding: 8px 10px;
    text-align: left;
  }
  .table-wrap {
    overflow-x: auto;
    margin: 12px 0;
  }

  code {
    background: #f1f5f9;
    border: 1px solid #e2e8f0;
    border-radius: 6px;
    padding: 2px 6px;
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    font-size: 0.95em;
  }
  pre code {
    display: block;
    padding: 12px;
    white-space: pre-wrap;
  }
`;

const TopRow = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 12px;
  margin-bottom: 8px;
`;

const BackBtn = styled(Link)`
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 6px 10px;
  border-radius: 10px;
  font-weight: 600;
  font-size: 14px;
  color: #0f172a;
  text-decoration: none;
  border: 1px solid #e2e8f0;
  background: #fff;
  &:hover {
    background: #f8fafc;
  }
  &:before {
    content: "←";
    font-size: 16px;
    line-height: 1;
  }
`;

const DateText = styled.div`
  font-size: 12px;
  color: #64748b;
`;

export default function Post() {
  const { slug } = useParams();
  const key = Object.keys(mdModules).find(k => k.endsWith(`${slug}.md`));
  const raw = key ? mdModules[key] : null;

  if (!raw) return <p style={{ color: "#64748b" }}>Post not found.</p>;

  const { data, body } = parseFrontMatter(raw);

  return (
    <PostBody>
      <TopRow>
        <BackBtn to="/documents" aria-label="Back to all documents">
          Back
        </BackBtn>
        {data.date && <DateText>{data.date}</DateText>}
      </TopRow>

      <h1 style={{ marginTop: 0 }}>{data.title || slug}</h1>

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
    </PostBody>
  );
}
