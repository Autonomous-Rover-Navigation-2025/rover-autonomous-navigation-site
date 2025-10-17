# Rover Autonomous Navigation — Website

A minimal React + Vite site (styled-components) to host your project intro, tech blog, and documents. Designed for serverless/static hosting on **Vercel**.

## Quick Start
```bash
# 1) Install deps
npm i

# 2) Run locally
npm run dev

# 3) Build
npm run build

# 4) Preview the production build
npm run preview
```

## Adding Content
- **Project description**: edit `src/pages/Home.jsx` (look for the italic placeholder).
- **Blog posts**: add Markdown files to `src/content/posts/` with frontmatter:
  ```md
  ---
  title: "Your Title"
  date: "YYYY-MM-DD"
  summary: "One-line summary"
  ---
  Your content here.
  ```
- **Documents**: place files in `src/content/docs/` (PDF, MD, TXT). They’ll show on the Documents page with a download link.

## Styling
- Uses **styled-components** for component-scoped styles and a small `GlobalStyle` in `src/styles.js`.

## Deploy (Vercel)
1. Push to GitHub.
2. Import the repo into Vercel.
3. Framework Preset: **Vite**.
4. Build Command: `npm run build` — Output Directory: `dist`.
5. Click Deploy.

> Optional: Add a `vercel.json` (below) to force SPA rewrites.

## vercel.json (optional)
```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "dist",
  "rewrites": [{ "source": "/(.*)", "destination": "/index.html" }]
}
```
