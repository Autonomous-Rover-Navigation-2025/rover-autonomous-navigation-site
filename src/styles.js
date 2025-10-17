import { createGlobalStyle } from "styled-components";

export const GlobalStyle = createGlobalStyle`
  :root { --page-max: 980px; }
  * { box-sizing: border-box; }
  html, body, #root { height: 100%; }
  body { margin: 0; font-family: system-ui, -apple-system, Segoe UI, Roboto, Ubuntu, 'Helvetica Neue', Arial, 'Apple Color Emoji', 'Segoe UI Emoji', 'Segoe UI Symbol'; color: #0f172a; background: #fff; }
  a { color: inherit; text-decoration: none; }
  pre, code { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace; }
`;
