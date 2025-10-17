import { Routes, Route } from "react-router-dom";
import Home from "./pages/Home";
import Post from "./pages/Post";
import Documents from "./pages/Documents";

export default function RoutesView() {
  return (
    <Routes>
      <Route path="/" element={<Home />} />
      <Route path="/documents" element={<Documents />} />
      <Route path="/documents/:slug" element={<Post />} />
      <Route path="*" element={<Home />} />
    </Routes>
  );
}
