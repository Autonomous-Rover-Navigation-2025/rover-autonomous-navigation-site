import { Routes, Route } from 'react-router-dom'
import Home from './pages/Home'
import Blog from './pages/Blog'
import Post from './pages/Post'
import Documents from './pages/Documents'

export default function RoutesView() {
  return (
    <Routes>
      <Route path="/" element={<Home />} />
      <Route path="/blog" element={<Blog />} />
      <Route path="/blog/:slug" element={<Post />} />
      <Route path="/documents" element={<Documents />} />
      <Route path="*" element={<Home />} />
    </Routes>
  )
}
