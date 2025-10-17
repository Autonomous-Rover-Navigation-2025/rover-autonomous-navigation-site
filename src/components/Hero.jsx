import { useEffect, useState, useRef } from "react";
import styled, { css, keyframes } from "styled-components";
import { Link } from "react-router-dom";

/* ===== HERO: gradient bg + non-cropped bg image (bottom) + white text ===== */
const Hero = styled.section`
  position: relative;
  width: 100vw; /* full-bleed */
  left: 50%;
  right: 50%;
  margin-left: -50vw;
  margin-right: -50vw;

  background: linear-gradient(180deg, #eef6ff 0%, #f6f7fb 45%, #edf1f5 100%);

  /* slightly less top padding so text sits higher */
  --topPad: 96px; /* tweak this (e.g., 88px or 104px) */
  --bottomPad: 96px;
  min-height: 72vh;
  padding: var(--topPad) 16px var(--bottomPad);

  overflow: hidden;
`;

/* Background image layer (no cropping) with parallax, pinned to bottom */
const BgImg = styled.div`
  position: absolute;
  inset: 0;
  pointer-events: none;

  background-image: linear-gradient(
      180deg,
      rgba(2, 6, 23, 0.55) 0%,
      rgba(2, 6, 23, 0.28) 45%,
      rgba(2, 6, 23, 0.08) 80%,
      rgba(2, 6, 23, 0) 100%
    ),
    url("/hero/rover-hero.png");

  /* gradient stretches; image height is a percentage so you can size it */
  --imgHeight: 50%; /* make smaller by lowering this (e.g., 45%) */
  background-size: 100% 100%, auto var(--imgHeight);
  background-position: center, bottom center;
  background-repeat: no-repeat, no-repeat;

  will-change: transform;
  transform: translateY(calc(var(--scroll, 0) * -8px));
  transition: transform 50ms linear;

  @media (max-width: 640px) {
    --imgHeight: 48%;
  }
`;

const Frame = styled.div`
  position: relative;
  max-width: 1100px;
  margin: 0 auto;
  height: 100%;
  display: grid;
  gap: 24px;
  justify-items: center;
  align-content: start; /* ⬅️ text sits higher in the hero */
`;

/* ---------- animations ---------- */
const fadeUp = keyframes`
  from { opacity: 0; transform: translateY(12px); }
  to   { opacity: 1; transform: translateY(0); }
`;

const TextBlock = styled.div`
  text-align: center;
  color: #ffffff;
  max-width: 920px;

  /* quick mount fade + tiny scroll nudge */
  opacity: 0;
  transform: translateY(calc(10px - var(--scroll, 0) * 6px));
  transition: opacity 260ms ease, transform 260ms ease;

  ${({ $ready }) =>
    $ready &&
    css`
      opacity: 1;
      transform: translateY(calc(0px - var(--scroll, 0) * 6px));
    `}

  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.25);
`;

const Title = styled.h1`
  margin: 0 0 10px;
  font-weight: 900;
  letter-spacing: -0.02em;
  font-size: clamp(32px, 6vw, 64px);
  line-height: 1.05;

  ${({ $ready }) =>
    $ready &&
    css`
      animation: ${fadeUp} 1s ease 100ms both;
    `}
`;

const Desc = styled.p`
  margin: 0 auto 18px;
  color: rgba(255, 255, 255, 0.92);
  font-size: clamp(15px, 1.7vw, 20px);
  line-height: 1.65;

  ${({ $ready }) =>
    $ready &&
    css`
      animation: ${fadeUp} 520ms ease 120ms both;
    `}
`;

const CTAs = styled.div`
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
  justify-content: center;

  ${({ $ready }) =>
    $ready &&
    css`
      animation: ${fadeUp} 520ms ease 200ms both;
    `}
`;

const CTA = styled(Link)`
  padding: 10px 16px;
  border-radius: 14px;
  font-weight: 800;
  font-size: 15px;

  color: #ffffff; /* white text */
  background: rgba(255, 255, 255, 0.3);
  border: 1.5px solid rgba(255, 255, 255, 0.65);
  backdrop-filter: saturate(140%) blur(2px);

  transition: background 160ms ease, border-color 160ms ease,
    transform 120ms ease;
  &:hover {
    background: rgba(255, 255, 255, 0.5);
    border-color: rgba(255, 255, 255, 0.8);
  }
  &:active {
    transform: translateY(1px);
  }
  &:focus-visible {
    outline: 2px solid rgba(255, 255, 255, 0.9);
    outline-offset: 2px;
  }
`;

const CTAext = styled.a`
  padding: 10px 16px;
  border-radius: 14px;
  font-weight: 800;
  font-size: 15px;
  text-decoration: none;

  color: #ffffff;
  background: rgba(255, 255, 255, 0.3);
  border: 1.5px solid rgba(255, 255, 255, 0.65);
  backdrop-filter: saturate(140%) blur(2px);

  transition: background 160ms ease, border-color 160ms ease,
    transform 120ms ease;
  &:hover {
    background: rgba(255, 255, 255, 0.5);
    border-color: rgba(255, 255, 255, 0.8);
  }
  &:active {
    transform: translateY(1px);
  }
  &:focus-visible {
    outline: 2px solid rgba(255, 255, 255, 0.9);
    outline-offset: 2px;
  }
`;

export function HeroAnimated() {
  const [ready, setReady] = useState(false);
  const ref = useRef(null);

  useEffect(() => {
    const t = requestAnimationFrame(() => setReady(true));
    return () => cancelAnimationFrame(t);
  }, []);

  useEffect(() => {
    const root = ref.current;
    if (!root) return;
    const onScroll = () => {
      const y = Math.min(1, Math.max(0, (window.scrollY || 0) / 200));
      root.style.setProperty("--scroll", y.toFixed(3));
    };
    onScroll();
    window.addEventListener("scroll", onScroll, { passive: true });
    window.addEventListener("resize", onScroll);
    return () => {
      window.removeEventListener("scroll", onScroll);
      window.removeEventListener("resize", onScroll);
    };
  }, []);

  return (
    <Hero ref={ref}>
      <BgImg />
      <Frame>
        <TextBlock $ready={ready}>
          <Title $ready={ready}>Rover Autonomous Navigation</Title>
          <Desc $ready={ready}>
            Perceives in <strong>2D</strong> and <strong>3D</strong> with LiDAR
            and a depth camera, enabling reliable localization, safe path
            planning, and obstacle avoidance—with hands-free{" "}
            <strong>LLM voice interaction</strong>.
          </Desc>
          <CTAs $ready={ready}>
            <CTA to="/documents">Documents</CTA>
            <CTAext
              href="https://github.com/Autonomous-Rover-Navigation-2025"
              target="_blank"
              rel="noopener noreferrer"
            >
              GitHub
            </CTAext>
          </CTAs>
        </TextBlock>
      </Frame>
    </Hero>
  );
}
