import { NavLink } from "react-router-dom";
import styled from "styled-components";

const Header = styled.header`
  position: sticky; /* stays at the top on scroll */
  top: 0;
  z-index: 1000;
  border-bottom: 1px solid #e2e8f0;
  background: rgba(255, 255, 255, 0.9);
  backdrop-filter: saturate(140%) blur(6px);
`;

const Bar = styled.div`
  max-width: 980px;
  margin: 0 auto;
  height: 56px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
`;

const Brand = styled(NavLink)`
  display: inline-flex;
  align-items: center;
  gap: 10px;
  font-weight: 800;
  font-size: 16px;
  color: #0f172a;
  text-decoration: none;
`;

const BrandLogo = styled.img`
  width: 40px;
  height: 40px;
  border-radius: 8px; /* the icon already has rounded corners; optional */
  display: block;
`;

const NavLinks = styled.nav`
  display: flex;
  gap: 8px;
`;

const LinkBtn = styled(NavLink)`
  padding: 6px 12px;
  border-radius: 10px;
  font-size: 14px;
  font-weight: 500;
  color: #0f172a;
  &:hover {
    background: #f1f5f9;
  }
`;

const ExternalLinkBtn = styled.a`
  padding: 6px 12px;
  border-radius: 10px;
  font-size: 14px;
  font-weight: 500;
  color: #0f172a;
  text-decoration: none;
  &:hover {
    background: #f1f5f9;
  }
`;

export default function Nav() {
  return (
    <Header>
      <Bar>
        <Brand to="/">
          <BrandLogo src="/favicon.svg" alt="Rover logo" />
        </Brand>
        <NavLinks>
          <LinkBtn to="/">Home</LinkBtn>
          <LinkBtn to="/documents">Documents</LinkBtn>
          <ExternalLinkBtn
            href="https://github.com/Autonomous-Rover-Navigation-2025"
            target="_blank"
            rel="noopener noreferrer"
          >
            GitHub
          </ExternalLinkBtn>
        </NavLinks>
      </Bar>
    </Header>
  );
}
