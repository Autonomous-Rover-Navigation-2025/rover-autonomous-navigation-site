import styled from 'styled-components'

const Foot = styled.footer`
  border-top: 1px solid #e2e8f0; background: #fff;
`;
const Inner = styled.div`
  max-width: 980px; margin: 0 auto; padding: 24px 16px; font-size: 12px; color: #64748b;
`;

export default function Footer() {
  return (
    <Foot>
      <Inner>© {new Date().getFullYear()} Rover Autonomous Navigation — All rights reserved.</Inner>
    </Foot>
  )
}
