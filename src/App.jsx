import RoutesView from "./routes";
import Nav from "./components/Nav";
import Footer from "./components/Footer";
import styled from "styled-components";

const Main = styled.main`
  flex: 1;
  padding: 0 16px;
`;

const Container = styled.div`
  max-width: 980px;
  margin: 0 auto;
  padding: 0 0 40px 0;
`;

const Shell = styled.div`
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  background: #fff;
  color: #0f172a;
`;

export default function App() {
  return (
    <Shell>
      <Nav />
      <Main>
        <Container>
          <RoutesView />
        </Container>
      </Main>
      <Footer />
    </Shell>
  );
}
