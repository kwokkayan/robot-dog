import Visual from "../visual/Visual";
import { Container, Box } from "@mui/material";

function Simulator() {

  return (
    <Container maxWidth="lg" sx={{ height: 'calc(80vh - 82px)', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
      <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        <Visual />
      </Box>
    </Container>
  );
}
export default Simulator;
