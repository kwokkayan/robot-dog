import Visual from "../visual/Visual";
import { Box } from "@mui/material";

function Simulator() {

  return (
    <Box sx={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        width: '90vw', 
        height: '100vh'
      }}><Visual /></Box>
  );
}
export default Simulator;
