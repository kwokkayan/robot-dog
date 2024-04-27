import * as React from 'react';
import { AppBar, Box, Button, Toolbar, IconButton, useMediaQuery, Grid} from '@mui/material';
import { Link } from 'react-router-dom'; 
import useTheme from '@mui/material/styles/useTheme';
import MenuIcon from '@mui/icons-material/Menu';
import LinkButton from '../linkbutton/LinkButton';
import "@fontsource-variable/jura";
import SignOut from '../login/signout';
import LivePage from '../live/LivePage';
import SimulatorPage from '../simulator/SimulatorPage';

export default function ButtonAppBar({goHome, goLive, goSimulator}) {
  const [pagename] = React.useState("Robot Dog");
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('sm'));
  const [menuOpen, setMenuOpen] = React.useState(false);

  const handleMenuClick = () => {
    setMenuOpen(!menuOpen);
  };

  const handleHomeClick = () => {
    goHome();
  };
  const handleLiveClick = () => {
    goLive();
  };

  const handleSimulatorClick = () => {
    goSimulator();
  };

  return (
    <Box sx={{ flexGrow: 1 }}>
      <AppBar position="fixed">
        <Toolbar>
          {isMobile && (
            <Grid container spacing={2} style={{ padding: theme.spacing(2)}}>
              <Grid item xs={2} sx={{display: 'flex', justifyContent: 'center', alignItems: 'center'}}>
            <IconButton
              size="large"
              edge="start"
              color="inherit"
              aria-label="menu"
              sx={{ mr: 2, position: 'absolute', left: '16px' }}
              onClick={handleMenuClick}
            >
              <MenuIcon />
            </IconButton>
            </Grid>
            <Grid item xs={8} sx={{display: 'flex', justifyContent: 'center', alignItems: 'center'}}>
            <Link to="/Home" >
            <img src="web_logo.png" alt="Robot Dog Logo" style={{position: 'absolute', height: '36px', top: '50%', left: '50%', transform: 'translate(-50%, -50%)'}} />
          </Link>
          </Grid>
          <Grid item xs={2}></Grid>
          </Grid>
          )}
          
          {!isMobile && (
            <Grid container spacing={2} style={{ padding: theme.spacing(2) }}>
              <Grid item xs={2}></Grid>
            <Grid item xs={8} sx={{ flexGrow: 1, display: 'flex', justifyContent: 'center', padding: 0, margin: 0}}>
            {/* <Link to="/Home" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
              <img src="web_logo.png" alt="Robot Dog Logo" style={{ height: '50px' }} />
            </Link> */}
            <IconButton color="inherit" onClick={handleHomeClick} style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
              <img src="web_logo.png" alt="Robot Dog Logo" style={{ height: '50px' }} />
            </IconButton>
          </Grid>
          <Grid item xs={2} sx={{display: 'flex', justifyContent: 'flex-end'}}>
            {/* <LinkButton color={"inherit"} href={"/simulator"} label={"Simulator"} sx={{}}></LinkButton> */}
            {/* <LinkButton color={"inherit"} href={"/live"} label={"Live"} sx={{}}></LinkButton> */}
            <Button color="inherit" onClick={handleSimulatorClick}>Simulator</Button>
            <Button color="inherit" onClick={handleLiveClick}>Live</Button>
            <SignOut/>
          </Grid>
          </Grid>
        )}
          {/* Use empty Box to balance the Toolbar layout */}
          {isMobile ? null : <Box sx={{ flexGrow: 1 }}></Box>}
        </Toolbar>
        {menuOpen && isMobile && (
          <Box sx={{display: 'flex', flexDirection: 'column'}}>
          <Box sx={{ display: 'flex', flexDirection: 'column', pb: 2 }}>
            {/* <LinkButton color={"inherit"} href={"/simulator"} label={"Simulator"} onClick={() => setMenuOpen(false)} />
            <LinkButton color={"inherit"} href={"/live"} label={"Live"} onClick={() => setMenuOpen(false)} /> */}
            <Button color="inherit" onClick={handleSimulatorClick}>Simulator</Button>
            <Button color="inherit" onClick={handleLiveClick}>Live</Button>
            <SignOut/>
          </Box>
          </Box>
        )}
      </AppBar>
    </Box>
  );
}
