import * as React from 'react';
import { AppBar, Box, Toolbar, IconButton, useMediaQuery, Grid} from '@mui/material';
import { Link } from 'react-router-dom'; 
import useTheme from '@mui/material/styles/useTheme';
import MenuIcon from '@mui/icons-material/Menu';
import LinkButton from '../linkbutton/LinkButton';
import "@fontsource-variable/jura";

export default function ButtonAppBar() {
  const [pagename] = React.useState("Robot Dog");
  const theme = useTheme();
  const isMobile = useMediaQuery(theme.breakpoints.down('sm'));
  const [menuOpen, setMenuOpen] = React.useState(false);

  const handleMenuClick = () => {
    setMenuOpen(!menuOpen);
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
            <Link to="/" >
            <img src="web_logo.png" alt="Robot Dog Logo" style={{position: 'absolute', height: '36px', top: '50%', left: '50%', transform: 'translate(-50%, -50%)'}} />
          </Link>
          </Grid>
          <Grid item xs={2}></Grid>
          </Grid>
          )}
          
          {!isMobile && (
            <Grid container spacing={2} style={{ padding: theme.spacing(2) }}>
              <Grid item xs={3}></Grid>
            <Grid item xs={6} sx={{ flexGrow: 1, display: 'flex', justifyContent: 'center', padding: 0, margin: 0}}>
            <Link to="/" style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
              <img src="web_logo.png" alt="Robot Dog Logo" style={{ height: '50px' }} />
            </Link>
          </Grid>
          <Grid item xs={3} sx={{display: 'flex', justifyContent: 'flex-end'}}>
            <LinkButton color={"inherit"} href={"/visual"} label={"Simulator"} sx={{}}></LinkButton>
            <LinkButton color={"inherit"} href={"/map"} label={"Map"} sx={{}}></LinkButton>
            <LinkButton color={"inherit"} href={"/live"} label={"Live"} sx={{}}></LinkButton>
          </Grid>
          </Grid>
        )}
          {/* Use empty Box to balance the Toolbar layout */}
          {isMobile ? null : <Box sx={{ flexGrow: 1 }}></Box>}
        </Toolbar>
        {menuOpen && isMobile && (
          <Box sx={{display: 'flex', flexDirection: 'column'}}>
          <Box sx={{ display: 'flex', flexDirection: 'column', pb: 2 }}>
            <LinkButton color={"inherit"} href={"/visual"} label={"Simulator"} onClick={() => setMenuOpen(false)} />
            <LinkButton color={"inherit"} href={"/map"} label={"Map"} onClick={() => setMenuOpen(false)} />
            <LinkButton color={"inherit"} href={"/live"} label={"Live"} onClick={() => setMenuOpen(false)} />
          </Box>
          </Box>
        )}
      </AppBar>
    </Box>
  );
}
