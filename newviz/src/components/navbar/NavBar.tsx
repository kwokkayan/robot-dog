import * as React from 'react';
import AppBar from '@mui/material/AppBar';
import Box from '@mui/material/Box';
import Toolbar from '@mui/material/Toolbar';
import Typography from '@mui/material/Typography';
import Button from '@mui/material/Button';
import IconButton from '@mui/material/IconButton';
import MenuIcon from '@mui/icons-material/Menu';
import LinkButton  from '../linkbutton/LinkButton';
import { Link } from 'react-router-dom';
import "@fontsource-variable/jura";

export default function ButtonAppBar() {
const [pagename, setPagename] = React.useState("Robot Dog");
  return (
    <Box sx={{ flexGrow: 1 }}>
      <AppBar position="fixed">
        <Toolbar>
          <IconButton
            size="large"
            edge="start"
            color="inherit"
            aria-label="menu"
            sx={{ mr: 2 }}
          >
            <MenuIcon />
          </IconButton>
          <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', width: '100vw' }}>
          <Button component={Link} color={"inherit"} to={"/"} sx={{ fontFamily: 'Jura, sans-serif', fontWeight: 700, fontSize: '1.5rem', textAlign: 'center' }}>{pagename}</Button>
          </div>
          <LinkButton color={"inherit"} href={"/visual"} label={"Simulator"}></LinkButton>
          <LinkButton color={"inherit"} href={"/map"} label={"Map"}></LinkButton>
          <LinkButton color={"inherit"} href={"/live"} label={"Live"}></LinkButton>
        </Toolbar>
      </AppBar>
    </Box>
  );
}