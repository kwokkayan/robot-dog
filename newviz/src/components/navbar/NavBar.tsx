import {
    AppBar,
    Badge,
    Box,
    IconButton,
    Toolbar,
    Typography,
  } from "@mui/material";
  import LinkButton from "../linkbutton";
  import { RosContext } from "../../contexts/RosContext";
  import { useContext, useEffect, useState } from "react";
  import { useNavigate } from "react-router-dom";
  import CSS from 'csstype';
  
  function NavBar() {

    const navigate = useNavigate();
  
    const logoStyle: CSS.Properties = {    
      height: "64px",
      width: "64px",
      mixBlendMode: "multiply",
    }
  
    return (
      <AppBar color={"primary"}>
        <Toolbar>
          <img src="/logo.jpg" alt="logo" style={logoStyle}/>
          <Box sx={{ flexGrow: 1, ml: 3 }}>
            <LinkButton color="inherit" href="/" label="Home" />
          </Box>
          <Box sx={{ display: "flex" }}>
          </Box>
        </Toolbar>
      </AppBar>
    );
  }
  export default NavBar;
  