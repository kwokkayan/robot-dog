import React, { useState } from 'react';
import SignIn from './signin';
import SignUp from './signup';
import Home from '../home/Home';
import SimulatorPage from '../simulator/SimulatorPage';
import LivePage from '../live/LivePage';
import './logincss.css';
import {Grid, Box, Typography, Button} from '@mui/material'

const LoginPage = () => {
    const winwidth = window.innerWidth;

    const [isLoggedIn, setIsLoggedIn] = useState(false);
    const [showSignUp, setShowSignUp] = useState(true);
    const SignInSuccess = () => {
        setIsLoggedIn(true); 
        setShowSignUp(false);
    };
    
    

    return (
        <Box className="login-page" sx={{ width: winwidth, display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
      {!isLoggedIn ? ( 
            <Box sx={{ margin: '40px' }}>
            <Typography variant="h4">Sign In</Typography>
            <SignIn SignInSuccess={SignInSuccess} />
            </Box>) 
        : <Home /> }

      {showSignUp && ( 
        <Box sx={{ margin: '40px' }} >
          <Typography variant="h4">Sign Up</Typography>
          <SignUp />
        </Box>
      )}
        
    </Box>
    );
};

export default LoginPage;