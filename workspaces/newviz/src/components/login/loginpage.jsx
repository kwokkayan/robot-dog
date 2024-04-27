import React from 'react';
import SignIn from './signin';
import SignUp from './signup';
import './logincss.css';
import {Grid, Box, Typography, Button} from '@mui/material'

const LoginPage = () => {
    const winwidth = window.innerWidth;

    return (
        <Grid container spacing={2} sx={{width: window.innerWidth}}>
            <Grid item xs={0} md={3}></Grid>
                <Grid item xs={12} md={3} sx={{padding: '30px'}}>
                    <Typography variant='h4'>Sign In</Typography>
                    <SignIn />
                </Grid>
                <Grid  item xs={12} md={3} className='section'>
                <Typography variant='h4'>Sign Up</Typography>
                    <SignUp />
                </Grid>
                <Grid item xs={0} md={3}></Grid>
        </Grid>
    );
};

export default LoginPage;