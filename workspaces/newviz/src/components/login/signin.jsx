import React, { useState } from 'react';
import { getAuth, signInWithEmailAndPassword, signInWithPopup, GoogleAuthProvider  } from 'firebase/auth';
import { auth } from '../../firebase'; 
import { useNavigate } from 'react-router-dom';
import './logincss.css';
import {Paper, Box, Typography, Button, Input} from '@mui/material';

const SignIn = ({SignInSuccess}) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [msg, setMsg] = useState('');
  const navigate = useNavigate();


  const provider = new GoogleAuthProvider();

  const handleSignIn = () => {
    const auth = getAuth();
    signInWithEmailAndPassword(auth, email, password)
      .then((userCredential) => {
          SignInSuccess();
          // navigate('/Home');
      })
      .catch((error) => {
        setMsg(error.message);
      });
  };

  const signInWithGoogle = () => {
    const auth = getAuth();
    const provider = new GoogleAuthProvider();
    signInWithPopup(auth, provider)
      .then((result) => {
        const credential = GoogleAuthProvider.credentialFromResult(result);
        const token = credential.accessToken;
        const user = result.user;
        navigate('/Home');
      })
      .catch((error) => {
        const errorCode = error.code;
        const errorMessage = error.message;
        const email = error.customData.email;
        const credential = GoogleAuthProvider.credentialFromError(error);
    
        console.error('Google sign-in error:', error);
      });
  };

  return (
    <Box className='block' sx={{borderColor: 'primary.dark', borderWidth: '2px', borderRadius: '8px', borderStyle: 'solid'}}>
      <Input className='inputfield'
        type="email"
        placeholder="Email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}
        style={{padding: '5px'}}/>

      <Input className='inputfield'
        type="password"
        placeholder="Password"
        value={password}
        onChange={(e) => setPassword(e.target.value)}/>

      <Button className='button' onClick={handleSignIn} color='primary' variant='contained' sx={{marginTop: '10px'}}>Sign In</Button>
      <Typography>--OR--</Typography>
      <Button variant="contained" onClick={signInWithGoogle}>Sign in with Google</Button>
      {msg && <p>{msg}</p>}
    </Box>
  );
};

export default SignIn;