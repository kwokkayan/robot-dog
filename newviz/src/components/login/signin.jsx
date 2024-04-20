import React, { useState } from 'react';
import { getAuth, signInWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../../firebase'; 
import { useNavigate } from 'react-router-dom';
import './logincss.css';
import {Paper, Box, Typography, Button, Input} from '@mui/material';

const SignIn = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [msg, setMsg] = useState('');
  const navigate = useNavigate();

  const handleSignIn = () => {
    const auth = getAuth();
    signInWithEmailAndPassword(auth, email, password)
      .then((userCredential) => {
          navigate('/Home');
      })
      .catch((error) => {
        setMsg(error.message);
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

      {msg && <p>{msg}</p>}
    </Box>
  );
};

export default SignIn;