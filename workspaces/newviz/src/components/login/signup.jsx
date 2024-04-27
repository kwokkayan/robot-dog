import React, { useState } from 'react';
import { getAuth, createUserWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../../firebase';
import './logincss.css';
import {Paper, Box, Typography, Button, Input} from '@mui/material';

const SignUp = () => {
    console.log("signup");
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [msg, setMsg] = useState('');

  const handleSignUp = () => {
    createUserWithEmailAndPassword(auth, email, password)
      .then((userCredential) => {
        const signupemail = userCredential.user.email;
        setMsg('Signed up successfully, '+signupemail+'!');
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

      <Button className='button' onClick={handleSignUp} color='primary' variant='contained' sx={{marginTop: '10px'}}>Sign Up</Button>

      {msg && <p>{msg}</p>}

      </Box>
  );
};

export default SignUp;