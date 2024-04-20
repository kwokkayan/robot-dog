import React, { useState } from 'react';
import { getAuth, signInWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../../firebase'; // Assuming firebase.js is in the same directory as SignIn.jsx
import { useNavigate } from 'react-router-dom';
import './logincss.css';

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
    <div className='block'>
      <input className='inputfield'
        type="email"
        placeholder="Email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}/>

      <input className='inputfield'
        type="password"
        placeholder="Password"
        value={password}
        onChange={(e) => setPassword(e.target.value)}/>

      <button className='button' onClick={handleSignIn}>Sign In</button>

      {msg && <p>{msg}</p>}
    </div>
  );
};

export default SignIn;