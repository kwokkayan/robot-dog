import React, { useState } from 'react';
import { getAuth, createUserWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../../firebase';
import './logincss.css';

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

      <button className='button' onClick={handleSignUp}>Sign Up</button>

      {msg && <p>{msg}</p>}

    </div>
  );
};

export default SignUp;