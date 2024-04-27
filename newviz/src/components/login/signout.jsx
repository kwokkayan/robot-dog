import React from 'react';
import { getAuth, signOut } from 'firebase/auth';
import { auth } from '../../firebase'; 
import { useNavigate } from 'react-router-dom';

const SignOut = () => {
  const navigate = useNavigate();

  const handleSignOut = () => {
    signOut(auth)
      .then(() => {
        // Sign-out successful.
        navigate('/');
        console.log('User signed out successfully.');
      })
      .catch((error) => {
        // An error happened.
        console.log('Sign out error:', error);
      });
  };

  return (
    <button onClick={handleSignOut}>Sign Out</button>
  );
};

export default SignOut;