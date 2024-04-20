import React from 'react';
import SignIn from './signin';
import SignUp from './signup';

const LoginPage = () => {
  return (
    <div>
      <h2>Sign In</h2>
      <SignIn />
      <h2>Sign Up</h2>
      <SignUp />
    </div>
  );
};

export default LoginPage;