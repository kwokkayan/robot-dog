import React from 'react';
import SignIn from './signin';
import SignUp from './signup';

const LoginPage = () => {
    const winwidth = window.innerWidth;

    return (
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', width: winwidth }}>
            <div style={{ flex: 1 }}>
                <h2>Sign In</h2>
                <SignIn />
            </div>
            <div style={{ flex: 1 }}>
                <h2>Sign Up</h2>
                <SignUp />
            </div>
        </div>
    );
};

export default LoginPage;