import React, { useState } from 'react';
import { getAuth, signInWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../../firebase'; // Assuming firebase.js is in the same directory as SignIn.jsx
import { useNavigate } from 'react-router-dom';
import './logincss.css'; // Import the CSS file

const SignIn = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const navigateTo = useNavigate();

  const handleSignIn = () => {
    const auth = getAuth();
    signInWithEmailAndPassword(auth, email, password)
      .then((userCredential) => {
        // User signed in successfully
        const user = userCredential.user;
        console.log('Signed in:', user);
        setError('');
        navigateTo('/home');
      })
      .catch((error) => {
        // Handle sign-in errors
        const errorCode = error.code;
        const errorMessage = error.message;
        console.error('Sign-in error:', errorCode, errorMessage);
        setError(errorMessage);
      });
  };

  return (
    <div className="container"> {/* Apply the "container" class */}
      <div className="box"> {/* Apply the "box" class */}
        <form className="form"> {/* Apply the "form" class */}
          <label>Email:</label>
          <input
            type="email"
            placeholder="Email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
          />

          <label>Password:</label>
          <input
            type="password"
            placeholder="Password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
          />

          <button className="button" onClick={handleSignIn}>Sign In</button> {/* Apply the "button" class */}
          {error && <p>{error}</p>}
        </form>
      </div>
    </div>
  );
};

export default SignIn;


// import React, { useState } from 'react';
// import { getAuth, signInWithEmailAndPassword } from 'firebase/auth';
// import { auth } from '../../firebase'; // Assuming firebase.js is in the same directory as SignIn.jsx
// import { useNavigate } from 'react-router-dom';

// const SignIn = () => {
//   const [email, setEmail] = useState('');
//   const [password, setPassword] = useState('');
//   const [error, setError] = useState('');
//   const navigateTo = useNavigate();

//   const handleSignIn = () => {
//     const auth = getAuth();
//     signInWithEmailAndPassword(auth, email, password)
//       .then((userCredential) => {
//         // User signed in successfully
//         const user = userCredential.user;
//         console.log('Signed in:', user);
//         setError('');
//         navigateTo('/Home')
//         // history.pushState('/home')
//       })
//       .catch((error) => {
//         // Handle sign-in errors
//         const errorCode = error.code;
//         const errorMessage = error.message;
//         console.error('Sign-in error:', errorCode, errorMessage);
//         setError(errorMessage);
//       });
//   };

//   return (
//     <div>
//       <input
//         type="email"
//         placeholder="Email"
//         value={email}
//         onChange={(e) => setEmail(e.target.value)}
//       />
//       <input
//         type="password"
//         placeholder="Password"
//         value={password}
//         onChange={(e) => setPassword(e.target.value)}
//       />
//       <button className="button" onClick={handleSignIn}>Sign In</button>
//       {error && <p>{error}</p>}
//     </div>
//   );
// };

// export default SignIn;