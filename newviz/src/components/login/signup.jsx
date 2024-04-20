import React, { useState } from 'react';
import { getAuth, createUserWithEmailAndPassword } from 'firebase/auth';
import { auth } from '../../firebase'; // Assuming firebase.js is in the same directory as SignUp.jsx
import './logincss.css'; // Import the CSS file

const SignUp = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');

  const handleSignUp = () => {
    createUserWithEmailAndPassword(auth, email, password)
      .then((userCredential) => {
        // User signed up successfully
        const user = userCredential.user;
        console.log('Signed up:', user);
        setError('');
      })
      .catch((error) => {
        // Handle sign-up errors
        const errorCode = error.code;
        const errorMessage = error.message;
        console.error('Sign-up error:', errorCode, errorMessage);
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

          <button className="button" onClick={handleSignUp}>Sign Up</button> {/* Apply the "button" class */}
          {error && <p>{error}</p>}
        </form>
      </div>
    </div>
  );
};

export default SignUp;

// import React, { useState } from 'react';
// import { getAuth, createUserWithEmailAndPassword } from 'firebase/auth';
// import { auth } from '../../firebase'; // Assuming firebase.js is in the same directory as SignUp.jsx

// const SignUp = () => {
//     console.log("signup");
//   const [email, setEmail] = useState('');
//   const [password, setPassword] = useState('');
//   const [error, setError] = useState('');

//   const handleSignUp = () => {
//     createUserWithEmailAndPassword(auth, email, password)
//       .then((userCredential) => {
//         // User signed up successfully
//         const user = userCredential.user;
//         console.log('Signed up:', user);
//         setError('');
//       })
//       .catch((error) => {
//         // Handle sign-up errors
//         const errorCode = error.code;
//         const errorMessage = error.message;
//         console.error('Sign-up error:', errorCode, errorMessage);
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
//       <button onClick={handleSignUp}>Sign Up</button>
//       {error && <p>{error}</p>}
//     </div>
//   );
// };

// export default SignUp;