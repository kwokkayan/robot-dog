// Import the functions you need from the SDKs you need
import { initializeApp } from "firebase/app";
import { getAnalytics } from "firebase/analytics";
import { getAuth } from "firebase/auth";

// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

Your web app's Firebase configuration
For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
  apiKey: import.meta.env.VITE_FIREBASE_API_KEY,
  authDomain: "robotdog-b9063.firebaseapp.com",
  projectId: "robotdog-b9063",
  storageBucket: "robotdog-b9063.appspot.com",
  messagingSenderId: "849893842441",
  appId: "1:849893842441:web:4d3714524d797b14f8a2d8",
  measurementId: "G-NFYEF444TW"
};


// Initialize Firebase
const app = initializeApp(firebaseConfig);
const analytics = getAnalytics(app);
const auth = getAuth(app);

export {auth};