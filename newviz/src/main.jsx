import React, { Suspense } from 'react'
import ReactDOM from 'react-dom'
import App from './App.jsx'
import './index.css'
import { createTheme } from "@mui/material";
import { ThemeProvider } from "@emotion/react";
import Home from './components/home/Home';
// import Live from './components/live/Live';
import LivePage from './components/live/LivePage';
import { RouterProvider, createBrowserRouter } from 'react-router-dom';
import { RosContextProvider } from './contexts/RosContextProvider.jsx';
// import Simulator from './components/simulator/Simulator';
import SimulatorPage from './components/simulator/SimulatorPage';
import SignUp from './components/login/signup';
import SignIn from './components/login/signin';
import LoginPage from './components/login/loginpage';

const theme = createTheme({
  palette: {
    primary: {
      main: "#524FA0",
      light: "#8080ff",
      dark: "26264c",
      contrastText: "#FFFFFF",
    },
    secondary: {
      main: "#b62467",
      light: "e589b7",
      dark: "#4c0f2e",
      contrastText: "#fff"
    },
    warning: {
      main: "#f05a22",
      contrastText: "#fff"
    },
    success: {
      main: "#c9da2a"
    }
  },
  transitions: {
    duration: {
      shortest: 150,
      shorter: 200,
      short: 250,
      standard: 300,
      complex: 375,
      enteringScreen: 225,
      leavingScreen: 195,
    },
  },
});

const router = createBrowserRouter([
  {
    path: "/",
    element: <App />,
    
  },
]);

ReactDOM.render(
  <React.StrictMode>
    <RosContextProvider>
      <ThemeProvider theme={theme}>
        <Suspense fallback={<div style={{display: 'flex', justifyContent: 'center', alignItems: 'center', width: window.innerWidth}}>Loading...</div>}>
         <RouterProvider router={router} />
         </Suspense>
      </ThemeProvider>
    </RosContextProvider>
  </React.StrictMode>,
  document.getElementById('root')
);

// ReactDOM.createRoot(document.getElementById('root')).render(
//   <React.StrictMode>
//     <RosContextProvider>
//       <ThemeProvider theme={theme}>
//         <RouterProvider router={router} />
//       </ThemeProvider>
//     </RosContextProvider>
//   </React.StrictMode>
// )
