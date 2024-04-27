import React, {useState, useEffect} from 'react'
import './App.css'
import { Outlet, useLocation } from "react-router-dom"
import NavBar from './components/navbar/NavBar'
import LoginPage from './components/login/loginpage'
import Home from './components/home/Home'
import LivePage from './components/live/LivePage'
import SimulatorPage from './components/simulator/SimulatorPage'
// import Notifications from './components/notification/Notification'
import { auth } from './firebase'

function App() {

  const [isLoggedIn, setIsLoggedIn] = useState(false);
  useEffect(() => {
    const unsubscribe = auth.onAuthStateChanged((user) => {
      setIsLoggedIn(!!user);
    });

    return () => unsubscribe(); 
  }, []);


  const [showHome, setShowHome] = useState(false);
  const goHome = () => {
    setShowHome(true);
    setShowSimulator(false);
    setShowLive(false);
  };
  const [showSimulator, setShowSimulator] = useState(false);
  const goSimulator = () => {
      setShowSimulator(true);
      setShowLive(false);
      setShowHome(false);
  };

  const [showLive, setShowLive] = useState(false);
  const goLive = () => {
      setShowLive(true);
      setShowSimulator(false);
      setShowHome(false);
  };
  return (
    // <div>

    <div>
    {isLoggedIn && <NavBar goHome={goHome} goLive={goLive} goSimulator={goSimulator}/>}
    {isLoggedIn ? (
      <>
        {showHome && <Home />}
        {!showSimulator && !showLive && <Home />}
        {showSimulator && <SimulatorPage />}
        {showLive && <LivePage /> }
      </>
    ) : <LoginPage />
    }
  </div>
  )
}

export default App;
