import React from 'react'
import './App.css'
import { Outlet, useLocation } from "react-router-dom"
import NavBar from './components/navbar/NavBar'
import loginpage from './components/login/loginpage'

function App() {

  const isLoginPage = useLocation().pathname === '/'
  
  return (
    <div>
      {!isLoginPage && <NavBar />}
      <Outlet />
    </div>
  )
}

export default App;
