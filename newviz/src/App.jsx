import React from 'react'
import './App.css'
import { Outlet } from "react-router-dom"
import NavBar from './components/navbar/NavBar'
function App() {

  return (
    <div>
      <NavBar />
      <Outlet />
    </div>
  )
}

export default App;
