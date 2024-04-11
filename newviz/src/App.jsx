import React from 'react'
import './App.css'
import { Outlet } from "react-router-dom"
import NavBar from './components/navbar/NavBar'

function App() {

  return (
    <>
      <NavBar />
      <div style={{ marginTop: '82px' }}>
        <Outlet />
      </div>
    </>
  )
}

export default App
