import React, { useEffect, useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'
import './TestButton.jsx'
import { RosContext } from './contexts/RosContext.js'
import ROSLIB from 'roslib'
import TestButton from './TestButton/'
import { Joystick } from 'react-joystick-component'
import { Link } from "react-router-dom"
import { Typography } from '@mui/material'
import NavBar from './components/navbar/NavBar'
import Visual from './components/visual/Visual'
import { Button } from "@mui/material";
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import Map from './components/map/Map'
import Home from './components/home/Home'
import Live from './components/live/Live'


// function MyJoystick() {

  
  
//   return (
//     <div>
//       <Joystick size={100} sticky={true} baseColor="#ffa500" stickColor="#ffcc66" />
//     </div>
//   );
// }
function App() {


  const [count, setCount] = useState(0)
  const [ msg, setMsg ] = useState("");
  const [robot, setRobot] = useState({"joints": [1, 2, 3, 4]})
  const [rosInstance, setRosInstance] = useState(undefined);
  // const [color, setColor] = useState("#05abcd");
  // const [href, setHref] = useState("/Joystick");
  // const [label, setLabel] = useState("link button");

  
  const handleMove = (event) => {
    //ref: https://www.npmjs.com/package/react-joystick-component
    //event: type, x, y, direction, distance
    console.log(event);
    const x = event.x;
    const y = event.y;
  };

  const handleEnd = () => {
    // console.log("joystick stopped");
  };



  useEffect(() => {
    if (count == 5) {
      setMsg("Please stop.");
    }
  }, [count]);
  
  useEffect(() => {
    console.log(robot);
  }, [robot]);

  useEffect(() => {
    const ros = new ROSLIB.Ros();
    ros.connect('ws://localhost:9090');

    ros.on('connection', () => {
      setRosInstance(ros); // we connected!      
      console.log('Connected to ROS');
      ros.getNodes((nodes) => console.log(nodes));
      
      ros.getTopics((topics) => console.log(topics));
      
      ros.getMessageDetails((msg) => console.log(msg));
      // var listener = new ROSLIB.Topic({
      //   ros : ros,
      //   name : '/joint_states',
      //   messageType : 'sensor_msgs/JointState'
      // });
      // listener.subscribe(function(message) {
      //   for (const i in message.position) {
      //     console.log(`${message.name[i]} ${message.position[i]}`);
      //   }
      //   listener.unsubscribe();
      // });
      

    });

    ros.on('error', (error) => {
      console.error('ROS error:', error);
    });

    ros.on('close', () => {
      setRosInstance(undefined);
      console.log('Disconnected from ROS');
    });

    
  }, []);

  return (
    <RosContext.Provider value={rosInstance}>
      <BrowserRouter>
      <NavBar />
      <Routes>
        <Route path="/" element={<Home />} />
        <Route path="/visual" element={<Visual label="simulator"/>} />
        <Route path="/map" element={<Map />} />
        <Route path="/live" element={<Live />} />
      </Routes>
        {/* <Joystick size={100} sticky={false} baseColor="#ffa500" stickColor="#ffcc66" move={handleMove } end={handleEnd} ></Joystick> */}
      </BrowserRouter>
    </RosContext.Provider>
    
  )
}

export default App
