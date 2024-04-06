import React, { useEffect, useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'
import './TestButton.jsx'
import { RosContext } from './contexts/RosContext.js'
import ROSLIB from 'roslib';
import TestButton from './TestButton/'
import { Joystick } from 'react-joystick-component'
import { Link } from "react-router-dom";
import RobotVisual from "./RobotVisual";

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
      <div>
        <a href="/Joystick" target="_blank">
          <button>Use Joystick!</button>
        </a>
        {/* <button id="joystickButton" onClick={console.log("joystickButton clicked")}>Use Joytick </button> */}
      </div>
      <div>
        <a href="https://vitejs.dev" target="_blank">
          <img src={viteLogo} className="logo" alt="Vite logo" />
        </a>
        <a href="https://react.dev" target="_blank">
          <img src={reactLogo} className="logo react" alt="React logo" />
        </a>
      </div>
      <h1>Vite + React</h1>
      <div className="card">
        <button onClick={() => setCount((count) => count + 1)}>
          count is {count}!
        </button>
        <button onClick={() => setRobot({"joints": robot.joints.concat([count])})}>
          add count to robot joint
        </button>
        <TestButton count={count}></TestButton>
        <Joystick size={100} sticky={false} baseColor="#ffa500" stickColor="#ffcc66" move={handleMove } end={handleEnd} ></Joystick>
        <p>{msg}</p>
        <p>
          Edit <code>src/App.jsx</code> and save to test HMR
        </p>
      </div>
      <p className="read-the-docs">
        Click on the Vite and React logos to learn more
      </p>

      <h1>My robot</h1>
      {/* <RobotModel/> */}

      <RobotVisual />
      {/* <script href=".../urdf-viewer-element.js"></script>
      <script>customElements.define('urdf-viewer', URDFViewer)</script>

      <body>
        <urdf-viewer package=".../package/dir/" urdf="T12/urdf/T12.URDF" up="Z+" display-shadow ambient-color="red"></urdf-viewer>
      </body> */}
    </RosContext.Provider>
  )
}

export default App
