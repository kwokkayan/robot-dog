import { createContext } from "react";
import React, { useEffect, useState } from 'react'
import ROSLIB from 'roslib'

export const RosContext = createContext();

export const RosContextProvider = (props) => {

  const [rosInstance, setRosInstance] = useState(undefined);

  useEffect(() => {
    if (rosInstance === undefined) {
      const ros = new ROSLIB.Ros();
      ros.connect(import.meta.env.VITE_WS_URL);

      ros.on('connection', () => {
        console.log('Connected to ROS');

        ros.getNodes((nodes) => console.log(nodes));

        ros.getTopics((topics) => console.log(topics));

        ros.getMessageDetails((msg) => console.log(msg));

        setRosInstance(ros);
      });

      ros.on('error', (error) => {
        ros.connect(import.meta.env.VITE_WS_URL);
      });

      ros.on('close', () => {
        setRosInstance(undefined);
      });
    }
  }, [rosInstance]);

  return (
    <RosContext.Provider value={rosInstance}>
      {props.children}
    </RosContext.Provider>
  );
}