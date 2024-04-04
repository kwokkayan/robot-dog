import React, { useEffect, useState } from 'react'
import { Typography } from '@mui/material'
import { Button } from "@mui/material";
import { Joystick } from 'react-joystick-component';


const Home = () => {
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
    return (
        <Joystick size={100} sticky={false} baseColor="#aabbcc" stickColor="#ffcc66" move={handleMove } end={handleEnd} ></Joystick>
    );
  };
  
  export default Home;