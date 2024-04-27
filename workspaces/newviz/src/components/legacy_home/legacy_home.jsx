import React, { useEffect, useState } from 'react'
import { Typography } from '@mui/material'
import { Button } from "@mui/material";
import { Joystick } from 'react-joystick-component';
import Grid from '@mui/material/Grid';
import { useTheme } from '@mui/material/styles';


const legacy_home = () => {
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
      const handleButtonClick = (command) => {
        console.log(`Executing command: ${command}`);
      };
      const theme = useTheme();
      const buttonStyle = {
        height: '10vh', 
        width: '10vh', 
        margin: theme.spacing(1), // Optional: add some space around the buttons
        transition: theme.transitions.create(['background-color', 'transform'], {
          duration: theme.transitions.duration.standard,
        }),
      };
    return (

      <Grid container spacing={2} style={{ padding: theme.spacing(2) }}>
      <Grid item xs={12} style={{ textAlign: 'center', marginBottom: theme.spacing(2) }}> {/* Adjust bottom margin for joystick */}
      <Joystick size={100} sticky={false} baseColor="#aabbcc" stickColor="#ffcc66" move={handleMove } end={handleEnd} />
      </Grid>
      <Grid item container xs={12} justifyContent="space-around" spacing={2} style={{ gap: `${theme.spacing(4)} 0` }}> {/* This line adjusts the vertical gap */}
        {['Forward', 'Backward', 'Left', 'Right', 'Start', 'Stop', 'Custom'].map((btn) => (
          <Grid item key={btn}>
            <Button
              variant="contained"
              color="primary"
              onClick={() => handleButtonClick(btn.toLowerCase())}
              style={buttonStyle}
            >
              {btn}
            </Button>
          </Grid>
        ))}
      </Grid>
    </Grid>
    );
  };
  
  export default legacy_home;