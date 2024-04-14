import React, { useEffect, useState, useRef } from 'react'
import { Button } from "@mui/material";
import { Paper, Typography, Box } from '@mui/material';


const Live = () => {
    const videoRef = useRef(null);

    useEffect(() => {
        const getVideo = async () => {
          try {
            const stream = await navigator.mediaDevices.getUserMedia({ video: true });
            if (videoRef.current) {
              videoRef.current.srcObject = stream;
            }
          } catch (err) {
            console.error('Error accessing the webcam', err);
          }
        };    

    getVideo();

    return () => {
        if (videoRef.current && videoRef.current.srcObject) {
          videoRef.current.srcObject.getTracks().forEach(track => track.stop());
        }
      };
    }, []);
  
    return (
        <Box sx={{
          display: 'flex',
          flexDirection: 'row',
          alignItems: 'center',
          justifyContent: 'center',
          height: '100vh', 
        }}>
          <Paper elevation={3} sx={{
            width: window.innerWidth > window.innerHeight? 'calc(55vw - 32px)' : '80vw',
            maxWidth: '100%',
            overflow: 'hidden',
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            padding: '8px',
            flexDirection: 'column',
          }}>
            <video
              ref={videoRef}
              autoPlay
              playsInline
              style={{
                width: '100%', 
                height: 'auto',
                maxWidth: '100%',
                maxHeight: '100%',
                objectFit: 'contain' 
              }}
            ></video>
          </Paper>
        </Box>
      );
  };
  
  export default Live;