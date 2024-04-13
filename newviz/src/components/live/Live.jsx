import React, { useEffect, useState, useRef, useCallback, useContext } from 'react'
import { Button } from "@mui/material";
import { Paper, Typography, Box } from '@mui/material';
import { RosContext } from '../../contexts/RosContextProvider';
import ROSLIB from 'roslib';

const Live = () => {
    const ros = useContext(RosContext);
    const canvasRef = useRef(undefined);
    const videoRef = useRef(null);
    useEffect(() => {
      if (canvasRef !== undefined && ros !== undefined && ros.isConnected) {
        const image_listener = new ROSLIB.Topic({
          ros,
          name: '/camera/color/image_raw',
          messageType: 'sensor_msgs/Image'
        });
        image_listener.subscribe((message) => {
          const ctx = canvasRef.current.getContext("2d");
          const imageData = ctx.createImageData(message.width, message.height);
          var ptr = 0;
          for (var i = 0; i < message.width * message.height; i += 3) {
            imageData.data[ptr++] = message.data.charCodeAt(i);
            imageData.data[ptr++] = message.data.charCodeAt(i+1);
            imageData.data[ptr++] = message.data.charCodeAt(i+2);
            imageData.data[ptr++] = 0;
          }
          ctx.putImageData(imageData, 0, 0);
        });
      }
    }, [ros, canvasRef])



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
          flexDirection: 'column',
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
            {/* <video
              ref={videoRef}
              autoPlay
              playsInline
              style={{
                width: '100%', 
                height: 'auto',
                maxWidth: '100%', 
              }}
            ></video> */}
            <canvas ref={canvasRef} style={{
                width: '100%', 
                height: 'auto',
                maxWidth: '100%', 
              }}>
            </canvas>
          </Paper>
        </Box>
      );
  };
  
  export default Live;