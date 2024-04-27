import React, { useEffect, useState, useRef } from 'react'
import Live from '../live/Live';

const LivePage = () => {
    const videoRef = useRef(null);
    const winwidth = window.innerWidth;
    const winheight = window.innerHeight
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


    return(
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: winheight, width: winwidth}}>
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
        </div>
    );
};    
export default LivePage;
