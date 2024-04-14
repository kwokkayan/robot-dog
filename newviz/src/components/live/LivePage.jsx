import React, { useEffect, useState, useRef } from 'react'
import Live from '../live/Live';

const LivePage = () => {
    const videoRef = useRef(null);
    const winwidth = window.innerWidth;
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
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100vh', width: winwidth}}>
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
// import React from 'react';
// import Live from '../live/Live';

// const LivePage = () => {

//     const height = window.innerHeight;

//     const styles = {

//     display: 'flex',
//     justifyContent: 'center',
//     alignItems: 'center',
//     height: `${height - 82}px`,
//     marginTop: '82px'

//     };
//     return (
//     <div style={styles}>
//         {/* <h1>testing</h1> */}
//         <Live />
//     </div>
//     );
// };



// export default LivePage;