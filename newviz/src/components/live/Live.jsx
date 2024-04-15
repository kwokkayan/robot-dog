import{ useEffect, useState } from 'react'
import { Paper, Box, Slider, Typography } from '@mui/material'

const Live = () => {
    const [feedUrl, setFeedURL] = useState(import.meta.env.VITE_CAMERA_STREAM_URL);
    const [quality, setQuality] = useState(5);
    const handleChange = (e, newValue) => {
      setQuality(newValue);
    };
    useEffect(() => {
      setFeedURL(`${import.meta.env.VITE_CAMERA_STREAM_URL}&quality=${quality}`);
    }, [quality]);
    return (
        <Box sx={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          rowGap: '20px',
          height: '100%',
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
            <img style={{
              maxWidth: '100%',
              maxHeight: '100%',
              objectFit: 'fill' 
            }} src={feedUrl}/>
          </Paper>
          <Box width={"100%"}>
          <Typography id="slider" sx={{color: "secondary.main"}}>
            Stream quality
          </Typography>
          <Slider min={1} max={100} value={quality} onChange={handleChange} valueLabelDisplay="auto" aria-labelledby="slider" sx={{color: "secondary.main"}}/>
          </Box>
        </Box>
      );
  };
  
  export default Live;