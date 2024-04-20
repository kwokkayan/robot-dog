import { useEffect, useState } from 'react';
import { Paper, Box, Slider, Typography, Button } from '@mui/material';
import { FFmpeg } from '@ffmpeg/ffmpeg';
import { fetchFile } from '@ffmpeg/util'

const Live = () => {
    const ffmpeg = new FFmpeg({ log: true });
    const [feedUrl, setFeedURL] = useState(import.meta.env.VITE_CAMERA_STREAM_URL);
    const [quality, setQuality] = useState(5);
    const [images, setImages] = useState([]);
    const [isRecording, setIsRecording] = useState(false);
    const [isProcessing, setIsProcessing] = useState(false);

    const handleChange = (e, newValue) => {
        setQuality(newValue);
    };

    useEffect(() => {
        setFeedURL(`${import.meta.env.VITE_CAMERA_STREAM_URL}&quality=${quality}`);
    }, [quality]);

    useEffect(() => {
        if (isRecording) {
            const interval = setInterval(() => {
                const img = document.querySelector('img');
                if (img) {
                    const canvas = document.createElement('canvas');
                    canvas.width = img.naturalWidth;
                    canvas.height = img.naturalHeight;
                    const ctx = canvas.getContext('2d');
                    ctx.drawImage(img, 0, 0, img.naturalWidth, img.naturalHeight);
                    canvas.toBlob(blob => {
                        setImages(prev => [...prev, blob]);
                    });
                }
            }, 100); // 10 fps
            return () => clearInterval(interval);
        }
    }, [isRecording]);

    const handleStartStopRecording = () => {
        setIsRecording(!isRecording);
        if (isRecording) { // If currently recording, stop and process the video
            processVideo();
        } else {
            setImages([]); // Clear previous images when starting new recording
        }
    };

    const processVideo = async () => {
        setIsProcessing(true);
        if (!ffmpeg.loaded) {
            await ffmpeg.load();
        }

        ffmpeg.FS('mkdir', 'images');
        await Promise.all(images.map((blob, index) =>
            ffmpeg.writeFile(`images/image${index}.jpg`, fetchFile(blob))
        ));

        await ffmpeg.run('-framerate', `${quality}`, '-i', 'images/image%d.jpg', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', 'output.mp4');
        const data = ffmpeg.FS('readFile', 'output.mp4');
        const videoUrl = URL.createObjectURL(new Blob([data.buffer], { type: 'video/mp4' }));
        const a = document.createElement('a');
        a.href = videoUrl;
        a.download = 'livestream_recording.mp4';
        a.click();
        URL.revokeObjectURL(videoUrl);
        setIsProcessing(false);
    };

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
                width: window.innerWidth > window.innerHeight ? 'calc(55vw - 32px)' : '80vw',
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
                }} src={feedUrl} />
            </Paper>
            <Box width={"100%"}>
                <Typography id="slider" sx={{ color: "white" }}>
                    Stream quality
                </Typography>
                <Slider min={1} max={100} value={quality} onChange={handleChange} valueLabelDisplay="auto" aria-labelledby="slider" sx={{ color: "white" }} />
                <Button onClick={handleStartStopRecording} color="primary" variant="contained">
                    {isRecording ? 'Stop Recording' : 'Start Recording'}
                </Button>
                {isProcessing && <p>Processing video...</p>}
            </Box>
        </Box>
    );
};

export default Live;