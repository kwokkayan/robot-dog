import React, { useState } from 'react';
import { Leva } from 'leva';
import Grid from '@mui/material/Grid';
import Live from '../live/Live';
import Visual from '../visual/Visual';

const Home = () => {
  const [size, setSize] = useState(50);

  const startResize = (event) => {
    const startX = event.clientX;
    const startSize = size;
    const viewW = Math.max(document.documentElement.clientWidth || 0, window.innerWidth || 0);

    const drag = (e) => {
      const dx = e.clientX - startX;
      const dxVw = (dx / viewW) * 100;
      const newSize = Math.max(20, Math.min(80, startSize + dxVw));
      setSize(newSize);
    };

    const stopDrag = () => {
      document.removeEventListener('mousemove', drag);
      document.removeEventListener('mouseup', stopDrag);
    };

    document.addEventListener('mousemove', drag);
    document.addEventListener('mouseup', stopDrag);
  };

  return (
    <div>
      <div>
        <Leva titleBar={{ position: { x: 0, y: 100 } }}/>
      </div>
      <div style={{ display: 'flex', flexDirection: 'row', height: '100vh', backgroundColor: '#333840' }}>
        <div style={{ width: `${size}vw`, margin: '0px 16px' }}>
          <Live></Live>
        </div>
        <div onMouseDown={startResize} style={{ cursor: 'ew-resize', width: '5px', background: '#888' }} />
        <div style={{ width: `${100 - size}vw` }}>
          <Visual></Visual>
        </div>
      </div>
    </div>
  );
};

export default Home;
