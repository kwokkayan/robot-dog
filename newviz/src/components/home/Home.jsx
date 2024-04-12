import React, { useEffect, useState } from 'react'
import { Typography } from '@mui/material'
import { Button } from "@mui/material";
import { Joystick } from 'react-joystick-component';
import Grid from '@mui/material/Grid';
import { useTheme } from '@mui/material/styles';
import Visual from '../visual/Visual';
import Live from '../live/Live';
import { Leva } from 'leva';


const Home = () => {
  const [sizes, setSizes] = useState({ wl: 40, wc: 40, wr: 20, levaDx: 0.2 * Math.max(document.documentElement.clientWidth || 0, window.innerWidth || 0)});

  const startResize = (event, left) => {
    const startX = event.clientX;
    const sizeNow = { ...sizes };
    const vieww = Math.max(document.documentElement.clientWidth || 0, window.innerWidth || 0);

    const drag = (e) => {
      const dx = e.clientX - startX;
      const dxVw = (dx / vieww) * 100;
      if (left) {
        const newwl = Math.max(20, Math.min(80 - sizeNow.wr, sizeNow.wl + dxVw));
        const newwc = 100 - sizeNow.wr - newwl;
        const levaDx = (sizeNow.wr / 100) * vieww;
        setSizes({ wl: newwl, wc: newwc, wr: sizeNow.wr, levaDx });
      } else {
        const newwr = Math.max(10, Math.min(90 - sizeNow.wl, sizeNow.wr - dxVw));
        const newwc = 100 - sizeNow.wl - newwr;
        const levaDx = (newwr / 100) * vieww;
        setSizes({ wl: sizeNow.wl, wc: newwc, wr: newwr, levaDx });
      }
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
        <Leva titleBar={{position:{x: -sizes.levaDx, y:100}}}/>
      </div>
      <div style={{ display: 'flex', flexDirection: 'row'}}>
        <div style={{ width: `${sizes.wl}vw`, margin: '0px 12px'}}><Live></Live></div>
        <div onMouseDown={(e) => startResize(e, true)} style={{ cursor: 'ew-resize', width: '5px', background: '#ccc' }} />
        <div style={{ width: `${sizes.wc}vw`}}><Visual></Visual></div>
        <div onMouseDown={(e) => startResize(e, false)} style={{ cursor: 'ew-resize', width: '5px', background: '#ccc' }} />
        <div style={{ width: `${sizes.wr}vw`}}>Right</div>
      </div>
    </div>
  );
  };
  
  export default Home;