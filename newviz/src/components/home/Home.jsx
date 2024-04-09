import React, { useEffect, useState } from 'react'
import { Typography } from '@mui/material'
import { Button } from "@mui/material";
import { Joystick } from 'react-joystick-component';
import Grid from '@mui/material/Grid';
import { useTheme } from '@mui/material/styles';
import Visual from '../visual/Visual';
import Live from '../live/Live';


const Home = () => {
  const [sizes, setSizes] = useState({ leftWidth: 40, centerWidth: 40, rightWidth: 20 });

  const startResize = (event, direction) => {
    const startX = event.clientX;
    const startSizes = { ...sizes };
    const viewportWidth = Math.max(document.documentElement.clientWidth || 0, window.innerWidth || 0);

    const doDrag = (dragEvent) => {
      const dx = dragEvent.clientX - startX;
      const dxVw = (dx / viewportWidth) * 100;

      if (direction === 'left') {
        const newLeftWidth = Math.max(20, Math.min(80 - startSizes.rightWidth, startSizes.leftWidth + dxVw));
        const newCenterWidth = 100 - startSizes.rightWidth - newLeftWidth;
        setSizes({ leftWidth: newLeftWidth, centerWidth: newCenterWidth, rightWidth: startSizes.rightWidth });
      } else {
        const newRightWidth = Math.max(10, Math.min(90 - startSizes.leftWidth, startSizes.rightWidth - dxVw));
        const newCenterWidth = 100 - startSizes.leftWidth - newRightWidth;
        setSizes({ leftWidth: startSizes.leftWidth, centerWidth: newCenterWidth, rightWidth: newRightWidth });
      }
    };

    const stopDrag = () => {
      document.removeEventListener('mousemove', doDrag);
      document.removeEventListener('mouseup', stopDrag);
    };

    document.addEventListener('mousemove', doDrag);
    document.addEventListener('mouseup', stopDrag);
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'row' }}>
      <div style={{ width: `${sizes.leftWidth}vw`, marginRight: '5px'}}><Live></Live></div>
      <div onMouseDown={(e) => startResize(e, 'left')} style={{ cursor: 'ew-resize', width: '5px', background: '#ccc' }} />
      <div style={{ width: `${sizes.centerWidth}vw`, marginRight: '5px'}}><Visual></Visual></div>
      <div onMouseDown={(e) => startResize(e, 'right')} style={{ cursor: 'ew-resize', width: '5px', background: '#ccc' }} />
      <div style={{ width: `${sizes.rightWidth}vw`}}>Right</div>
    </div>
  );
  };
  
  export default Home;