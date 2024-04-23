import { useState } from 'react';
import { Leva } from 'leva';
import Live from '../live/Live';
import Visual from '../visual/Visual';
import Notifications from '../notifications';

const Home = () => {
  const [size, setSize] = useState(30);

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
      <Notifications></Notifications>
      <div>
        <Leva theme={{sizes: {rootWidth: "20vw", rowHeight: "3rem"}, fontSizes:{root: "1rem"}, space: {rowGap: "10px"}}} titleBar={{ position: { x: 0, y: 100 } }}/>
      </div>
      <div style={{ display: 'flex', flexDirection: 'row', backgroundColor: '#333840'}}>
        <div style={{ width: `${size}vw`, padding: "0px 16px"}}>
          <Live></Live>
        </div>
        <div onMouseDown={startResize} style={{ cursor: 'ew-resize', width: '5px', background: '#888', height: '100vh' }} />
        <div style={{ width: `${100 - size}vw`}}>
          <Visual></Visual>
        </div>
      </div>
    </div>
  );
};

export default Home;
