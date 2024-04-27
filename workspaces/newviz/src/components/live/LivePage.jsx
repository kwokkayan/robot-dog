import React, { useEffect, useState, useRef } from 'react'
import Live from '../live/Live';

function LivePage() {
  const winwidth = window.innerWidth;

  return(
      

      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100vh', width: winwidth}}>
          <Live/>
      </div>
  );
};
export default LivePage;
