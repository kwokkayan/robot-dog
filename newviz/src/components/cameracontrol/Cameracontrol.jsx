import * as THREE from 'three'
import { memo, useRef, forwardRef } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import { Grid, Center, AccumulativeShadows, RandomizedLight, Environment, useGLTF, CameraControls } from '@react-three/drei'
import { useControls, button, buttonGroup, folder } from 'leva'
import { suspend } from 'suspend-react'
import { useContext, useEffect, useState } from "react";
import URDFViewer from "../../URDFViewer/URDFViewer";
import { RosContext } from "../../contexts/RosContextProvider";
import Control from "../control/Control";



export default function Cameracontrol() {
    const ros = useContext(RosContext);
    const [isConnected, setConnected] = useState(false);
    useEffect(() => setConnected(ros !== undefined && ros.isConnected), [ros])
    if (!isConnected) {
      return (<div>{`Waiting on ${import.meta.env.VITE_WS_URL}`}</div> );
    }
    return (
      <div style={{ width: "75vw", height: "75vh" }}><URDFViewer /><Control/></div>
    );
};


