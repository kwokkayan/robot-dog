import { createRoot } from 'react-dom/client'
import React, { useEffect, useRef, useState } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import URDFLoader from 'urdf-loader';
import { LoadingManager } from 'three';

function URDFViewer(props) {

  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader( manager );
    loader.packages = {
      packageName : '/home/deck/Desktop/robot-dog/IK_engine/src/'            // The equivalent of a (list of) ROS package(s):// directory
    };
    loader.load(
      'test_config/urdf/quadruped.urdf',                    // The path to the URDF within the package OR absolute
      robot => {
        console.log(robot);
      },
      progress => {
        console.log(progress);
      },
      error => {
        console.log(error);
      }
    );
  }, [])

  return (
    <Canvas>
      <ambientLight intensity={0.1} />
      <directionalLight color="red" position={[0, 0, 5]} />
      <mesh>
        <boxGeometry />
        <meshStandardMaterial />
      </mesh>
    </Canvas>

  );
}

export default URDFViewer;