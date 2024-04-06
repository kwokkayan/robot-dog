import React, { useEffect, useRef } from 'react';
import URDFLoader from 'urdf-loader';
import { LoaderUtils } from 'three';
import { XacroLoader } from 'xacro-parser';
import * as THREE from 'three';
import { OrbitControls } from "https://unpkg.com/three@0.112/examples/jsm/controls/OrbitControls.js";

const RobotModel = () => {
    const mountRef = useRef(null);
  
    useEffect(() => {
      const mount = mountRef.current;
  
      // Create a scene, camera, and renderer
      const scene = new THREE.Scene();
      const camera = new THREE.PerspectiveCamera(
        75,
        mount.clientWidth / mount.clientHeight,
        0.1,
        1000
      );
      const renderer = new THREE.WebGLRenderer();
      renderer.setSize(mount.clientWidth, mount.clientHeight);
      mount.appendChild(renderer.domElement);
  
      // Create an instance of URDFLoader
      const loader = new XacroLoader();
  
      // Load the xacro file
      const url = './robot_dog_meshes_xacro/urdf/quadruped.urdf.xacro';
      loader.load(url, (xml) => {
        
        // Add the loaded robot to the scene
        const urdfLoader = new URDFLoader();
        urdfLoader.workingPath = LoaderUtils.extractUrlBase( url );

        const robot = urdfLoader.parse( xml );

        scene.add(robot);
  
        // Adjust the camera position and controls
        camera.position.z = 5;
        const controls = new OrbitControls(camera, renderer.domElement);
  
        // Render the scene
        function animate() {
          requestAnimationFrame(animate);
          controls.update();
          renderer.render(scene, camera);
        }
        animate();
      });
  
      // Clean up the scene when the component is unmounted
      return () => {
        mount.removeChild(renderer.domElement);
      };
    }, []);
  
    return <div ref={mountRef} />;
  };

  export default RobotModel