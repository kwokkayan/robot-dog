import { useCallback, useEffect, useRef, useState, useContext } from 'react'
import { Canvas } from '@react-three/fiber'
import URDFLoader from 'urdf-loader';
import { LoadingManager, Euler, Quaternion } from 'three';
import { EffectComposer, Bloom, ToneMapping } from '@react-three/postprocessing';
import { Stage, Grid, OrbitControls, Environment } from "@react-three/drei";
import ROSLIB from 'roslib';
import { Buffer } from "buffer";
import { RosContext } from '../contexts/RosContextProvider';
import Control from "../components/control/Control";
import PointCloud from './PointCloud';


/*
Reference coordinate frames for THREE.js and ROS.
Both coordinate systems are right handed so the URDF is instantiated without
frame transforms. The resulting model can be rotated to rectify the proper up,
right, and forward directions

THREE.js
   Y
   |
   |
   .-----X
 ／
Z

ROS URDf
       Z
       |   X
       | ／
 Y-----.

*/

function URDFViewer() {
  const ros = useContext(RosContext);
  const [robot, setRobot] = useState(undefined);
  const meshRef = useRef();
  const js_callback = useCallback((message) => {
    if (robot) {
      for (const i in message.position) {
        robot.joints[message.name[i]].setJointValue(message.position[i]);
      }
      setRobot(robot);
    }
  }, [robot]);

  const odom_callback = useCallback((message) => {
    if (robot) {
      robot.position.x = message.pose.pose.position.y;
      robot.position.y = message.pose.pose.position.z;
      robot.position.z = message.pose.pose.position.x;
      const { x, y, z, w } = message.pose.pose.orientation;
      const q = new Quaternion(x, y, z, w);
      const angles = new Euler();
      angles.setFromQuaternion(q);
      robot.rotation.x = -Math.PI / 2 + angles.x;
      robot.rotation.y = angles.y;
      robot.rotation.z = -Math.PI / 2 + angles.z;
      setRobot(robot);
    }
  }, [robot]);

  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = (pkg) => `${import.meta.env.VITE_ROS_PACKAGE}/${pkg}`
    loader.load(
      import.meta.env.VITE_URDF_PATH,                    // The path to the URDF within the package OR absolute
      robot => {
        manager.onLoad = () => {
          robot.rotation.x = -Math.PI / 2;
          robot.rotation.z = -Math.PI / 2;
          robot.traverse(c => {
            c.castShadow = true;
          });
          robot.updateMatrixWorld(true);
          setRobot(robot);
        }
      },
      progress => {
        console.log(progress);
      },
      error => {
        console.log(error);
      }
    );
  }, []);

  useEffect(() => {
    if (ros !== undefined && ros.isConnected) {
      const js_listener = new ROSLIB.Topic({
        ros,
        name: '/joint_states',
        messageType: 'sensor_msgs/JointState'
      });
      js_listener.subscribe(js_callback);
      const odom_listener = new ROSLIB.Topic({
        ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
      });
      odom_listener.subscribe(odom_callback);
      // const pointCloud_listener = new ROSLIB.Topic({
      //   ros,
      //   name: '/nvblox_node/static_esdf_pointcloud',
      //   messageType: 'sensor_msgs/PointCloud2'
      // });
      // pointCloud_listener.subscribe((message) => {
      //   const num_points = message.row_step / message.point_step;
      //   console.log(message)
      //   // assume 4 bytes
      //   var points = [];
      //   var ptr = 0;
      //   for (var i = 0; i < num_points; i++) {
      //     const line = new Uint8Array(message.data, ptr, message.point_step)
      //     // console.log(line)
      //     break
      //     points.push(

      //     )
      //     ptr += message.point_step;
      //   }
      // });
    }
  }, [ros, robot, js_callback, odom_callback]);

  return (
    <Canvas flat shadows camera={{ fov: 25 }}>
      <Control meshRef={meshRef}/>
      <Stage intensity={0.5} environment="city" shadows={{ type: 'accumulative', bias: -0.001, intensity: Math.PI }} adjustCamera={false}>
        {robot !== undefined && <primitive ref={meshRef} castShadow receiveShadow object={robot} />}
        {/* <PointCloud/> */}
      </Stage>
      <Grid renderOrder={-1} position={[0, -0.15, 0]} infiniteGrid cellSize={0.6} cellThickness={0.6} sectionSize={3.3} sectionThickness={1.5} sectionColor={[0.5, 0.5, 10]} fadeDistance={30} />
      <OrbitControls makeDefault minPolarAngle={Math.PI / 2} maxPolarAngle={Math.PI / 2} />
      <EffectComposer disableNormalPass>
        <Bloom luminanceThreshold={2} mipmapBlur />
        <ToneMapping />
      </EffectComposer>
      <Environment background preset="sunset" blur={0.8} />
    </Canvas>
  );
}

export default URDFViewer;