import * as THREE from 'three'
import { memo, useRef, forwardRef } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import { Grid, Center, AccumulativeShadows, RandomizedLight, Environment, useGLTF, CameraControls } from '@react-three/drei'
import { useControls, button, buttonGroup, folder} from 'leva'
import { suspend } from 'suspend-react'
import { useContext, useEffect, useState } from "react";
import URDFViewer from "../../URDFViewer/URDFViewer";
import { RosContext } from "../../contexts/RosContextProvider";



const { DEG2RAD } = THREE.MathUtils



export default function Control() {
    const meshRef = useRef()
    const cameraControlsRef = useRef()
  
    var params;
    const contextValue = useContext(RosContext);
      // console.log(contextValue.Topic);
    // const handleGetParam = () => {
    //   if(contextValue){
    //     contextValue.getParams((params) =>{
    //       console.log(params);
    //     });
    //   }
    // };

    contextValue.getParams((params)=>console.log(params));

    contextValue.getParams('array', (value) => {
      
    })

    const { camera } = useThree()
  
    // All same options as the original "basic" example: https://yomotsu.github.io/camera-controls/examples/basic.html
    const { minDistance, enabled, verticalDragToForward, dollyToCursor, infinityDolly } = useControls({
      top:'50px',
      
      ShowCloseControlPanel: folder(
        {
        thetaGrp: buttonGroup({
          label: 'rotate theta',
          opts: {
            '+45º': () => cameraControlsRef.current?.rotate(45 * DEG2RAD, 0, true),
            '-45º': () => cameraControlsRef.current?.rotate(-45 * DEG2RAD, 0, true),
            '+180º': () => cameraControlsRef.current?.rotate(180 * DEG2RAD, 0, true)
          }
        }),
        phiGrp: buttonGroup({
          label: 'rotate phi',
          opts: {
            '+30º': () => cameraControlsRef.current?.rotate(0, 30 * DEG2RAD, true),
            '-30º': () => cameraControlsRef.current?.rotate(0, -30 * DEG2RAD, true)
          }
        }),
        truckGrp: buttonGroup({
          label: 'truck',
          opts: {
            '(1,0)': () => cameraControlsRef.current?.truck(1, 0, true),
            '(0,1)': () => cameraControlsRef.current?.truck(0, 1, true),
            '(-1,-1)': () => cameraControlsRef.current?.truck(-1, -1, true)
          }
        }),
        dollyGrp: buttonGroup({
          label: 'dolly',
          opts: {
            '1': () => cameraControlsRef.current?.dolly(1, true),
            '-1': () => cameraControlsRef.current?.dolly(-1, true)
          }
        }),
        zoomGrp: buttonGroup({
          label: 'zoom',
          opts: {
            '/2': () => cameraControlsRef.current?.zoom(camera.zoom / 2, true),
            '/-2': () => cameraControlsRef.current?.zoom(-camera.zoom / 2, true)
          }
        }),
        minDistance: { value: 0 },
        moveTo: folder(
          {
            vec1: { value: [3, 5, 2], label: 'vec' },
            'moveTo(…vec)': button((get) => cameraControlsRef.current?.moveTo(...get('moveTo.vec1'), true))
          },
          { collapsed: true }
        ),
        'fitToBox(mesh)': button(() => cameraControlsRef.current?.fitToBox(meshRef.current, true)),
        setPosition: folder(
          {
            vec2: { value: [-5, 2, 1], label: 'vec' },
            'setPosition(…vec)': button((get) => cameraControlsRef.current?.setPosition(...get('setPosition.vec2'), true))
          },
          { collapsed: true }
        ),
        setTarget: folder(
          {
            vec3: { value: [3, 0, -3], label: 'vec' },
            'setTarget(…vec)': button((get) => cameraControlsRef.current?.setTarget(...get('setTarget.vec3'), true))
          },
          { collapsed: true }
        ),
        setLookAt: folder(
          {
            vec4: { value: [1, 2, 3], label: 'position' },
            vec5: { value: [1, 1, 0], label: 'target' },
            'setLookAt(…position, …target)': button((get) => cameraControlsRef.current?.setLookAt(...get('setLookAt.vec4'), ...get('setLookAt.vec5'), true))
          },
          { collapsed: true }
        ),
        lerpLookAt: folder(
          {
            vec6: { value: [-2, 0, 0], label: 'posA' },
            vec7: { value: [1, 1, 0], label: 'tgtA' },
            vec8: { value: [0, 2, 5], label: 'posB' },
            vec9: { value: [-1, 0, 0], label: 'tgtB' },
            t: { value: Math.random(), label: 't', min: 0, max: 1 },
            'f(…posA,…tgtA,…posB,…tgtB,t)': button((get) => {
              return cameraControlsRef.current?.lerpLookAt(
                ...get('lerpLookAt.vec6'),
                ...get('lerpLookAt.vec7'),
                ...get('lerpLookAt.vec8'),
                ...get('lerpLookAt.vec9'),
                get('lerpLookAt.t'),
                true
              )
            })
          },
          { collapsed: true }
        ),
        saveState: button(() => cameraControlsRef.current?.saveState()),
        reset: button(() => cameraControlsRef.current?.reset(true)),
        enabled: { value: true, label: 'controls on' },
        verticalDragToForward: { value: false, label: 'vert. drag to move forward' },
        dollyToCursor: { value: false, label: 'dolly to cursor' },
        infinityDolly: { value: false, label: 'infinity dolly' }
      },
      {collapsed: true}
      )
    })
  
    return (
      <>

        {/* <div><button onClick={(handleGetParam)}>get</></div> */}

        <group position-y={-0.5}>
          <Center top>
          </Center>
          {/* <Ground /> */}
          <CameraControls 
            ref={cameraControlsRef}
            minDistance={minDistance}
            enabled={enabled}
            verticalDragToForward={verticalDragToForward}
            dollyToCursor={dollyToCursor}
            infinityDolly={infinityDolly}
          />
          {/* <Environment files={suspend(city).default} /> */}
        </group>
      </>
    )

    
    useEffect(() => {
      const ros = new ROSLIB.Ros();
      ros.connect('ws://localhost:9090');
  
      ros.on('connection', () => {
        setRosInstance(ros); // we connected!      
        console.log('Connected to ROS');
        console.log('hihihihi');
        ros.getNodes((nodes) => console.log(nodes+"hi"));
        // console.log("topics")
        // ros.getTopics((topics) => console.log(topics));
        
        // ros.getMessageDetails((msg) => console.log(msg));
        var listener = new ROSLIB.Topic({
          ros : ros,
          name : '/imu/data',
          messageType : 'sensor_msgs/Imu'
        });

        // listener.subscribe(function(message){
        //   for(const i in message.position){

        //   }
        // };
        


        listener.subscribe(function(message) {
          for (const i in message.position) {
            // console.log(`${message.name[i]} ${message.position[i]}`);
          }
          
          listener.unsubscribe();
        });
        
  
      });
  
      ros.on('error', (error) => {
        console.error('ROS error:', error);
      });
  
      ros.on('close', () => {
        setRosInstance(undefined);
        console.log('Disconnected from ROS');
      });
  
      
    }, []);


  }


  
//   function Ground() {
//     const gridConfig = {
//       cellSize: 0.5,
//       cellThickness: 0.5,
//       cellColor: '#6f6f6f',
//       sectionSize: 3,
//       sectionThickness: 1,
//       sectionColor: '#9d4b4b',
//       fadeDistance: 30,
//       fadeStrength: 1,
//       followCamera: false,
//       infiniteGrid: true
//     }
//     return <Grid position={[0, -0.01, 0]} args={[10.5, 10.5]} {...gridConfig} />
//   }
  
//   const Shadows = memo(() => (
//     <AccumulativeShadows temporal frames={100} color="#9d4b4b" colorBlend={0.5} alphaTest={0.9} scale={20}>
//       <RandomizedLight amount={8} radius={4} position={[5, 5, -10]} />
//     </AccumulativeShadows>
//   ))
  
//   const Suzi = forwardRef((props, ref) => {
//     const { nodes } = useGLTF(suspend(suzi).default)
//     return (
//       <>
//         <mesh ref={ref} castShadow receiveShadow geometry={nodes.mesh.geometry} {...props}>
//           <meshStandardMaterial color="#9d4b4b" />
//         </mesh>
//       </>
//     )
//   })
  