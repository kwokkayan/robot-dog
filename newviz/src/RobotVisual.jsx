import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader'
import URDFLoader from 'urdf-loader';
import * as THREE from 'three';
import { LoaderUtils } from 'three';
import { XacroLoader } from 'xacro-parser';
import React from 'react';

// ...init three.js scene...
class RobotVisual extends React.Component{
    componentDidMount(){
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 0.1, 1000 );

    const renderer = new THREE.WebGLRenderer();
    renderer.setSize( window.innerWidth, window.innerHeight );
    document.body.appendChild( renderer.domElement );

    const loader = new URDFLoader();
    loader.loadMeshCb = function( path, manager, onComplete ) {

        const gltfLoader = new GLTFLoader( manager );
        gltfLoader.load(
            path,
            result => {

                onComplete( result.scene );

            },
            undefined,
            err => {
            
                // try to load again, notify user, etc
            
                onComplete( null, err );
            
            }
        );

    };
    console.log("read1");


    const url = '../../robot_dog_meshes_xacro/urdf/newquadruped.urdf.xacro';
    const xacroLoader = new XacroLoader();
    let robot;

    xacroLoader.load( url, xml => {
        if(xml){
            console.log("yes xml");

            const urdfLoader = new URDFLoader();
            urdfLoader.workingPath = LoaderUtils.extractUrlBase( url );
            console.log(urdfLoader.workingPath);
            console.log(xml);

            console.log("start");
            
            robot = urdfLoader.parse( xml );
            console.log(urdfLoader.parser.getErrors());
            console.log("here");
            console.log(robot);
            scene.add( robot );
        }
        console.log("read2");
    } );
    // console.log(robot.BoxGeometry);
    console.log("end");

    // const geometry = new THREE.BoxGeometry( 1, 1, 1 );
    // const material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
    // const cube = new THREE.Mesh( geometry, material );
    // scene.add( cube );

    camera.position.z = 5;
    // camera.position.set(0, 0, 10);
    // robot.scale.set(0.1, 0.1, 0.1);
    // robot.position.set(0, 0, -5);

    function animate() {
        requestAnimationFrame( animate );
        renderer.render( scene, camera );
    }
    animate();
    }

    render(){
        return null;
    }
}
export default RobotVisual;