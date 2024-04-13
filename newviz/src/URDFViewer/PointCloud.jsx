import { useFrame, useThree } from '@react-three/fiber'
import { useContext, useEffect, useState } from 'react';
import { PointCloud2} from 'ros3d';
import ROSLIB from 'roslib';
import { RosContext } from '../contexts/RosContextProvider';

function PointCloud() {
    const { scene } = useThree();
    const ros = useContext(RosContext);
    const [pointCloud, setPointCloud] = useState(undefined);
    useEffect(() => {
        if (scene !== undefined && ros !== undefined && ros.isConnected) {
            const tfClient = new ROSLIB.TFClient({
                ros: ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: '/base_link'
            });
            const pointCloud = new PointCloud2({
                ros: ros,
                topic: '/nvblox_node/static_esdf_pointcloud',
                tfClient,
                material: { size: 1000, color: 0xff0000 },
                rootObject: scene
            });
            setPointCloud(pointCloud);
        }
    }, [ros, scene]);
    return (
        <>
            { pointCloud !== undefined && <primitive object={pointCloud}/> }
        </>
    );
}
export default PointCloud;