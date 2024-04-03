import React, { useContext, useEffect } from 'react'
import { RosContext } from './contexts/RosContext.js'
import ROSLIB from 'roslib';

function TestButton({count}) {
    const ros = useContext(RosContext);
    useEffect(() => {
        if (ros === undefined) return;
        var listener = new ROSLIB.Topic({
            ros : ros,
            name : '/joint_states',
            messageType : 'sensor_msgs/JointState'
        });
        listener.subscribe(function(message) {
        for (const i in message.position) {
            console.log(`${message.name[i]} ${message.position[i]}`);
        }
        listener.unsubscribe();
        });
    }, [ros]);
    return (
        <button>Click me! {count}</button>
    );
}
export default TestButton;