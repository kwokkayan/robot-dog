import { useEffect, useState, useContext } from 'react';
import { Box, Typography } from '@mui/material';
import { RosContext } from "../../contexts/RosContextProvider";
import ROSLIB from 'roslib';

const Notifications = () => {
    const contextValue = useContext(RosContext);
    const [logMsg, setLogMsg] = useState(null);

    useEffect(() => {
        if (contextValue !== undefined && contextValue.isConnected) {
          
          const listener = new ROSLIB.Topic({
            ros: contextValue,
            name: 'detections_output',
            messageType: 'vision_msgs/Detection2DArray'
          }); 
          listener.subscribe((message) => {
            console.log(message);
            console.log('successful');
            setLogMsg(message);
            alert(`Received message: ${JSON.stringify(message)}`);
          });
          
    
          // Clean up the subscription when the component unmounts
          return () => {
            listener.unsubscribe();
          };
        }
      }, [contextValue, logMsg]);

    return null;
};    
export default Notifications;
