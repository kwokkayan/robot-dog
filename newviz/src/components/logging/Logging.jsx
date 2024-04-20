import { useEffect, useState, useContext } from 'react';
import { Box, Typography } from '@mui/material';
import { RosContext } from "../../contexts/RosContextProvider";
import ROSLIB from 'roslib';

const Logging = () => {
    const contextValue = useContext(RosContext);
    const [logMsg, setLogMsg] = useState(null);

    useEffect(() => {
        if (contextValue !== undefined && contextValue.isConnected) {
          
          const listener = new ROSLIB.Topic({
            ros: contextValue,
            name: '/joint_states',
            messageType: 'sensor_msgs/JointState'
          });
          // setImuData(listener.subscribe((message) => {console.log(message)}));
  
          listener.subscribe((message) => {
            // console.log('Received IMU data:', message);
            setLogMsg(logMsg + JSON.stringify(message['position'], null, 2) + '\n');
          })

          
    
          // Clean up the subscription when the component unmounts
          return () => {
            listener.unsubscribe();
          };
        }
      }, [contextValue, logMsg]);

    return(
        <Box sx={{
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            rowGap: '20px',
            height: '100%',
        }}>
        {/* <Typography sx={{alignSelf: 'center'}}>{logMsg}</Typography> */}
        </Box>
    );
};    
export default Logging;
