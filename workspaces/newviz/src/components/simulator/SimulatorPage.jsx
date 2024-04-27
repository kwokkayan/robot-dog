import Visual from "../visual/Visual";
import { Leva } from 'leva';

function SimulatorPage() {
    const winwidth = window.innerWidth;

    return(
        

        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100vh', width: winwidth}}>
            <Leva titleBar={{ position: { x: 0, y: 100 } }}/>
            <Visual/>
        </div>
    );
};
export default SimulatorPage;

// import { useContext, useEffect, useState } from "react";
// import URDFViewer from "../../URDFViewer/URDFViewer";
// import { RosContext } from "../../contexts/RosContextProvider";

// function SimulatorPage() {
//   const ros = useContext(RosContext);
//   const [isConnected, setConnected] = useState(false);
//   useEffect(() => setConnected(ros !== undefined && ros.isConnected), [ros])
//   if (!isConnected) {
//     return (<div>{`Waiting on ${import.meta.env.VITE_WS_URL}`}</div>);
//   }
//   return (
    

//     <div style={{ display: 'flex',
//     // flexDirection: 'column',
//     alignItems: 'center',
//     justifyContent: 'center', height: '100%', width:'100%'}}>
//         <h1>testing</h1><URDFViewer/></div>
//   );
// }
// export default SimulatorPage;
