import { useContext, useEffect, useState } from "react";
import URDFViewer from "../../URDFViewer/URDFViewer";
import { RosContext } from "../../contexts/RosContextProvider";

function Visual() {
  const ros = useContext(RosContext);
  const [isConnected, setConnected] = useState(false);
  useEffect(() => setConnected(ros !== undefined && ros.isConnected), [ros])
  if (!isConnected) {
    return (<div>{`Waiting on ${import.meta.env.VITE_WS_URL}`}</div>);
  }
  return (
    <div style={{ display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center', height: '100%', width:'100%'}}><URDFViewer/></div>
  );
}
export default Visual;
