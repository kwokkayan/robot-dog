import { useContext, useEffect, useState } from "react";
import Control from "../control/Control";
import { RosContext } from "../../contexts/RosContextProvider";
import { Canvas } from "@react-three/fiber";
import { Stage, Grid, OrbitControls, Environment, CameraControls } from "@react-three/drei";

function Panel() {
  return (
    <div style={{width: '50px'}}><Canvas><Control /></Canvas></div>
  );
}
export default Panel;
