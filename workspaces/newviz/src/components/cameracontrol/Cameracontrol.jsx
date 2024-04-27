import { extend } from '@react-three/fiber';
// import { Div } from 'third-party-library';
import Control from "../control/Control";
// extend({ Div });


extend({ div ,h1});

export default function Cameracontrol() {
  return (
    <div>
      <Control />
      {/* <h1>test</h1> */}
    </div>
  );
}