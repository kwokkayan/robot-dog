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
