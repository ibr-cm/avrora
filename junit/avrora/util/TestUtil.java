package avrora.util;

import avrora.arch.avr.AVRArchitecture;
import avrora.core.Program;
import avrora.sim.MockInterpreterFactory;
import avrora.sim.Simulator;
import avrora.sim.mcu.ATMega128;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.platform.MicaZ;
import avrora.sim.types.SingleSimulation;

/**
 * Test Utility class
 * 
 * @author Matthias Linder
 */
public final class TestUtil {
    
    private TestUtil() {}
    
    /**
     * Creates a default {@link Simulator} object
     * @return
     */
    public static Simulator createSimulator(Microcontroller mcu) {
        return new SingleSimulation().createSimulator(0, new MockInterpreterFactory(), mcu, null);
    }
    
    /**
     * Creates a program.
     * @return
     */
    public static Program createProgram() {
        return new Program(new AVRArchitecture(), 0, 128);
    }
    
    /**
     * Creates the MicaZ platform.
     * @return
     */
    public static MicaZ createMicaZ() {
        return (MicaZ) new MicaZ.Factory().newPlatform(0, new SingleSimulation(), createProgram());
    }
    
    /**
     * Creates an ATMega128 mcu
     * @return
     */
    public static ATMega128 createATMega128() {
        return (ATMega128) createMicaZ().getMicrocontroller();
    }
}
