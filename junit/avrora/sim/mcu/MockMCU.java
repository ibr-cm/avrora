package avrora.sim.mcu;

import avrora.sim.Simulator;
import avrora.sim.clock.ClockDomain;
import avrora.sim.platform.Platform;
import avrora.util.TestUtil;

/**
 * Mock for the {@link Microcontroller} interface.
 * 
 * @author Matthias Linder
 */
public class MockMCU implements Microcontroller {
    
    public Simulator simulator;
    public Platform platform;
    private ClockDomain clockDomain;
    
    public MockMCU() {
        clockDomain = new ClockDomain(1000);
        this.simulator = TestUtil.createSimulator(this);
    }
    

    @Override
    public Simulator getSimulator() {
        return simulator;
    }

    @Override
    public Platform getPlatform() {
        return platform;
    }

    @Override
    public void setPlatform(Platform p) {
        platform = p;
    }

    @Override
    public Pin getPin(String name) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Pin getPin(int num) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void sleep() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int wakeup() {
        throw new UnsupportedOperationException();
    }

    @Override
    public ClockDomain getClockDomain() {
        return clockDomain;
    }

    @Override
    public RegisterSet getRegisterSet() {
        throw new UnsupportedOperationException();
    }

    @Override
    public MCUProperties getProperties() {
        return null;
    }
}
