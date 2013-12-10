package avrora.sim.mcu;

import avrora.sim.mcu.Microcontroller.Pin.Input;

/**
 * Mock implementation of a {@link Microcontroller.Pin.InputListener}
 * 
 * @author Matthias Linder
 */
public class MockPinInputListener implements Microcontroller.Pin.InputListener {
    
    public int called = 0;
    public boolean newValue;
    public Input input;
    
    @Override
    public void onInputChanged(Input input, boolean newValue) {
        called++;
        this.input = input;
        this.newValue = newValue;
    }

}
