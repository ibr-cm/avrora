package avrora.sim.mcu;

import avrora.sim.mcu.Microcontroller.Pin.ListenableInput;

/**
 * Mock implementation of a {@link ListenableInput}
 * 
 * @author Matthias Linder
 */
public class MockInput extends ListenableInput {
    
    public boolean readValue;

    @Override
    public boolean read() {
        return readValue;
    }
    
    public void setAndNotify(boolean newVal) {
        readValue = newVal;
        this.notifyListeners(readValue);
    }
}
