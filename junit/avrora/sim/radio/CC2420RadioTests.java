package avrora.sim.radio;

import junit.framework.TestCase;
import avrora.sim.mcu.MockMCU;
import avrora.sim.mcu.MockPinInputListener;
import avrora.sim.state.Register;
import avrora.sim.state.RegisterUtil;
import cck.util.Arithmetic;

/**
 * Tests for the {@link CC2420Radio}
 * 
 * @author Matthias Linder
 */
public class CC2420RadioTests extends TestCase {
    
    public void testCC2420Output_listener() { /* partial integration test */
        Register reg = new Register(8);
        CC2420Radio radio = new CC2420Radio(new MockMCU(), 1000);
        CC2420Radio.CC2420Output output = radio.new CC2420Output("foobar", RegisterUtil.booleanView(reg, 0));
        MockPinInputListener listener = new MockPinInputListener();
        
        output.registerListener(listener);
        
        // Register modifications
        reg.setValue(Arithmetic.packBits(0, 0, 0, 0, 0, 0, 0, 0));
        assertEquals(0, listener.called);
        
        reg.setValue(Arithmetic.packBits(1, 1, 1, 1, 1, 1, 1, 0));
        assertEquals(0, listener.called);
        
        reg.setValue(Arithmetic.packBits(0, 0, 0, 0, 0, 0, 0, 1));
        assertEquals(1, listener.called); assertTrue(listener.newValue);
        
        reg.setValue(Arithmetic.packBits(0, 0, 0, 0, 0, 0, 1, 0));
        assertEquals(2, listener.called); assertFalse(listener.newValue);
        
        // Direct access to the boolean view
        output.setLevel(true);
        assertEquals(3, listener.called); assertTrue(listener.newValue);
        
        output.setLevel(true);
        assertEquals(3, listener.called); assertTrue(listener.newValue);
        
        output.setLevel(false);
        assertEquals(4, listener.called); assertFalse(listener.newValue);
    }
}
