package avrora.sim.state;

import junit.framework.TestCase;

/** 
 * Tests for the BooleanView class
 * 
 * @author Matthias Linder
 */
public class BooleanRegisterTests extends TestCase {
    
    public void testValueChanged() {
        MockBooleanValueSetListener listener = new MockBooleanValueSetListener(); 
        
        BooleanRegister reg = new BooleanRegister();
        
        reg.setValueSetListener(listener);
        assertFalse(listener.wasCalled);
        
        reg.setValue(false);
        assertTrue(listener.wasCalled); assertFalse(listener.newValue); listener.wasCalled = false;
        
        reg.setValue(true);
        assertTrue(listener.wasCalled); assertTrue(listener.newValue);
    }
}
