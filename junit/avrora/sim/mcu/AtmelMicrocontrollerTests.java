package avrora.sim.mcu;

import avrora.sim.mcu.ATMegaFamily.FlagRegister;
import avrora.sim.state.Register;
import avrora.sim.state.RegisterUtil;
import avrora.util.TestUtil;
import junit.framework.TestCase;

/**
 * Tests for the {@link AtmelMicrocontroller} class.
 * 
 * @author Matthias Linder
 */
public class AtmelMicrocontrollerTests extends TestCase {
    
    public void testINTPin_simple() {
        ATMega128 mcu = TestUtil.createATMega128();
        FlagRegister eifr = new FlagRegister(mcu.interpreter, new int[] { 0, 1, 2, 3, 4, 5, 6, 7 } );
        Register eicrb = new Register(2);
        AtmelMicrocontroller.INTPin pin = mcu.new INTPin(0, eifr, 0, RegisterUtil.bitRangeView(eicrb, 0, 1));
        
        // Connect the pin to a fake input
        MockInput trigger = new MockInput();
        trigger.readValue = true;
        pin.connectInput(trigger);
        eicrb.setValue(10); // any change causes interrupt
        assertEquals(0, eifr.getValue());
        
        // Create some interrupts
        trigger.setAndNotify(false);		
        assertEquals(1, eifr.getValue());
        eifr.setValue(0);
        
        trigger.setAndNotify(false);		
        assertEquals(0, eifr.getValue());
        
        trigger.setAndNotify(true);		
        assertEquals(0, eifr.getValue());
    }
    
    public void testINTPin_edges() {
        ATMega128 mcu = TestUtil.createATMega128();
        FlagRegister eifr = new FlagRegister(mcu.interpreter, new int[] { 0, 1, 2, 3, 4, 5, 6, 7 } );
        Register eicrb = new Register(2);
        AtmelMicrocontroller.INTPin pin = mcu.new INTPin(0, eifr, 0, RegisterUtil.bitRangeView(eicrb, 0, 1));
        
        // Connect the pin to a fake input
        MockInput trigger = new MockInput();
        trigger.readValue = true;
        pin.connectInput(trigger);
        
        eicrb.setValue(2); // falling edge
        
        // Create some interrupts
        trigger.setAndNotify(true);
        assertEquals(0, eifr.getValue());
        trigger.setAndNotify(true);
        assertEquals(0, eifr.getValue());
        trigger.setAndNotify(false);
        assertEquals(1, eifr.getValue());
        trigger.setAndNotify(false);
        assertEquals(1, eifr.getValue());
        eifr.setValue(0);
        
        eicrb.setValue(3); // rising edge
        trigger.setAndNotify(true);
        assertEquals(1, eifr.getValue());
        eifr.setValue(0);
        trigger.setAndNotify(true);
        assertEquals(0, eifr.getValue());
        trigger.setAndNotify(false);
        assertEquals(0, eifr.getValue());
    }
}
