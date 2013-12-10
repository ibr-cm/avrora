package avrora.sim.state;

import junit.framework.TestCase;

/**
 * Tests for the RegisterUtil
 * 
 * @author Matthias Linder
 */
public class RegisterUtilTests extends TestCase {
    
    public void testBoolViewNotify() {
        MockBooleanValueSetListener listener = new MockBooleanValueSetListener();
        Register register = new Register(8);
        BooleanView view = RegisterUtil.booleanView(register, 3); // fourth bit
        view.setValueSetListener(listener);
        assertFalse(listener.wasCalled);
        
        register.setValue(0xFF);
        assertTrue(listener.wasCalled); assertTrue(listener.newValue); 
        listener.wasCalled = false;
        
        register.setValue(1 << 3);
        assertFalse(listener.wasCalled);
        
        register.setValue(0x00);
        assertTrue(listener.wasCalled); assertFalse(listener.newValue);
    }
    
    public void testByteArrayViewNotify() {
        MockRegisterValueSetListener listener = new MockRegisterValueSetListener();
        RegisterView view = new RegisterUtil.ByteArrayView(new byte[3], 1);
        view.registerValueSetListener(listener);
        assertEquals(0, listener.called);
        
        view.setValue(1);
        assertEquals(1, listener.called); assertEquals(0, listener.oldValue); assertEquals(1, listener.newValue);
        
        view.setValue(2);
        assertEquals(2, listener.called); assertEquals(1, listener.oldValue); assertEquals(2, listener.newValue);
    }
    
    public void testCharArrayViewNotify() {
        MockRegisterValueSetListener listener = new MockRegisterValueSetListener();
        RegisterView view = new RegisterUtil.CharArrayView(new char[3], 1);
        view.registerValueSetListener(listener);
        assertEquals(0, listener.called);
        
        view.setValue(1);
        assertEquals(1, listener.called); assertEquals(0, listener.oldValue); assertEquals(1, listener.newValue);
        
        view.setValue(2);
        assertEquals(2, listener.called); assertEquals(1, listener.oldValue); assertEquals(2, listener.newValue);
    }
    
    public void testBitRangeViewNotify() {
        Register reg = new Register(8);
        
        MockRegisterValueSetListener listener = new MockRegisterValueSetListener();
        RegisterView view = new RegisterUtil.BitRangeView(reg, (byte)3, (byte)4);
        view.registerValueSetListener(listener);
        assertEquals(0, listener.called);
        
        reg.setValue((1 << 5));
        reg.setValue((1 << 2));
        assertEquals(0, listener.called);
        
        reg.setValue((1 << 3));
        assertEquals(1, listener.called); assertEquals(0, listener.oldValue); assertEquals(1, listener.newValue);
        
        view.setValue((1 << 1) | 1);
        view.setValue((1 << 1) | 1);
        assertEquals(2, listener.called); assertEquals(1, listener.oldValue); assertEquals((1 << 1) | 1, listener.newValue);
        
        view.setValue(0);
        view.setValue(1);
        view.setValue(2);
        view.setValue(3);
        assertEquals(2+4, listener.called);
    }
}
