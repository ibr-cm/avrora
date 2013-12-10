/**
 * Copyright (c) 2004-2005, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the University of California, Los Angeles nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package avrora.sim.mcu;

import avrora.arch.avr.AVRProperties;
import avrora.sim.*;
import avrora.sim.InterruptTable.Notification;
import avrora.sim.clock.ClockDomain;
import avrora.sim.clock.MainClock;
import avrora.sim.mcu.ATMegaFamily.FlagRegister;
import avrora.sim.state.RegisterUtil.BitRangeView;
import cck.text.Printer;
import java.util.HashMap;

/**
 * The <code>AtmelMicrocontroller</code> class represents the common functionality among microcontrollers
 * for the Atmel series. These all contain a clock domain (collection of internal, external clocks), a
 * simulator, an interpreter, microcontroller properties, and a mapping between string names and IO reg
 * addresses, etc.
 *
 * @author Ben L. Titzer
 */
public abstract class AtmelMicrocontroller extends DefaultMCU {

    protected final MainClock mainClock;
    protected AtmelInterpreter interpreter;

    public final AVRProperties properties;

    protected final HashMap<String, AtmelInternalDevice> devices;
    public static final int MODE_ACTIVE = 0;

    /**
     * The <code>sleep()</code> method is called by the interpreter when the program executes a SLEEP
     * instruction. This method transitions the microcontroller into a sleep mode, including turning
     * off any devices, shutting down clocks, and transitioning the sleep FSM into a sleep mode.
     *
     * @see Microcontroller#sleep()
     */
    public void sleep() {
        // transition to the sleep state in the MCUCR register
        sleepState.transition(getSleepMode());
    }

    protected abstract int getSleepMode();

    /**
     * The <code>wakeup()</code> method is called by the interpreter when the microcontroller is
     * woken from a sleep mode by an interrupt or other event. This method transitions the
     * microcontroller back into active mode, turning back on devices. This method returns
     * the number of clock cycles necessary to wake the MCU from sleep.
     *
     * @return cycles it takes to wake up
     * @see Microcontroller#wakeup()
     */
    public int wakeup() {
        // transition to the active state (may insert transition event into event queue)
        sleepState.transition(MODE_ACTIVE);
        // return the number of cycles consumed by waking up
        return sleepState.getTransitionTime(sleepState.getCurrentState(), MODE_ACTIVE);
    }

    protected AtmelMicrocontroller(ClockDomain cd, AVRProperties p, FiniteStateMachine fsm) {
        super(cd, p.num_pins, p.getRegisterLayout().instantiate(), fsm);
        mainClock = cd.getMainClock();
        properties = p;
        devices = new HashMap<String, AtmelInternalDevice>();
    }

    /**
     * The <code>installIOReg()</code> method installs an IO register with the specified name. The register
     * layout for this microcontroller is used to get the address of the register (if it exists) and
     * install the <code>ActiveRegister</code> object into the correct place.
     * @param name the name of the IO register as a string
     * @param reg the register to install
     */
    protected ActiveRegister installIOReg(String name, ActiveRegister reg) {
        interpreter.installIOReg(properties.getIOReg(name), reg);
    return reg;
    }

    /**
     * The <code>getIOReg()</code> method gets a reference to the active register currently installed for
     * the specified name. The register layout for this microcontroller is used to get the correct address.
     * @param name the name of the IO register as a string
     * @return a reference to the active register object if it exists
     */
    protected ActiveRegister getIOReg(String name) {
        return interpreter.getIOReg(properties.getIOReg(name));
    }

    /**
     * The <code>addDevice()</code> method adds a new internal device to this microcontroller so that it can
     * be retrieved later with <code>getDevice()</code>
     * @param d the device to add to this microcontroller
     */
    protected void addDevice(AtmelInternalDevice d) {
        devices.put(d.name, d);
    }

    /**
     * The <code>getDevice()</code> method is used to get a reference to an internal device with the given name.
     * For example, the ADC device will be under the name "adc" and Timer0 will be under the name "timer0". This
     * is useful for external devices that need to connect to the input of internal devices.
     *
     * @param name the name of the internal device as a string
     * @return a reference to the internal device if it exists; null otherwise
     */
    public AtmelInternalDevice getDevice(String name) {
        return devices.get(name);
    }

    public static void addPin(HashMap<String, Integer> pinMap, int p, String n) {
        pinMap.put(n, new Integer(p));
    }

    public static void addPin(HashMap<String, Integer> pinMap, int p, String n1, String n2) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
    }

    public static void addPin(HashMap<String, Integer> pinMap, int p, String n1, String n2, String n3) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
    }

    public static void addPin(HashMap<String, Integer> pinMap, int p, String n1, String n2, String n3, String n4) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
        pinMap.put(n4, i);
    }

    public static void addPin(HashMap pinMap, int p, String n1, String n2, String n3, String n4, String n5) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
        pinMap.put(n4, i);
        pinMap.put(n5, i);
    }

    public static void addPin(HashMap pinMap, int p, String n1, String n2, String n3, String n4, String n5, String n6) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
        pinMap.put(n4, i);
        pinMap.put(n5, i);
        pinMap.put(n6, i);
    }

    public static void addPin(HashMap pinMap, int p, String n1, String n2, String n3, String n4, String n5, String n6, String n7) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
        pinMap.put(n4, i);
        pinMap.put(n5, i);
        pinMap.put(n6, i);
        pinMap.put(n7, i);
    }
 
    public static void addPin(HashMap pinMap, int p, String n1, String n2, String n3, String n4, String n5, String n6, String n7, String n8) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
        pinMap.put(n4, i);
        pinMap.put(n5, i);
        pinMap.put(n6, i);
        pinMap.put(n7, i);
        pinMap.put(n8, i);
    }

    public static void addPin(HashMap pinMap, int p, String n1, String n2, String n3, String n4, String n5, String n6, String n7, String n8, String n9) {
        Integer i = new Integer(p);
        pinMap.put(n1, i);
        pinMap.put(n2, i);
        pinMap.put(n3, i);
        pinMap.put(n4, i);
        pinMap.put(n5, i);
        pinMap.put(n6, i);
        pinMap.put(n7, i);
        pinMap.put(n8, i);
        pinMap.put(n9, i);
    }



    public static void addInterrupt(HashMap<String, Integer> iMap, String n, int i) {
        iMap.put(n, new Integer(i));
    }

    /**
     * The <code>getPin()</code> method looks up the named pin and returns a reference to that pin. Names of
     * pins should be UPPERCASE. The intended users of this method are external device implementors which
     * connect their devices to the microcontroller through the pins.
     *
     * @param n the name of the pin; for example "PA0" or "OC1A"
     * @return a reference to the <code>Pin</code> object corresponding to the named pin if it exists; null
     *         otherwise
     */
    public Microcontroller.Pin getPin(String n) {
        return pins[properties.getPin(n)];
    }

    /**
     * The <code>getProperties()</code> method gets a reference to the microcontroller properties for this
     * microcontroller instance.
     * @return a reference to the microcontroller properties for this instance
     */
    public MCUProperties getProperties() {
        return properties;
    }
    
    private enum InterruptType {
        
        /**
         * Low Level of pin generates an interrupt.
         */
         LowLevel ((0 << 1) | 0),
         
         /**
          * Any change of level generates an interrupt. (Only for EICRB interrupts).
          */
         AnyLevel ((0 << 1) | 1),
         
         /**
          * A falling edge causes an interrupt.
          */
         FallingEdge((1 << 1) | 0),
         
         /**
          * A raising edge causes an interrupt.
          */
         RisingEdge((1 << 1) | 1);
                 
         private int bitValue;
         
         InterruptType(int bitValue) {
           this.bitValue = bitValue;
         }
         
         /**
          * Returns the bit value of this setting.
          */
         public int getBitValue() {
             return bitValue;
         }
    }

    /**
     * The <code>INTPin</code> class implements a model of a pin of the Atmel family which can
     * cause Interrupts when a change in level occurs.
     */
    protected class INTPin extends Pin implements Pin.InputListener {
        
        /** 
         * Notification for events occurring on an InterruptTable. Used for handling repetitive LowLevel-interrupts. 
         */
        class InterruptTableNotification implements InterruptTable.Notification {
            
            /** 
             * Returns true if this notification is still valid and in use.
             */
            public boolean isStillValid() {
                if (EICRx_bits.getValue() != InterruptType.LowLevel.getBitValue()) {
                    return false; // InterruptType was changed
                }
                if (read() != false) {
                    return false; // no longer a low level.
                }
                return true;
            }
            
            private Notification underlyingNotification;

            public InterruptTableNotification(InterruptTable.Notification underlyingNotification) {
                this.underlyingNotification = underlyingNotification;
            }
            
            @Override
            public void force(int inum) {
                if (underlyingNotification != null) {
                    underlyingNotification.force(inum);
                }
            }

            @Override
            public void invoke(int inum) {
                if (underlyingNotification != null) {
                    underlyingNotification.invoke(inum);
                }
                
                if (isStillValid()) {
                    // Re-post the interrupt
                    EIFR_reg.flagBit(intNum);
                } 
                else {
                    
                    // Remove the notification, and restore the original one.
                    EIFR_reg.interpreter.getInterruptTable().registerInternalNotification(underlyingNotification, intNum);
                    
                }
            }
        }
        
        private boolean oldValue;
        private FlagRegister EIFR_reg;
        private int intNum;
        private BitRangeView EICRx_bits;
        private InterruptTableNotification notification;
        
        protected INTPin(int pinNum, FlagRegister eifr, int flagNum, BitRangeView eicrb) {
            super(pinNum);
            
            EIFR_reg = eifr;
            this.intNum = flagNum;
            EICRx_bits = eicrb;

            oldValue = read();
        }

        public void connectInput(Input i) {
            if (input != null) {
                input.unregisterListener(this);
            }
            super.connectInput(i);
            if (i != null) {
                try {
                    i.registerListener(this);
                } catch (UnsupportedOperationException ex) {
                    Printer.STDERR.println("[WARN] Input target "+i+" does not support Listeners. EIFR #"+intNum+" won't trigger.");
                }
                updateSensedLevel(read());
            }
        }

        protected void write(boolean value) {
            super.write(value);
            
            if (outputDir) {
                // Write to a PORT may also trigger an interrupt
                updateSensedLevel(read());
            }
        }

        public void onInputChanged(Input input, boolean newValue) {
            updateSensedLevel(newValue);
        }
        
        private boolean triggersInterrupt(boolean oldValue, boolean newValue) {
            int eicrb = EICRx_bits.getValue();
            
            if (eicrb == InterruptType.LowLevel.getBitValue()) { // no level change required
                if (!newValue) {
                    return true;
                }
            }
            
            if (oldValue == newValue) {
                return false;
            }
            
            
            if (eicrb == InterruptType.AnyLevel.getBitValue()) {
                // NOTE(mlinder): INT0-INT3 on ATMega128 should not necessarily support this ("reserved for future use")
                return true;
            } else if (eicrb == InterruptType.FallingEdge.getBitValue()) {
                if (oldValue && !newValue) {
                    return true;
                }
            } else if (eicrb == InterruptType.RisingEdge.getBitValue()) {
                if (!oldValue && newValue) {
                    return true;
                }
            }
            return false;
        }
        
        private void updateSensedLevel(boolean newValue) {
            // Check if we can trigger an interrupt			
            if (triggersInterrupt(oldValue, newValue)) {
                if (EICRx_bits.getValue() == InterruptType.LowLevel.getBitValue()) {
                    // Add a notification so that this interrupt is re-triggered
                    InterruptTable table = EIFR_reg.interpreter.getInterruptTable();
                    Notification oldNotification = table.getInternalNotification(intNum);
                    if (oldNotification != notification) {
                        notification = new InterruptTableNotification(oldNotification);
                        table.registerInternalNotification(notification, intNum); // Replace/Proxy the FlagRegister.Notification
                    }
                }
                
                // Modify the interrupt EIFR table
                EIFR_reg.flagBit(intNum); // use .flagBit as .setValue won't trigger an interrupt 
            }
            
            oldValue = newValue;
        }
    }
}
