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

import avrora.sim.*;
import avrora.sim.state.*;
import avrora.arch.avr.AVRProperties;

/**
 * The <code>ADC</code> class represents an on-chip device on the ATMega series of microcontroller that is
 * capable of converting an analog voltage value into a 10-bit digital value.
 *
 * @author Daniel Lee
 * @author Ben L. Titzer
 */
public class ADC extends AtmelInternalDevice {
    /* Default values for Vcc, Aref, bandgap, ground in volts */
    public static float VCC_LEVEL  = 3.3f;
    public static float AREF_LEVEL = 3.3f;
    public static float V256_LEVEL = 2.56f;
    public static float VBG_LEVEL  = 1.0f;
    public static float GND_LEVEL  = 0.0f;
    private static float voltageRef;

    private final ADCInput VCC_INPUT = new ADCInput() {
        public float getVoltage() {
       //     System.out.println("VCC_INPUT returns "+VCC_LEVEL);
            return VCC_LEVEL;
        }
    };   
    private final ADCInput VBG_INPUT = new ADCInput() {
        public float getVoltage() {
       //     System.out.println("VBG_INPUT returns "+VBG_LEVEL);
            return VBG_LEVEL;
        }
    };
    private static final ADCInput GND_INPUT = new ADCInput() {
        public float getVoltage() {
       //     System.out.println("GND_LEVEL returns "+GND_LEVEL);
            return GND_LEVEL;
        }
    };

    final MUXRegister ADMUX_reg = new MUXRegister();
    final ControlRegister ADCSRA_reg = new ControlRegister();
    final RWRegister ADCH_reg = new RWRegister();
    final RWRegister ADCL_reg = new RWRegister();

    final int channels;
    final int interruptNum;
    final int ioregSize; //Used to determine whether ATTiny or ATMega

    final ADCInput[] connectedDevices;

    /**
     * The <code>ADCInput</code> interface is used by inputs into the analog to digital converter.
     */
    public interface ADCInput {

        /**
         * Report the current voltage level of the input.
         *
         * @return an integer value representing the voltage level of the input, in millivolts
         */
        public float getVoltage();
    }


    public ADC(AtmelMicrocontroller m, int channels) {
        super("adc", m);

        this.channels = channels;

        connectedDevices = new ADCInput[channels + 3];

        // the last three channels correspond to Vcc, VBG, and GND
        // Application can overwrite these connected devices for dynamic changing,
        // or change the default values they return.
        connectedDevices[channels + 0] = VCC_INPUT;
        connectedDevices[channels + 1] = VBG_INPUT;
        connectedDevices[channels + 2] = GND_INPUT;

        interruptNum = m.getProperties().getInterrupt("ADC");
        ioregSize = ((AVRProperties)m.getProperties()).ioreg_size;
        if (ioregSize <=64 ) System.out.println("ADC: Assuming this is an ATTiny85");

        installIOReg("ADMUX", ADMUX_reg);
        installIOReg("ADCH", ADCH_reg);
        installIOReg("ADCL", ADCL_reg);
        installIOReg("ADCSRA", ADCSRA_reg);

        interpreter.getInterruptTable().registerInternalNotification(ADCSRA_reg, interruptNum);
    }
    /**
     * The <code>setVoltageRef()</code> method sets the external (Vref) voltage for the ADC converter.
     * @param vref the voltage reference in volts
     */
    public void setVoltageRef(float vref) {
        System.out.println("setvoltageref passed " + vref);
        voltageRef = vref;
    }

    /**
     * The <code>getVoltageRef()</code> method returns the external (Vref) voltage that is currently
     * being used by the ADC converter.
     * @return the voltage reference in volts
     */
    public float getVoltageRef() {
    System.out.println("getvoltageref returns " + voltageRef);
        return voltageRef;
    }

    /**
     * The <code>connectADCInput()</code> method connects an <code>ADCInput</code> object to the specified
     * input port on the ADC chip.
     *
     * @param input the <code>ADCInput</code> object to attach to the input
     * @param num   the input port number to attach the device to
     */
    public void connectADCInput(ADCInput input, int num) {
        connectedDevices[num] = input;
    }

    static final byte[] SINGLE_ENDED_INPUT = {
             0,  1,  2,  3,  4,  5,  6,  7,
            -1, -1, -1, -1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1, -1,  9, 10
    };

    static final short[] GAIN = {
             -1, -1,  -1,  -1,  -1,  -1,  -1,  -1,
             10, 10, 200, 200,  10,  10, 200, 200,
             1,   1,   1,   1,   1,   1,   1,   1,
             1,   1,   1,   1,   1,   1,  -1,  -1
    };

    static final byte[] POS_INPUT = {
            -1, -1, -1, -1, -1, -1, -1, -1,
             0,  1,  0,  1,  2,  3,  2,  3,
             0,  1,  2,  3,  4,  5,  6,  7,
             0,  1,  2,  3,  4,  5, -1, -1
    };

    static final byte[] NEG_INPUT = {
            -1, -1, -1, -1, -1, -1, -1, -1,
             0,  0,  0,  0,  2,  2,  2,  2,
             1,  1,  1,  1,  1,  1,  1,  1,
             2,  2,  2,  2,  2,  2, -1, -1
    };

//ATTiny85 - only 16 mux values
    static final byte[] TINY_SINGLE_ENDED_INPUT = {
             0,  1,  2,  3, -1, -1, -1, -1,
            -1, -1, -1, -1,  6,  7, -1,  4
    };

    static final short[] TINY_GAIN = {
             -1, -1, -1,  -1,  1, 20,  1,  20,
             1,  20,  1,  20, -1, -1, -1, -1
    };

    static final byte[] TINY_POS_INPUT = {
            -1, -1, -1, -1, 2, 2, 2, 2,
             0,  0,  0,  0, -1, -1, -1, -1
    };

    static final byte[] TINY_NEG_INPUT = {
            -1, -1, -1, -1, 2, 2, 3, 3,
             0,  0,  1,  1,-1, -1, -1, -1
    };


    /**
     * <code>MUXRegister</code> defines the behavior of the ADMUX register.
     */
    protected class MUXRegister extends RWRegister {

        final RegisterView _mux = RegisterUtil.bitRangeView(this, 0, 4);
        final RegisterView _ref = RegisterUtil.bitRangeView(this, 6, 2);

        float getReference() {
            if (ioregSize > 64) {
                //ATMega
                int value = _ref.getValue();
                if (value == 0) return AREF_LEVEL;  //AREF, internal vref off
                if (value == 1) return connectedDevices[channels].getVoltage();      //AVCC with capacitor on AREF pin
                if (value == 2) return VBG_LEVEL;   //internal bandgap with capacitor on AREF (not on all megas)
                if (value == 3) return V256_LEVEL;  //internal 2v56 with capacitor on AREF
            } else {
                //ATTiny
                int value = _ref.getValue() & 0x03;
                if (value == 0) return connectedDevices[channels].getVoltage();  //Vcc disconnected from AREF
                if (value == 1) return AREF_LEVEL; //AREF, internal vref off
                //ATTiny uses bit 4 for REFS2
                value |= ((_mux.getValue() & 0x10) >> 2);
                if (value == 2) return VBG_LEVEL; //internal bandgap with capacitor on AREF
            //  if (value == 3) return 0;//reserved?
            //  if (value == 6) return V256_LEVEL;  //2.56 without external capacitor
            //  if (value == 7) return V256_LEVEL;   //2.56 with external capacitor             
                return V256_LEVEL;
            }
            return 0;
        }

        boolean isSingleEnded() {
            return getSingleIndex() >= 0;
        }

        int getSingleIndex() {
            if (ioregSize > 64)
                return SINGLE_ENDED_INPUT[_mux.getValue()];
            else
                return TINY_SINGLE_ENDED_INPUT[_mux.getValue()];
        }

        int getPosIndex() {
            if (ioregSize > 64)
                return POS_INPUT[_mux.getValue()];
            else
                return TINY_POS_INPUT[_mux.getValue()];
        }

        int getNegIndex() {
            if (ioregSize > 64)
                return NEG_INPUT[_mux.getValue()];
            else
                return TINY_NEG_INPUT[_mux.getValue()];
        }

        int getGain() {
            if (ioregSize > 64)
                return GAIN[_mux.getValue()];
            else
                return TINY_GAIN[_mux.getValue()];
        }
    }

    static final short[] PRESCALER = { 2, 2, 4, 8, 16, 32, 64, 128 };

    /**
     * <code>ControlRegister</code> defines the behavior of the ADC control register,
     */
    protected class ControlRegister extends RWRegister implements InterruptTable.Notification {

        final ConversionEvent conversion = new ConversionEvent();

        final BooleanView _aden = RegisterUtil.booleanView(this, 7);
        final BooleanView _adsc = RegisterUtil.booleanView(this, 6);
        final BooleanView _adfr = RegisterUtil.booleanView(this, 5);//ADATE on attiny, assumes ADTS2:0 = 0 for free running mode
        final BooleanView _adif = RegisterUtil.booleanView(this, 4);
        final BooleanView _adie = RegisterUtil.booleanView(this, 3);
        final RegisterView _prescaler = RegisterUtil.bitRangeView(this, 0, 2);

        int cycles = 25;

        boolean converting;

        private void unpostADCInterrupt() {
            _adif.setValue(false);
            interpreter.setPosted(interruptNum, false);
        }

        public void write(byte nval) {
            //System.out.println("ADCwrite " + nval);
            value = nval;

            if (_aden.getValue()) {
                // if enabled and start conversion
                if (_adsc.getValue()) startConversion();
            } else {
                // else, stop conversion
                stopConversion();
            }

            // reset the flag bit if written by the user
            if ( _adif.getValue() ) unpostADCInterrupt();

            // enable the interrupt if the flag is set
            interpreter.setEnabled(interruptNum, _adie.getValue());
        }

        private void startConversion() {
            if ( !converting ) {
                // queue event for converting
                converting = true;
                insertConversion();
            }
        }

        private void insertConversion() {
            mainClock.insertEvent(conversion, getPrescaler() * cycles);
            /*
            if (ADMUX_reg.isSingleEnded()) {
                if (devicePrinter != null) {
                    devicePrinter.println("ADC: beginning sample of channel " + ADMUX_reg.getSingleIndex());
                }
            } else {
                if (devicePrinter != null) {
                    devicePrinter.println("ADC: beginning sample of channels " + ADMUX_reg.getPosIndex() + " - " + ADMUX_reg.getNegIndex());
                }
            }
            */
            cycles = 13;
        }

        private void stopConversion() {
            _adsc.setValue(false);
            if ( converting ) {
                converting = false;
                mainClock.removeEvent(conversion);
            }
        }

        private int getPrescaler() {
     //   System.out.println("getprescaler returns " + PRESCALER[_prescaler.getValue()]);
            return PRESCALER[_prescaler.getValue()];
        }

        /**
         * The conversion event for the ADC. It is fired at a certain delay after the start conversion bit
         * in the control register is set.
         */
        private class ConversionEvent implements Simulator.Event {

            public void fire() {

                int val = convertVoltage();
                write16(val, ADCH_reg, ADCL_reg);
        //        if (devicePrinter != null) {
         //           devicePrinter.println("ADC: conversion completed -> " + val);
        //        }
                _adif.setValue(true);
                //value = Arithmetic.setBit(value, ADIF, adif = true);
                interpreter.setPosted(interruptNum, true);

                if ( _adfr.getValue() ) {
                    // in free running mode, start the next conversion
                    insertConversion();
                } else {
                    // otherwise, stop conversion
                    stopConversion();
                }
            }
        }

        private int convertVoltage() {
            if (ADMUX_reg.isSingleEnded()) {
                // single ended conversions don't require amplification
                ADCInput dev = connectedDevices[ADMUX_reg.getSingleIndex()];
                float vin = dev != null ? dev.getVoltage() : 0;
                float vref = ADMUX_reg.getReference();
          //      System.out.println("ADC_" + ADMUX_reg.getSingleIndex() + " " + vin + "/" + vref);
                if ( vin >= vref ) return 0x3ff;
                return 0x3ff & (int)(vin * 1024 / vref);
            } else {
                // use the differential gain amplifier.
                ADCInput pos = connectedDevices[ADMUX_reg.getPosIndex()];
                ADCInput neg = connectedDevices[ADMUX_reg.getNegIndex()];
                float vpos = pos != null ? pos.getVoltage() : 0;
                float vneg = neg != null ? neg.getVoltage() : 0;
                float vref = ADMUX_reg.getReference();
                float val = ((vpos - vneg) * ADMUX_reg.getGain() * 512 / vref);
                System.out.println("ADC_"+ADMUX_reg.getPosIndex()+"-"+ADMUX_reg.getNegIndex()+" (" + vpos + "-" + vneg + ")/" + vref + "*" + ADMUX_reg.getGain() + " = " + val);
                if ( val < -512 ) return 0x3ff;
                if ( val > 511 ) return 0x1ff;
                return 0x3ff & (int)val;
            }
        }


        public void force(int inum) {
            // set the interrupt flag accordingly
            System.err.println("setadcinterrupt");
            _adif.setValue(true);
        }

        public void invoke(int inum) {
            System.err.println("invokeadcinterrupt");
            unpostADCInterrupt();
        }
    }
}
