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

package avrora.sim.platform;

import avrora.core.Program;
import avrora.sim.Simulator;
import avrora.sim.Simulation;
import avrora.sim.clock.ClockDomain;
import avrora.sim.mcu.*;
import avrora.sim.platform.sensors.LightSensor;
import avrora.sim.platform.sensors.SensorBoard;
//import avrora.sim.platform.sensors.AccelSensor;
//import avrora.sim.platform.sensors.AccelSensorPower;
import avrora.sim.radio.*;
import cck.text.Terminal;
import avrora.sim.types.SingleSimulation;

/**
 * The <code>Raven</code> class is an implementation of the <code>Platform</code> interface that represents
 * both a specific microcontroller and the devices connected to it. This implements the Atmel Raven
 * configuration, an ATMega1284p with SPI connection to an AT86RF230 radio, and no other peripherals.
 *
 * @author Ben L. Titzer, Daniel Lee, David Kopf
 */
public class Raven extends Platform {

    protected static final int MAIN_HZ = 8000000;
    protected static final Simulation mysim = null;
    public static class Factory implements PlatformFactory {

        /**
         * The <code>newPlatform()</code> method is a factory method used to create new instances of the
         * <code>Raven</code> class.
         * @param id the integer ID of the node
         * @param sim the simulation
         * @param p the program to load onto the node @return a new instance of the <code>Mica2</code> platform
         */
        public Platform newPlatform(int id, Program p) {
            ClockDomain cd = new ClockDomain(MAIN_HZ);
            cd.newClock("external", 32768);

            return new Raven(new ATMega1284p(id, new SingleSimulation(), cd, p));
        }
    }

    protected final Simulator sim;
    protected ATmega128RFA1Radio radio;
    protected LED.LEDGroup ledGroup;

    private Raven(Microcontroller m) {
        super(m);
        sim = m.getSimulator();
        addDevices();
    }

    /**
     * The <code>addDevices()</code> method is used to add the external (off-chip) devices to the
     * platform. The Raven 1284p has no external devices except for the SPI connected RF230 radio,
     * but LED's can be added for simulation debugging.
     * Warning: running such code on the hardware will drive the selected port outputs!
     */
    protected void addDevices() {
        LED red = new LED(sim, Terminal.COLOR_RED, "Red");
        LED green = new LED(sim, Terminal.COLOR_GREEN, "Green");
        LED blue = new LED(sim, Terminal.COLOR_BLUE, "Blue");



        ledGroup = new LED.LEDGroup(sim, new LED[] { red, green, blue });
        addDevice("leds", ledGroup);

        AtmelMicrocontroller amcu = (AtmelMicrocontroller)mcu;
        if (true) {
            mcu.getPin("PD5").connectOutput(blue);
            mcu.getPin("PD6").connectOutput(green);
            mcu.getPin("PD7").connectOutput(red);
        }
        // install the new AT86RF230 radio. Actually an AT86RF231.
        AT86RF231Radio radio = new AT86RF231Radio(mcu, MAIN_HZ * 2);
 //     mcu.getPin("PB7").connectOutput(radio.SCLK_pin);
        mcu.getPin(3).connectOutput(radio.SCLK_pin); //PB7
        mcu.getPin(1).connectOutput(radio.MOSI_pin); //PB5
        mcu.getPin(2).connectInput(radio.MISO_pin);  //PB6
        mcu.getPin(41).connectOutput(radio.RSTN_pin); //PB1
        mcu.getPin(43).connectOutput(radio.SLPTR_pin);//PB3
        mcu.getPin(44).connectOutput(radio.CS_pin);   //PB4

        SPI spi = (SPI)amcu.getDevice("spi");
        spi.connect(radio.spiInterface);
        addDevice("radio", radio);
        radio.RF231_interrupt = mcu.getProperties().getInterrupt("TIMER1 CAPT");
    }

}
