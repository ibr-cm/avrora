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
import avrora.sim.platform.sensors.AccelSensor;
import avrora.sim.platform.sensors.AccelSensorPower;
import avrora.sim.radio.*;
import cck.text.Terminal;
import avrora.sim.types.SingleSimulation;
/**
 * The <code>Tiny85</code> class is an implementation of the <code>Platform</code> interface that represents
 * both a specific microcontroller and the devices connected to it. This implementation therefore uses the
 * bare ATTiny85 microcontroller with uart and some LEDs connected.
 *
 * @author Ben L. Titzer, Daniel Lee
 */
public class Tiny85 extends Platform {

    protected static final int MAIN_HZ = 8000000;

    public static class Factory implements PlatformFactory {

        /**
         * The <code>newPlatform()</code> method is a factory method used to create new instances of the
         * <code>Tiny85</code> class.
         * @param id the integer ID of the node
         * @param sim the simulation
         * @param p the program to load onto the node
         * @return a new instance of the <code>Tiny85</code> platform
         */
         //             protected static final Simulation mysim = null;
        public Platform newPlatform(int id, Program p) {
            ClockDomain cd = new ClockDomain(MAIN_HZ);
          //  cd.newClock("external", 32768);
            return new Tiny85(new ATTiny85(id, new SingleSimulation(), cd, p));
        }
    }

    protected final Simulator sim;
    protected LED.LEDGroup ledGroup;

    private Tiny85(Microcontroller m) {
        super(m);
        sim = m.getSimulator();
        addDevices();
    }

    /**
     * The <code>addDevices()</code> method is used to add the external (off-chip) devices to the
     * platform. For the mica2, these include the LEDs, the radio, and the sensor board.
     */
    protected void addDevices() {
        LED red = new LED(sim, Terminal.COLOR_RED, "Red");
        LED green = new LED(sim, Terminal.COLOR_GREEN, "Green");
        LED blue = new LED(sim, Terminal.COLOR_BLUE, "Blue");
        ledGroup = new LED.LEDGroup(sim, new LED[] { red, green, blue });
        addDevice("leds", ledGroup);

        AtmelMicrocontroller amcu = (AtmelMicrocontroller)mcu;

        mcu.getPin("PB0").connectOutput(red);//pin 5
        mcu.getPin("PB4").connectOutput(green);//pin 3
        mcu.getPin("PB3").connectOutput(blue);//pin 2

        ADC adc = (ADC)((AtmelMicrocontroller)mcu).getDevice("adc");
        adc.VCC_LEVEL = 3.0f;
        adc.AREF_LEVEL = 3.0f;
        adc.VBG_LEVEL = 1.1f;
    //    SPI spi = (SPI)amcu.getDevice("spi");

        // install the input capture pin.
      //  Timer16Bit timer1 = (Timer16Bit)amcu.getDevice("timer1");

    }

}
