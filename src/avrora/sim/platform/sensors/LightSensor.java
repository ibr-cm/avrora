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

package avrora.sim.platform.sensors;

import avrora.sim.FiniteStateMachine;
import avrora.sim.mcu.*;

/**
 * The <code>LightSensor</code> class implements a light sensor.
 * 
 * This light sensor is connected to and ADC.
 *
 * @author Ben L. Titzer
 */
public class LightSensor extends Sensor {

    private SensorSource source;
    protected final AtmelMicrocontroller mcu;
    protected final int channel;

    protected final FiniteStateMachine fsm;

    protected static final String[] names = { "power down", "on" };
    protected boolean on;
    public ADC adcDevice;

    /**
     * Creates a light sensor.
     * 
     * @param m Microcontroller reference
     * @param adcChannel Adc channel to connect to
     * @param onPin Pin name to connect to
     */
    public LightSensor(AtmelMicrocontroller m, int adcChannel, String onPin) {
        super();
        mcu = m;
        channel = adcChannel;
        mcu.getPin(onPin).connectOutput(new OnPin());
        fsm = new FiniteStateMachine(mcu.getClockDomain().getMainClock(), 0, names, 0);
        adcDevice = (ADC)mcu.getDevice("adc");
        adcDevice.connectADCInput(new ADCInput(), channel);
    }

    final Channel[] channels = new Channel[]{
        new Channel("light", -1.0, -1.0, -1.0)
    };

    @Override
    public Channel[] getChannels() {
        return channels;
    }

    @Override
    public void setSensorSource(SensorSource src) {
        source = src;
    }

    class OnPin implements Microcontroller.Pin.Output {
        @Override
        public void write(boolean val) {
            on = val;
            fsm.transition(state());
        }
    }

    private int state() {
        if (on) {
            return 1;
        } else {
            return 0;
        }
    }

    class ADCInput implements ADC.ADCInput {

        @Override
        public float getVoltage() {
            if (!on) {
                return ADC.GND_LEVEL;
            }
            // fetch data
            double data = source.read(0);
            // scale the read back to a voltage.
            return (float) (adcDevice.getVoltageRef() * (data) / 0x3ff);
        }
    }
}
