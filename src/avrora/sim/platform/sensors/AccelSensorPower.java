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
 * The <code>AccelSensorPower</code> handles the Power pin for the acceleration sensor
 *
 * @author Ben L. Titzer
 * @author Zainul M. Charbiwala
 * @author Daniel Minder
 */
public class AccelSensorPower {

    protected final FiniteStateMachine fsm;

    protected static final String[] names = { "power down", "on" };
    protected boolean on;

    public AccelSensorPower(AtmelMicrocontroller m, String onPin) {
        m.getPin(onPin).connectOutput(new OnPin());
        fsm = new FiniteStateMachine(m.getClockDomain().getMainClock(), 0, names, 0);
    }

    class OnPin implements Microcontroller.Pin.Output {
        public void write(boolean val) {
            on = val;
            fsm.transition(state());
        }
    }

    private int state() {
        if ( !on ) return 0;
        else return 1;
    }
    
    public boolean isOn() {
      return on;
    }
}
