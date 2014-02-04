/**
 * Copyright (c) 2013-2014, TU Braunschweig
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

package avrora.sim.util;

import avrora.sim.mcu.Microcontroller.Pin.Input;
import avrora.sim.mcu.Microcontroller.Pin.InputListener;
import avrora.sim.mcu.Microcontroller.Pin.Output;

/**
 *
 * @author S. Willenborg
 */
public class LCX138 {

    public final OPin A0 = new OPin("A0");
    public final OPin A1 = new OPin("A1");
    public final OPin A2 = new OPin("A2");

    public final OPin E1 = new OPin("E1");
    public final OPin E2 = new OPin("E2");
    public final OPin E3 = new OPin("E3");

    public final IPin O0 = new IPin("O0", 0);
    public final IPin O1 = new IPin("O1", 1);
    public final IPin O2 = new IPin("O2", 2);
    public final IPin O3 = new IPin("O3", 3);
    public final IPin O4 = new IPin("O4", 4);
    public final IPin O5 = new IPin("O5", 5);
    public final IPin O6 = new IPin("O6", 6);
    public final IPin O7 = new IPin("O7", 7);

   
    private int state = -1;
   
    private void update() {
        if (!E1.value && !E2.value && E3.value) {
            state = (A0.value ? 1 : 0) | (A1.value ? 2 : 0) | (A2.value ? 4 : 0);
        } else
            state = -1;
    }
    public class OPin implements Output {
        final String name;
        boolean value = false;

        public OPin(String name) {
            this.name = name;
        }

        @Override
        public void write(boolean level) {
            value = level;
            update();
        }
    }

    public class IPin implements Input {
        final String name;
        final int index;

        public IPin(String name, int index) {
            this.name = name;
            this.index = index;
        }

        @Override
        public boolean read() {
            return state != index;
        }

        @Override
        public void registerListener(InputListener listener) {
        }

        @Override
        public void unregisterListener(InputListener listener) {
        }

    }
}
