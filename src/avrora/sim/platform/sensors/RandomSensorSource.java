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

import java.util.Random;

/**
 * Sensor source that generates random data.
 *
 * @author Ben L. Titzer
 * @author Enrico Jorns
 */
public class RandomSensorSource implements SensorSource {

    protected final Random random;
    protected final double lbound;
    protected final double ubound;

    /**
     * Random source that emits n random values.
     *
     * @param r Random instance
     * @param lbound Lower bound of data range to generate
     * @param ubound Uppper bound of data range to generate
     */
    public RandomSensorSource(Random r, double lbound, double ubound) {
        random = r;
        this.lbound = lbound;
        this.ubound = ubound;
    }

    /**
     * Random source that emits a random value in range [0.0, 1.0]
     *
     * @param r Random instance
     */
    public RandomSensorSource(Random r) {
        this(r, 0.0, 1.0);
    }

    @Override
    public double read(int idx) {
        return (ubound - lbound) * random.nextDouble() + lbound;
    }
}
