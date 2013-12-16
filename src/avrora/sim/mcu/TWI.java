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

import avrora.sim.RWRegister;

/**
 *
 * @author swillenborg
 */
class TWI extends AtmelInternalDevice {

    TWBReg TWBR_reg;
    TWCReg TWCR_reg;
    TWSReg TWSR_reg;
    TWDReg TWDR_reg;
    TWAReg TWAR_reg;
    TWAMReg TWAMR_reg;

    public TWI(AtmelMicrocontroller m) {
        super("TWI", m);
        TWBR_reg = new TWBReg();
        TWCR_reg = new TWCReg();
        TWSR_reg = new TWSReg();
        TWDR_reg = new TWDReg();
        TWAR_reg = new TWAReg();
        TWAMR_reg = new TWAMReg();

        installIOReg("TWBR", TWBR_reg);
        installIOReg("TWCR", TWCR_reg);
        installIOReg("TWSR", TWSR_reg);
        installIOReg("TWDR", TWDR_reg);
        installIOReg("TWAR", TWAR_reg);
        installIOReg("TWAMR", TWAMR_reg);

    }
    /**
     * TWCR - TWI Control Register
     */
    private static class TWCReg extends RWRegister {

        static final int TWINT = 7;
        static final int TWEA = 6;
        static final int TWSTA = 5;
        static final int TWSTO = 4;
        static final int TWWC = 3;
        static final int TWEN = 2;
        static final int TWIE = 0;
        static final int write_mask = 0xF5;

        public TWCReg() {
            setValue(0x00);
        }

        @Override
        public void write(byte val) {
            
            // TODO: Mark transmission as stoped
            val = (byte) (val & ~(1 << TWSTO));
            // Bits 1 and 3 are read-only
            super.write((byte)((val & ~write_mask) | (val & write_mask)));
            
        }

    }

    /**
     * TWSR - TWI Status register
     */
    private static class TWSReg extends RWRegister {

        static final int TWS7 = 7;
        static final int TWS6 = 6;
        static final int TWS5 = 5;
        static final int TWS4 = 4;
        static final int TWS3 = 3;

        static final int TWPS1 = 1;
        static final int TWPS0 = 0;
        static final int write_mask = 0x03;

        public TWSReg() {
            setValue(0xF8);
        }

        @Override
        public void write(byte val) {
            // only bits 0 and 1 are writeable
            super.write((byte)((val & ~write_mask) | (val & write_mask)));
        }

    }

    /**
     * TWDR - TWI Data Register
     */
    private static class TWDReg extends RWRegister {

        static final int TWD7 = 7;
        static final int TWD6 = 6;
        static final int TWD5 = 5;
        static final int TWD4 = 4;
        static final int TWD3 = 3;
        static final int TWD2 = 2;
        static final int TWD1 = 1;
        static final int TWD0 = 0;

        public TWDReg() {
            setValue(0xFF);
        }
    }

    /**
     * TWI - TWI (Slave) Address Register
     */
    private static class TWAReg extends RWRegister {
        static final int TWA6 = 7;
        static final int TWA5 = 6;
        static final int TWA4 = 5;
        static final int TWA3 = 4;
        static final int TWA2 = 3;
        static final int TWA1 = 2;
        static final int TWA0 = 1;
        static final int TWGCE = 0;
        
        public TWAReg() {
            setValue(0xFE);
        }
    }

    /**
     * TWAMR - TWI (slave) Address Mask Register
     */
    private static class TWAMReg extends RWRegister {
        static final int TWAM6 = 7;
        static final int TWAM5 = 6;
        static final int TWAM4 = 5;
        static final int TWAM3 = 4;
        static final int TWAM2 = 3;
        static final int TWAM1 = 2;
        static final int TWAM0 = 1;

        static final int write_mask = 0xFE;
        public TWAMReg() {
            setValue(0x00);
        }

        @Override
        public void write(byte val) {
            // bit 0 is read only
            super.write((byte)((val & ~write_mask) | (val & write_mask)));
        }

    }

    /**
     * TWBR - TWI Bit Rate Register
     */
    class TWBReg extends RWRegister {
        static final int TWBR7 = 7;
        static final int TWBR6 = 6;
        static final int TWBR5 = 5;
        static final int TWBR4 = 4;
        static final int TWBR3 = 3;
        static final int TWBR2 = 2;
        static final int TWBR1 = 1;
        static final int TWBR0 = 0;

        public TWBReg() {
            setValue(0x00);
        }
    }

}
