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

import avrora.sim.InterruptTable;
import avrora.sim.RWRegister;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import java.util.LinkedList;

/**
 *
 * @author swillenborg
 */
public class TWI extends AtmelInternalDevice implements InterruptTable.Notification {

    final TWBReg TWBR_reg;
    final TWCReg TWCR_reg;
    final TWSReg TWSR_reg;
    final TWDReg TWDR_reg;
    final TWAReg TWAR_reg;
    final TWAMReg TWAMR_reg;
    final LinkedList<TWIDevice> devices = new LinkedList<>();
    int interruptNum;
    int period = 1;

    public TWI(AtmelMicrocontroller m) {
        super("twi", m);
        TWBR_reg = new TWBReg();
        TWCR_reg = new TWCReg();
        TWSR_reg = new TWSReg();
        TWDR_reg = new TWDReg();
        TWAR_reg = new TWAReg();
        TWAMR_reg = new TWAMReg();

        interruptNum = m.getProperties().getInterrupt("TWI");

        installIOReg("TWBR", TWBR_reg);
        installIOReg("TWCR", TWCR_reg);
        installIOReg("TWSR", TWSR_reg);
        installIOReg("TWDR", TWDR_reg);
        installIOReg("TWAR", TWAR_reg);
        installIOReg("TWAMR", TWAMR_reg);
        interpreter.getInterruptTable().registerInternalNotification(this, interruptNum);


    }

    @Override
    public void force(int inum) {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void invoke(int inum) {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    /**
     * TWCR - TWI Control Register
     */
    private class TWCReg extends RWRegister {

        static final int TWINT = 7;
        static final int TWEA = 6;
        static final int TWSTA = 5;
        static final int TWSTO = 4;
        static final int TWWC = 3;
        static final int TWEN = 2;
        static final int TWIE = 0;
        static final int write_mask = 0xf5;
        /**
         * TWINT: TWI Interrupt Flag
         */
        final RegisterView _int = RegisterUtil.bitView(this, TWINT);
        /**
         * TWEA: TWI Enable Acknowledge Bit
         */
        final RegisterView _ea = RegisterUtil.bitView(this, TWEA);

        /**
         * TWSTA: TWI START Condition Bit
         */
        final RegisterView _sta = RegisterUtil.bitView(this, TWSTA);

        /**
         * TWSTO: TWI STOP Condition Bit
         */
        final RegisterView _sto = RegisterUtil.bitView(this, TWSTO);

        /**
         * TWWC: TWI Write Collision Flag
         */
        final RegisterView _wc = RegisterUtil.bitView(this, TWWC);

        /**
         * TWEN: TWI: Enable Bit
         */
        final RegisterView _en = RegisterUtil.bitView(this, TWEN);

        /**
         * TWIE: TWI Enable Bit
         */
        final RegisterView _ie = RegisterUtil.bitView(this, TWIE);

        public TWCReg() {
            setValue(0x00);
        }
        boolean started = false;
        boolean rep = false;
        boolean addr = false;
        boolean write = false;
        @Override
        public void write(byte val) {
            val = (byte) ((this.value & ~write_mask) | (val & write_mask));
            if ((val & (1 << TWINT)) != 0) {
                val &= ~(1 << TWINT);
            }
            super.write(val);
            
            if (_en.getValue() != 0) {
                

                if (_sta.getValue() != 0) {
                    
                    
                    if (!started) {
                        //System.out.printf("Do start\n");
                        TWSR_reg.setValue(TWSReg.TWI_STATUS_START);
                        started = true;
                    } else {
                        //System.out.printf("Do rep start\n");
                        TWSR_reg.setValue(TWSReg.TWI_STATUS_START_REP);

                    }
                    addr = true;
                    val |= (1 << TWINT);
                } else if (_sto.getValue() != 0) {
                    //System.out.printf("Do stop\n");
                    for (TWIDevice dev : devices) {
                        dev.stop();
                    }
                    val = (byte) (val & ~(1 << TWSTO));
                    started = false;
                } else {
                    //System.out.print(" twea " + _ea.getValue());
                    boolean ack_flag = (_ea.getValue() != 0);
                    Boolean ack = null;
                    if (addr) {
                        addr = false;
                        write = ((TWDR_reg.value & 1) == 0);
                        //System.out.printf(" Do addr %x, write %s\n", (int) (TWDR_reg.value) & 0xfe, write);
                        
                        
                        for (TWIDevice dev : devices) {
                            Boolean dev_ack = dev.start((byte) ((TWDR_reg.value) & 0xfe), rep, ack_flag);
                            if (dev_ack != null) {
                                ack = dev_ack;
                            }
                        rep = true;

                        }
                        if (write) {

                            if (ack) {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_SLA_W_ACK);
                            } else {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_SLA_W_NACK);
                            }
                        } else {
                            if (ack) {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_SLA_R_ACK);
                            } else {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_SLA_R_NACK);
                            }
                        }
                    } else {
                        if (write) {
                            for (TWIDevice dev : devices) {
                                Boolean dev_ack = dev.writeByte(TWDR_reg.value, ack_flag);
                                if (dev_ack != null) {
                                    ack = dev_ack;
                                }
                            }
                            //System.out.printf(" Write data %x\n", (int) TWDR_reg.value & 0xff);
                            if (ack) {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_DATA_W_ACK);
                            } else {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_DATA_W_NACK);
                            }

                        } else {
                            Byte result = null;
                            for (TWIDevice dev : devices) {
                                TWIData data = dev.readByte(ack_flag);

                                if (data != null) {
                                    ack = data.ack;
                                    result = data.data;
                                    //TWDR_reg.setValue(data.data);
                                    
                                }
                            }
                            if (result != null) {
                                TWDR_reg.setValue(result);
                                //System.out.printf(" read data %x\n", (int) result & 0xff);
                            } else {
                                //System.out.printf(" read data ?\n");
                            }
                            if (ack) {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_DATA_R_ACK);
                            } else {
                                TWSR_reg.setValue(TWSReg.TWI_STATUS_DATA_R_NACK);
                            }
                        }
                    }
                    val |= (1 << TWINT);
                    /*if (_ea.getValue() != 0) {
                     TWSR_reg.setValue(status_ack);
                     } else {
                        TWSR_reg.setValue(status_nack);
                     }*/
                    
                }
                // start 2
                // data 9
                // stop 2
            }
            // Bits 1 and 3 are read-only
            super.write(val);
            
        }

    }

    /**
     * TWSR - TWI Status register
     */
    private class TWSReg extends RWRegister {

        /**
         * Bus error due to an illegal START or STOP condition
         */
        static final int TWI_STATUS_ERROR = 0x0;

        /**
         * A START condition has been transmitted
         */
        static final int TWI_STATUS_START = 0x8;

        /**
         * A repeated START condition has been transmitted
         */
        static final int TWI_STATUS_START_REP = 0x10;

        /**
         * SLA+W has been transmitted; ACK has been received
         */
        static final int TWI_STATUS_SLA_W_ACK = 0x18;

        /**
         * SLA+W has been transmitted; NOT ACK has been received
         */
        static final int TWI_STATUS_SLA_W_NACK = 0x20;

        /**
         * Data byte has been transmitted; ACK has been received
         */
        static final int TWI_STATUS_DATA_W_ACK = 0x28;

        /**
         * Data byte has been transmitted; NOT ACK has been received
         */
        static final int TWI_STATUS_DATA_W_NACK = 0x30;

        /**
         * Arbitration lost in SLA+W or data bytes
         */
        static final int TWI_STATUS_ARBITRATION_LOST = 0x38;

        /**
         * SLA+R has been transmitted; ACK has been received
         */
        static final int TWI_STATUS_SLA_R_ACK = 0x40;

        /**
         * SLA+R has been transmitted; NOT ACK has been received
         */
        static final int TWI_STATUS_SLA_R_NACK = 0x48;

        /**
         * Data byte has been received; ACK has been returned
         */
        static final int TWI_STATUS_DATA_R_ACK = 0x50;

        /**
         * Data byte has been received; NOT ACK has been returned
         */
        static final int TWI_STATUS_DATA_R_NACK = 0x58;

        /**
         * Own SLA+W has been received; ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_SLA_W_ACK = 0x60;

        /**
         * Arbitration lost in SLA+R/W as Master; own SLA+W has been received;
         * ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_SLA_W_ACK_ARBITRATION_LOST = 0x68;

        /**
         * General call address has been received; ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_ADDRESS_ACK = 0x70;

        /**
         * Arbitration lost in SLA+R/W as Master; General call address has been
         * received; ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_ADDRESS_ACK_ARBITRATION_LOST = 0x78;

        /**
         * Previously addressed with own SLA+W; data has been received; ACK has
         * been returned
         */
        static final int TWI_STATUS_SLAVE_SLA_W_ADDRESSED_ACK = 0x80;

        /**
         * Previously addressed with own SLA+W; data has been received; NOT ACK
         * has been returned
         */
        static final int TWI_STATUS_SLAVE_SLA_W_ADDRESSED_NACK = 0x88;

        /**
         * Previously addressed with general call; data has been received; ACK
         * has been returned
         */
        static final int TWI_STATUS_SLAVE_GENERAL_DATA_ACK = 0x90;

        /**
         * Previously addressed with general call; data has been received; NOT
         * ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_GENERAL_DATA_NACK = 0x98;

        /**
         * A STOP condition or repeated START condition has been received while
         * still addressed as Slave
         */
        static final int TWI_STATUS_SLAVE_STOP_OR_REP_START = 0xa0;

        /**
         * Own SLA+R has been received; ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_SLA_R_ACK = 0xa8;

        /**
         * Arbitration lost in SLA+R/W as Master; own SLA+R has been received;
         * ACK has been returned
         */
        static final int TWI_STATUS_SLAVE_SLA_R_ACK_ARBITRATION_LOST = 0xb0;

        /**
         * Data byte in TWDR has been transmitted; ACK has been received
         */
        static final int TWI_STATUS_SLAVE_TWDR_SENT_ACK = 0xb8;

        /**
         * Data byte in TWDR has been transmitted; NOT ACK has been received
         */
        static final int TWI_STATUS_SLAVE_TWDR_SENT_NACK = 0xc0;

       
        /**
         * No relevant state information available; TWINT = “0”
         */
        static final int TWI_STATUS_DEFAULT = 0xf8;


        static final int TWS7 = 7;
        static final int TWS6 = 6;
        static final int TWS5 = 5;
        static final int TWS4 = 4;
        static final int TWS3 = 3;

        static final int TWPS1 = 1;
        static final int TWPS0 = 0;
        static final int write_mask = 0x03;

        public TWSReg() {
            setValue(TWI_STATUS_DEFAULT);
        }

        @Override
        public void write(byte val) {
            // only bits 0 and 1 are writeable
            super.write((byte) ((this.value & ~write_mask) | (val & write_mask)));
        }

    }

    /**
     * TWDR - TWI Data Register
     */
    private class TWDReg extends RWRegister {

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
    private class TWAReg extends RWRegister {
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
    private class TWAMReg extends RWRegister {
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
            super.write((byte) ((this.value & ~write_mask) | (val & write_mask)));
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

    public void connect(TWIDevice device) {
        if (!devices.contains(device)) {
            devices.add(device);
        }
    }
}
