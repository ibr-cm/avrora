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
import avrora.sim.Simulator;
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
    final TransferEvent transferEvent = new TransferEvent();
    int interruptNum;
    int clock = 1;

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
        
    }

    @Override
    public void invoke(int inum) {
        
    }
    enum TransferMode {
        NULL, START, STOP, ADDRESS, DATA_WRITE, DATA_READ;
    };

    protected class TransferEvent implements Simulator.Event {

        boolean transmitting;
        int status;
        TransferMode mode = TransferMode.NULL;
        
        private byte data;
        private boolean write;
        private boolean rep_start;
        private boolean ack_flag;

        protected void enableWriteTransfer(byte data, boolean ack_flag) {
            if (transmitting) {
                return;
            }
            transmitting = true;
            this.data = data;
            this.ack_flag = ack_flag;
            mode = TransferMode.DATA_WRITE;
            mainClock.insertEvent(this, clock * 7);
    
        }

        protected void enableReadTransfer(boolean ack_flag) {
            if (transmitting) {
                return;
            }
            transmitting = true;
            this.ack_flag = ack_flag;
            mode = TransferMode.DATA_READ;
            mainClock.insertEvent(this, clock * 7);
        }

        protected void enableAddressTransfer(byte address, boolean write, boolean rep_start, boolean ack_flag) {
            //(byte) (TWDR_reg.value & 0xfe), write, rep, ack_flag);
            if (transmitting) {
                return;
            }
            transmitting = true;
            this.data = address;
            this.write = write;
            this.rep_start = rep_start;
            this.ack_flag = ack_flag;
            mode = TransferMode.ADDRESS;
            mainClock.insertEvent(this, clock * 7);
            
        }

        protected void enableStopTransfer() {
            if (transmitting) {
                return;
            }
            transmitting = true;
            mode = TransferMode.STOP;
            mainClock.insertEvent(this, clock * 2);
            
        }
        protected void enableStartTransfer(int status) {
            if (transmitting) {
                return;
            }
            transmitting = true;
            this.status = status;
            mode = TransferMode.START;
            mainClock.insertEvent(this, clock * 2);
        }

        @Override
        public void fire() {
            if (!transmitting) {
                return;
            }
            Boolean ack = null;
            switch (mode) {
                case NULL:
                    break;
                case START:
                    TWSR_reg.setStatus(status);
                    TWCR_reg.setValue(TWCR_reg.value | (1 << TWCReg.TWINT));
                    break;
                case STOP:
                    for (TWIDevice dev : devices) {
                        dev.stop();
                    }
                    TWCR_reg.setValue(TWCR_reg.value & ~(1 << TWCReg.TWSTO));
                    break;
                case ADDRESS:
                    
                    for (TWIDevice dev : devices) {
                        Boolean dev_ack = dev.start(data, write, rep_start, ack_flag);
                        if (dev_ack != null) {
                            ack = dev_ack;
                        }
                    }
                    if (ack != null) {
                        if (write) {
                            if (ack) {
                                TWSR_reg.setStatus(TWSReg.TWI_STATUS_SLA_W_ACK);
                            } else {
                                TWSR_reg.setStatus(TWSReg.TWI_STATUS_SLA_W_NACK);
                            }
                        } else {
                            if (ack) {
                                TWSR_reg.setStatus(TWSReg.TWI_STATUS_SLA_R_ACK);
                            } else {
                                TWSR_reg.setStatus(TWSReg.TWI_STATUS_SLA_R_NACK);
                            }
                        }
                    }
                    TWCR_reg.setValue(TWCR_reg.value | (1 << TWCReg.TWINT));
                    break;
                case DATA_READ:
                    Byte result = null;
                    for (TWIDevice dev : devices) {
                        TWIData result_data = dev.readByte(ack_flag);
                        if (result_data != null) {
                            ack = result_data.ack;
                            result = result_data.data;
                        }
                    }
                    if (result != null) {
                        TWDR_reg.setValue(result);
                    }
                    if (ack != null) {
                        if (ack) {
                            TWSR_reg.setStatus(TWSReg.TWI_STATUS_DATA_R_ACK);
                        } else {
                            TWSR_reg.setStatus(TWSReg.TWI_STATUS_DATA_R_NACK);
                        }
                    }
                    TWCR_reg.setValue(TWCR_reg.value | (1 << TWCReg.TWINT));
                    break;
                case DATA_WRITE:
                    for (TWIDevice dev : devices) {
                        Boolean dev_ack = dev.writeByte(data, ack_flag);
                        if (dev_ack != null) {
                            ack = dev_ack;
                        }
                    }
                    if (ack != null) {
                        if (ack) {
                            TWSR_reg.setStatus(TWSReg.TWI_STATUS_DATA_W_ACK);
                        } else {
                            TWSR_reg.setStatus(TWSReg.TWI_STATUS_DATA_W_NACK);
                        }
                    }
                    TWCR_reg.setValue(TWCR_reg.value | (1 << TWCReg.TWINT));
                    break;
                default:
            }
            mode = TransferMode.NULL;
            transmitting = false;
        }
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
        final RegisterView _interrupt = RegisterUtil.bitView(this, TWINT);
        /**
         * TWEA: TWI Enable Acknowledge Bit
         */
        final RegisterView _enable_ack = RegisterUtil.bitView(this, TWEA);

        /**
         * TWSTA: TWI START Condition Bit
         */
        final RegisterView _start = RegisterUtil.bitView(this, TWSTA);

        /**
         * TWSTO: TWI STOP Condition Bit
         */
        final RegisterView _stop = RegisterUtil.bitView(this, TWSTO);

        /**
         * TWWC: TWI Write Collision Flag
         */
        final RegisterView _write_collision = RegisterUtil.bitView(this, TWWC);

        /**
         * TWEN: TWI: Enable Bit
         */
        final RegisterView _enable = RegisterUtil.bitView(this, TWEN);

        /**
         * TWIE: TWI Interrupt Enable Bit
         */
        final RegisterView _interrupt_enable = RegisterUtil.bitView(this, TWIE);

        public TWCReg() {
            setValue(0x00);
        }
        boolean started = false;
        boolean rep_start = false;
        boolean next_is_addr = false;
        boolean write_mode = false;

        @Override
        public void write(byte val) {
            // Bits 1 and 3 are read-only
            val = (byte) ((this.value & ~write_mask) | (val & write_mask));

            if ((val & (1 << TWINT)) != 0) {
                val &= ~(1 << TWINT);
            }
            super.write(val);
            
            if (_enable.getValue() != 0) {
                if (_start.getValue() != 0) {
                    if (!started) {
                        transferEvent.enableStartTransfer(TWSReg.TWI_STATUS_START);
                        started = true;
                    } else {
                        transferEvent.enableStartTransfer(TWSReg.TWI_STATUS_START_REP);
                    }
                    next_is_addr = true;
                } else if (_stop.getValue() != 0) {
                    transferEvent.enableStopTransfer();
                    started = false;
                } else {
                    boolean ack_flag = (_enable_ack.getValue() != 0);
                    if (next_is_addr) {
                        next_is_addr = false;
                        write_mode = ((TWDR_reg.value & 1) == 0);
                        transferEvent.enableAddressTransfer((byte) (TWDR_reg.value & 0xfe), write_mode, rep_start, ack_flag);
                        rep_start = true;
                    } else {
                        if (write_mode) {
                            transferEvent.enableWriteTransfer(TWDR_reg.value, ack_flag);
                        } else {
                            transferEvent.enableReadTransfer(ack_flag);
                        }
                    }
                    
                }
            }
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

        final RegisterView _twps = RegisterUtil.bitRangeView(this, TWPS0, TWPS1);

        static final int write_mask = 0x03;

        public TWSReg() {
            setValue(TWI_STATUS_DEFAULT);
        }

        @Override
        public void write(byte val) {
            // only bits 0 and 1 are writeable
            super.write((byte) ((this.value & ~write_mask) | (val & write_mask)));
            updateClock();
        }

        public void setStatus(int status) {
            setValue((status | (0x03 & this.value)));
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

        @Override
        public void write(byte val) {
            super.write(val);
            updateClock();
        }

    }

    private void updateClock() {
        int twps = TWSR_reg._twps.getValue();
        int twbr = TWBR_reg.getValue();
        clock = 16 + 2 * (twbr) << (2 * twps);
    }

    public void connect(TWIDevice device) {
        if (!devices.contains(device)) {
            devices.add(device);
        }
    }
}
