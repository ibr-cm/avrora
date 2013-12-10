/**
 * Copyright (c) 2007, Regents of the University of California
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
 * THIS SOFTWARE ISIS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
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
 *
 * Created Oct 10, 2007
 */
package avrora.sim.radio;

import java.util.Arrays;
import java.util.Random;

import avrora.sim.FiniteStateMachine;
import avrora.sim.Simulator;
import avrora.sim.clock.Synchronizer;
import avrora.sim.energy.Energy;
import avrora.sim.mcu.ADC;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.output.SimPrinter;
import avrora.sim.state.BooleanRegister;
import avrora.sim.state.BooleanView;
import avrora.sim.state.ByteFIFO;
import avrora.sim.state.Register;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import cck.text.StringUtil;
import cck.util.Arithmetic;
import cck.util.Util;

/**
 * The <code>CC2420Radio</code> implements a simulation of the CC2420 radio
 * chip.The CC2420 radio is used with the Micaz and telosb platforms.
 * Verbose printers for this class include "radio.cc2420"
 *
 * @author Ben L. Titzer
 * @author Rodolfo de Paz
 */
public class CC2420Radio implements Radio {

    //-- Register addresses ---------------------------------------------------
    public static final int MAIN = 0x10;
    public static final int MDMCTRL0 = 0x11;
    public static final int MDMCTRL1 = 0x12;
    public static final int RSSI = 0x13;
    public static final int SYNCWORD = 0x14;
    public static final int TXCTRL = 0x15;
    public static final int RXCTRL0 = 0x16;
    public static final int RXCTRL1 = 0x17;
    public static final int FSCTRL = 0x18;
    public static final int SECCTRL0 = 0x19;
    public static final int SECCTRL1 = 0x1a;
    public static final int BATTMON = 0x1b;
    public static final int IOCFG0 = 0x1c;
    public static final int IOCFG1 = 0x1d;
    public static final int MANFIDL = 0x1e;
    public static final int MANFIDH = 0x1f;
    public static final int FSMTC = 0x20;
    public static final int MANAND = 0x21;
    public static final int MANOR = 0x22;
    public static final int AGCCTRL0 = 0x23;
    public static final int AGCTST0 = 0x24;
    public static final int AGCTST1 = 0x25;
    public static final int AGCTST2 = 0x26;
    public static final int FSTST0 = 0x27;
    public static final int FSTST1 = 0x28;
    public static final int FSTST2 = 0x29;
    public static final int FSTST3 = 0x2a;
    public static final int RXBPFTST = 0x2b;
    public static final int FSMSTATE = 0x2c;
    public static final int ADCTST = 0x2d;
    public static final int DACTST = 0x2e;
    public static final int TOPTST = 0x2f;
    public static final int TXFIFO = 0x3e;
    public static final int RXFIFO = 0x3f;

    //-- Command strobes ---------------------------------------------------
    public static final int SNOP = 0x00;
    public static final int SXOSCON = 0x01;
    public static final int STXCAL = 0x02;
    public static final int SRXON = 0x03;
    public static final int STXON = 0x04;
    public static final int STXONCCA = 0x05;
    public static final int SRFOFF = 0x06;
    public static final int SXOSCOFF = 0x07;
    public static final int SFLUSHRX = 0x08;
    public static final int SFLUSHTX = 0x09;
    public static final int SACK = 0x0a;
    public static final int SACKPEND = 0x0b;
    public static final int SRXDEC = 0x0c;
    public static final int STXENC = 0x0d;
    public static final int SAES = 0x0e;

    //-- Other constants --------------------------------------------------
    private static final int NUM_REGISTERS = 0x40;
    private static final int FIFO_SIZE = 128;
    private static final int RAMSECURITYBANK_SIZE = 113;

    private static final int XOSC_START_TIME = 1000;// oscillator start time

    //-- Simulation objects -----------------------------------------------
    protected final Microcontroller mcu;
    protected final Simulator sim;

    //-- Radio state ------------------------------------------------------
    protected final int xfreq;
    protected final char[] registers = new char[NUM_REGISTERS];
    protected final byte[] RAMSecurityRegisters = new byte[RAMSECURITYBANK_SIZE];
    protected final ByteFIFO txFIFO = new ByteFIFO(FIFO_SIZE);
    protected final ByteFIFO rxFIFO = new ByteFIFO(FIFO_SIZE);
    protected double BERtotal = 0.0D;
    protected int BERcount = 0;

    protected Medium medium;
    protected Transmitter transmitter;
    protected Receiver receiver;

    //-- Strobes and status ----------------------------------------------
    // note that there is no actual "status register" on the CC2420.
    // The register here is used in the simulation implementation to
    // simplify the handling of radio states and state transitions.
    protected final Register statusRegister = new Register(8);
    protected boolean startingOscillator = false;

    //-- Views of bits in the status "register" ---------------------------
    protected final BooleanView oscStable = RegisterUtil.booleanView(statusRegister, 6);
    protected final BooleanView txUnderflow = RegisterUtil.booleanView(statusRegister, 5);
    protected final BooleanView txActive = RegisterUtil.booleanView(statusRegister, 3);
    protected final BooleanView signalLock = RegisterUtil.booleanView(statusRegister, 2);
    protected final BooleanView rssiValid = RegisterUtil.booleanView(statusRegister, 1);
    //-- Views of bits in the status "register" ---------------------------
    protected final RegisterView MDMCTRL0_reg = new RegisterUtil.CharArrayView(registers, MDMCTRL0);
    protected final BooleanView autoACK = RegisterUtil.booleanView(MDMCTRL0_reg, 4);
    protected final BooleanView autoCRC = RegisterUtil.booleanView(MDMCTRL0_reg, 5);
    protected final BooleanView ADR_DECODE = RegisterUtil.booleanView(MDMCTRL0_reg, 11);
    protected final BooleanView PAN_COORDINATOR = RegisterUtil.booleanView(MDMCTRL0_reg, 12);
    protected final BooleanView RESERVED_FRAME_MODE = RegisterUtil.booleanView(MDMCTRL0_reg, 13);

    protected final RegisterView IOCFG0_reg = new RegisterUtil.CharArrayView(registers, IOCFG0);
    protected final BooleanView BCN_ACCEPT = RegisterUtil.booleanView(IOCFG0_reg, 11);

    protected final BooleanView CCA_assessor = new ClearChannelAssessor();
    protected BooleanView SFD_value = new BooleanRegister();

    //-- Pins ------------------------------------------------------------
    public final CC2420Pin SCLK_pin = new CC2420Pin("SCLK");
    public final CC2420Pin MISO_pin = new CC2420Pin("MISO");
    public final CC2420Pin MOSI_pin = new CC2420Pin("MOSI");
    public final CC2420Pin CS_pin = new CC2420Pin("CS");
    public final CC2420Pin VREN_pin = new CC2420Pin("VREN");
    public final CC2420Pin RSTN_pin = new CC2420Pin("RSTN");
    public final CC2420Output FIFO_pin = new CC2420Output("FIFO", new BooleanRegister());
    public final CC2420Output FIFOP_pin = new CC2420Output("FIFOP", new BooleanRegister());
    public final CC2420Output CCA_pin = new CC2420Output("CCA", CCA_assessor);
    public final CC2420Output SFD_pin = new CC2420Output("SFD", SFD_value);

    public final SPIInterface spiInterface = new SPIInterface();
    public final ADCInterface adcInterface = new ADCInterface();

    public int FIFOP_interrupt = -1;

    protected final SimPrinter printer,myprinter;

    // the CC2420 allows reversing the polarity of these outputs.
    protected boolean FIFO_active;// selects active high (true) or active low.
    protected boolean FIFOP_active;
    protected boolean CCA_active;
    protected boolean SFD_active;

    //Acks variables
    public static final int SENDACK_NONE = 0;
    public static final int SENDACK_NORMAL = 1;
    public static final int SENDACK_PEND = 2;
    protected int SendAck;
    protected boolean AutoAckPend;
    protected boolean lastCRCok;
    protected byte DSN;

    //Address recognition variables
    protected byte[] PANId;
    protected byte[] macPANId;
    protected byte[] ShortAddr;
    protected byte[] macShortAddr;
    protected static final byte[] SHORT_BROADCAST_ADDR = {-1, -1};
    protected byte[] LongAdr;
    protected byte[] IEEEAdr;
    protected static final byte[] LONG_BROADCAST_ADDR = {-1, -1, -1, -1, -1, -1, -1, -1};

    //LUT from cubic spline interpolation with all transmission power values
    protected static final double [] POWER_dBm = {-37.917,-32.984,-28.697,-25,
    -21.837,-19.153,-16.893,-15,-13.42,-12.097,-10.975,-10,-9.1238,-8.3343,
    -7.6277,-7,-6.4442,-5.9408,-5.467,-5,-4.5212,-4.0275,-3.5201,-3,-2.4711,
    -1.9492,-1.4526,-1,-0.6099,-0.3008,-0.0914,0};

    //LUT for max and min correlation values depending on PER
    protected static final int [] Corr_MAX = {110,109,109,109,107,107,107,107,107,
    107,107,107,103,102,102,102,101,101,101,101,99,94,92,94,101,97,98,97,97,97,97,97,
    94,94,94,94,94,94,94,94,94,94,94,94,92,89,89,89,89,89,88,88,88,88,88,86,86,86,
    86,86,86,86,86,86,85,85,85,85,85,85,83,83,83,83,83,83,83,83,79,78,78,78,78,78,
    76,76,76,74,74,74,74,74,74,74,74,74,74,66,65,65,65};
    protected static final int [] Corr_MIN = {95,95,94,91,90,90,89,89,89,88,88,88,82,
    82,82,82,76,76,76,76,76,76,74,74,74,74,74,74,72,72,72,72,72,72,72,72,69,69,69,69,
    69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,67,67,67,67,67,67,65,65,65,65,65,
    65,65,64,64,63,63,63,63,63,63,63,63,63,61,61,61,60,60,60,58,58,56,56,56,55,55,55,
    50,50,50,50,50,50,50};
    protected double Correlation;

    //CC2420Radio energy
    protected static final String[] allModeNames = CC2420Energy.allModeNames();
    protected static final int[][] ttm = FiniteStateMachine.buildSparseTTM(allModeNames.length, 0);
    protected final FiniteStateMachine stateMachine;

    //Clear TxFIFO flag boolean value
    protected boolean ClearFlag;

    /**
     * The constructor for the CC2420 class creates a new instance connected
     * to the specified microcontroller with the given external clock frequency.
     *
     * @param mcu   the microcontroller unit to which this radio is attached
     * @param xfreq the external clock frequency supplied to the CC2420 radio chip
     */
    public CC2420Radio(Microcontroller mcu, int xfreq) {
        // set up references to MCU and simulator
        this.mcu = mcu;
        this.sim = mcu.getSimulator();
        this.xfreq = xfreq;

        // create a private medium for this radio
        // the simulation may replace this later with a new one.
        setMedium(createMedium(null, null));

        //setup energy recording
        stateMachine = new FiniteStateMachine(sim.getClock(), CC2420Energy.startMode, allModeNames, ttm);
        new Energy("Radio", CC2420Energy.modeAmpere, stateMachine, sim.getEnergyControl());

        // reset all registers
        reset();

        // get debugging channel.
      myprinter = sim.getPrinter("radio.cc2420");
    printer = null;
    }

    /**
     * The <code>getFiniteStateMachine()</code> method gets a reference to the finite state
     * machine that represents this radio's state. For example, there are states corresponding
     * to "on", "off", "transmitting", and "receiving". The state names and numbers will vary
     * by radio implementation. The <code>FiniteStateMachine</code> instance allows the user
     * to instrument the state transitions in order to gather information during simulation.
     * @return a reference to the finite state machine for this radio
     */
    public FiniteStateMachine getFiniteStateMachine() {
        return stateMachine;
    }

    private void reset() {
        for (int cntr = 0; cntr < NUM_REGISTERS; cntr++) {
            resetRegister(cntr);
        }

        // clear FIFOs.
        txFIFO.clear();
        rxFIFO.clear();

        // reset the status register
        statusRegister.setValue(0);

        // restore default CCA and SFD values.
        CCA_pin.level = CCA_assessor;
        SFD_pin.level = SFD_value;

        FIFO_active = true;// the default is active high for all of these pins
        FIFOP_active = true;
        CCA_active = true;
        SFD_active = true;

        SendAck = SENDACK_NONE;  // reset these internal variables
        AutoAckPend = false;
        lastCRCok = false;
        ClearFlag = false;

        // reset pins.
        FIFO_pin.level.setValue(!FIFO_active);
        FIFOP_pin.level.setValue(!FIFOP_active);

        transmitter.shutdown();
        receiver.shutdown();
    }

    public void setSFDView(BooleanView sfd) {
        if (SFD_pin.level == SFD_value) {
            SFD_pin.level = sfd;
        }
        SFD_value = sfd;
    }

    /**
     * The <code>readRegister()</code> method reads the value from the specified register
     * and takes any action(s) that are necessary for the specific register.
     *
     * @param addr the address of the register
     * @return an integer value representing the result of reading the register
     */
    public int readRegister(int addr) {
        int val = registers[addr];
        if (printer != null) {
            printer.println("CC2420 " + regName(addr) + " => " + StringUtil.toMultirepString(val, 16));
        }
        return val;
    }

    /**
     * The <code>writeRegister()</code> method writes the specified value to the specified
     * register, taking any action(s) necessary and activating any command strobes as
     * required.
     *
     * @param addr the address of the register
     * @param val  the value to write to the specified register
     */
    void writeRegister(int addr, int val) {
        if (printer != null) {
            printer.println("CC2420 " + regName(addr) + " <= " + StringUtil.toMultirepString(val, 16));
        }
        registers[addr] = (char) val;
        switch (addr) {
            case MAIN:
                if ((val & 0x8000) == 0) {
                    reset();
                    stateMachine.transition(1);//change to power down state
                }
                break;
            case IOCFG1:
                int ccaMux = val & 0x1f;
                int sfdMux = (val >> 5) & 0x1f;
                setCCAMux(ccaMux);
                setSFDMux(sfdMux);
                break;
            case IOCFG0:
                // set the polarities for the output pins.
                FIFO_active = !Arithmetic.getBit(val, 10);
                FIFOP_active = !Arithmetic.getBit(val, 9);
                SFD_active = !Arithmetic.getBit(val, 8);
                CCA_active = !Arithmetic.getBit(val, 7);
                break;
        }
        computeStatus();
    }

    private void setSFDMux(int sfdMux) {
        // TODO: SFD multiplexor
        if (sfdMux != 0) {
            throw Util.unimplemented();
        }
    }

    private void setCCAMux(int ccaMux) {
        // TODO: handle all the possible CCA multiplexing sources
        // and possibility of active low.
        if (ccaMux == 24) CCA_pin.level = oscStable;
        else {
            if (ccaMux != 0) {
                throw Util.unimplemented();
            }
            CCA_pin.level = CCA_assessor;
        }
    }

    void strobe(int addr) {
     //   if (printer != null) {
      //      printer.println("CC2420 Strobe " + strobeName(addr));
      //  }
        if (!oscStable.getValue()) {
            if (addr == SXOSCON) {
                startOscillator();
            }
        }
        else {
            switch (addr) {
                case SNOP:
                    break;
                case SXOSCON:
                    // was handled above
                    break;
                case STXCAL:
                    break;
                case SRXON:
                    if (!txActive.getValue()){
                        // should not interrupt transmissions according to state machine
                        // after a transmission RX mode is turned on automatically
                        transmitter.shutdown();
                        receiver.startup();
                    }
                    break;
                case STXONCCA:
                    if (CCA_assessor.getValue()) {
                        receiver.shutdown();
                        transmitter.startup();
                    }
                    break;
                case STXON:
                    receiver.shutdown();
                    transmitter.startup();
                    break;
                case SRFOFF:
                    //change to idle state
                    receiver.shutdown();
                    transmitter.shutdown();
                    stateMachine.transition(2);  // idle state
                    break;
                case SXOSCOFF:
                    // stop the tickers in receiver and transmitter
                    receiver.shutdown();
                    transmitter.shutdown();
                    // it is not clear from the data sheet if we have to do more here, e.g. resetting the FIFO pins
                    oscStable.setValue(false);
                    stateMachine.transition(1);//change to power down state
                    break;
                case SFLUSHRX:
                    rxFIFO.clear();
                    receiver.resetOverflow();
                    FIFO_pin.level.setValue(!FIFO_active);
                    FIFOP_pin.level.setValue(!FIFOP_active);
                    SFD_value.setValue(!SFD_active);  // needed in case of an overflow
                    break;
                case SFLUSHTX:
                    txFIFO.clear();
                    txUnderflow.setValue(false);
                    break;
                case SACK:
                    AutoAckPend = false;  // AutoAck sends pending flag until SACK is issued
                    if (!receiver.inPacket()) {
                        // if reception is over ACK is only sent when the CRC was ok
                        if (lastCRCok) {
                            SendAck = SENDACK_NORMAL;
                            receiver.shutdown();
                            transmitter.startup();
                        }
                    }
                    else {
                        // otherwise it might be sent when packet is complete
                        SendAck = SENDACK_NORMAL;
                    }
                    break;
                case SACKPEND:
                    AutoAckPend = true;  // AutoAck sends pending flag until SACK is issued
                    if (!receiver.inPacket()) {  // see SACK
                        if (lastCRCok) {
                            SendAck = SENDACK_PEND;
                            receiver.shutdown();
                            transmitter.startup();
                        }
                    }
                    else {
                        SendAck = SENDACK_PEND;
                    }
                    break;
                case SRXDEC:
                    // start RXFIFO in-line decryption/authentication as set by SPI_SEC_MODE
                    throw Util.unimplemented();
                case STXENC:
                    // start TXFIFO in-line encryption/authentication as set by SPI_SEC_MODE
                    throw Util.unimplemented();
                case SAES:
                    // SPI_SEC_MODE is not required to be 0, but the encrypt. module must be idle; else strobe is ignored
                    throw Util.unimplemented();
            }
        }
    }

    private void startOscillator() {
        if (!oscStable.getValue() && !startingOscillator) {
            startingOscillator = true;
            sim.insertEvent(new Simulator.Event() {
                public void fire() {
                    if (startingOscillator) {  // just in case the voltage regulator has been switched off in the meantime
                        oscStable.setValue(true);
                        startingOscillator = false;
                        stateMachine.transition(2);//change to idle state
                        if (printer != null) {
                            printer.println("CC2420 Oscillator established");
                        }
                    }
                }
            }, toCycles(XOSC_START_TIME));
        }
    }

    /**
     * The <code>resetRegister()</code> method resets the specified register's value
     * to its default.
     *
     * @param addr the address of the register to reset
     */
    void resetRegister(int addr) {
        char val = 0x0000;
        switch (addr) {
            case MAIN:
                val = 0xf800;
                break;
            case MDMCTRL0:
                val = 0x0ae2;
                break;
            case RSSI:
                val = 0xe080;
                break;
            case SYNCWORD:
                val = 0xa70f;
                break;
            case TXCTRL:
                val = 0xa0ff;
                break;
            case RXCTRL0:
                val = 0x12e5;
                break;
            case RXCTRL1:
                val = 0x0a56;
                break;
            case FSCTRL:
                val = 0x4165;
                break;
            case IOCFG0:
                val = 0x0040;
                break;
        }
        registers[addr] = val;
    }

    /**
     * The <code>computeStatus()</code> method computes the status byte of the radio.
     */
    void computeStatus() {
        // do nothing.
    }

    protected static final int CMD_R_REG = 0;
    protected static final int CMD_W_REG = 1;
    protected static final int CMD_R_RX = 2;
    protected static final int CMD_W_RX = 3;
    protected static final int CMD_R_TX = 4;
    protected static final int CMD_W_TX = 5;
    protected static final int CMD_R_RAM = 6;
    protected static final int CMD_W_RAM = 7;

    //-- state for managing configuration information
    protected int configCommand;
    protected int configByteCnt;
    protected int configRegAddr;
    protected byte configByteHigh;
    protected int configRAMAddr;
    protected int configRAMBank;

    protected byte receiveConfigByte(byte val) {
        configByteCnt++;
        if (configByteCnt == 1) {
            // the first byte is the address byte
            byte status = getStatus();
            if (Arithmetic.getBit(val, 7)) {
                // RAM/register bit is set, i.e. RAM should be accessed
                // byte2 determines R or R/W!
                configCommand = CMD_R_RAM;
                configRAMAddr = val & 0x7f;
            }
            else {
                // command strobes or register access
                boolean readop = Arithmetic.getBit(val, 6);
                configRegAddr = val & 0x3f;
                if (configRegAddr <= 15) {
                    // execute the command strobe
                    strobe(configRegAddr);
                    configByteCnt = 0;
                } else {
                    if (configRegAddr == TXFIFO) configCommand = readop ? CMD_R_TX : CMD_W_TX;
                    else if (configRegAddr == RXFIFO) configCommand = readop ? CMD_R_RX : CMD_W_RX;
                    else configCommand = readop ? CMD_R_REG : CMD_W_REG;
                }
            }
            return status;
        } else if (configByteCnt == 2) {
            if (!oscStable.getValue() && configCommand != CMD_R_REG && configCommand != CMD_W_REG) {
                // with crystal oscillator disabled only register access is possible
                return 0;
            }
            // the second byte is the MSB for a write, unused for read
            switch (configCommand) {
                case CMD_R_REG:
                    return Arithmetic.high(readRegister(configRegAddr));
                case CMD_W_REG:
                    configByteHigh = val;
                    return 0;
                case CMD_R_TX:
                    return readFIFO(txFIFO);
                case CMD_R_RX:
                    return readFIFO(rxFIFO);
                case CMD_W_TX:
                    return writeFIFO(txFIFO, val, true);
                case CMD_W_RX:
                    return writeFIFO(rxFIFO, val, false);
                case CMD_R_RAM:
                    configRAMBank = (val >> 6) & 0x3;
                    if (!Arithmetic.getBit(val, 5))
                        configCommand = CMD_W_RAM;
                    return 0;
            }
        } else {
            if (!oscStable.getValue() && configCommand != CMD_R_REG && configCommand != CMD_W_REG) {
                // with crystal oscillator disabled only register access is possible
                return 0;
            }
            // the third byte completes a read or write register
            // while subsequent bytes are valid for fifo and RAM accesses
            switch (configCommand) {
                case CMD_R_REG:
                    configByteCnt = 0;
                    return Arithmetic.low(readRegister(configRegAddr));
                case CMD_W_REG:
                    configByteCnt = 0;
                    writeRegister(configRegAddr, Arithmetic.word(val, configByteHigh));
                    return 0;
                case CMD_R_TX:
                    return readFIFO(txFIFO);
                case CMD_R_RX:
                    return readFIFO(rxFIFO);
                case CMD_W_TX:
                    return writeFIFO(txFIFO, val, true);
                case CMD_W_RX:
                    return writeFIFO(rxFIFO, val, false);
                case CMD_R_RAM:
                    byte retval;
                    if (configRAMBank == 0x00)
                        retval = txFIFO.peek(configRAMAddr);
                    else if (configRAMBank == 0x01)
                        retval = rxFIFO.peek(configRAMAddr);
                    else if (configRAMBank == 0x02)
                        retval = ReadSecurityBank(configRAMAddr);
                    else
                        retval = 0;
                    configRAMAddr++;
                    return retval;
                case CMD_W_RAM:
                    if (configRAMBank == 0x00) {
                        retval = txFIFO.poke(configRAMAddr, val);
                    }
                    else if (configRAMBank == 0x01)
                        retval = rxFIFO.poke(configRAMAddr, val);
                    else if (configRAMBank == 0x02)
                        retval = WriteSecurityBank(configRAMAddr, val);
                    else
                        retval = 0;
                    configRAMAddr++;
                    return retval;
            }
        }
        return 0;
    }

    private byte getStatus() {
        byte status = (byte) statusRegister.getValue();
    //    if (printer != null) {
  //          printer.println("CC2420 status: " + StringUtil.toBin(status, 8));
   //     }
        return status;
    }

    protected byte ReadSecurityBank(int address) {
        int value = RAMSecurityRegisters[address];
        if (printer != null) {
            printer.println("CC2420 " + SecurityRAMName(address) + "(addr " + StringUtil.to0xHex(address, 2) + ") -> " + StringUtil.toMultirepString(value, 8));
        }
        return (byte) value;
    }

    protected byte WriteSecurityBank(int address, byte value) {
        if (printer != null) {
            printer.println("CC2420 " + SecurityRAMName(address) + "(addr " + StringUtil.to0xHex(address, 2) + ") <= " + StringUtil.toMultirepString(value, 8));
        }
        RAMSecurityRegisters[address] = value;
        //If RAM PANId = 0xffff set IOCFG0.BCN_ACCEPT
        // Changed: NO! The data sheet says that BCN_ACCEPT should be set when PANId=0xffff, but it's not set automatically!
//        if ((RAMSecurityRegisters[104] == 255) && (RAMSecurityRegisters[105] == 255)) {
//            BCN_ACCEPT.setValue(true);
//        }
        return value;
    }

    protected byte readFIFO(ByteFIFO fifo) {
        byte val = fifo.remove();
        if (printer != null) {
            printer.println("CC2420 Read " + fifoName(fifo) + " -> " + StringUtil.toMultirepString(val, 8));
        }
        if (fifo == rxFIFO) {
            if (fifo.empty()) {
                // reset the FIFO pin when the read FIFO is empty.
                FIFO_pin.level.setValue(!FIFO_active);
            }
            if (fifo.size() < getFIFOThreshold()) {
                // reset FIFOP pin when the number of bytes in the FIFO is below threshold
                FIFOP_pin.level.setValue(!FIFOP_active);
            }
        }
        return val;
    }

    protected byte writeFIFO(ByteFIFO fifo, byte val, boolean st) {
        if (printer != null) {
            printer.println("CC2420 Write " + fifoName(fifo) + " <= " + StringUtil.toMultirepString(val, 8));
        }
        byte result = st ? getStatus() : 0;
        if (fifo == txFIFO && getClearFlag()){  // clear flag is only valid for txFIFO
            fifo.clear();
            ClearFlag = false;
        }
        fifo.add(val);
        computeStatus();
        return result;
    }

    protected boolean getClearFlag(){
        return ClearFlag;
    }

    protected void setClearFlag(){
        ClearFlag = true;
    }

    private int getFIFOThreshold() {
        // get the FIFOP_THR value from the configuration register
        return registers[IOCFG0] & 0x7f;
    }

    public Simulator getSimulator() {
        return sim;
    }

    public double getPower() {
        //return power in dBm
        return POWER_dBm[(readRegister(TXCTRL) & 0x1f)];
    }

    public double getFrequency() {
        //return frequency in Mhz
        return (2048 + (readRegister(FSCTRL) & 0x03ff));
    }

    public class ClearChannelAssessor implements BooleanView {
        public void setValue(boolean val) {
             // ignore writes.
        }

        public boolean getValue() {
            if (!receiver.getRssiValid())
                // CC2420 sets the pin inactive when RSSI is invalid!
                return false;
            else
                return receiver.isChannelClear(readRegister(RSSI),readRegister(MDMCTRL0));
        }
    }

    public class SPIInterface implements SPIDevice {

        public SPI.Frame exchange(SPI.Frame frame) {
    //        if (printer != null) {
    //            printer.println("CC2420 new SPI frame exchange " + StringUtil.toMultirepString(frame.data, 8));
     //       }
            if (!CS_pin.level && VREN_pin.level && RSTN_pin.level) {
                // configuration requires CS pin to be held low, and VREN pin and RSTN pin to be held high
                return SPI.newFrame(receiveConfigByte(frame.data));
            } else {
                return SPI.newFrame((byte) 0);
            }
        }

        public void connect(SPIDevice d) {
            // do nothing.
        }
    }

    public class ADCInterface implements ADC.ADCInput {

        public float getVoltage() {
            throw Util.unimplemented();
        }
    }

    private void pinChange_CS(boolean level) {
        // a change in the CS level always restarts a config command.
        configByteCnt = 0;
    }

    private void pinChange_VREN(boolean level) {
        if (level) {
            // the voltage regulator has been switched on
                if (printer != null) {
                    printer.println("CC2420 VREN current state = "+stateMachine.getCurrentState());
                }

            if (stateMachine.getCurrentState() == 0) {
                // actually, there is a startup time for the voltage regulator
                // but we assume here that it starts immediately
                stateMachine.transition(1);//change to power down state
                if (printer != null) {
                    printer.println("CC2420 Voltage Regulator started");
                }
            }
        }
        else {
            if (stateMachine.getCurrentState() > 0) {
                // switch the chip off, but stop all things first
                startingOscillator = false;
                oscStable.setValue(false);
                transmitter.shutdown();
                receiver.shutdown();
                stateMachine.transition(0);//change to off state
                if (printer != null) {
                    printer.println("CC2420 Voltage Regulator switched off");
                }
            }
        }
    }

    private void pinChange_RSTN(boolean level) {
        if (!level) {
            // high->low indicates reset
            reset();
            stateMachine.transition(1);//change to power down state
            if (printer != null) {
                printer.println("CC2420 reset by pin");
            }
        }
    }

    // TODO: According to the CC2420 data sheet a 0xF in syncword means that
    // this symbol is ignored (or transmitted as 0). Therefore, for the default
    // syncword 00 7A should be transmitted (and not 0F 7A). The receiver should
    // look for a 0 symbol, then for the syncword, but also ignoring 0xF. Therefore,
    // for the default syncword 00 7A should be searched. Since for the default
    // syncword this might result only in a minor difference I decided not to change it now.

    private static final int TX_IN_PREAMBLE = 0;
    private static final int TX_SFD_1 = 1;
    private static final int TX_SFD_2 = 2;
    private static final int TX_LENGTH = 3;
    private static final int TX_IN_PACKET = 4;
    private static final int TX_CRC_1 = 5;
    private static final int TX_CRC_2 = 6;
    private static final int TX_END = 7;
    private static final int TX_WAIT = 8;

   protected static final int[] reverse_bits = {
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8, 0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4, 0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec, 0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2, 0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea, 0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6, 0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee, 0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1, 0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9, 0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5, 0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed, 0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, 0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb, 0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7, 0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef, 0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff};

    public class Transmitter extends Medium.Transmitter {

        protected int state;
        protected int counter;
        protected int length;
        protected short crc;
        protected boolean wasAck;

        public Transmitter(Medium m) {
            super(m, sim.getClock());
        }

        public byte nextByte() {
            byte val = 0;
            switch (state) {
                case TX_IN_PREAMBLE:
                    counter++;
                    if (counter >= getPreambleLength()) {
                        state = TX_SFD_1;
                    }
                    break;
                case TX_SFD_1:
                    state = TX_SFD_2;
                                        val = 0;
                   // val = Arithmetic.low(registers[SYNCWORD]);//this is an 0x0f
                    break;
                case TX_SFD_2:
                    state = TX_LENGTH;
                    val = 0x7a; //sky uses this
                 //   val = Arithmetic.high(registers[SYNCWORD]);
                    break;
                case TX_LENGTH:
                    if (SendAck != SENDACK_NONE) {//ack frame
                        wasAck = true;
                        length = 5;
                    } else {//data frame
                        wasAck = false;
                        txFIFO.saveState();  // save FIFO state for later refill
                        length = txFIFO.remove() & 0x7f;
                    }
                    state = TX_IN_PACKET;
                    counter = 0;
                    crc = 0;
                    val = (byte) length;
                    // remark: it is not clear from the data sheet if SFD becomes active for an ACK frame. Probably not...
                    SFD_value.setValue(SFD_active);
                    break;
                case TX_IN_PACKET:
                    if (SendAck == SENDACK_NONE) {//data frame
                        if (txFIFO.empty()) {
                            if (printer != null) {
                              printer.println("CC2420 txFIFO underflow");
                            }
                            // a transmit underflow has occurred. set the flag and stop transmitting.
                            txUnderflow.setValue(true);
                            val = 0;
                            state = TX_END;
                            break;
                        }
                        //  no underflow occurred.
                        val = txFIFO.remove();
                        counter++;
                    } else {//ack frame
                        switch (counter) {
                            case 0://FCF_low
                                if (SendAck == SENDACK_NORMAL) {
                                    val = 2;
                                    break;
                                } else if (SendAck == SENDACK_PEND) {
                                    val = 0x12;  //  type ACK + frame pending flag
                                    break;
                                }
                            case 1://FCF_hi
                                val = 0;
                                break;
                            case 2://Sequence number
                                val = DSN;
                                SendAck = SENDACK_NONE;
                                break;
                        }
                        counter++;
                    }
                    //Calculate CRC and switch state if necessary
                    if (autoCRC.getValue()) {
                        // accumulate CRC if enabled.
                    //    crc = crcAccumulate(crc, val);
                        crc = crcAccumulate(crc, (byte) reverse_bits[(val) & 0xff]);
                        if (counter >= length - 2) {
                            // switch to CRC state if when 2 bytes remain.
                            state = TX_CRC_1;
                        }
                    } else if (counter >= length) {
                        // AUTOCRC not enabled, switch to packet end mode when done.
                        state = TX_END;
                    }
                    break;
                case TX_CRC_1:
                    state = TX_CRC_2;
                    val = Arithmetic.high(crc);
                    val = (byte) reverse_bits[(val) & 0xff];
                    break;
                case TX_CRC_2:
                    val = Arithmetic.low(crc);
                    val = (byte) reverse_bits[(val) & 0xff];
                    state = TX_END;
                    break;
            }
            if (printer != null) {
                printer.println("CC2420 " + StringUtil.to0xHex(val, 2) + " --------> ");
            }
            // common handling of end of transmission
            if (state == TX_END) {
                // actually, this should be set low AFTER the crc, but we are not called at that time...
                SFD_value.setValue(!SFD_active);

                if (!wasAck && !txUnderflow.getValue()) {  //data frame only and only when no underflow
                    //After complete tx of data frame the txFIFO is automatically refilled
                    txFIFO.refill();
                    //writing txFIFO after frame transmitted will cause it to be flushed
                    setClearFlag();
                }

                // auto transition back to receive mode.
                shutdown();
                receiver.startup();// auto transition back to receive mode.

                // transmitter stays in this state until it is really switched off
                state = TX_WAIT;
            }
            return val;
        }

        private int getPreambleLength() {
            int val = registers[MDMCTRL0] & 0xf;
            return val + 1;
        }

        void startup() {
            if (!txActive.getValue()){
                // the PLL lock time is implemented as the leadCycles in Medium!
                txActive.setValue(true);
                stateMachine.transition((readRegister(TXCTRL) & 0x1f)+4);//change to Tx(Level) state
                state = TX_IN_PREAMBLE;
                counter = 0;
                beginTransmit(getPower(),getFrequency());
                if (printer != null) {
                    printer.println("CC2420 TX started");
                }
            }
        }

        void shutdown() {
            // note that the stateMachine.transition() is not called here any more!
            txActive.setValue(false);
            endTransmit();
            if (printer != null) {
                printer.println("CC2420 TX shutdown");
            }
        }
    }

    int crcAccumulatet(int crc, byte val) {
	    crc = ((crc >> 8) & 0xff) | (crc << 8) & 0xffff;
	    crc ^= (val & 0xff);
	    crc ^= (crc & 0xff) >> 4;
	    crc ^= (crc << 12) & 0xffff;
	    crc ^= (crc & 0xff) << 5;
	    crc = crc & 0xffff;
        return crc;
    }

    short crcAccumulate(short crc, byte val) {
        int i = 8;
        crc = (short) (crc ^ val << 8);
        do {
            if ((crc & 0x8000) != 0) crc = (short) (crc << 1 ^ 0x1021);
            else crc = (short) (crc << 1);
        } while (--i > 0);
        return crc;
    }

    private static final int RECV_SFD_SCAN = 0;
    private static final int RECV_SFD_MATCHED_1 = 1;
    private static final int RECV_SFD_MATCHED_2 = 2;
    private static final int RECV_IN_PACKET = 3;
    private static final int RECV_CRC_1 = 4;
    private static final int RECV_CRC_2 = 5;
    private static final int RECV_END_STATE = 6;
    private static final int RECV_OVERFLOW = 7;
    private static final int RECV_WAIT = 8;

    public class Receiver extends Medium.Receiver {
        protected int state;
        protected int counter;
        protected int length;
        protected short crc;
        protected byte crcLow;

        public Receiver(Medium m) {
            super(m, sim.getClock());
        }

        private void setRssiValid (boolean v){
            rssiValid.setValue(v);
            //RSSI valid: rssi initialized to KTB - Rssi_offset=-91-45=-136
            //RSSI not valid: rssi_val = -128
            int rssi_val = v ? -91 : -128;
            int cca_thr = (readRegister(RSSI) & 0xff00);
            rssi_val = cca_thr | (rssi_val & 0x00ff);
            writeRegister(RSSI,rssi_val);
        }

        private boolean getRssiValid (){
            return rssiValid.getValue();
        }

        public double getCorrelation (){
            int PERindex = (int)(getPER()*100);
            Random random = new Random();
            //get the range, casting to long to avoid overflow problems
            long range = (long)Corr_MAX[PERindex] - (long)Corr_MIN[PERindex] + 1;
            // compute a fraction of the range, 0 <= frac < range
            long fraction = (long)(range * random.nextDouble());
            double corr = fraction + Corr_MIN[PERindex];
            return corr;
        }

        public void setRSSI (double Prec){
            //compute Rssi as Pr + RssiOffset
            int rssi_val =(int)Math.rint(Prec + 45.0D);
            // -127 <= rssi_val <= 127 since -128 is "invalid RSSI"
            rssi_val = Math.min(127,Math.max(-127,rssi_val));
            rssi_val = rssi_val & 0x00ff;
            int cca_thr = (readRegister(RSSI) & 0xff00);
            rssi_val = rssi_val | cca_thr;
            writeRegister(RSSI,rssi_val);
        }

        public void setBER (double BER){
            BERcount++;
            if (BERcount > 5) {
                BERtotal += BER;
            }
        }

        public double getPER (){
            double PER = 0.0D;
            if (BERcount > 5) {
                //compute average BER after SHR
                PER = BERtotal/(BERcount-5);
                //considering i.i.s errors i compute PER
                PER = 1D-Math.pow((1D-PER),(BERcount-5)*8);
            }
            clearBER();
            return PER;
        }

        public void clearBER() {
            BERcount = 0;
            BERtotal = 0.0D;
        }

        public byte nextByte(boolean lock, byte b) {
            if (state == RECV_END_STATE) {
                state = RECV_SFD_SCAN; // to prevent loops when calling shutdown/endReceive
                // packet ended before
                if (SendAck != SENDACK_NONE && lastCRCok) {//Send Ack?
                    shutdown();
                    transmitter.startup();
                } else {
                    if (lock) {
                        // the medium is still locked, so there could be more packets!
                        // fire the probes manually
                        if (probeList != null) probeList.fireAfterReceiveEnd(Receiver.this);
                    }
                }
                return b;
            }

            if (!lock) {
                // the transmission lock has been lost
                switch (state) {
                    case RECV_SFD_MATCHED_2:
                    case RECV_IN_PACKET:
                    case RECV_CRC_1:
                    case RECV_CRC_2:
                        //packet lost in middle -> drop frame
                        dropFrame();
                        // fall through

                    case RECV_SFD_MATCHED_1: // packet has just started
                        state = RECV_SFD_SCAN;
                        SFD_value.setValue(!SFD_active);
                        break;
                }
                return b;
            }

            if (printer != null) {
                printer.println("CC2420 <======== " + StringUtil.to0xHex(b, 2));
            }
            switch (state) {
                case RECV_SFD_MATCHED_1:
                    // check against the second byte of the SYNCWORD register.
                 //   if (b == Arithmetic.high(registers[SYNCWORD])) {
                    if (b == (byte) 0x7A) {
 //myprinter.println("CC2420 SFD");
                        state = RECV_SFD_MATCHED_2;
                        SFD_value.setValue(SFD_active);
                        break;
                    }
                    // fallthrough if we failed to match the second byte
                    // and try to match the first byte again.
                case RECV_SFD_SCAN:
   // myprinter.println("CC2420 scan");
                    // check against the first byte of the SYNCWORD register.
              //      if (b == Arithmetic.low(registers[SYNCWORD])) {
                     if (b == (byte) 0x00) {
                        state = RECV_SFD_MATCHED_1;
                    } else {
                        state = RECV_SFD_SCAN;
                    }
                    break;

                case RECV_SFD_MATCHED_2:
                    // SFD matched. read the length from the next byte.
                    length = b & 0x7f;
          //          myprinter.println("CC2420 length = " + length);
                    if (length == 0) {  // ignore frames with 0 length
                        SFD_value.setValue(!SFD_active);
                        state = RECV_SFD_SCAN;
                        break;
                    }

                    // save current position in rxFIFO as start of frame
                    rxFIFO.saveState();
                    if (fifoAdd(b)) {
                        counter = 0;
                        state = RECV_IN_PACKET;
                        crc = 0;
                    }
                    break;

                case RECV_IN_PACKET:
                    // we are in the body of the packet.
                    counter++;
                    if (!fifoAdd(b)) {
                        myprinter.println("OVERFLOW");
                        break;  // overflow
                    }

                    //Address Recognition
                    if (ADR_DECODE.getValue()) {
                        if (counter <= 13) {
                            boolean satisfied = matchAddress(b, counter);
                            if (!satisfied) {
                                //reject frame
                              //  myprinter.println("Dropped, no match");
                                dropFrame();
                                // wait for end of packet
                                SFD_value.setValue(!SFD_active);
                                state = RECV_WAIT;
                                break;
                            }
                        }
                    }
                    else {
                        // sequence number - save it outside of address recognition since it is needed for SACK/SACKPEND commands as well
                        if (counter == 3 && (rxFIFO.peek(1) & 0x07) != 0 && (rxFIFO.peek(1) & 0x04) != 4) {
                            DSN = b;
                            lastCRCok = false;  // we have a new DSN now. Therefore, we cannot send an ACK for the last frame any more.
                        }
                    }

                    // no overflow occurred and address ok
                    if (autoCRC.getValue()) {
                     //   crc = crcAccumulate(crc, b);
                        crc = crcAccumulate(crc, (byte) reverse_bits[(b) & 0xff]);
                        if (counter == length - 2) {
                            // transition to receiving the CRC.
                         //   myprinter.println("CC2420 crc1");
                            state = RECV_CRC_1;
                        }
                    } else if (counter == length) {
                           myprinter.println("CC2420 noautocrc");
                        // no AUTOCRC, but reached end of packet.
                        signalFIFOP();
                        SFD_value.setValue(!SFD_active);
                        clearBER();  // will otherwise be done by getCorrelation() in state RECV_CRC_2
                        lastCRCok = false;
                        state = RECV_END_STATE;
                    }

                    break;
                case RECV_CRC_1:
                    crcLow = b;
                    state = RECV_CRC_2;
                    //RSSI value is written in this position of the rxFIFO
                    b = (byte)(readRegister(RSSI)&0x00ff);
                    fifoAdd(b);
                    break;

                case RECV_CRC_2:
                    state = RECV_END_STATE;
                    crcLow = (byte)reverse_bits[(crcLow) & 0xff];
                    b = (byte)reverse_bits[(b) & 0xff];
                    short crcResult = Arithmetic.word(b, crcLow);
                    //Corr value and CRCok are written in this position of the rxFIFO
                    b = (byte) ((byte)getCorrelation() & 0x7f);
              //      myprinter.println(" crcResult " + crcResult + " crc " + crc);
                    if (crcResult == crc) {
                        b |= 0x80;
                        lastCRCok = true;
                        if (printer != null) {
                            printer.println("CC2420 CRC passed");
                        }
                    }
                    else {
                        // According to the CC2420 data sheet the frame is not rejected if the CRC is invalid!!!
                        // reset ACK flags set by the SACK/SACKPEND commands since ACK is only sent when CRC is valid
                        lastCRCok = false;
                        SendAck = SENDACK_NONE;
                        if (printer != null) {
                            printer.println("CC2420 CRC failed");
                        }
                    }

                    if (fifoAdd(b)) {
                        // signal FIFOP and unsignal SFD
                        signalFIFOP();
                        SFD_value.setValue(!SFD_active);
                        if (lastCRCok && autoACK.getValue() && (rxFIFO.peek(1) & 0x20) == 0x20) {//autoACK
                            //send ack if we are not receiving ack frame
                            if ((rxFIFO.peek(1) & 0x07) != 2) {
                                // the type of the ACK only depends on a previous received SACK or SACKPEND
                                SendAck = AutoAckPend ? SENDACK_PEND : SENDACK_NORMAL;
                            }
                        }
                    }

                    break;
                case RECV_OVERFLOW:
                    // do nothing. we have encountered an overflow.
                    break;
                case RECV_WAIT:
                    // just wait for the end of the packet
                    if (++counter == length) {
                        clearBER();  // will otherwise be done by getCorrelation()
                        state = RECV_SFD_SCAN;
                        SendAck = SENDACK_NONE;  // just in case we received SACK(PEND) in the meantime
                    }
                    break;
            }
            return b;
        }

        // helper function to handle the addition of a byte to the rxFIFO
        private boolean fifoAdd(byte c) {
            if (printer != null) {
                printer.println("CC2420 Add to RXFIFO: " + StringUtil.toMultirepString(c, 8));
            }
            rxFIFO.add(c);
            if (rxFIFO.overFlow()) {
                // an RX overflow has occurred.
                FIFO_pin.level.setValue(!FIFO_active);
                signalFIFOP();
                state = RECV_OVERFLOW;
                lastCRCok = false;
                return false;
            }
            else {
                FIFO_pin.level.setValue(FIFO_active);
                if (rxFIFO.size() >= getFIFOThreshold()) {
                    // TODO: when address recognition is on FIFOP will remain
                    // low until the frame passed address recognition completely.
                    // The current matchAddress() does not support this.
                    signalFIFOP();
                }
            }
            return true;
        }

        private boolean matchAddress(byte b, int counter) {
            if (counter > 1 && (rxFIFO.peek(1) & 0x04) == 4 && RESERVED_FRAME_MODE.getValue()) {
                // no further address decoding is done for reserved frames
                return true;
            }
            switch (counter) {
                case 1://frame type subfield contents an illegal frame type?
                    if ((rxFIFO.peek(1) & 0x04) == 4 && !(RESERVED_FRAME_MODE.getValue()))
                        return false;
                    break;
                case 3://Sequence number
                    if ((rxFIFO.peek(1) & 0x07) != 0 && (rxFIFO.peek(1) & 0x04) != 4) DSN = b;
                    break;
                case 5:
                    PANId = rxFIFO.peekField(4, 6);
                    macPANId = ByteFIFO.copyOfRange(RAMSecurityRegisters, 104, 106);
                    if (((rxFIFO.peek(2) >> 2) & 0x02) != 0) {//DestPANId present?
                        if (!Arrays.equals(PANId, macPANId) && !Arrays.equals(PANId, SHORT_BROADCAST_ADDR))
                            return false;
                    } else
                    if (((rxFIFO.peek(2) >> 2) & 0x03) == 0) {//DestPANId and dest addresses are not present
                        if (((rxFIFO.peek(2) >> 6) & 0x02) != 0) {//SrcPANId present
                            if ((rxFIFO.peek(1) & 0x07) == 0) {//beacon frame: SrcPANid shall match macPANId unless macPANId = 0xffff
                                if (!Arrays.equals(PANId, macPANId) && !Arrays.equals(macPANId, SHORT_BROADCAST_ADDR) && !BCN_ACCEPT.getValue())
                                    return false;
                            } else
                            if (((rxFIFO.peek(1) & 0x07) == 1) || ((rxFIFO.peek(1) & 0x07) == 3)) {//data or mac command
                                if (!PAN_COORDINATOR.getValue() || !Arrays.equals(PANId,macPANId)) return false;
                            }
                        }
                    }
                    break;
                case 7://If 32-bit Destination Address exits check if  match
                    if (((rxFIFO.peek(2) >> 2) & 0x03) == 2) {
                        ShortAddr = rxFIFO.peekField(6, 8);
                        macShortAddr = ByteFIFO.copyOfRange(RAMSecurityRegisters, 106, 108);
                       //myprinter.println("shortadr " + ShortAddr[0]+ShortAddr[1]);
                       //myprinter.println("macshortaddr " + macShortAddr[0]+" "+macShortAddr[1]);
                        if (!Arrays.equals(ShortAddr, macShortAddr) && !Arrays.equals(ShortAddr, SHORT_BROADCAST_ADDR))
                            return false;
                    }
                    break;
             // case 12://If 64-bit Destination Address exits check if match
                //dak bumped this up a byte, works with sky. The SFD change caused this?
                case 13://If 64-bit Destination Address exits check if match
                    if (((rxFIFO.peek(2) >> 2) & 0x03) == 3) {
                     // LongAdr = rxFIFO.peekField(8, 16);
                        LongAdr = rxFIFO.peekField(6, 14);//dak
                        IEEEAdr = ByteFIFO.copyOfRange(RAMSecurityRegisters, 96, 104);
                        if (!Arrays.equals(LongAdr, IEEEAdr) && !Arrays.equals(LongAdr, LONG_BROADCAST_ADDR)) {
                       //     myprinter.println(" longadr " + LongAdr[0]+LongAdr[1]+LongAdr[2]+LongAdr[3]+LongAdr[4]+LongAdr[5]+LongAdr[6]+LongAdr[7]);
                        //    myprinter.println(" IEEEAdr " + IEEEAdr[0]+IEEEAdr[1]+IEEEAdr[2]+IEEEAdr[3]+IEEEAdr[4]+IEEEAdr[5]+IEEEAdr[6]+IEEEAdr[7]);
                            return false;
                        }
                    }
                    break;
            }
            return true;
        }

        private void dropFrame() {
            // do not clear complete FIFO, only reject frame.
            rxFIFO.dropLast();
            // unset FIFO/FIFOP pins if necessary
            if (rxFIFO.empty()) {
                FIFO_pin.level.setValue(!FIFO_active);
            }
            if (rxFIFO.size() < getFIFOThreshold()) {
                FIFOP_pin.level.setValue(!FIFOP_active);
            }
        }

        private void signalFIFOP() {
            FIFOP_pin.level.setValue(FIFOP_active);
            if (FIFOP_interrupt > 0) {
                sim.getInterpreter().getInterruptTable().post(FIFOP_interrupt);
            }
        }

        private void unsignalFIFOP() {
            FIFOP_pin.level.setValue(!FIFOP_active);
            if (FIFOP_interrupt > 0) {
                sim.getInterpreter().getInterruptTable().unpost(FIFOP_interrupt);
            }
        }

        protected boolean inPacket() {
            return state == RECV_SFD_MATCHED_1
                || state == RECV_SFD_MATCHED_2
                || state == RECV_IN_PACKET
                || state == RECV_CRC_1
                || state == RECV_CRC_2;
        }

        /**
         * The <code>RssiValid</code> class implements a Simulator Event
         * that is fired when the RSSI becomes valid after 8 symbols
         */
        protected class RssiValid implements Simulator.Event {
            public void fire() {
                if (activated) {
                    setRssiValid(true);
                }
            }
        }
        protected RssiValid rssiValidEvent = new RssiValid();

        void startup() {
            stateMachine.transition(3);//change to receive state
            state = RECV_SFD_SCAN;
            clearBER();
            beginReceive(getFrequency());
            clock.insertEvent(rssiValidEvent, 4*cyclesPerByte);  // 8 symbols = 4 bytes
            if (printer!=null) {
                printer.println("CC2420 RX started");
            }
        }

        void shutdown() {
            // note that stateMachine.transition() is not called here any more
            endReceive();
            setRssiValid(false);
            if (printer != null) {
                printer.println("CC2420 RX shutdown");
            }
        }

        void resetOverflow() {
            state = RECV_SFD_SCAN;
        }
    }

    /**
     * The <code>CC2420Pin</code>() class models pins that are inputs and outputs to the CC2420 chip.
     */
    public class CC2420Pin implements Microcontroller.Pin.Input, Microcontroller.Pin.Output {
        protected final String name;
        protected boolean level;

        public CC2420Pin(String n) {
            name = n;
        }

        public void write(boolean level) {
            if (this.level != level) {
                // level changed
                this.level = level;
                if (this == CS_pin) pinChange_CS(level);
                else if (this == VREN_pin) pinChange_VREN(level);
                else if (this == RSTN_pin) pinChange_RSTN(level);
           //     if (printer != null) {
            //        printer.println("CC2420 Write pin " + name + " -> " + level);
           //     }
            }
        }

        public boolean read() {
          //  if (printer != null) {
          //      printer.println("CC2420 Read pin " + name + " -> " + level);
         //   }
            return level;
        }
    }

    public class CC2420Output implements Microcontroller.Pin.Input {

        protected BooleanView level;
        protected final String name;

        public CC2420Output(String n, BooleanView lvl) {
            name = n;
            level = lvl;
        }

        public boolean read() {
            boolean val = level.getValue();
        //    if (printer != null) {
        //        printer.println("CC2420 Read pin " + name + " -> " + val);
        //    }
            return val;
        }
    }

    public static String regName(int reg) {
        switch (reg) {
            case MAIN:
                return "MAIN    ";
            case MDMCTRL0:
                return "MDMCTRL0";
            case MDMCTRL1:
                return "MDMCTRL1";
            case RSSI:
                return "RSSI    ";
            case SYNCWORD:
                return "SYNCWORD";
            case TXCTRL:
                return "TXCTRL  ";
            case RXCTRL0:
                return "RXCTRL0 ";
            case RXCTRL1:
                return "RXCTRL1 ";
            case FSCTRL:
                return "FSCTRL  ";
            case SECCTRL0:
                return "SECCTRL0";
            case SECCTRL1:
                return "SECCTRL1";
            case BATTMON:
                return "BATTMON ";
            case IOCFG0:
                return "IOCFG0  ";
            case IOCFG1:
                return "IOCFG1  ";
            case MANFIDL:
                return "MANFIDL ";
            case MANFIDH:
                return "MANFIDH ";
            case FSMTC:
                return "FSMTC   ";
            case MANAND:
                return "MANAND  ";
            case MANOR:
                return "MANOR   ";
            case AGCCTRL0:
                return "AGCCTRL0";
            case AGCTST0:
                return "AGCTST0 ";
            case AGCTST1:
                return "AGCTST1 ";
            case AGCTST2:
                return "AGCTST2 ";
            case FSTST0:
                return "FSTST0  ";
            case FSTST1:
                return "FSTST1  ";
            case FSTST2:
                return "FSTST2  ";
            case FSTST3:
                return "FSTST3  ";
            case RXBPFTST:
                return "RXBPFTST";
            case FSMSTATE:
                return "FSMSTATE";
            case ADCTST:
                return "ADCTST  ";
            case DACTST:
                return "DACTST  ";
            case TOPTST:
                return "TOPTST  ";
            case TXFIFO:
                return "TXFIFO  ";
            case RXFIFO:
                return "RXFIFO  ";
            default:
                return StringUtil.to0xHex(reg, 2) + "    ";
        }
    }

    public static String strobeName(int strobe) {
        switch (strobe) {
            case SNOP:
                return "SNOP    ";
            case SXOSCON:
                return "SXOSCON ";
            case STXCAL:
                return "STXCAL  ";
            case SRXON:
                return "SRXON   ";
            case STXON:
                return "STXON   ";
            case STXONCCA:
                return "STXONCCA";
            case SRFOFF:
                return "SRFOFF  ";
            case SXOSCOFF:
                return "SXOSCOFF";
            case SFLUSHRX:
                return "SFLUSHRX";
            case SFLUSHTX:
                return "SFLUSHTX";
            case SACK:
                return "SACK    ";
            case SACKPEND:
                return "SACKPEND";
            case SRXDEC:
                return "SRXDEC  ";
            case STXENC:
                return "STXENC  ";
            case SAES:
                return "SAES    ";
            default:
                return StringUtil.to0xHex(strobe, 2) + "    ";
        }
    }

    String fifoName(ByteFIFO fifo) {
        if (fifo == txFIFO) return "TX FIFO";
        if (fifo == rxFIFO) return "RX FIFO";
        return "XX FIFO";
    }

    public static String SecurityRAMName(int address) {
        if (address < 16) return "KEY0";
        else if (address < 32) return "RX_NONCE_COUNTER";
        else if (address < 48) return "SABUF";
        else if (address < 64) return "KEY1";
        else if (address < 80) return "TX_NONCE_COUNTER";
        else if (address < 96) return "CBCSTATE";
        else if (address < 104) return "IEEADR";
        else if (address < 106) return "PANID";
        else if (address < 112) return "SHORTADR";
        else return " ";
    }

    private long toCycles(long us) {
        return us * sim.getClock().getHZ() / 1000000;
    }

    public static Medium createMedium(Synchronizer synch, Medium.Arbitrator arbitrator) {
        return new Medium(synch, arbitrator, 250000, 48, 8, 8 * 128);
    }

    public Medium.Transmitter getTransmitter() {
        return transmitter;
    }

    public Medium.Receiver getReceiver() {
        return receiver;
    }

    public void setMedium(Medium m) {
        medium = m;
        transmitter = new Transmitter(m);
        receiver = new Receiver(m);
    }

    public Medium getMedium() {
        return medium;
    }

}
