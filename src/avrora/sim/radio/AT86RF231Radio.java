/*
 * Copyright (c) 2012, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

package avrora.sim.radio;

import avrora.sim.AtmelInterpreter;
import avrora.sim.FiniteStateMachine;
import avrora.sim.RWRegister;
import avrora.sim.Simulator;
import avrora.sim.clock.Synchronizer;
import avrora.sim.energy.Energy;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.output.SimPrinter;
import avrora.sim.state.BooleanRegister;
import avrora.sim.state.BooleanView;
import avrora.sim.state.ByteFIFO;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import cck.text.StringUtil;
import cck.util.Arithmetic;
import java.security.InvalidKeyException;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Random;
import javax.crypto.BadPaddingException;
import javax.crypto.Cipher;
import javax.crypto.IllegalBlockSizeException;
import javax.crypto.NoSuchPaddingException;
import javax.crypto.spec.SecretKeySpec;


/**
 * The <code>AT86RF231Radio</code> implements a simulation of the Atmel
 * AT86RF231 radio, and can also be used for the AT86RF230.
 * Verbose printers for this class include "radio.rf231"
 *
 * @author David A. Kopf
 */
public class AT86RF231Radio implements Radio {
    private final static boolean DEBUG   = false;  //state changes, interrupts
    private final static boolean DEBUGV  = false;  //pin changes
    private final static boolean DEBUGRX = false;  //receiver
    private final static boolean DEBUGTX = false;  //transmitter
    private final static boolean DEBUGA  = false;  //ACKs
    private final static boolean DEBUGC  = false;  //CCA, CSMA
    private final static boolean DEBUGQ  = true;   //"should not happen" debugs

    //-- Radio states, confusingly contained in the TRX_STATUS register --------
    public  byte rf231Status = 0;
    public static final byte STATE_BUSY_RX      = 0x01;
    public static final byte STATE_BUSY_TX      = 0x02;
    public static final byte STATE_RX_ON        = 0x06;
    public static final byte STATE_TRX_OFF      = 0x08;
    public static final byte STATE_PLL_ON       = 0x09;
    public static final byte STATE_SLEEP        = 0x0F;
    public static final byte STATE_BUSY_RX_AACK = 0x11;
    public static final byte STATE_BUSY_TX_ARET = 0x12;
    public static final byte STATE_RX_AACK_ON   = 0x16;
    public static final byte STATE_TX_ARET_ON   = 0x19;
    public static final byte STATE_TRANSITION   = 0x1F;
    //-- Radio commands--------------------------------------------------------
    public static final byte CMD_NOP            = 0x00;
    public static final byte CMD_TX_START       = 0x02;
    public static final byte CMD_FORCE_TRX_OFF  = 0x03;
    public static final byte CMD_FORCE_PLL_ON   = 0x04;
    public static final byte CMD_RX_ON          = 0x06;
    public static final byte CMD_TRX_OFF        = 0x08;
    public static final byte CMD_TX_ON          = 0x09;
    public static final byte CMD_RX_AACK_ON     = 0x16;
    public static final byte CMD_TX_ARET_ON     = 0x19;
    //-- Register addresses ---------------------------------------------------
    // The rf230 does not use all registers but initializes them anyway.
    public static final int TRX_STATUS   = 0x01;   //CCA_DONE, CCA_STATUS, TST_STATUS, TRX_STATUS[4:0]
    public static final int TRX_STATE    = 0x02;   //TRAC_STATUS[2:0],TRX_CMD[4:0]
    public static final int TRX_CTRL_0   = 0x03;
    public static final int TRX_CTRL_1   = 0x04;//rf231
    public static final int PHY_TX_PWR   = 0x05;
    public static final int PHY_RSSI     = 0x06;
    public static final int PHY_ED_LEVEL = 0x07;
    public static final int PHY_CC_CCA   = 0x08;
    public static final int CCA_THRES    = 0x09;
    public static final int RX_CTRL      = 0x0A;//rf231
    public static final int SFD_VALUE    = 0x0B;//rf231
    public static final int TRX_CTRL_2   = 0x0C;//rf231
    public static final int ANT_DIV      = 0x0D;//rf231
    public static final int IRQ_MASK     = 0x0E;
    public static final int IRQ_STATUS   = 0x0F;
    public static final int VREG_CTRL    = 0x10;
    public static final int BATMON       = 0x11;
    public static final int XOSC_CTRL    = 0x12;
    public static final int RX_SYN       = 0x15;//rf231
    public static final int XAH_CTRL_1   = 0x17;//rf231
    public static final int PLL_CF       = 0x1A;
    public static final int PLL_DCU      = 0x1B;
    public static final int PART_NUM     = 0x1C;
    public static final int VERSION_NUM  = 0x1D;
    public static final int MAN_ID_0     = 0x1E;
    public static final int MAN_ID_1     = 0x1F;
    public static final int SHORT_ADDR_0 = 0x20;
    public static final int SHORT_ADDR_1 = 0x21;
    public static final int PAN_ID_0     = 0x22;
    public static final int PAN_ID_1     = 0x23;
    public static final int IEEE_ADDR_0  = 0x24;
    public static final int IEEE_ADDR_1  = 0x25;
    public static final int IEEE_ADDR_2  = 0x26;
    public static final int IEEE_ADDR_3  = 0x27;
    public static final int IEEE_ADDR_4  = 0x28;
    public static final int IEEE_ADDR_5  = 0x29;
    public static final int IEEE_ADDR_6  = 0x2A;
    public static final int IEEE_ADDR_7  = 0x2B;
    public static final int XAH_CTRL_0   = 0x2C;
    public static final int CSMA_SEED_0  = 0x2D;
    public static final int CSMA_SEED_1  = 0x2E;
    public static final int CSMA_BE      = 0x2F;//rf231
    //-- AES RAM Addresses -----------------------------------------------------
    public static final int AES_RAM_STATUS = 0x02;
    public static final int AES_RAM_CONFIG = 0x03;
    public static final int AES_RAM_DATA = 0x04;
    public static final int AES_RAM_DATA_END = 0x13;
    public static final int AES_RAM_CONFIG_SHADOW = 0x14;

    //-- Other constants --------------------------------------------------
    private static final int NUM_REGISTERS = 0x3F;
    private static final int FIFO_SIZE     = 128;
    private static final int XOSC_START_TIME = 1000;// oscillator start time

    //-- Simulation objects -----------------------------------------------
    protected final AtmelInterpreter interpreter;
    protected final Microcontroller mcu;
    protected final Simulator sim;

    //-- Radio state ------------------------------------------------------
    protected final int xfreq;
    protected final byte[] registers = new byte[NUM_REGISTERS];
    protected final ByteFIFO trxFIFO = new ByteFIFO(FIFO_SIZE);
    protected double BERtotal = 0.0D;
    protected int BERcount = 0;
    protected boolean txactive = false,rxactive = false;
    protected boolean sendingAck, waitingAck, handledAck;
    protected byte frameRetries, csmaRetries;
    protected boolean lastCRCok;
    protected byte DSN;
    protected Random random = new Random();

    protected Medium medium;
    protected Transmitter transmitter;
    protected Receiver receiver;

    //-- Pins ------------------------------------------------------------
    public final RF231Pin SCLK_pin   = new RF231Pin("SCLK");
    public final RF231Pin MISO_pin   = new RF231Pin("MISO");
    public final RF231Pin MOSI_pin   = new RF231Pin("MOSI");
    public final RF231Pin CS_pin     = new RF231Pin("SELN");
    public final RF231Pin SLPTR_pin  = new RF231Pin("SLP_TR");
    public final RF231Pin RSTN_pin   = new RF231Pin("RSTN");
    public final RF231Output IRQ_pin = new RF231Output("IRQ", new BooleanRegister());

    public final SPIInterface spiInterface = new SPIInterface();

    protected final AESModule aes = new AESModule();

    public int RF231_interrupt = -1;  //platform sets this to the correct interrupt number

    protected final SimPrinter printer;

    //RF231 energy
    protected static final String[] allModeNames = AT86RF231Energy.allModeNames();
    protected static final int[][] ttm = FiniteStateMachine.buildSparseTTM(allModeNames.length, 0);
    protected final FiniteStateMachine stateMachine;

    /**
     * The constructor for the AT86RF231Radio class creates a new instance of an
     * AT86RF231 radio connected to a microcontroller via SPI.
     * to the specified microcontroller with the given clock frequency.
     *
     * @param mcu   the microcontroller unit
     * @param xfreq the clock frequency of this microcontroller
     */
    public AT86RF231Radio(Microcontroller mcu, int xfreq) {
        // set up references to MCU and simulator
        this.mcu = mcu;
        this.sim = mcu.getSimulator();
        this.interpreter = (AtmelInterpreter)sim.getInterpreter();

        this.xfreq = xfreq;

        // create a private medium for this radio
        // the simulation may replace this later with a new one.
        setMedium(createMedium(null, null));

        //setup energy recording
        stateMachine = new FiniteStateMachine(sim.getClock(), AT86RF231Energy.startMode, allModeNames, ttm);
        new Energy("Radio", AT86RF231Energy.modeAmpere, stateMachine, sim.getEnergyControl());

        // get debugging channel.
        printer = sim.getPrinter("radio.rf231");

        // set all registers to reset values, and clear FIFO
        reset();
        trxFIFO.clear();

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

    public void reset() {
        if (DEBUG && printer!= null) printer.println("RF231: RESET");
        for (int cntr = 0; cntr < NUM_REGISTERS; cntr++) {
           resetRegister(cntr);
        }
        lastCRCok = false;
        rf231Status = STATE_TRX_OFF;
        txactive = rxactive = true;
        transmitter.shutdown();
        receiver.shutdown();
    }


    //ccaDelay fires after the cca or energy detect delay.
    protected class ccaDelay implements Simulator.Event {
        public void fire() {
            //Construct PHY_ED_LEVEL from PHY_RSSI
            registers[PHY_ED_LEVEL] = (byte) ((registers[PHY_RSSI] & 0x1f) * 3);

            //update cca done and cca_status
            boolean ccaBusy = (registers[PHY_RSSI] & 0x1f) > ((registers[CCA_THRES] & 0x0f) << 1);
 /*
            //TODO: Carrier sense
            switch (mode) {
                case 0: //Carrier sense OR energy above threshold
                case 1: //Energy above threshold
                case 2: //Carrier sense only
                case 3: //Carrier sense AND energy above threshold
                break;
            }
*/
            if (rf231Status == STATE_BUSY_TX_ARET) {
                if (ccaBusy) {
                    if (csmaRetries > 0) {
                        if (DEBUGC && printer!=null) printer.println("RF231: csma busy, retry count " + csmaRetries);
                        csmaRetries--;
                        //Wait for a random number of 20 symbol periods, between 0 and 2**backoff-1
                        int backoff = 1;
                        int minBE = (registers[CSMA_BE] & 0xFF);
                        int maxBE = minBE >> 4;
                        if (maxBE > 0) {
                            minBE = minBE & 0x0F;
                            if (minBE > 0) minBE = 1 << minBE;
                            maxBE = 1 << maxBE;
                            backoff = minBE + (int) ((maxBE - minBE) * random.nextDouble());
                            if (DEBUGC && printer!=null) printer.println("RF231: minBE, maxBE, backoff " + minBE + " " + maxBE + " " + backoff);
                        }
                        //TODO: slotted transmissions start the wait at the next slot time
                        receiver.clock.insertEvent(ccaDelayEvent, 10*backoff*receiver.cyclesPerByte);
                    } else {
                        //Set TRAC_STATUS to no CHANNEL_ACCESS_FAILURE and return to TX_ARET_ON
                        registers[TRX_STATE] = (byte) ((registers[TRX_STATE] & 0X1F) | 0x20);

                        //rf231Status = STATE_TX_ARET_ON;
                        rf231Status = STATE_PLL_ON;//??
                        registers[TRX_STATUS] = rf231Status;
                        if (rxactive) printer.println("rxactive after csma failure");
                        //receiver.shutdown();//??
                        //possibly sending an autoack?
                        if (txactive) printer.println("txactive after csma failure");
                        transmitter.endTransmit(); //this seems to be necessary...
                      //  if (txactive) transmitter.shutdown();
                        if (DEBUGC & printer !=null) printer.println("RF231: TRX_END interrupt, csma failure");
                        postInterrupt(INT_TRX_END);
                    }
                } else {
                  //  if (!rxactive) printer.println("rx not active during cca");
                    if (rxactive) receiver.shutdown(); //should receiver get shutdown regardless?
                    if (DEBUGC && printer!=null) printer.println("RF231: Starting tx after csma");
                    transmitter.startup();
                }
                return;
            }
          //  registers[TRX_STATUS] = (byte) registers[TRX_STATUS] & 0x3f;
            if (ccaBusy) {
                registers[TRX_STATUS] = (byte) (0x80 | (registers[TRX_STATUS] & 0x3f));
            } else {
                registers[TRX_STATUS] = (byte) (0xC0 | (registers[TRX_STATUS] & 0x3f));
            }
          //  printer.println("after cca status is " + rf231Status);
            if (rf231Status == STATE_BUSY_RX_AACK) {
            //receiving a packet during the cca. How to handle this?
               if (DEBUGC && printer !=null)printer.println("RF231: BUSY_RX_AACK after CCA");
              //  rf231Status = STATE_RX_AACK_ON;
            }
        //    if (rxactive) printer.println("rxactive after cca");
            //receiver.shutdown();
            postInterrupt(INT_CCA_ED_DONE);
        }
    }
    protected ccaDelay ccaDelayEvent = new ccaDelay();

    /**
     * The <code>readRegister()</code> method reads the value from the specified register
     * and takes any action(s) that are necessary for the specific register.
     *
     * @param addr the address of the register
     * @return an integer value representing the result of reading the register
     */
    public byte readRegister(int addr) {
        byte val = registers[addr];
        if (DEBUGV && printer!=null) printer.println("RF231 " + regName(addr) + " => " + StringUtil.to0xHex(val, 2));
        switch (addr) {
            case IRQ_STATUS:
                //reading IEQ_STATUS clears all interrupts
                registers[addr] = 0;
                break;
            case PHY_RSSI:
                val |= (random.nextInt() & 0x60);
                break;
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
    void writeRegister(int addr, byte val) {
        if (DEBUGV && printer!=null) printer.println("RF231 " + regName(addr) + " <= " + StringUtil.to0xHex(val, 2));
        registers[addr] = val;
        switch (addr) {
            case TRX_STATE:
                newCommand((byte) (val & 0x1F));
                break;
            //In basic operation a CCA is triggered by a write to the top bit of PHY_CC_CCA
            //In extended operation it is triggered by a write to the PHY_ED_LEVEL register.
            //See data sheet for the various possibilities. In extended mode rx done is set immediately with no CCA.
            //In BUSY_RX carrier sense only the result is available immediately.
            case PHY_ED_LEVEL:
                val = (byte) 0x80; //fall through with the upper bit set?
            case PHY_CC_CCA:
                //upper bit is cca request, is cleared automatically
                 if ((val & 0x80) != 0) {
                    if (rf231Status == STATE_RX_AACK_ON) {  //or BUSY?
                        byte mode = (byte) ((val & 0x60) >> 5);
                        if ((registers[PHY_RSSI] & 0x1f) != 0) {
                            if (DEBUGC & printer!=null) printer.println("RF231: CCA_ED_REQUEST, mode = " + mode + " PHYRSSI = " +(registers[PHY_RSSI] & 0x1f) + " CCATHRES " + (registers[CCA_THRES] & 0x0f));
                        }
                        boolean tbusy = (registers[PHY_RSSI] & 0x1f) >= ((registers[CCA_THRES] & 0x0f) << 1);
                        //TODO: Carrier sense
                        switch (mode) {
                            case 0: //Carrier sense OR energy above threshold
                            case 1: //Energy above threshold
                            case 2: //Carrier sense only
                            case 3: //Carrier sense AND energy above threshold
                            break;
                        }
                        //clear status and done bit
                        registers[TRX_STATUS] &= 0x3f;
                        registers[TRX_STATUS] |= 0x80;
                        if (!tbusy) registers[TRX_STATUS] |= 0x40;
                        postInterrupt(INT_CCA_ED_DONE);
                        break;
                    }
                    if (DEBUGC & printer!=null) printer.println("RF231: CCA_REQUEST, mode = " + " PHYRSSI = " +registers[PHY_RSSI] + " CCATHRESH " + (registers[CCA_THRES] & 0x0f));
                    registers[addr] = (byte) (val & 0x7f);
                    //clear status and done bit
                    registers[TRX_STATUS] = (byte) (registers[TRX_STATUS] & 0x3F);

                    //wait 140 usec (8.75 symbol periods)
                    receiver.clock.insertEvent(ccaDelayEvent, 875*receiver.cyclesPerByte/200);
                }
                break;
        }
    }

    /**
     * The <code>newCommand()</code> method alters the radio state according to
     * the rules for possible state transitions.
     *
     * @param byte the command
     */
     void newCommand(byte val) {
    //A state change does not affect the frame buffer contents.
    //TODO:The driver should be checking for valid transitions, but diagnostics could be added here.
        switch (val) {
            case (byte) 0x42:
                printer.println("RF231: not an RFA1!!");
                break;
            case CMD_NOP:
                if (DEBUG && printer!=null) printer.println("RF231: NOP");
                break;
            case CMD_TX_START:
                if (DEBUG && printer!=null) printer.println("RF231: TX_START");
                // set TRAC_STATUS bits to INVALID?
                // registers[TRX_STATE] = (byte) (CMD_TX_START | 0xE0);
                rf231Status = STATE_BUSY_TX;
                sendingAck = false;
                if (rxactive) receiver.shutdown();
                if (!txactive) transmitter.startup();
                break;
            case CMD_FORCE_TRX_OFF:
                if (DEBUG && printer!=null) printer.println("RF231: FORCE_TRX_OFF");
                if (txactive) transmitter.shutdown();
                if (rxactive) receiver.shutdown();
                rf231Status = STATE_TRX_OFF;
                break;
            case CMD_FORCE_PLL_ON:
                if (DEBUG && printer!=null) printer.println("RF231: FORCE_PLL_ON");
                rf231Status = STATE_PLL_ON;
                break;
            case CMD_RX_ON:
                if (DEBUG && printer!=null) printer.println("RF231: RX_ON");
                rf231Status = STATE_RX_ON;
                if (txactive) transmitter.shutdown();
                if (!rxactive) receiver.startup();
                break;
            case CMD_TRX_OFF:
                if (DEBUG && printer!=null) printer.println("RF231: TRX_OFF");
                if (txactive) transmitter.shutdown();
                if (rxactive) receiver.shutdown();
                rf231Status = STATE_TRX_OFF;
                break;
            case CMD_TX_ON:
                if (DEBUG && printer!=null) printer.println("RF231: PLL_ON");
                rf231Status = STATE_PLL_ON;
                break;
            case CMD_RX_AACK_ON:
                if (DEBUG && printer!=null) printer.println("RF231: RX_AACK_ON");
           //     if (rf231Status == etc.
                rf231Status = STATE_RX_AACK_ON;
                // set TRAC_STATUS bits to INVALID
                registers[TRX_STATE] = (byte) (CMD_TX_START | 0xE0);
                // reset autoack flag
                if (waitingAck) System.out.println("spi sets waitingAck false");
                waitingAck = false;
                if (txactive) transmitter.shutdown();
                if (!rxactive) receiver.startup();
                break;
            case CMD_TX_ARET_ON:
                if (DEBUG && printer!=null) printer.println("RF231: TX_ARET_ON");
                rf231Status = STATE_BUSY_TX_ARET;
                sendingAck = false;
                // set TRAC_STATUS bits to INVALID
                // reset frame retry and csma retry count
                frameRetries = (byte) ((registers[XAH_CTRL_0] & 0xF0) >> 4);
                csmaRetries  = (byte) ((registers[XAH_CTRL_0] & 0x0E) >> 1);
                if (DEBUGC && printer!=null) printer.println("RF231: Frame, csma retries = " + frameRetries + " " + csmaRetries);
                //slottedOperation = (registers[XAH_CTRL_0] & 0x01) != 0; //TODO: slotted operation
                registers[TRX_STATE] = (byte) (CMD_TX_START | 0xE0);
                if (csmaRetries == 7) {
                    csmaRetries = 0;
                    //A value of 7 initiates an immediate transmission with no CSMA
                    if (rxactive) receiver.shutdown();
                    // Transmission starts on the rising edge of SLPTR
                    if (!txactive) transmitter.startup();
                } else if (csmaRetries == 6) {
                    csmaRetries = 0;
                    //6 is reserved
                    if (DEBUGQ && printer!=null) printer.println("RF231: csma retry of 6 is reserved");
                } else {
                    //TODO:should csma should wait for rising edge of slptr?
                    if (!rxactive) receiver.startup();
                    //wait 140 usec (8.75 symbol periods)
                    receiver.clock.insertEvent(ccaDelayEvent, 875*receiver.cyclesPerByte/200);
                }
                break;
            default:
                if (DEBUG && printer!=null) printer.println("RF231: Invalid TRX_CMD, treat as NOP" + val);
                break;
        }

        registers[TRX_STATUS] = (rf231Status);
    }
    /**
     * The <code>resetRegister()</code> method resets the specified register's value
     * to its default.
     *
     * @param addr the address of the register to reset
     */
    void resetRegister(int addr) {
        if (DEBUG && printer!=null) printer.println("RF231: Reset registers");
        byte val = 0x00;
        switch (addr) {
            case TRX_CTRL_0:
                val = (byte) 0x19;
                break;
            case TRX_CTRL_1:
                val = (byte) 0x20;//rf230 sets to zero
                break;
            case PHY_TX_PWR:
                val = (byte) 0xC0;//rf230 sets to zero
                break;
            case PHY_ED_LEVEL:
                val = (byte) 0xFF;//rf230 sets to zero
                break;
            case PHY_CC_CCA:
                val = (byte) 0x2B;
                break;
            case CCA_THRES:
                val = (byte) 0xC7;
                break;
            case RX_CTRL:
                val = (byte) 0xB7;//rf230 sets to 0xBC
                break;
            case SFD_VALUE:
                val = (byte) 0xA7;
                break;
            case TRX_CTRL_2:
                val = (byte) 0x00;//rf230 sets to 0x04
                break;
            case ANT_DIV:
                val = (byte) 0x03;//rf230 sets to zero
                break;
            case IRQ_MASK:
                val = (byte) 0x00;//NB: rf230 sets to 0xFF
                break;
            case VREG_CTRL:
                //val = (byte) 0x00;//reads as 4 when access is possible
                break;
            case BATMON:
                val = (byte) 0x02;//reads as 0x22 when access is possible
                break;
            case XOSC_CTRL:
                val = (byte) 0xF0;
                break;
            case 0x18:
                val = (byte) 0x58;
                break;
            case 0x19:
                val = (byte) 0x55;
                break;
            case PLL_CF:
                val = (byte) 0x57;//rf230 sets to 0x5F
                break;
            case PLL_DCU:
                val = (byte) 0x20;
                break;
            case PART_NUM:
                val = (byte) 0x03;//rf230 sets of 0x02
                break;
            case VERSION_NUM:
                val = (byte) 0x02;
                break;
            case MAN_ID_0:
                val = (byte) 0x1F;
                break;
            case SHORT_ADDR_0:
            case SHORT_ADDR_1:
            case PAN_ID_0:
            case PAN_ID_1:
                val = (byte) 0xFF;//rf230 sets to zeros
                break;
            case XAH_CTRL_0:
                val = (byte) 0x38;
                break;
            case CSMA_SEED_0:
                val = (byte) 0xEA;
                break;
            case CSMA_SEED_1:
                val = (byte) 0x42;//rf230 sets to 0xC2
                break;
            case CSMA_BE:
                val = (byte) 0x53;//rf230 sets to zero
                break;
            case 0x39:
               val = (byte) 0x40;
               break;
        }
        registers[addr] = val;
    }
    /**
     * The <code>postInterrupt()</code> method posts the single RF231 interrupt if the cause
     * is enabled in the IRQ_MASK, or if interrupt polling is enabled.
     *
     * @theBit A byte with bit set corresponding to the cause
     */
     //The same bit numbering is used in the IRQ_MASK and IRQ_STATUS registers
    protected static final byte INT_PLL_LOCK    = 0x01;
    protected static final byte INT_PLL_UNLOCK  = 0x02;
    protected static final byte INT_RX_START    = 0x04;
    protected static final byte INT_TRX_END     = 0x08;
    protected static final byte INT_CCA_ED_DONE = 0x10;
    protected static final byte INT_AMI         = 0x20;
    protected static final byte INT_TRX_UR      = 0x40;
    protected static final byte INT_BAT_LOW     = (byte) 0x80;

    void postInterrupt(byte theBit) {
      if ((registers[IRQ_MASK] & theBit) !=0) {
        //interrupt is enabled
        registers[IRQ_STATUS] |= theBit;
        if (RF231_interrupt > 0) interpreter.setPosted(RF231_interrupt, true);
        if (DEBUGV && printer!=null) printer.println("RF231: interrupt posted");
      } else if ((registers[TRX_CTRL_1] & 0x02) == 1) {
        //polling is enabled
        registers[IRQ_STATUS] |= 0x08;
      }
    }

    /* ------------------------------------------SPI Transfers --------------------------------*/
    protected static final int CMD_R_REG = 0;
    protected static final int CMD_W_REG = 1;
    protected static final int CMD_R_BUF = 2;
    protected static final int CMD_W_BUF = 3;
    protected static final int CMD_R_RAM = 4;
    protected static final int CMD_W_RAM = 5;

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
          // the first byte is the command byte
          //  byte status = getStatus();
            if (Arithmetic.getBit(val, 7)) {
                // register bit is set, bit2 determines write or read
                configCommand = Arithmetic.getBit(val,6) ? CMD_W_REG : CMD_R_REG;
                configRegAddr = val & 0x3f;
            }
            else if (Arithmetic.getBit(val,5)) {
                // frame buffer access
                configCommand = Arithmetic.getBit(val,6) ? CMD_W_BUF : CMD_R_BUF;
            } else {
                // SRAM access
                configCommand = Arithmetic.getBit(val,6) ? CMD_W_RAM : CMD_R_RAM;
            }
            return 0;
        } else if (configByteCnt == 2) {
       //    if (!oscStable.getValue() && configCommand != CMD_R_REG && configCommand != CMD_W_REG) {
                // with crystal oscillator disabled only register access is possible
          //      return 0;
         //   }
            // the second byte is the value to read or write, or the starting SRAM address
            switch (configCommand) {
                case CMD_R_REG:
                    return readRegister(configRegAddr);
                case CMD_W_REG:
                    writeRegister(configRegAddr, val);
                    return 0;
                case CMD_R_BUF:
                    //TODO:diagnostics if FIFO is empty
                    if (DEBUGRX && printer!=null) {
                        byte length = readFIFO(trxFIFO);
                        printer.println("RF231: Sending rx length of " + length);
                        return length;
                    }
                    return readFIFO(trxFIFO);
                case CMD_W_BUF:
                    //The first write clears the FIFO
                    trxFIFO.clear();
                    return writeFIFO(trxFIFO, val, true);
                case CMD_R_RAM:
                case CMD_W_RAM:
                    configRAMAddr = val & 0x7F;
                    return 0;
            }
        } else {
         //   if (!oscStable.getValue() && configCommand != CMD_R_REG && configCommand != CMD_W_REG) {
                // with crystal oscillator disabled only register access is possible
           //     return 0;
          //  }
            // subsequent bytes are valid for fifo and RAM accesses
            switch (configCommand) {
                case CMD_R_BUF:
                    return readFIFO(trxFIFO);
                case CMD_W_BUF:
                    return writeFIFO(trxFIFO, val, true);
                case CMD_R_RAM:
                    return readRAM(configRAMAddr++);
                case CMD_W_RAM:
                    return writeRAM(configRAMAddr++, val);
            }
        }
        return 0;
    }

    protected byte readRAM(int addr) {
        if (addr == AES_RAM_STATUS) {
            return aes.getStatus();
        }
        return trxFIFO.getAbsoluteByte(addr);
    }

    protected byte writeRAM(int addr, byte val) {
        byte return_val = trxFIFO.setAbsoluteByte(addr, val);
        // Transfer of AES-data or AES-key done
        switch (addr) {
            case AES_RAM_CONFIG:
                aes.setConfig(val);
                break;
            case AES_RAM_CONFIG_SHADOW:
                aes.setConfigShadow(val);
                break;
            case AES_RAM_DATA_END:
                byte[] data = new byte[16];
                for (int i = 0; i < 16; i++) {
                    data[i] = trxFIFO.getAbsoluteByte(AES_RAM_DATA + i);
                }
                aes.setData(data);
                break;
            default:
        };

        return return_val;
    }


    protected byte readFIFO(ByteFIFO fifo) {
        byte val = fifo.remove();
        if (DEBUGV && printer!=null) printer.println("RF231 Read FIFO -> " + StringUtil.toMultirepString(val, 8));
        /*
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
        */
        return val;
    }

    protected byte writeFIFO(ByteFIFO fifo, byte val, boolean st) {
        if (DEBUGV && printer!=null) printer.println("rf231 Write FIFO <= " + StringUtil.toMultirepString(val, 8));
        fifo.add(val);
        return 0;
    }

    public Simulator getSimulator() {
        return sim;
    }

    public double getPower() {
        //return power in dBm
        double power=0;
        switch (registers[PHY_TX_PWR]&0x0f) {
            case  0:power=  3.0;break;
            case  1:power=  2.8;break;
            case  2:power=  2.3;break;
            case  3:power=  1.8;break;
            case  4:power=  1.3;break;
            case  5:power=  0.7;break;
//          case  6:power=  0.0;break;
            case  7:power= -1.0;break;
            case  8:power= -2.0;break;
            case  9:power= -3.0;break;
            case 10:power= -4.0;break;
            case 11:power= -5.0;break;
            case 12:power= -7.0;break;
            case 13:power= -9.0;break;
            case 14:power=-12.0;break;
            case 15:power=-17.0;break;
        }
        if (DEBUGV && printer!=null) printer.println("RF231: getPower returns "+ power + " dBm");
        return power;
    }

    public int getChannel() {
        if (DEBUGV && printer!=null) printer.println("RF231: getChannel returns "+ (registers[PHY_CC_CCA] & 0x1F));
        return registers[PHY_CC_CCA] & 0x1F;
    }

    public double getFrequency() {
        double frequency = 2405 + 5*(getChannel() - 11 );
        if (DEBUGV && printer!=null) printer.println("RF231: getFrequency returns "+frequency+" MHz (channel " + getChannel() + ")");
        return frequency;
    }

    public class ClearChannelAssessor implements BooleanView {
        public void setValue(boolean val) {
             if (DEBUGV && printer!=null) printer.println("RF231: set clearchannel");
             // ignore writes.
        }

        public boolean getValue() {
          if (DEBUGV && printer!=null) printer.println("RF231: CleanChannelAssesor.getValue");
          return true;
        }

        public void setValueSetListener(ValueSetListener listener) {
            if (printer != null) {
                // TODO: Implement value-change events for this view
                printer.println("AT86RF231 WARN: ValueSet events for the ClearChannelAssessor are not implemented.");
            }
        }
    }

    public class SPIInterface implements SPIDevice {

        public SPI.Frame exchange(SPI.Frame frame) {
            if (DEBUGV && printer!=null) printer.println("RF231 new SPI frame exchange " + StringUtil.toMultirepString(frame.data, 8));
            if (!CS_pin.level && RSTN_pin.level) {
                // configuration requires CS pin to be held low and RSTN pin to be held high
                return SPI.newFrame(receiveConfigByte(frame.data));
            } else {
                return SPI.newFrame((byte) 0);
            }
        }

        public void connect(SPIDevice d) {
            // do nothing.
        }
    }


    private void pinChange_CS(boolean level) {
        // a change in the CS level always restarts a config command.
        configByteCnt = 0;
    }

    //wakupDelay fires after transition from SLEEP or RESET to TRX_OFF. The nominal delay is 240 usec from SLEEP or
    //32 usec after RESET, but board capacitance can delay the oscillator startup to as much as 1000 usec.
    protected class wakeupDelay implements Simulator.Event {
        public void fire() {
            rf231Status = STATE_TRX_OFF;
            registers[TRX_STATUS] = rf231Status;
            if (DEBUG && printer !=null) printer.println("RF231: WAKE interrupt");
            //TODO: Is this a wake interrupt?
            postInterrupt(INT_PLL_LOCK);
        }
    }
    protected wakeupDelay wakeupDelayEvent = new wakeupDelay();

    private void pinChange_SLPTR(boolean level) {
        if (level) {  //pin was raised
            if (DEBUG && printer!=null) printer.println("RF231 SLPTR pin raised");
            switch (rf231Status) {
                //off -> sleep
                case STATE_TRX_OFF:
                    if (rxactive) receiver.shutdown();
                    if (txactive) transmitter.shutdown();
                    //Frame buffer contents is lost on sleep
                    trxFIFO.clear();
                    stateMachine.transition(0);//change to off state
                    rf231Status = STATE_SLEEP;
                    break;
                case STATE_PLL_ON:
                    rf231Status = STATE_BUSY_TX;
                    break;
                case STATE_BUSY_TX_ARET:
                //    rf231Status = STATE_BUSY_TX_ARET;
                    break;
                default:
                System.out.println("RF231: should not be in this state " + rf231Status);
                    //dont know what to do here
                    break;
            }
        } else {    //pin was lowered
            if (DEBUG && printer!=null) printer.println("RF231 SLPTR pin lowered");
            switch (rf231Status) {
                //Go to idle if sleeping
                case STATE_SLEEP:
                    if (rxactive) receiver.shutdown();
                    if (txactive) transmitter.shutdown();
                    stateMachine.transition(3);//change to on state
                    transmitter.clock.insertEvent(wakeupDelayEvent, 12*transmitter.cyclesPerByte);
                    break;

                case STATE_BUSY_TX:
                case STATE_BUSY_TX_ARET:
                    //Raised after tx initiation
                    break;

                default:
                    if (DEBUGQ && printer!=null) printer.println("RF231: SLP pin lowered but not sleeping, state = " + rf231Status);
                    //dont know what to do here
                    break;
            }
        }
        registers[TRX_STATUS] = (rf231Status);
    }

    private void pinChange_RSTN(boolean level) {
        if (!level) {
            // high->low indicates reset
            reset();
            stateMachine.transition(1);//change to power down state
            if (DEBUGQ && printer!=null) printer.println("RF231 reset by RSTN pin");
        }
    }
    /* ------------------------------------------CRC Computation --------------------------------*/
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
    //TODO: which is faster
/*
    int crcAccumulatet(int crc, byte val) {
	    crc = ((crc >> 8) & 0xff) | (crc << 8) & 0xffff;
	    crc ^= (val & 0xff);
	    crc ^= (crc & 0xff) >> 4;
	    crc ^= (crc << 12) & 0xffff;
	    crc ^= (crc & 0xff) << 5;
	    crc = crc & 0xffff;
        return crc;
    }
*/
    short crcAccumulate(short crc, byte val) {
        int i = 8;
        crc = (short) (crc ^ val << 8);
        do {
            if ((crc & 0x8000) != 0) crc = (short) (crc << 1 ^ 0x1021);
            else crc = (short) (crc << 1);
        } while (--i > 0);
        return crc;
    }

    /* ------------------------------------------Transmitter-------------------------------------*/
    public class Transmitter extends Medium.Transmitter {
        private static final int TX_IN_PREAMBLE = 0;
        private static final int TX_SFD = 1;
        private static final int TX_LENGTH = 3;
        private static final int TX_IN_PACKET = 4;
        private static final int TX_CRC_1 = 5;
        private static final int TX_CRC_2 = 6;
        private static final int TX_END = 7;
        private static final int TX_WAIT = 8;

        protected int state, counter, length;
        protected short crc;
        protected boolean waitForAck;

        //ackTimeOut fires 54 symbol periods (864 usec) after tx transition to rx to receive an ack
        protected class ackTimeOut implements Simulator.Event {
            public void fire() {
            //  if (activated) {
                    waitingAck = false;
                    if (!handledAck) {
                        //Ack not received, abort or retry
                        handledAck = true;
                        if (false && frameRetries != 0) {
                            //Autoretry
                            System.out.println("tx_aret #" + frameRetries);
                            frameRetries--;
                             if (rxactive) receiver.shutdown();
                             if (!txactive) transmitter.startup();
                        } else {
                           //Set TRAC_STATUS to no ack and return to TX_ARET_ON
                            registers[TRX_STATE] = (byte) (0xA0 | (registers[TRX_STATE] & 0x1F));
                            rf231Status = STATE_TX_ARET_ON;
                            registers[TRX_STATUS] = (rf231Status);
                            state = TX_WAIT;//???

                            if (!rxactive) printer.println("rx1 not active before shutdown");
                            receiver.shutdown();
                            if (DEBUGA & printer !=null) printer.println("RF2311: TRX_END interrupt, no ack");
                            postInterrupt(INT_TRX_END);
                        }
                    }
            //  }
            }
        }
        protected ackTimeOut ackTimeOutEvent = new ackTimeOut();

        public Transmitter(Medium m) {
            super(m, sim.getClock());
        }

        public byte nextByte() {
            if (rxactive) printer.println("ractive while transmitting");
            if (!txactive) printer.println("tx not active while transmitting");
            byte val = 0;
            switch (state) {
                case TX_IN_PREAMBLE:
                    //always 4 octects of zero
                    if (++counter >= 4) state = TX_SFD;
                    break;
                case TX_SFD:
                   // val = registers[SFD_VALUE];
                    val = (byte) 0x7A;//sky radio compatibility
                  //  val = (byte) 0xA7;
                    state = TX_LENGTH;
                    break;
                case TX_LENGTH:
                    if (sendingAck) {
                        length = 5;
                    } else {//data frame
                        // length is the first byte in the FIFO buffer
                        length = trxFIFO.remove();
                    }
                    if (DEBUGTX && printer!=null) printer.println("RF231: Tx frame length " + length);
                    state = TX_IN_PACKET;
                    counter = 0;
                    crc = 0;
                    val = (byte) length;
                    break;
                case TX_IN_PACKET:
                    if (sendingAck) {
                        switch (counter) {
                            case 0://FCF_low
                                val = 2;
                             // val = 0x12;  //TODO:handle pending flag
                                 break;
                            case 1://FCF_hi
                                val = 0;
                                break;
                            case 2://Sequence number
                                val = DSN;
                                waitForAck = false;
                                state = TX_END;
                                break;
                        }
                        counter++;
                    } else {
                        val = trxFIFO.remove();
                        counter++;
                        if (counter == 1) {
                            //check the ack request bit in the FCF
                            waitForAck = (val & 0x20) != 0;
                        }
                    }
                    //Calculate CRC and switch state if necessary
                    if ((registers[TRX_CTRL_1] & 0x20) !=0) {  //Test TX_AUTO_CRC_ON bit
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
            if (DEBUGTX && printer!=null) printer.println("RF231 " + StringUtil.to0xHex(val, 2) + " --------> ");
            // common handling of end of transmission
            if (state == TX_END) {
                //Set the TRAC status bits in the TRX_STATE register
                //0 success 1 pending 2 waitforack 3 accessfail 5 noack 7 invalid
                if (waitForAck && (rf231Status == STATE_BUSY_TX_ARET)) {
                    //Show waiting for ack, and switch to rx mode
                    waitingAck = true;
                    handledAck = false;
                    registers[TRX_STATE] = (byte) (0x40 | (registers[TRX_STATE] & 0x1F));
                    //wait 54 symbol periods (864 usec, 27 bytes) to receive the ack
                    clock.insertEvent(ackTimeOutEvent, 27*cyclesPerByte);
                    if (!txactive) printer.println("tx not active at txend");
                    transmitter.shutdown();
                    if (rxactive) printer.println("rx active at txend");
                    receiver.startup();
                    state = TX_WAIT;
                    return val;
                } else {
                    rf231Status = STATE_PLL_ON;
                    registers[TRX_STATUS] = (rf231Status);
                    registers[TRX_STATE] = (byte) (registers[TRX_STATE] & 0x1F);
                    transmitter.shutdown();
                }
                postInterrupt(INT_TRX_END);
                // transmitter stays in this state until it is really switched off
                state = TX_WAIT;
            }
            return val;
        }

        void startup() {
            if (!txactive) {
                txactive = true;
                // the PLL lock time is implemented as the leadCycles in Medium
                stateMachine.transition(6 + 15 - (registers[PHY_TX_PWR]&0x0f));//change to Tx(power) state
                state = TX_IN_PREAMBLE;
                counter = 0;
                beginTransmit(getPower(),getFrequency());
                if (DEBUGTX && printer!=null) printer.println("RF231: TX Startup");
            } else printer.println("tx startup while active");
        }

        void shutdown() {
            // note that the stateMachine.transition() is not called here any more!
            if (txactive) {
                txactive = false;
                endTransmit();
                     //           trxFIFO.clear();
                if (DEBUGTX && printer!=null) printer.println("RF231: TX shutdown");
                stateMachine.transition(3);//change to RX TODO:FIX this
            }else printer.println("tx shutdown while not active");
        }
    }

    /* ------------------------------------------Receiver------------------------------------*/
    private static final int RECV_SFD_SCAN = 0;
    private static final int RECV_SFD_MATCHED_1 = 1;
    private static final int RECV_SFD_MATCHED_2 = 2;
    private static final int RECV_IN_PACKET = 3;
    private static final int RECV_CRC_1 = 4;
    private static final int RECV_CRC_2 = 5;
    private static final int RECV_END_STATE = 6;
    private static final int RECV_OVERFLOW = 7;
    private static final int RECV_WAIT = 8;

    //Address recognition variables
    protected byte[] PANId;
    protected byte[] macPANId = new byte[2];
    protected byte[] ShortAddr;
    protected byte[] macShortAddr = new byte[2];
    protected static final byte[] SHORT_BROADCAST_ADDR = {-1, -1};
    protected byte[] LongAdr;
    protected byte[] IEEEAdr = new byte[8];
    protected static final byte[] LONG_BROADCAST_ADDR = {-1, -1, -1, -1, -1, -1, -1, -1};

    public class Receiver extends Medium.Receiver {
        protected int state;
        protected int counter;
        protected int length;
        protected short crc;
        protected byte crcLow;

        public Receiver(Medium m) {
            super(m, sim.getClock());
        }

        public byte nextByte(boolean lock, byte b) {
           //The receiver will continue to get some byte runon after the force TRX_OFF command
           // if (!rxactive) printer.println("rx not active while receiving");
            if (!rxactive) return 0;
            if (txactive) printer.println("txactive while receiving");

            if (state == RECV_END_STATE) {
                state = RECV_SFD_SCAN; // to prevent loops when calling shutdown/endReceive
                // packet ended before
                if (sendingAck && lastCRCok) { //send ack?
                    if (DEBUGRX && printer!=null) printer.println("RF231: sendack");
                    receiver.shutdown();
                    transmitter.startup();
                } else {
                    if (lock) {
                        // the medium is still locked, so there could be more packets!
                        if (DEBUGQ && printer!=null) printer.println("RF231: still locked");
                        // fire the probes manually
                        if (probeList != null) {
                            System.out.println("probeList is not null!");
                            probeList.fireAfterReceiveEnd(Receiver.this);
                        }
                    }
                }
                return b;
            }


            if (!lock) {
                if (DEBUGRX && printer!=null) printer.println("RF231 notlock, state= "+state);
                // the reception lock has been lost
                switch (state) {
                    case RECV_SFD_MATCHED_2:
                    case RECV_IN_PACKET:
                    case RECV_CRC_1:
                    case RECV_CRC_2:
                         if (DEBUGQ && printer != null) printer.println("RF231: PLL_UNLOCK Interrupt");
                        postInterrupt(INT_PLL_UNLOCK);
                        //packet lost in middle -> drop frame
                        // fall through
                    case RECV_SFD_SCAN:
                    case RECV_SFD_MATCHED_1: // packet has just started
                        if (DEBUGRX && printer != null) printer.println("RFA1: lock lost");
                        state = RECV_SFD_SCAN;
                        if (rf231Status == STATE_BUSY_RX_AACK) {
                            rf231Status = STATE_RX_AACK_ON;
                        } else if (rf231Status == STATE_BUSY_RX) {
                            rf231Status = STATE_RX_ON;
                        } else if (rf231Status == STATE_BUSY_TX_ARET) {
                            //that's one ack were not going to get
                        } else if (rf231Status == STATE_RX_AACK_ON) {
                            //we are getting jammed
                        } else if (rf231Status == STATE_RX_ON) {
                            //we are getting jammed
                        } else if (rf231Status == STATE_PLL_ON) {
                            //whats up with that
                        } else {
                            if (DEBUGQ && printer!=null) printer.println("RF231: Bad state when lock lost " + rf231Status);
                        }
                        registers[TRX_STATUS] = (byte) (rf231Status | (registers[TRX_STATUS] & 0xE0));

        /*
                        if (DEBUGRX && printer!=null) printer.println("RF231 packet started");
                        state = RECV_SFD_SCAN;
                        //rx start invalidates CRC and ED register
                        registers[PHY_RSSI] &= 0x7f; //Clear RX_CRC_VALID
                        registers[PHY_ED_LEVEL] = (byte) 0xff;
                        //switch to busy state unless waiting for ack in BUSY_TX_ARET
                        if (rf231Status == STATE_RX_AACK_ON) {
                            rf231Status = STATE_BUSY_RX_AACK;
                        } else if (rf231Status == STATE_RX_ON) {
                        printer.println("setting busyrx state her!");
                            rf231Status = STATE_BUSY_RX;
                        }
                        registers[TRX_STATUS] = (byte) (rf231Status | (registers[TRX_STATUS] & 0xE0));
                        postInterrupt(INT_RX_START);
                   */
                        break;
                    default:
                        if (DEBUGQ && printer!=null) printer.println("RF231: RX badstate " + state);
                                                state = RECV_SFD_SCAN;
                        break;

                }
                return b;
            }
            if (txactive)  {
                printer.println("txactive while receiving5 " + state);
                transmitter.shutdown();
            }
            if (DEBUGRX && printer!=null) printer.println("RF231 <======== " + StringUtil.to0xHex(b, 2));

            switch (state) {
                case RECV_SFD_MATCHED_1:
                   // if (b == (byte) 0xA7) {
                    if (b == (byte) 0x7A) {  //sky compatibility
                    // check against the second byte of the SYNCWORD register.
                        state = RECV_SFD_MATCHED_2;
                        //If waiting on ack in BUSY_TX_ARET don't make any status changes
                        if (waitingAck) break;
                        if (rf231Status == STATE_TX_ARET_ON) printer.println("busy txaret but not waiting for ack");
                        if (rf231Status == STATE_BUSY_TX_ARET) {
                      //       printer.println("busy txaret but not waiting for ack");
                            //ack timeout occurred. Probably this is not our ack anyway
                            if (rxactive) receiver.shutdown();
                            return 0;
                        }
                        //rx start invalidates CRC and ED register
                        registers[PHY_RSSI] &= 0x7f; //Clear RX_CRC_VALID
                         registers[PHY_ED_LEVEL] = (byte) 0xff;
                        if (rf231Status == STATE_RX_AACK_ON) {
                            rf231Status = STATE_BUSY_RX_AACK;
                        } else if (rf231Status == STATE_RX_ON) {
                            rf231Status = STATE_BUSY_RX;
                        } else if (rf231Status == STATE_BUSY_RX_AACK) {
                            printer.println("already in BUSY_RX_AACK on, probably a missed ack");
                        } else if (rf231Status == STATE_BUSY_RX) {
                            printer.println("already in BUSY_RX, should not be");
                        } else {
                            printer.println("not in a receive state! " + rf231Status + " " + registers[TRX_STATUS]);
                            if (rf231Status == STATE_SLEEP) {
                                printer.println("RF231: Sleep while receiving");
                                //we were turned off while receiving
                                return 0;
                            }
                            //hmmm whats going on
                            printer.println("rxactive = " + rxactive + "   txactive = " + txactive);
                            return 0;
                        }
                        registers[TRX_STATUS] = (byte) (rf231Status | (registers[TRX_STATUS] & 0xE0));
                        if (DEBUGRX && printer != null) printer.println("RF231: RX_START interrupt");
                        postInterrupt(INT_RX_START);
                        break;
                    }
                    // fallthrough if we failed to match the second byte
                    // and try to match the first byte again.
                case RECV_SFD_SCAN:
                    // check against the first byte of the SYNCWORD register.
                    if (b == (byte) 0x00) {
                        state = RECV_SFD_MATCHED_1;
                    } else {
                        state = RECV_SFD_SCAN;
                    }
                    break;

                case RECV_SFD_MATCHED_2:
                    // SFD matched. read the length from the next byte.
                    length = b & 0x7f;
                    if (length == 0) {  // ignore frames with 0 length
                        state = RECV_SFD_SCAN;
                        break;
                    }
                    counter = 0;
                    state = RECV_IN_PACKET;
                    if (waitingAck) {
                        if (length != 5) {
                         //not an ack, show failure and abort rx
                          //  waitingAck = false;
                            if (DEBUGA && printer!=null) printer.println("RF231: Expecting ack, got something else");
                            handledAck = true;
                            //Status goes from BUSY_TX_ARET to TX_ARET_ON
                            rf231Status = STATE_TX_ARET_ON;
                            //TODO:should upper bits change?
                            registers[TRX_STATUS] = (byte) (rf231Status | (registers[TRX_STATUS] & 0xE0));
                            //Set TRAC_STATUS bits to failure
                            registers[TRX_STATE] = (byte) (0xA0 | (registers[TRX_STATE] & 0x1F));
                            state = RECV_SFD_SCAN;
                            //Post the TRX_END interrupt
                            postInterrupt(INT_TRX_END);
                            if (!rxactive) {
                                printer.println("rx2 not active before shutdown");
                                return(0);
                            }
                            receiver.shutdown();
                        }
                    } else {
                        // Start transferring bytes to FIFO
                        trxFIFO.clear();
                        trxFIFO.add((byte) length);
                        crc = 0;
                        //Update the energy detect register
                        //This should have been an average rssi over the previous 8 symbols
                        //but basically is 3 times PHY_RSSI
                        registers[PHY_ED_LEVEL] = (byte)((registers[PHY_RSSI] & 0x1f) * 3);
                    }
                    break;

                case RECV_IN_PACKET:
                    // we are in the body of the packet.
                    counter++;
                    if (waitingAck) {
                        if (handledAck) {
                            printer.println("waiting but handled during reception");
                        }
                        //show success after complete ack reception
                        //TODO: verify checksum
                        if (counter == 5) {
                            if (DEBUGA && printer!=null) printer.println("RF231: Got ack");
                            handledAck = true;
                            rf231Status = STATE_TX_ARET_ON;
                            registers[TRX_STATUS] = (byte) (rf231Status | (registers[TRX_STATUS] & 0xE0));
                            registers[TRX_STATE] = (byte) (registers[TRX_STATE] & 0x1F);
                            if (!rxactive) printer.println("rf231 rx3 not active before shutdown");
                            if (txactive) printer.println("rf231 tx active before startup");
                            receiver.shutdown();
                          //  transmitter.startup();
                            if (DEBUGA & printer !=null) printer.println("RF231: TRX24 TX_END interrupt, ack");
                            postInterrupt(INT_TRX_END);
                            return(b);
                        } else if (counter > 5) {
                            if (DEBUGA && printer!=null) printer.println("RF231: can an not get here");
                        }
                        break;
                    }
                    trxFIFO.add(b);
                    //Address Recognition and sequence number
                    if (counter <= 13) {
                        boolean satisfied = matchAddress(b, counter);
                        // address match enabled only in RA_AACK mode TODO: should be BUSY_RX_AACK
                        if (rf231Status == STATE_RX_AACK_ON) {
                            // is AACK_I_AM_COORD set?
                            if ((registers[CSMA_SEED_1] & 0x04) == 0) {
                                if (!satisfied) {
                                    //reject frame
                                    if (DEBUGRX && (printer!=null)) printer.println("Dropped, no address match with counter "+ counter);
                                    // wait for end of packet
                                    state = RECV_WAIT;
                                    break;
                                } else if (counter == 13) { //TODO: use the correct number based on short/long address
                                    postInterrupt(INT_AMI);
                                }
                            }
                        }
                    }

                    // no overflow occurred and address ok
                 //   if (autoCRC.getValue()) {
                    if (true) { //todo: check crc request bits if any
                        crc = crcAccumulate(crc, (byte) reverse_bits[(b) & 0xff]);

                        if (counter == length - 2) {
                            // transition to receiving the CRC.
                            state = RECV_CRC_1;
                        }
                    } else if (counter == length) {
                        // no AUTOCRC, but reached end of packet.
                        clearBER();  // will otherwise be done by getCorrelation() in state RECV_CRC_2
                        lastCRCok = false;
                        state = RECV_END_STATE;
                    }

                    break;
                case RECV_CRC_1:
                    trxFIFO.add(b);
                    crcLow = b;
                    state = RECV_CRC_2;
                    break;

                case RECV_CRC_2:
                    state = RECV_END_STATE;
                    trxFIFO.add(b);
                    crcLow = (byte)reverse_bits[(crcLow) & 0xff];
                    b = (byte)reverse_bits[(b) & 0xff];
                    short crcResult = Arithmetic.word(b, crcLow);

                    //LQI is written in this position
                    b = (byte) ((byte)getCorrelation() & 0x7f);
                    if (crcResult == crc) { //TODO:LQI increases when CRC valid?
                        b |= 0x80;
                        lastCRCok = true;
                        if (DEBUGRX && printer!=null) printer.println("RF231: CRC passed");
                        registers[PHY_RSSI] |= 0x80; //Set RX_CRC_VALID
                      //  registers[PHY_ED_LEVEL] = update?
                    }
                    else {
                        // Frame is not rejected if CRC is invalid
                        // reset ACK flags set by the SACK/SACKPEND commands since ACK is only sent when CRC is valid
                        lastCRCok = false;
                //        SendAck = SENDACK_NONE;
                        if (DEBUGA && printer!=null) printer.println("RF231: CRC received " + StringUtil.to0xHex(crcResult, 4) + " calculated " + StringUtil.to0xHex(crc, 4));
                    }

                    trxFIFO.add(b);
                    postInterrupt(INT_TRX_END);

                    if (rf231Status == STATE_BUSY_TX_ARET) {
                            printer.println(" STATE_BUSY_TX_ARET here! " + rf231Status + " " + registers[TRX_STATUS]);
                    } else if (rf231Status == STATE_TX_ARET_ON) {
                            System.out.println("TX_ARET state in receiver here");
                    }
                    if (lastCRCok && (rf231Status == STATE_BUSY_RX_AACK) && (trxFIFO.getRelativeByte(1) & 0x20) == 0x20) {
                        //send ack if we are not receiving ack frame
                        if ((registers[CSMA_SEED_1] & 0x10) == 0) {
                            if ((trxFIFO.getRelativeByte(1) & 0x07) != 2) {
                                sendingAck = true;
                                handledAck = false;
                            }
                        }
                    } else {
                        if (DEBUGA && printer!=null) {
                            if (!lastCRCok) printer.println("RF231: No ack, CRC failed");
                            if (rf231Status != STATE_BUSY_RX_AACK) printer.println("RF231: status no longer rxaack: "+ rf231Status);
                        }
                        if (rf231Status == STATE_BUSY_RX_AACK) {
                            rf231Status = STATE_RX_AACK_ON;
                        } else if (rf231Status == STATE_BUSY_RX) {
                              //received frame during cca?
                               rf231Status = STATE_RX_ON;
                        } else {
                            if (DEBUGQ && printer!=null) printer.println("RF231: not busy at end of reception");
                            rf231Status = STATE_RX_AACK_ON;
                        }
                         registers[TRX_STATUS] = (byte) (rf231Status | (registers[TRX_STATUS] & 0xE0));
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
                     //   SendAck = SENDACK_NONE;  // just in case we received SACK(PEND) in the meantime
                    }
                    break;
                default:
                    if (DEBUGRX && printer!=null) printer.println("RF231: Unknown state in receiver");
                    break;
            }
            return b;
        }

        private boolean matchAddress(byte b, int counter) {
            if (counter > 1 && (trxFIFO.getRelativeByte(1) & 0x04) == 4) {
                // no further address decoding is done for reserved frames
                return true;
            }
            switch (counter) {
                case 1://frame type subfield contents an illegal frame type?
                    if ((trxFIFO.getRelativeByte(1) & 0x04) == 4)
                        return false;
                    break;
                case 3://Sequence number
                    if ((trxFIFO.getRelativeByte(1) & 0x07) != 0 && (trxFIFO.getRelativeByte(1) & 0x04) != 4) {
                        DSN = b;
                        lastCRCok = false;  // we have a new DSN now. Therefore, we cannot send an ACK for the last frame any more.
                    }
                    break;
                case 5:
                    PANId = trxFIFO.getRelativeField(4, 6);
                    macPANId[0] = registers[PAN_ID_0];
                    macPANId[1] = registers[PAN_ID_1];
                  //  printer.println("PANId " + PANId[0] + " " +PANId[1]);
                  //  printer.println("macPANId " + macPANId[0]+" "+macPANId[1]);
                    if (((trxFIFO.getRelativeByte(2) >> 2) & 0x02) != 0) {//DestPANId present?
                        if (!Arrays.equals(PANId, macPANId) && !Arrays.equals(PANId, SHORT_BROADCAST_ADDR)) {
                            if (DEBUGRX && printer!=null) printer.println("RF231: Not broadcast and PANid does not match");
                            return false;
                        }
                    } else
                    if (((trxFIFO.getRelativeByte(2) >> 2) & 0x03) == 0) {//DestPANId and dest addresses are not present
                        if (((trxFIFO.getRelativeByte(2) >> 6) & 0x02) != 0) {//SrcPANId present
                            if ((trxFIFO.getRelativeByte(1) & 0x07) == 0) {//beacon frame: SrcPANid shall match macPANId unless macPANId = 0xffff
                              //  if (!Arrays.equals(PANId, macPANId) && !Arrays.equals(macPANId, SHORT_BROADCAST_ADDR) && !BCN_ACCEPT.getValue())
                                if (!Arrays.equals(PANId, macPANId) && !Arrays.equals(macPANId, SHORT_BROADCAST_ADDR))
                                    return false;
                            } else
                            if (((trxFIFO.getRelativeByte(1) & 0x07) == 1) || ((trxFIFO.getRelativeByte(1) & 0x07) == 3)) {//data or mac command
                                if (rf231Status == STATE_BUSY_RX_AACK) {
                                    // is AACK_I_AM_COORD set?
                                    if ((registers[CSMA_SEED_1] & 0x04) == 0) return false;
                                }
                                if (!(Arrays.equals(PANId,macPANId))) return false;
                            }
                        }
                    }
                    break;
                case 7://If 32-bit Destination Address exits check if  match
                    if (((trxFIFO.getRelativeByte(2) >> 2) & 0x03) == 2) {
                        ShortAddr = trxFIFO.getRelativeField(6, 8);
                        macShortAddr[0] = registers[SHORT_ADDR_0];
                        macShortAddr[1] = registers[SHORT_ADDR_1];
                     //   printer.println("shortadr " + ShortAddr[0]+ShortAddr[1]);
                    //    printer.println("macshortaddr " + macShortAddr[0]+" "+macShortAddr[1]);
                        if (!Arrays.equals(ShortAddr, macShortAddr) && !Arrays.equals(ShortAddr, SHORT_BROADCAST_ADDR))
                            return false;
                    }
                    break;
             // case 12://If 64-bit Destination Address exits check if match
                //dak bumped this up a byte, works with sky. The SFD change caused this?
                case 13://If 64-bit Destination Address exits check if match
                    if (((trxFIFO.getRelativeByte(2) >> 2) & 0x03) == 3) {
                     // LongAdr = rxFIFO.peekField(8, 16);
                        LongAdr = trxFIFO.getRelativeField(6, 14);//dak
                        IEEEAdr[0] = registers[IEEE_ADDR_0];
                        IEEEAdr[1] = registers[IEEE_ADDR_1];
                        IEEEAdr[2] = registers[IEEE_ADDR_2];
                        IEEEAdr[3] = registers[IEEE_ADDR_3];
                        IEEEAdr[4] = registers[IEEE_ADDR_4];
                        IEEEAdr[5] = registers[IEEE_ADDR_5];
                        IEEEAdr[6] = registers[IEEE_ADDR_6];
                        IEEEAdr[7] = registers[IEEE_ADDR_7];
                        if (!Arrays.equals(LongAdr, IEEEAdr) && !Arrays.equals(LongAdr, LONG_BROADCAST_ADDR)) {
                            if (DEBUGRX && printer != null) {
                              printer.println(" longadr " + LongAdr[0]+LongAdr[1]+LongAdr[2]+LongAdr[3]+LongAdr[4]+LongAdr[5]+LongAdr[6]+LongAdr[7]);
                              printer.println(" IEEEAdr " + IEEEAdr[0]+IEEEAdr[1]+IEEEAdr[2]+IEEEAdr[3]+IEEEAdr[4]+IEEEAdr[5]+IEEEAdr[6]+IEEEAdr[7]);
                            }
                            return false;
                        }
                    }
                    break;
            }
            return true;
        }
        void startup() {
            if (!rxactive) {
                rxactive = true;
                stateMachine.transition(3);//change to receive state TODO:low sensitivity state
                state = RECV_SFD_SCAN;
                clearBER();
                beginReceive(getFrequency());
             //   clock.insertEvent(rssiValidEvent, 4*cyclesPerByte);  // 8 symbols = 4 bytes
                if (DEBUGRX && printer!=null) printer.println("RF231: RX startup");
            } else printer.println("receiver startup while active");
        }

        void shutdown() {
            if (rxactive) {
                rxactive = false;
                endReceive();
                state = RECV_SFD_SCAN;
                stateMachine.transition(1);//change to idle state
             //   setRssiValid(false);
                if (DEBUGRX && printer!=null) printer.println("RF231: RX shutdown");
            } else printer.println("receiver shutdown while not active");
        }
    /* -----------------------------------RSSI, LQI, PER------------------------------------*/
        //LUT for max and min correlation values depending on PER
        private final int [] Corr_MAX = {110,109,109,109,107,107,107,107,107,
        107,107,107,103,102,102,102,101,101,101,101,99,94,92,94,101,97,98,97,97,97,97,97,
        94,94,94,94,94,94,94,94,94,94,94,94,92,89,89,89,89,89,88,88,88,88,88,86,86,86,
        86,86,86,86,86,86,85,85,85,85,85,85,83,83,83,83,83,83,83,83,79,78,78,78,78,78,
        76,76,76,74,74,74,74,74,74,74,74,74,74,66,65,65,65};
        private final int [] Corr_MIN = {95,95,94,91,90,90,89,89,89,88,88,88,82,
        82,82,82,76,76,76,76,76,76,74,74,74,74,74,74,72,72,72,72,72,72,72,72,69,69,69,69,
        69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,67,67,67,67,67,67,65,65,65,65,65,
        65,65,64,64,63,63,63,63,63,63,63,63,63,61,61,61,60,60,60,58,58,56,56,56,55,55,55,
        50,50,50,50,50,50,50};
        protected double Correlation;
/*
        private void setRssiValid (boolean v){
            if (DEBUGA && printer!=null) printer.println("RF231: setrssivalid "+ rssi_val);
            registers[PHY_RSSI] |= 0x80; //Set RX_CRC_VALID
        }

        private boolean getRssiValid (){
            if (DEBUGA && printer!=null) printer.println("RF231: getRssivalid");
            //RF231 RSSI is always valid in non-extended mode
            return true;
        }
 */
        public double getCorrelation (){
            int PERindex = (int)(getPER()*100);
            Random random = new Random();
            //get the range, casting to long to avoid overflow problems
            long range = (long)Corr_MAX[PERindex] - (long)Corr_MIN[PERindex] + 1;
            // compute a fraction of the range, 0 <= frac < range
            long fraction = (long)(range * random.nextDouble());
            double corr = fraction + Corr_MIN[PERindex];
            if (DEBUGRX && printer!=null) printer.println("RF231: returncorr " + corr);
            return corr;
        }

        public void setRSSI (double Prec){
            //RSSI register in units of 3dBm, 0 <-90dBm, 28 >=-10 dBm
            int rssi_val = (((int) Math.rint(Prec) + 90) / 3) +1;
            if (rssi_val < 0) rssi_val = 0;
            if (rssi_val > 28) rssi_val = 28;
    //      if (DEBUGRX && printer!=null) if (rssi_val > 0) printer.println("RF231: setrssi " + rssi_val);
            registers[PHY_RSSI] = (byte) (rssi_val | (registers[PHY_RSSI] & 0xE0));
        }

        public double getRSSI (){
            int rssi_val = registers[PHY_RSSI] & 0x1F;
            return -90 + 3*(rssi_val-1);
        }

        public void setBER (double BER){
        //  if (DEBUGRX && printer!=null) printer.println("RF231: setBER");
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
         // if (DEBUGRX && printer!=null) printer.println("RF231: getPER " + PER);
            return PER;
        }

        public void clearBER() {
            BERcount = 0;
            BERtotal = 0.0D;
        }


 /**
         * The <code>RssiValid</code> class implements a Simulator Event
         * that is fired when the RSSI becomes valid after 8 symbols
         */
         /*
        protected class RssiValid implements Simulator.Event {

            public void fire() {
                if (activated) {
              //      setRssiValid(true);
                }
            }
        }
        protected RssiValid rssiValidEvent = new RssiValid();
*/
    }

    private long toCycles(long us) {
        return us * sim.getClock().getHZ() / 1000000;
    }

   // public static Medium createMedium(Synchronizer synch, Medium.Arbitrator arbitrator) {
    public Medium createMedium(Synchronizer synch, Medium.Arbitrator arbitrator) {
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

    /**
     * The <code>RF231Pin</code>() class models pins that are inputs and outputs to the RF231 chip.
     */
    public class RF231Pin implements Microcontroller.Pin.Input, Microcontroller.Pin.Output {

        protected LinkedList<Microcontroller.Pin.InputListener> listeners = new LinkedList<>();
        protected final String name;
        protected boolean level;

        public RF231Pin(String n) {
            name = n;
        }

        public void write(boolean level) {
            if (this.level != level) {
                // level changed
                this.level = level;
                if (this == CS_pin) pinChange_CS(level);
                else if (this == RSTN_pin) pinChange_RSTN(level);
                else if (this == SLPTR_pin) pinChange_SLPTR(level);
                if (DEBUGV && printer!=null) printer.println("RF231 Write pin " + name + " -> " + level);
            }
        }

        public boolean read() {
            if (DEBUGV && printer!=null) printer.println("RF231 Read pin " + name + " -> " + level);
            return level;
        }
        public void registerListener(Microcontroller.Pin.InputListener listener) {
             listeners.add(listener);
         }

         public void unregisterListener(Microcontroller.Pin.InputListener listener) {
             listeners.remove(listener);
         }
    }

    public class RF231Output extends Microcontroller.Pin.ListenableBooleanViewInput implements Microcontroller.Pin.Input {

        //protected BooleanView level;
        protected final String name;

        public RF231Output(String n, BooleanView lvl) {
            super(lvl);
            name = n;
        }

        public boolean read() {
            boolean val = super.read();
            if (DEBUGV && printer!=null)
              printer.println("RF231 Read (output) pin " + name + " -> " + val);
            return val;
        }
    }

    public static String regName(int reg) {
        switch (reg) {
            case TRX_STATUS:
                return "TRX_STATUS  ";
            case TRX_STATE:
                return "TRX_STATE   ";
            case TRX_CTRL_0:
                return "TRX_CTRL_0  ";
            case TRX_CTRL_1:
                return "TRX_CTRL_1  ";
            case PHY_TX_PWR:
                return "PHY_TX_PWR  ";
            case PHY_RSSI:
                return "PHY_RSSI    ";
            case PHY_ED_LEVEL:
                return "PHY_ED_LEVEL";
            case PHY_CC_CCA:
                return "PHY_CC_CCA  ";
            case CCA_THRES:
                return "CCA_THRES   ";
            case RX_CTRL:
                return "RX_CTRL     ";
            case SFD_VALUE:
                return "SFD_VALUE   ";
            case TRX_CTRL_2:
                return "TRX_CTRL_2  ";
            case ANT_DIV:
                return "ANT_DIV     ";
            case IRQ_MASK:
                return "IRQ_MASK    ";
            case IRQ_STATUS:
                return "IRQ_STATUS  ";
            case VREG_CTRL:
                return "VREG_CTRL   ";
            case BATMON:
                return "BATMON      ";
            case XOSC_CTRL:
                return "XOSC_CTRL   ";
            case RX_SYN:
                return "RX_SYN      ";
            case XAH_CTRL_1:
                return "XAH_CTRL_1  ";
            case PLL_CF:
                return "PLL_CF      ";
            case PLL_DCU:
                return "PLL_DCU     ";
            case PART_NUM:
                return "PART_NUM    ";
            case VERSION_NUM:
                return "VERSION_NUM ";
            case MAN_ID_0:
                return "MAN_ID_0    ";
            case MAN_ID_1:
                return "MAN_ID_1    ";
            case SHORT_ADDR_0:
                return "SHORT_ADDR_0";
            case SHORT_ADDR_1:
                return "SHORT_ADDR_1";
            case PAN_ID_0:
                return "PAN_ID_0    ";
            case PAN_ID_1:
                return "PAN_ID_1    ";
            case IEEE_ADDR_0:
                return "IEEE_ADDR_0 ";
            case IEEE_ADDR_1:
                return "IEEE_ADDR_1 ";
            case IEEE_ADDR_2:
                return "IEEE_ADDR_2 ";
            case IEEE_ADDR_3:
                return "IEEE_ADDR_3 ";
            case IEEE_ADDR_4:
                return "IEEE_ADDR_4 ";
            case IEEE_ADDR_5:
                return "IEEE_ADDR_5 ";
            case IEEE_ADDR_6:
                return "IEEE_ADDR_6 ";
            case IEEE_ADDR_7:
                return "IEEE_ADDR_7 ";
            case XAH_CTRL_0:
                return "XAH_CTRL_0  ";
            case CSMA_SEED_0:
                return "CSMA_SEED_0 ";
            case CSMA_SEED_1:
                return "CSMA_SEED_1 ";
            case CSMA_BE:
                return "CSMA_BE     ";
            default:
                return StringUtil.to0xHex(reg, 2) + "    ";
        }
    }
    enum AESKeyStatus {
        INIT, ENCRYPT, DECRYPT;
    }
    /**
     * Performs AES-Operations
     */
    protected class AESModule {

        static final int AES_MODE_ECB = 0;
        static final int AES_MODE_KEY = 1;
        static final int AES_MODE_CBC = 2;

        static final int AES_DIR_ENCRYPTION  = 0;
        static final int AES_DIR_DECRYPTION = 1;

        protected class AES_STATUS_reg extends RWRegister {

            static final int AES_DONE = 7;
            static final int AES_ER = 7;
            final RegisterView _aes_done = RegisterUtil.bitView(this, AES_DONE);
            final RegisterView _aes_er = RegisterUtil.bitView(this, AES_ER);
        }

        protected class AES_CTRL_reg extends RWRegister {
            static final int AES_DIR = 3;
            static final int AES_MODE0 = 4;
            static final int AES_MODE1 = 6;
            static final int AES_REQUEST = 7;

            final RegisterView _aes_dir = RegisterUtil.bitView(this, AES_DIR);
            final RegisterView _aes_mode = RegisterUtil.bitRangeView(this, AES_MODE0, AES_MODE1);
            final RegisterView _aes_request = RegisterUtil.bitView(this, AES_REQUEST);
            static final int write_mask = 0xf8;
            static final int read_mask = 0x7f;

            @Override
            public void write(byte val) {
                super.write((byte) ((this.value & ~write_mask) | (val & write_mask)));
            }

            @Override
            public byte read() {
                return (byte) (super.read() & read_mask);
            }
        }

        private final short[] sbox = {0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f,
            0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82,
            0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c,
            0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc,
            0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23,
            0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27,
            0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52,
            0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed,
            0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58,
            0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9,
            0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92,
            0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
            0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e,
            0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a,
            0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0,
            0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62,
            0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e,
            0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78,
            0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b,
            0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
            0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98,
            0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55,
            0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41,
            0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};

        private final short[] rconst = {0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
            0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e,
            0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef,
            0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25,
            0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb,
            0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36,
            0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97,
            0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72,
            0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66,
            0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04,
            0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d,
            0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3,
            0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61,
            0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a,
            0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40,
            0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc,
            0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5,
            0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a,
            0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d,
            0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c,
            0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35,
            0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4,
            0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc,
            0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb};

        AES_STATUS_reg AesStatusReg;
        AES_CTRL_reg AesCtrlReg;
        AES_CTRL_reg AesCtrlRegShadow;
        SecretKeySpec aes_key;
        Cipher aes_cipher;
        byte[] aes_data;
        byte[] saved_aes_key;
        byte[] readable_aes_key = new byte[16];
        AESKeyStatus aes_key_status;

        public AESModule() {
            AesStatusReg = new AES_STATUS_reg();
            AesCtrlReg = new AES_CTRL_reg();
            AesCtrlRegShadow = new AES_CTRL_reg();
        }

        public byte getStatus() {
            return AesStatusReg.value;
        }

        public void setConfig(byte config) {
            AesCtrlReg.write(config);
            if (AesCtrlReg._aes_mode.getValue() == AES_MODE_KEY && aes_key != null) {
                for (int i = 0; i < 16; i++) {
                    trxFIFO.setAbsoluteByte(AES_RAM_DATA + i, readable_aes_key[i]);
                }
            }
        }

        public void setData(byte[] data) {
            switch (AesCtrlReg._aes_mode.getValue()) {
                case AES_MODE_KEY:
                    saved_aes_key = data;
                    aes_key_status = AESKeyStatus.INIT;
                    break;
                case AES_MODE_CBC:
                    // Make sure aes_data has size 16
                    if (aes_data.length < 16) {
                        byte[] pre_data = new byte[16];
                        int i = 0;
                        for (byte b : aes_data) {
                            pre_data[i++] = b;
                        }
                        aes_data = pre_data;
                    }
                    for (int i = 0; i < 16; i++) {
                        aes_data[i] ^= data[i];
                    }
                    break;
                default:
                    aes_data = data;
            }
        }

        public void setConfigShadow(byte config) {
            AesCtrlRegShadow.write(config);
            if (AesCtrlRegShadow._aes_request.getValue() != 0) {
                if (AesCtrlReg._aes_dir.getValue() == AES_DIR_ENCRYPTION) {
                    performEncryption();
                } else {
                    performDecryption();
                }
            }
        }

        private void performEncryption() {
            try {
                if (aes_key_status != AESKeyStatus.ENCRYPT) {
                    aes_key_status = AESKeyStatus.ENCRYPT;
                    aes_key = new SecretKeySpec(saved_aes_key, "AES");
                    readable_aes_key = getLastRoundKey();
                    try {
                        aes_cipher = Cipher.getInstance("AES/ECB/NoPadding");
                    } catch (NoSuchAlgorithmException | NoSuchPaddingException e) {
                        throw new RuntimeException(e);
                    }
                }
                aes_cipher.init(Cipher.ENCRYPT_MODE, aes_key);
                byte[] output = aes_cipher.doFinal(aes_data);
                for (int i = 0; i < 16; i++) {
                    trxFIFO.setAbsoluteByte(AES_RAM_DATA + i, output[i]);
                }
                aes_data = output;
            } catch (InvalidKeyException | IllegalBlockSizeException | BadPaddingException e) {
                throw new RuntimeException(e);
            }
        }

        private void performDecryption() {
            try {
                if (aes_key_status != AESKeyStatus.DECRYPT) {
                    aes_key_status = AESKeyStatus.DECRYPT;
                    readable_aes_key = getFirstRoundKey();
                    aes_key = new SecretKeySpec(readable_aes_key, "AES");
                    //readable_aes_key = getLastRoundKey();
                    try {
                        aes_cipher = Cipher.getInstance("AES/ECB/NoPadding");
                    } catch (NoSuchAlgorithmException | NoSuchPaddingException e) {
                        throw new RuntimeException(e);
                    }
                }
                aes_cipher.init(Cipher.DECRYPT_MODE, aes_key);
                byte[] output = aes_cipher.doFinal(aes_data);
                for (int i = 0; i < 16; i++) {
                    trxFIFO.setAbsoluteByte(AES_RAM_DATA + i, output[i]);
                }
                aes_data = output;

            } catch (InvalidKeyException | IllegalBlockSizeException | BadPaddingException e) {
                throw new RuntimeException(e);
            }
        }

        private void aesExpandMix(byte[] word, int round) {
            // rotate bytewise to left, apply sbox and round constant
            byte tmp = word[0];
            word[0] = (byte) ((sbox[word[1] & 0xff] ^ rconst[round]) & 0xff);
            word[1] = (byte) (sbox[word[2] & 0xff] & 0xff);
            word[2] = (byte) (sbox[word[3] & 0xff] & 0xff);
            word[3] = (byte) (sbox[tmp & 0xff] & 0xff);
        }

        private byte[] getLastRoundKey() {
            byte[] expanded_key = new byte[16 * 11];
            byte[] temp = new byte[4];
            int round = 1;
            System.arraycopy(saved_aes_key, 0, expanded_key, 0, 16);
            int position = 16;
            while (position < 16 * 11) {
                for (int i = 0; i < 4; i++) {
                    temp[i] = expanded_key[position - 4 + i];
                }
                if (position % 16 == 0) {
                    aesExpandMix(temp, round++);
                }
                for (int i = 0; i < 4; i++) {
                    expanded_key[position] = (byte) ((expanded_key[position - 16] ^ temp[i]) & 0xff);
                    position++;
                }
            }
            byte[] key = new byte[16];
            System.arraycopy(expanded_key, 160, key, 0, 16);
            return key;
        }

        private byte[] getFirstRoundKey() {
            byte[] expanded_key = new byte[16 * 11];
            byte[] temp1 = new byte[4];
            byte[] temp2 = new byte[4];
            int round = 10;
            System.arraycopy(saved_aes_key, 0, expanded_key, 160, 16);
            int position = 16 * 11;
             while (position > 16) {
                System.arraycopy(expanded_key, position - 4, temp1, 0, 4);
                System.arraycopy(expanded_key, position - 8, temp2, 0, 4);
                position -= 4;
                if (position % 16 == 0) {
                    aesExpandMix(temp2, round--);
                }
                for (int i = 0; i < 4; i++) {
                    expanded_key[position - 16 + i] = (byte) ((temp1[i] ^ temp2[i]) & 0xff);
                }
            }

            byte[] key = new byte[16];
            System.arraycopy(expanded_key, 0, key, 0, 16);
            return key;

        }
    }

}
