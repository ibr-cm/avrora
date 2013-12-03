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

package avrora.sim.mcu;

import avrora.sim.*;
import avrora.sim.radio.Radio;
import avrora.sim.radio.Medium;
import avrora.sim.radio.ATmega128RFA1Radio;
import avrora.sim.state.*;
import cck.text.StringUtil;
import cck.util.Arithmetic;
import cck.util.Util;

/**
 * Internal Radio interface. Used on the <code>ATMega128RFA1</code> platform for radio communication.
 *
 * @author David Kopf
 */
public class RADIO extends AtmelInternalDevice implements RADIODevice, InterruptTable.Notification {

    private TRX_STATE_Reg TRX_STATE_reg;
    private TRXPR_Reg TRXPR_reg;
    private PHY_ED_LEVEL_Reg PHY_ED_LEVEL_reg;

    private ATmega128RFA1Radio connectedRadio;
    private RADIODevice connectedDevice;
 
 /* Radio states are contained in the TRX_STATUS register.
  * Since that register has no trigger actions it can be read
  * directly from RAM. The connected ATmega128RFA1Radio has a public
  * rf231Status which should mirror the RAM value.
  */
//  private static final int  TRX_STATUS         = 0x141;
    private static final byte STATE_BUSY_RX      = 0x01;
    private static final byte STATE_BUSY_TX      = 0x02;
    private static final byte STATE_RX_ON        = 0x06; 
    private static final byte STATE_TRX_OFF      = 0x08;
    private static final byte STATE_PLL_ON       = 0x09;
    private static final byte STATE_SLEEP        = 0x0F;
    private static final byte STATE_BUSY_RX_AACK = 0x11;
    private static final byte STATE_BUSY_TX_ARET = 0x12;
    private static final byte STATE_RX_AACK_ON   = 0x16;
    private static final byte STATE_TX_ARET_ON   = 0x19;
  //private static final byte STATE_TRANSITION   = 0x1F;

    public void connect(ATmega128RFA1Radio r, RADIODevice d) {
        connectedRadio = r;
        connectedDevice = d;
    }
    
    public RADIO(AtmelMicrocontroller m) {
        super("radio", m);
        TRX_STATE_reg    = new TRX_STATE_Reg();
        TRXPR_reg        = new TRXPR_Reg();
        PHY_ED_LEVEL_reg = new PHY_ED_LEVEL_Reg();
        installIOReg("TRX_STATE", TRX_STATE_reg);
        installIOReg("TRXPR", TRXPR_reg);
        installIOReg("PHY_ED_LEVEL", PHY_ED_LEVEL_reg);              
    }

    /**
     * TRXPR register. Emulates the TRXRST and SLPTR pins of an external RF231 radio.
     */
    protected class TRXPR_Reg extends RWRegister {
        protected byte lastSLP = 0;
/*  
        //The read override is only needed for debugging, the super should be returning the correct value
        public byte read() {
            byte curSLP = super.read();
            if (devicePrinter != null) {
                byte rf231Status = connectedRadio.rf231Status;
                if (curSLP != 0) {
                    if (rf231Status == 18) {
                        //busy txaret
                    } else if (rf231Status == 8) {
                        devicePrinter.println("RADIO : SLP bit set when radio in TRX_OFF state");   
                    } else if (rf231Status != 0x0F ) {
                        devicePrinter.println("SLP bit reads high but status is not sleep " + rf231Status);
                    }
                }
                if (lastSLP !=curSLP) if (devicePrinter !=null) devicePrinter.println("RADIO: lastSLP " + lastSLP + " but curSLP is " + curSLP);
            }
            return curSLP;
        }
*/
        public void write(byte val) {
 //         if (devicePrinter != null) devicePrinter.println("RADIO: wrote " + StringUtil.toMultirepString(val, 8) + " to TRXPR");
            super.write(val);
            
            //There are several ways to get the radio status; use the fastest!
         // byte rf231status = interpreter.getDataByte(0x141);
         // byte rf231status = interpreter.getDataByte(connectedRadio.TRX_STATUS);
         // byte rf231status = interpreter.getDataByte(TRX_STATUS);
         // if (rf231status != connectedRadio.rf231State) devicePrinter.println("RADIO:status conflict "+ rf231status + " " + connectedRadio.rf231Status);
            byte rf231status = connectedRadio.rf231Status;
            /* TODO: Figure out what the datasheed means by this!
               "After initiating a state change
                by a rising edge at Bit SLPTR in radio transceiver states TRX_OFF, RX_ON or
                RX_AACK_ON, the radio transceiver remains in the new state as long as the pin is
                logical high and returns to the preceding state with the falling edge."
            */
            // The SLPTR bit is only effective on a toggle
            if ((val&0x02) != lastSLP) {
                if (lastSLP == 0 ) {
      //            devicePrinter.println("RADIO: SLP pin raised, state = " + TRX_STATE_reg.read() + " status = " + rf231status);
                    switch (rf231status) {
                       //if TRX_OFF, -> SLEEP
                        case STATE_TRX_OFF:
                            connectedRadio.pinChangeSLP(val);
                            break;
                        //if PLL_ON or TX_ARET_ON -> BUSY_TX
                        //TRX_STATUS will have been set to the corresponding busy state
                       case STATE_PLL_ON:
                       case STATE_TX_ARET_ON:
                            connectedRadio.pinChangeSLP(val);
                            break;
                        case STATE_BUSY_TX:
                        case STATE_BUSY_TX_ARET:
                        devicePrinter.println("slp pin changed in busy state");
                            connectedRadio.pinChangeSLP(val);
                            break;
                        default:
                            devicePrinter.println("slp pin changed in unknown state " + rf231status);
                            break;
                     }
                } else {
      //            devicePrinter.println("RADIO: SLP pin lowered, state = " + TRX_STATE_reg.read() + " status = " + rf231status);
                    switch (rf231status) {
                        //if SLEEP, ->TRX_OFF
                        case STATE_SLEEP:
                            connectedRadio.pinChangeSLP(val);
                            break;
                    }
                }
                lastSLP = (byte) (val&0x02);
            }
            /* RST high resets all registers to default values and is cleared automatically.
             * The microcontroller has to set SLPTR to the default value
             */
            if ((val&0x01) != 0) {
                connectedRadio.reset();
                 //->TRX_OFF
                 //delay?
                 super.write((byte)(val&0x02));
            }
        }
    }
    /**
     * TRX_STATE control register.
     * The TRAC_STATUS bits may be set in response to a command, which would cause a recursive call
     * to this routine if done in the connected radio. To avoid this, newCommand() returns those bits
     * so they can be written here.
     * Other writes from the radio to the TRAC_STATUS field are detected by the upper bits set,
     * or a value of zero. This means the NOP command won't work!
     * TODO:fix this
     */
    protected class TRX_STATE_Reg extends RWRegister {
        public void write(byte val) {
            //if the command is FORCE_TRX_OFF clear the TRAC_STATUS bits. These are readonly bits from the MCU side.
            //However the rf230 driver does a subregister write which or's the TRAC_STATUS bits with the command before writing.
            //We thus have no way of telling whether the write was from the driver or the internal ATmega128RFA1 radio.
            //A workaround is to clear those bits only on the FORCE_TRX_OFF command.
            //TODO:use the same method as isWritingEDLevel which flags radio writes to the readonly ED register
            if ((val & 0x1F) == 0x03) {
           // System.out.println("removing trac status from force trx off command");
                val = (byte) (val & 0x1F);
            }

            if ((val == 0) || ((val & 0xE0) != 0)) {         
                val = (byte) (val & 0xE0);
                byte cmd = (byte) super.read();
                cmd = (byte) (cmd & 0x1F);
                super.write((byte) (val | cmd));
                return;
            } else {
                //Send the command field to the radio.
                //The return value adds the readonly TRAC_STATUS bits for the actual register write
                val = connectedRadio.newCommand((byte) (val & 0x1F));
            //    System.out.println("newCommand returns " + StringUtil.to0xHex(val, 2));
                super.write(val);
              //  super.write(connectedRadio.newCommand((byte) (val & 0x1F)));
            }
        }
    }
    /**
     * PHY_ED_LEVEL register.
     * In extended operation a write to this register initiates a CCA
     * Since both MCU and radio write to it, isWritingtoED is used as a flag
     * to avoid infinite recursion.
     */
    protected class PHY_ED_LEVEL_Reg extends RWRegister {
        public void write(byte val) {
            if (connectedRadio.isWritingEDLevel) {
                super.write(val);
            } else {
                //Send the CCA command field to the radio.
                //This is obviously a hack.  0x42 is an arbitrary number.
                connectedRadio.newCommand((byte) 0x42);
            }
        }
    }


    public void force(int inum) {
        if (devicePrinter != null) devicePrinter.println("RADIO: force " + inum);
    }
    public void invoke(int inum) {
        if (devicePrinter != null) devicePrinter.println("RADIO: invoke " + inum);
    }

}
