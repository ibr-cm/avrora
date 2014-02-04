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

package avrora.sim.platform;

import avrora.sim.Simulator;
import avrora.sim.mcu.USART;
import avrora.sim.util.SimUtil;
import cck.text.Terminal;
import cck.text.StringUtil;
import cck.util.Util;

/**
 * The <code>SerialForwarder</code> class implements a serial forwarder that takes traffic
 * to and from a socket and directs it into the UART chip of a simulated device.
 *
 * @author Olaf Landsiedel
 * @author Ben L. Titzer
 */
public class SerialLogger implements USART.USARTDevice {
    final static int SYNC_BYTE = 0x7e;
    final static int ESCAPE_BYTE = 0x7d;
    final static int MTU = 256;

    private byte[] receiveBuffer;
      
    private boolean inSync = false;
    private int count = 0;
    private boolean escaped = false;

    private Simulator simulator;

    public SerialLogger(USART usart, Simulator s) {
        usart.connect(this);

        this.simulator = s;
        this.receiveBuffer = new byte[MTU];
    }

    public USART.Frame transmitFrame() {
      throw Util.failure("transmitFrame should not be called in SerialLogger");
    }


    public void receiveFrame(USART.Frame frame) {
      String buf = null;
      byte b = (byte)frame.value;
      
      if (!inSync) {
        if (b == SYNC_BYTE) {
          inSync = true;
          count = 0;
          escaped = false;
        }
      }
      else {
        if (count >= MTU) {
          buf = new String("UART: Packet to long");
          inSync = false;
        }
        else {
          if (escaped) {
            if (b == SYNC_BYTE) {
              // sync byte following escape is an error, resync
              buf = new String("UART: Unexpected sync byte");
              inSync = false;
            }
            else {
              b ^= 0x20;
              receiveBuffer[count++] = b;
              escaped = false;
            }
          }
          else if (b == ESCAPE_BYTE) {
            escaped = true;
          }
          else if (b == SYNC_BYTE) {
            if (count < 4) {
              // too-small frames are ignored
              count = 0;
            }
            else {
              int readCrc = (receiveBuffer[count - 2]  & 0xff) |
                  (receiveBuffer[count - 1] & 0xff) << 8;
              int computedCrc = calc(receiveBuffer, count-2);

              // create output
              StringBuffer pbuf;
              if (readCrc == computedCrc) {
                pbuf = new StringBuffer("UART: received ");
              }
              else {
                pbuf = new StringBuffer("UART: bad packet ");
              }
              for (int i=0; i < count - 2; i++) {
                StringUtil.toHex(pbuf, receiveBuffer[i], 2);
                pbuf.append(":");
              }
              buf = pbuf.toString();
              count = 0;
            }
          }
          else { // normal byte
            receiveBuffer[count++] = b;
          } 
        }  // count < MTU
      }  // inSync
      
      if (buf != null) {
        StringBuffer idtime = new StringBuffer(45);
        SimUtil.getIDTimeString(idtime, simulator);
        synchronized ( Terminal.class) {
          Terminal.print(idtime.toString());
          Terminal.println(buf);
        }
      }
    }
    
    public static int calcByte(int crc, int b) {
      crc = crc ^ (int)b << 8;

      for (int i = 0; i < 8; i++) {
        if ((crc & 0x8000) == 0x8000)
          crc = crc << 1 ^ 0x1021;
        else
          crc = crc << 1;
      }

      return crc & 0xffff;
    }

    public static int calc(byte[] packet, int index, int count) {
        int crc = 0;
        while (count > 0) {
            crc = calcByte(crc, packet[index++]);
            count--;
        }
        return crc;
    }

    public static int calc(byte[] packet, int count) {
        return calc(packet, 0, count);
    }

}
