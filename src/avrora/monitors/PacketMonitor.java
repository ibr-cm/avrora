/**
 * Created on 17.11.2004
 *
 * Copyright (c) 2004-2005, Olaf Landsiedel, Protocol Engineering and
 * Distributed Systems, University of Tuebingen
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
 * Neither the name of the Protocol Engineering and Distributed Systems
 * Group, the name of the University of Tuebingen nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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

package avrora.monitors;

import avrora.sim.Simulator;
import avrora.sim.platform.Platform;
import avrora.sim.radio.*;
import avrora.sim.output.SimPrinter;
import cck.text.*;
import cck.util.Option;

import java.util.*;
import java.text.StringCharacterIterator;

/**
 * Packet monitor implementation. This class logs the number of packets, e.g. bytes sent and received.
 *
 * @author Olaf Landsiedel
 * @author Ben L. Titzer
 */
public class PacketMonitor extends MonitorFactory {

    private static final int INITIAL_BUFFER_SIZE = 64;

    protected Option.Bool BITS = newOption("show-bits", false,
            "This option enables the printing of packets as they are transmitted.");
    protected Option.Bool PACKETS = newOption("show-packets", true,
            "This option enables the printing of packet contents in bits rather than in bytes.");
    protected Option.Str START_SYMBOL = newOption("start-symbol", "",
            "When this option is not blank, the packet monitor will attempt to match the " +
            "start symbol of packet data in order to display both the preamble, start " +
            "symbol, and packet contents.");

    protected List monitors = new LinkedList();

    class Mon implements Monitor, Medium.Probe {
        char[] bufferData;
        int bufferPos;

        final Simulator simulator;
        final SimPrinter printer;
        final boolean showPackets;
        final boolean bits;

        int bytesTransmitted;
        int packetsTransmitted;
        int bytesReceived;
        int packetsReceived;
        int bytesCorrupted;
        int packetsLostinMiddle;
        boolean matchStart;
        byte startSymbol;
        long startCycle;
        boolean cc2420radio;

        Mon(Simulator s) {
            simulator = s;
            Platform platform = simulator.getMicrocontroller().getPlatform();
            Radio radio = (Radio)platform.getDevice("radio");
            radio.getTransmitter().insertProbe(this);
            radio.getReceiver().insertProbe(this);
            printer = simulator.getPrinter();
            showPackets = PACKETS.get();
            bits = BITS.get();

            // compute the start symbol
            if (!START_SYMBOL.isBlank()) {
                matchStart = true;
                startSymbol = (byte) StringUtil.readHexValue(new StringCharacterIterator(START_SYMBOL.get()), 2);
            } else {
                if (radio instanceof CC1000Radio) {
                    cc2420radio = false;
                    matchStart = true;
                    startSymbol = (byte)0x33;
                }
                if (radio instanceof CC2420Radio) {
                    cc2420radio = true;
                    matchStart = true;
                    startSymbol = (byte)0xA7;
                }
            }
            monitors.add(this);
        }

        private void append(char c) {
            if (bufferData == null) {
                bufferData = new char[INITIAL_BUFFER_SIZE];
            } else if (bufferData.length == bufferPos) {
                char[] newData = new char[bufferData.length * 2];
                System.arraycopy(bufferData, 0, newData, 0, bufferData.length);
                bufferData = newData;
            }
            bufferData[bufferPos++] = c;
        }

        private void clear() {
            bufferPos = 0;
            bufferData = null;
        }

        public void fireBeforeTransmit(Medium.Transmitter t, byte val) {
            if (bufferPos == 0) startCycle = simulator.getClock().getCount();
            append((char) (val & 0xff));
            bytesTransmitted++;
        }

        public void fireBeforeTransmitEnd(Medium.Transmitter t) {
            packetsTransmitted++;
            if (showPackets) {
                printer.printBuffer(renderPacket("----> ", 0, 0, bufferPos));
            }
            clear();
        }

        public void fireAfterReceive(Medium.Receiver r, char val) {
            if (bufferPos == 0) startCycle = simulator.getClock().getCount();
            if (Medium.isCorruptedByte(val)) bytesCorrupted++;
            bytesReceived++;
            append(val);
        }

        public void fireAfterReceiveEnd(Medium.Receiver r) {
            if (bufferPos == 0 || bufferData == null) {
                return;
            }
            if (cc2420radio) {
                    boolean foundAtLeastOne = false;
                    int packetStart = 0;
                    // we could still find a new packet here (2 bytes syncword, 1 byte length, 1 byte payload)
                    while (packetStart + 4 <= bufferPos) {
                        boolean found = false;
                        // look for the beginning of the packet since the transmission might start before
                        // Note: we assume that the syncword is 0FA7 since it's implemented like this in CC2420Radio although it's not correct!
                        int syncwordStart;
                        int addPreamble = 0;
                        for (syncwordStart = packetStart; syncwordStart <= bufferPos - 4; syncwordStart++) {
                            if (((int)bufferData[syncwordStart] & 0xff) == 0x0F && ((int)bufferData[syncwordStart+1] & 0xff) == 0xA7) {
                                // found a new packet, determine alignment
                                if (syncwordStart < packetStart + 3) {
                                    // preamble in packet is less than 3, calculate additional preamble bytes for output
                                    addPreamble = 3- (syncwordStart - packetStart);
                                }
                                else {
                                    // enough bytes for preamble, calculate correct start of packet
                                    packetStart = syncwordStart - 3;
                                }
                                found = true;
                                break;
                            }
                        }

                        // check end
                        if (found) {
                            int packetLength = (int)bufferData[syncwordStart+2] & 0xff;
                            int packetEnd = syncwordStart + 3 + packetLength;
                            if (packetLength > 0  && packetEnd <= bufferPos) {
                                // packets with 0 length are ignored. packet must fit in buffer completely
                                packetsReceived++;
                                foundAtLeastOne = true;
                                if ( showPackets) {
                                    printer.printBuffer(renderPacket("<==== ", addPreamble, packetStart, packetEnd));
                                }
                            }
                            // set the new packetStart for the next iteration
                            packetStart = packetEnd;
                        }
                        else {
                            packetStart = bufferPos;  // no further iteration
                        }
                    }
                    
                    if (bufferPos >= 5 && !foundAtLeastOne) {  // smaller packets are artefacts and are not counted as lost
                        packetsLostinMiddle++;
                    }

            } else {
                packetsReceived++;
                if ( showPackets ) {
                    printer.printBuffer(renderPacket("<==== ", 0, 0, bufferPos));
                }
            }
            clear();
        }

        private StringBuffer renderPacket(String prefix, int addPreamble, int start, int end) {
            int packetLen = end - start;
            StringBuffer buf = printer.getBuffer(3 * (addPreamble + packetLen) + 15);
            Terminal.append(Terminal.COLOR_BRIGHT_CYAN, buf, prefix);
            // additional preamble bytes
            for (int cntr = 0; cntr < addPreamble; cntr++) {
                renderByte(cntr, '\0', true, buf);
                buf.append('.');
            }
            // real packet
            boolean inPreamble = true;
            for (int cntr = 0; cntr < packetLen; cntr++) {
                char t = bufferData[start + cntr];
                inPreamble = renderByte(cntr + addPreamble, t, inPreamble, buf);
                if (cntr < packetLen - 1) buf.append('.');
            }
            appendTime(buf);
            return buf;
        }
        
        private void appendTime(StringBuffer buf) {
            long cycles = simulator.getClock().getCount() - startCycle;
            double ms = simulator.getClock().cyclesToMillis(cycles);
            buf.append("  ");
            buf.append(StringUtil.toFixedFloat((float)ms, 3));
            buf.append(" ms");
        }

        private boolean renderByte(int cntr, char value, boolean inPreamble, StringBuffer buf) {
            int color = Terminal.COLOR_DEFAULT;
            byte bval = (byte)value;
            if (!bits && Medium.isCorruptedByte(value)) {
                // this byte was corrupted during transmission.
                color = Terminal.COLOR_RED;
            } else if (matchStart && cntr > 4) {
                // should we match the start symbol?
                if (inPreamble && cntr == 5) {
                    if (bval == startSymbol) {
                        color = Terminal.COLOR_YELLOW;
                        inPreamble = false;
                    }
                } else if (!inPreamble && cntr > 5) {
                    color = Terminal.COLOR_GREEN;
                }
            }
            renderByte(buf, color, value);
            return inPreamble;
        }

        private void renderByte(StringBuffer buf, int color, char value) {
            if (bits) {
                byte corrupted = Medium.getCorruptedBits(value);
                for (int i = 7; i >= 0; i--) {
                    boolean bit = (value >> i & 1) != 0;
                    if ( ((corrupted >> i) & 1) != 0 )
                        Terminal.append(Terminal.COLOR_RED, buf, bit ? "1" : "0");
                    else
                        Terminal.append(color, buf, bit ? "1" : "0");
                }
            } else {
                Terminal.append(color, buf, StringUtil.toHex((byte)value, 2));
            }
        }

        public void report() {
            if (monitors != null) {
                TermUtil.printSeparator(Terminal.MAXLINE, "Packet monitor results");
                if (cc2420radio) Terminal.printGreen("Node     sent (b/p)          recv (b/p)    corrupted (b)   lostinMiddle(p)");
                else Terminal.printGreen("Node     sent (b/p)          recv (b/p)    corrupted (b)");
                Terminal.nextln();
                TermUtil.printThinSeparator();
                Iterator i = monitors.iterator();
                while (i.hasNext()) {
                    Mon mon = (Mon)i.next();
                    Terminal.print(StringUtil.rightJustify(mon.simulator.getID(), 4));
                    Terminal.print(StringUtil.rightJustify(mon.bytesTransmitted, 10));
                    Terminal.print(" / ");
                    Terminal.print(StringUtil.leftJustify(mon.packetsTransmitted, 8));

                    Terminal.print(StringUtil.rightJustify(mon.bytesReceived, 10));
                    Terminal.print(" / ");
                    Terminal.print(StringUtil.leftJustify(mon.packetsReceived, 8));
                    Terminal.print(StringUtil.rightJustify(mon.bytesCorrupted, 10));
                    if (cc2420radio) Terminal.print(StringUtil.rightJustify(mon.packetsLostinMiddle, 8));
                    Terminal.nextln();
                }
                monitors = null;
                Terminal.nextln();
            }
        }
    }

    /**
     * create a new monitor
     */
    public PacketMonitor() {
        super("The \"packet\" monitor tracks packets sent and received by nodes in a sensor network.");
    }

    /**
     * create a new monitor, calls the constructor
     *
     * @see MonitorFactory#newMonitor(Simulator)
     */
    public Monitor newMonitor(Simulator s) {
        return new Mon(s);
    }
}

