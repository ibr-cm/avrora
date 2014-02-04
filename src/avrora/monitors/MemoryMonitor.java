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

package avrora.monitors;

import avrora.arch.avr.AVRProperties;
import avrora.arch.legacy.LegacyState;
import avrora.core.Program;
import avrora.core.SourceMapping;
import avrora.sim.Simulator;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.util.MemoryProfiler;
import cck.text.*;
import cck.util.Option;
import cck.util.Util;

import java.util.HashSet;
import java.util.List;

/**
 * The <code>MemoryMonitor</code> class implements a monitor that collects information about how the program
 * accesses the data memory over its execution. For each RAM address it keeps an account of the number of
 * reads and the number of writes and reports that information after the program is completed.
 *
 * @author Ben L. Titzer
 * @author Daniel Minder
 */
public class MemoryMonitor extends MonitorFactory {

    public final Option.List LOCATIONS = newOptionList("locations", "",
            "This option, when set, specifies a list of memory locations to instrument. When " +
            "this option is not specified, the monitor will instrument all reads and writes to " +
            "memory. Locations can be given as symbols or addresses in hex format. For symbols " +
            "all memory addresses of this symbol are monitored (works only with ELF input!).");

    public final Option.Bool LOWER_ADDRESS = newOption("low-addresses", false,
            "When this option is enabled, the memory monitor will be inserted for lower addresses, " +
            "recording reads and writes to the general purpose registers on the AVR and also IO registers " +
            "through direct and indirect memory reads and writes.");
    
    public final Option.Bool DUMP_WRITES = newOption("dump-writes", false,
            "When this option is enabled, the memory monitor will dump each write access to all " +
            "instrumented addresses. The address of the instruction that causes the write is included.");

    public class Monitor implements avrora.monitors.Monitor {
        public final Simulator simulator;
        public final Microcontroller microcontroller;
        public final Program program;
        public final int ramsize;
        public final int memstart;
        public final MemoryProfiler memprofile;

        Monitor(Simulator s) {
            simulator = s;
            microcontroller = simulator.getMicrocontroller();
            program = simulator.getProgram();
            AVRProperties p = (AVRProperties)microcontroller.getProperties();
            ramsize = p.sram_size + p.ioreg_size + LegacyState.NUM_REGS;
            if ( LOWER_ADDRESS.get() ) {
                memstart = 0;
            } else {
                memstart = LegacyState.IOREG_BASE + p.ioreg_size;
            }
            // when output is switched off just pass null for the printer
            memprofile = new MemoryProfiler(ramsize, DUMP_WRITES.get() ? s.getPrinter() : null);

            insertWatches();
        }

        private void insertWatches() {
            if (!LOCATIONS.get().isEmpty() ) {
                // instrument only the locations specified
                List<String> l = LOCATIONS.get();
                HashSet<Integer> locset = new HashSet<Integer>(l.size()*2);
                SourceMapping sm = program.getSourceMapping();
                for (String val : l) {
                    if (val.matches("^0[xX][0-9a-fA-F]+$")) {
                        // hexadecimal number is direct address
                        int memaddr = -1;
                        try {
                            memaddr = Integer.parseInt(val.substring(2), 16);
                        }
                        catch (java.lang.NumberFormatException nfe) {
                            cck.util.Util.userError("Not a hex memory address", val);
                        }
                        if (memaddr < 0 || memaddr >= ramsize) {
                            cck.util.Util.userError("Memory address not in ram", val);
                        }
                        if (memaddr != -1) {
                            locset.add(new Integer(memaddr));
                        }
                    }
                    else {
                        // treat it as a label, try to convert it to address
                        SourceMapping.Location loc = sm.getLocation(val);
                        if (loc == null)
                            Util.userError("Label unknown", val);
                        int addr = loc.vma_addr & 0xffff;
                        for (int i = 0; i < loc.size; i++)
                            locset.add(new Integer(addr+i));
                    }
                }
                // insert watches
                for (Integer val : locset) {
                        simulator.insertWatch(memprofile, val.intValue());
                }
            } else {
                // instrument the entire memory
                for (int cntr = memstart; cntr < ramsize; cntr++) {
                    simulator.insertWatch(memprofile, cntr);
                }
            }
        }

        public void report() {
            TermUtil.printSeparator("Memory profiling results for node "+simulator.getID());
            Terminal.printGreen("   Address     Reads               Writes");
            Terminal.nextln();
            TermUtil.printThinSeparator(Terminal.MAXLINE);
            double rtotal = 0;
            long[] rcount = memprofile.rcount;
            double wtotal = 0;
            long[] wcount = memprofile.wcount;
            int imax = rcount.length;

            // compute the total for percentage calculations
            for (int cntr = 0; cntr < imax; cntr++) {
                rtotal += rcount[cntr];
                wtotal += wcount[cntr];
            }

            int zeroes = 0;

            for (int cntr = memstart; cntr < imax; cntr++) {
                long r = rcount[cntr];
                long w = wcount[cntr];

                if (r == 0 && w == 0)
                    zeroes++;
                else
                    zeroes = 0;

                // skip long runs of zeroes
                if (zeroes == 2) {
                    Terminal.println("                   .                    .");
                    continue;
                } else if (zeroes > 2) continue;

                String addr = StringUtil.addrToString(cntr);
                printLine(addr, r, rtotal, w, wtotal);

            }
            printLine("total ", (long)rtotal, rtotal, (long)wtotal, wtotal);
            Terminal.nextln();
        }

        private void printLine(String addr, long r, double rtotal, long w, double wtotal) {
            String rcnt = StringUtil.rightJustify(r, 8);
            float rpcnt = (float)(100 * r / rtotal);
            String rpercent = StringUtil.rightJustify(StringUtil.toFixedFloat(rpcnt, 4),8) + " %";

            String wcnt = StringUtil.rightJustify(w, 8);
            float wpcnt = (float)(100 * w / wtotal);
            String wpercent = StringUtil.rightJustify(StringUtil.toFixedFloat(wpcnt, 4),8) + " %";

            Terminal.printGreen("    " + addr);
            Terminal.print(": ");
            Terminal.printBrightCyan(rcnt);
            Terminal.print(' ' + ("  " + rpercent));
            Terminal.printBrightCyan(wcnt);
            Terminal.println(' ' + ("  " + wpercent));
        }

    }

    public MemoryMonitor() {
        super("The \"memory\" monitor collects information about the " +
                "memory usage statistics of the program, including the number " +
                "of reads and writes to every byte of data memory.");
    }

    public avrora.monitors.Monitor newMonitor(Simulator s) {
        return new Monitor(s);
    }
}
