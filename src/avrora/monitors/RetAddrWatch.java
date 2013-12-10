/**
 * Copyright (c) 2005, Regents of the University of California
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
 *
 * Created May 16th, 2012
 */
package avrora.monitors;

import avrora.arch.legacy.LegacyInstr;
import avrora.core.Program;
import avrora.core.SourceMapping;
import avrora.sim.InterruptTable;
import avrora.sim.Simulator;
import avrora.sim.State;

/**
 * The <code>retaddr</code> monitor inserts a watch on the stack dynamically
 * after each CALL/ICALL/RCALL or interrupt invocation to check if the return
 * address is not modified. Before a RET or RETI the watch is removed.
 *
 * Note: This monitor is tested with TinyOS programs for AVR platforms. It is not guaranteed
 * it will work with other OS or platforms!
 * Note 2: this still experimental :-)
 *
 * @author Daniel Minder
 */
public class RetAddrWatch extends MonitorFactory {

    protected class WatchRetAddr extends Simulator.Watch.Empty {
        public void fireBeforeWrite(State state, int data_addr, byte value) {
            state.getSimulator().getPrinter().println("Instruction at " + Integer.toHexString(state.getPC())
                + " destroyed return address on stack address " + Integer.toHexString(data_addr));
        }
    }
    
    WatchRetAddr myWatch = new WatchRetAddr();
    
    // when a value is pushed on the stack on AVR processors the SP is postdecremented!
    // this is different from, e.g., x86 platforms where SP is first decremented.
    protected class ProbeCall extends Simulator.Probe.Empty {
        public void fireAfter(State state, int pc) {
            int sp = state.getSP();
            state.getSimulator().insertWatch(myWatch, sp+1);
            state.getSimulator().insertWatch(myWatch, sp+2);
        }
    }

    protected class ProbeInterrupt extends Simulator.InterruptProbe.Empty {
        public void fireAfterInvoke(State state, int inum) {
            int sp = state.getSP();
            state.getSimulator().insertWatch(myWatch, sp+1);
            state.getSimulator().insertWatch(myWatch, sp+2);
        }
    }

    protected class ProbeReturn extends Simulator.Probe.Empty {
        public void fireBefore(State state, int pc) {
            int sp = state.getSP();
            state.getSimulator().removeWatch(myWatch, sp+1);
            state.getSimulator().removeWatch(myWatch, sp+2);
        }

    }

    /**
      * The actual monitoring class. It attaches probes to each CALL/RCALL/ICALL/RET/RETI instruction
      * and attaches also an interrupt invocation probe */
    
    class Mon implements Monitor {
        private final Simulator simulator;
        private final ProbeCall probeCall = new ProbeCall();
        private final ProbeInterrupt probeInterrupt = new ProbeInterrupt();
        private final ProbeReturn probeReturn = new ProbeReturn();

        Mon(Simulator s) {
            simulator = s;
            attachInstructionProbes();
            attachInterruptProbes();
        }
        
        private void attachInstructionProbes() {
            Program p = simulator.getProgram();
            SourceMapping sourceMap = simulator.getProgram().getSourceMapping();
            for ( int pc = 0; pc < p.program_end; pc = p.getNextPC(pc)) {
                LegacyInstr i = (LegacyInstr)p.readInstr(pc);
                if ( i != null ) {
                    if ( i instanceof LegacyInstr.CALL ) {
                        // determine target of call
                        int target = ((LegacyInstr.CALL)i).imm1 * 2;
                        String targetName = sourceMap.getName(target);
                        // do not insert a probe when main is called since main will set SP again
                        if (!targetName.equals("main"))
                            simulator.insertProbe(probeCall, pc);
                    }
                    else if ( i instanceof LegacyInstr.RCALL || i instanceof LegacyInstr.ICALL)
                        simulator.insertProbe(probeCall, pc);
                    else if ( i instanceof LegacyInstr.RET || i instanceof LegacyInstr.RETI )
                        simulator.insertProbe(probeReturn, pc);
                }
            }
        }

        private void attachInterruptProbes() {
            InterruptTable table = simulator.getInterpreter().getInterruptTable();
            table.insertProbe(probeInterrupt);
        }
        
        public void report() {
            // nothing to report
        }
    }
    
    /**
     * The constructor for the <code>RetAddrWatch</code> class simply initializes the help for this
     * class. Monitors are also help categories, so they will have an options section in their help
     * that explains each option and its use.
     */
    public RetAddrWatch() {
        super("The \"retaddr\" monitor inserts watches on all return addresses on the stack "+
            "and prints the program counter of the instruction modifying them.");
    }

    /**
     * The <code>newMonitor()</code> method simply creates a new retaddrwatch  monitor for each simulator.
     * @param s the simulator to create a new monitor for
     * @return a new monitor that adds watches for each return address on the stack
     */
    public Monitor newMonitor(Simulator s) {
        return new Mon(s);
    }
}
