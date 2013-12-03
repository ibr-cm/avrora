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

package avrora.sim;

import avrora.sim.util.MulticastInterruptProbe;
import cck.util.Arithmetic;

/**
 * The <code>InterruptTable</code> class encapsulates the functionality relating to handling
 * the state of the interrupts in the simulation. It allows probes to be inserted on each
 * interrupt (and globally, for each interrupt) that allow the user's code to be called back
 * when the state of interrupts changes.
 *
 * @author Ben L. Titzer
 */
public class InterruptTable {

    protected final int numInterrupts;
    protected long posted1, posted2;
    protected long pending1, pending2;
    protected long enabled1, enabled2;
    protected MulticastInterruptProbe globalProbe;
    protected final MulticastInterruptProbe[] probes;
    protected final Notification[] notify;
    protected final Interpreter interpreter;
    protected final State state;

    /**
     * The <code>Notification</code> interface serves a very specific role in simulation;
     * for device implementations to be notified when an interrupt that a device may have
     * posted is executed, or when the user forces an interrupt to be notified. Some interrupts
     * are "auto-clear" which means that they automatically unpost themselves when their
     * handler is invoked. To support this functionality, a <code>Notification</code> is inserted
     * on them that allows the flag register to be updated when the handler is invoked.
     *
     * As a user of the simulation, you SHOULD NOT implement a notification.
     *
     * @see Simulator.InterruptProbe for probing interrupts
     */
    public interface Notification {
        public void force(int inum);
        public void invoke(int inum);
    }

    public InterruptTable(Interpreter interp, int numInterrupts) {
        interpreter = interp;
        probes = new MulticastInterruptProbe[numInterrupts];
        notify = new Notification[numInterrupts];
        state = interpreter.getState();
        this.numInterrupts = numInterrupts;
    }

    /**
     * The <code>post()</code> method is called by the interpreter when an interrupt is posted.
     * The interrupt table will adjust the posted and pending masks appropriately and notify
     * any probes for the interrupt.
     * @param inum the interrupt number to post
     */
    public void post(int inum) {
        interpreter.innerLoop = false;
        if (inum < 64) {
            posted1 = Arithmetic.setBit(posted1, inum, true);
            pending1 = posted1 & enabled1;
        } else {
            posted2 = Arithmetic.setBit(posted2, inum - 64, true);
            pending2 = posted2 & enabled2;
        }
        MulticastInterruptProbe probe = probes[inum];
        if ( globalProbe != null ) globalProbe.fireWhenPosted(state, inum);
        if ( probe != null ) probe.fireWhenPosted(state, inum);
    }

    /**
     * The <code>force()</code> method is called by the interpreter when the user attempts to
     * force an interrupt to become posted. The interrupt table will adjust the posted and pending
     * masks appropriately and notify any probes for the interrupt.
     * @param inum the interrupt number to force
     */
    void force(int inum) {
        post(inum);
        Notification n = notify[inum];
        if ( n != null ) n.force(inum);
    }

    /**
     * The <code>unpost()</code> method is called by the interpreter when an interrupt is unposted.
     * The interrupt table will adjust the posted and pending masks appropriately and notify
     * any probes for the interrupt.
     * @param inum the interrupt number to unpost
     */
    public void unpost(int inum) {
        if (inum < 64) {
            posted1 = Arithmetic.setBit(posted1, inum, false);
            pending1 = posted1 & enabled1;
        } else {
            posted2 = Arithmetic.setBit(posted2, inum - 64, false);
            pending2 = posted2 & enabled2;
        }
        MulticastInterruptProbe probe = probes[inum];
        if ( globalProbe != null ) globalProbe.fireWhenUnposted(state, inum);
        if ( probe != null ) probe.fireWhenUnposted(state, inum);
    }

    /**
     * The <code>enable()</code> method is called by the interpreter when an interrupt is enabled.
     * The interrupt table will adjust the posted and pending masks appropriately and notify
     * any probes for the interrupt.
     * @param inum the interrupt number to enable
     */
    void enable(int inum) {
        if (inum < 64) {
            enabled1 = Arithmetic.setBit(enabled1, inum, true);
            pending1 = posted1 & enabled1;
        } else {
            enabled2 = Arithmetic.setBit(enabled2, inum - 64, true);
            pending2 = posted2 & enabled2;
        }
        MulticastInterruptProbe probe = probes[inum];
        if ( globalProbe != null ) globalProbe.fireWhenEnabled(state, inum);
        if ( probe != null ) probe.fireWhenEnabled(state, inum);
    }

    /**
     * The <code>disable()</code> method is called by the interpreter when an interrupt is disabled.
     * The interrupt table will adjust the posted and pending masks appropriately and notify
     * any probes for the interrupt.
     * @param inum the interrupt number to disable
     */
    public void disable(int inum) {
    //If an ATTiny do nothing
        if (((AtmelInterpreter)interpreter).getFlashByte(0) != 0x0c) return;
         if (inum < 64) {
            enabled1 = Arithmetic.setBit(enabled1, inum, false);
            pending1 = posted1 & enabled1;
        } else {
            enabled2 = Arithmetic.setBit(enabled2, inum - 64, false);
            pending2 = posted2 & enabled2;
        }
    //    System.err.println("inum is " + inum);
        MulticastInterruptProbe probe = probes[inum];
        if ( globalProbe != null ) globalProbe.fireWhenDisabled(state, inum);
        if ( probe != null ) probe.fireWhenDisabled(state, inum);
    }

    /**
     * The <code>enableAll()</code> method is called by the interpreter when the all of
     * the interrupts are enabled by setting the global interrupt enable bit.
     */
    public void enableAll() {
        if ( globalProbe != null ) globalProbe.fireWhenEnabled(state, 0);
    }

    /**
     * The <code>disableAll()</code> method is called by the interpreter when the all of
     * the interrupts are disabled by clearing the global interrupt enable bit.
     */
    public void disableAll() {
        if ( globalProbe != null ) globalProbe.fireWhenDisabled(state, 0);
    }

    /**
     * The <code>beforeInvoke()</code> method is called by the interpreter before it
     * invokes an interrupt handler.
     * @param inum the interrupt number about to be invoked
     */
    public void beforeInvoke(int inum) {
        MulticastInterruptProbe probe = probes[inum];
        if ( globalProbe != null ) globalProbe.fireBeforeInvoke(state, inum);
        if ( probe != null ) probe.fireBeforeInvoke(state, inum);
        Notification n = notify[inum];
        if ( n != null ) n.invoke(inum);
    }

    /**
     * The <code>afterInvoke()</code> method is called by the interpreter after it
     * invokes an interrupt handler.
     * @param inum the interrupt number that was just invoked
     */
    public void afterInvoke(int inum) {
        MulticastInterruptProbe probe = probes[inum];
        if ( globalProbe != null ) globalProbe.fireAfterInvoke(state, inum);
        if ( probe != null ) probe.fireAfterInvoke(state, inum);
    }

    /**
     * The <code>registerInternalNotification()</code> method is used by devices that
     * require notifications when their interrupt numbers are either forced or invoked.
     * @param n the notification to register for this interrupt
     * @param inum the interrupt number for which to register the notification
     */
    public void registerInternalNotification(Notification n, int inum) {
        notify[inum] = n;
    }

    /**
     * The <code>getPostedInterrupts()</code> method returns a long integer that represents a bit map
     * of which interrupts are currently posted. For example, bit 1 of this long corresponds to whether
     * interrupt 1 is currently posted.
     * @return a long integer representing a bit map of the posted interrupts
     */
     /*
    public long getPostedInterrupts() {
        return posted;
    }
*/
    /**
     * The <code>getPendingInterrupts()</code> method returns a long integer that represents a bit map
     * of which interrupts are currently pending, meaning they are both posted and enabled. For example, bit 1
     * of this long corresponds to whether
     * interrupt 1 is currently pending.
     * @return a long integer representing a bit map of the pending interrupts
     */
    public long getPendingInterrupts1() {
        return pending1;
    }
    public long getPendingInterrupts2() {
        return pending2;
    }

    /**
     * The <code>getEnabledInterrupts()</code> method returns a long integer that represents a bit map
     * of which interrupts are currently enabled, meaning not masked out. For example, bit 1 of this long
     * corresponds to whether
     * interrupt 1 is currently enabled.
     * @return a long integer representing a bit map of the posted interrupts
     */
     /*
    public long getEnabledInterrupts() {
        return enabled;
    }
*/
    /**
     * The <code>isPosted()</code> method checks whether the specified interrupt is currently posted.
     * @param inum the interrupt number to check
     * @return true if the specified interrupt is currently posted; false otherwise
     */
    public boolean isPosted(int inum) {
        if (inum < 64) {
            return Arithmetic.getBit(posted1, inum);
        } else {
            return Arithmetic.getBit(posted2, inum - 64);
        }
    }

    /**
     * The <code>isPending()</code> method checks whether the specified interrupt is currently pending.
     * @param inum the interrupt number to check
     * @return true if the specified interrupt is currently pending; false otherwise
     */
    public boolean isPending(int inum) {
        if (inum < 64) {
            return Arithmetic.getBit(pending1, inum);
        } else {
            return Arithmetic.getBit(pending2, inum - 64);
        }
    }

    /**
     * The <code>isEnabled()</code> method checks whether the specified interrupt is currently enabled.
     * @param inum the interrupt number to check
     * @return true if the specified interrupt is currently enabled; false otherwise
     */
    public boolean isEnabled(int inum) {
        if (inum < 64) {
            return Arithmetic.getBit(enabled1, inum);
        } else {
            return Arithmetic.getBit(enabled2, inum - 64);
        }
    }

    /**
     * The <code>insertProbe()</code> method inserts an interrupt probe on the specified interrupt. When
     * the specified interrupt changes state, i.e. it is either posted, unposted, or invoked, the probe
     * will be notified.
     * @param p the probe to insert on the specified interrupt
     * @param inum the interrupt on which to insert the probe
     */
    public void insertProbe(Simulator.InterruptProbe p, int inum) {
        MulticastInterruptProbe mp = probes[inum];
        if ( mp == null ) probes[inum] = mp = new MulticastInterruptProbe();
        mp.add(p);
    }

    /**
     * The <code>removeProbe()</code> method removes a probe from an interrupt.
     * @param p the probe to remove from this interrupt
     * @param inum the interrupt number from which to remove this probe
     */
    public void removeProbe(Simulator.InterruptProbe p, int inum) {
        MulticastInterruptProbe mp = probes[inum];
        if ( mp != null ) mp.remove(p);
    }

    /**
     * The <code>insertProbe()</code> method inserts a global probe on all of the interrupts. This probe
     * will be notified when any of the interrupts changes state, and also when the global interrupt flag
     * is enabled or disabled. When the global interrupt flag (master bit) is changed, the special interrupt
     * number <code>0</code> will be passed to the <code>fireWhenEnabled()</code> method of the probe.
     * @param p the probe to insert on all the interrupts
     */
    public void insertProbe(Simulator.InterruptProbe p) {
        if ( globalProbe == null ) globalProbe = new MulticastInterruptProbe();
        globalProbe.add(p);
    }

    /**
     * The <code>removeProbe()</code> method removes a global probe from all of the interrupts.
     * @param p the probe to remove from all of the interrupts
     */
    public void removeProbe(Simulator.InterruptProbe p) {
        if ( globalProbe != null ) globalProbe.remove(p);
    }

    /**
     * The <code>getNumberOfInterrupts()</code> method returns the number of interrupts in this interrupt table.
     * @return the number of interrupts in this interrupt table
     */
    public int getNumberOfInterrupts() {
        return numInterrupts;
    }
}
