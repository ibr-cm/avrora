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
 *
 */

package org.contikios.cooja.avrmote.interfaces;
import javax.swing.JPanel;
import org.apache.log4j.Logger;

import org.contikios.cooja.Mote;
import org.contikios.cooja.MoteMemory;
import org.contikios.cooja.MoteMemory.MemoryEventType;
import org.contikios.cooja.interfaces.MoteID;

/**
 * @author Fredrik Osterlind
 */
public class AvroraMoteID extends MoteID {
  private static Logger logger = Logger.getLogger(AvroraMoteID.class);

  private Mote mote;
  private MoteMemory moteMem = null;

  private int moteID = -1;

  private org.contikios.cooja.MoteMemory.MemoryMonitor memoryMonitor;

  /**
   * Creates an interface to the mote ID at mote.
   *
   * @param mote
   * @see Mote
   * @see org.contikios.cooja.MoteInterfaceHandler
   */
  public AvroraMoteID(Mote m) {
    this.mote = m;
    this.moteMem = mote.getMemory();
  }

  public int getMoteID() {
    return moteID;
  }

  public void setMoteID(int newID) {
    if (moteID != newID) {
      setChanged();
    }
    moteID = newID;

    writeID();
    if (memoryMonitor == null) {
      memoryMonitor = new org.contikios.cooja.MoteMemory.MemoryMonitor() {
        public void memoryChanged(MoteMemory memory, MemoryEventType type, int address) {
          if (type != MemoryEventType.WRITE) {
            return;
          }
          writeID();
        }
      };
      addMonitor("node_id", memoryMonitor);
      addMonitor("TOS_NODE_ID", memoryMonitor);
      addMonitor("ActiveMessageAddressC__addr", memoryMonitor);
      addMonitor("ActiveMessageAddressC$addr", memoryMonitor);
    }

    notifyObservers();
  }

  private void writeID() {
    int id = getMoteID();
    if (moteMem.variableExists("node_id")) {
      moteMem.setIntValueOf("node_id", id);

      /* Set Contiki random seed variable if it exists */
      if (moteMem.variableExists("rseed")) {
        moteMem.setIntValueOf("rseed", (int) (mote.getSimulation().getRandomSeed() + id));
      }
    }
    if (moteMem.variableExists("TOS_NODE_ID")) {
      moteMem.setIntValueOf("TOS_NODE_ID", id);
    }
    if (moteMem.variableExists("ActiveMessageAddressC__addr")) {
      moteMem.setIntValueOf("ActiveMessageAddressC__addr", id);
    }
    if (moteMem.variableExists("ActiveMessageAddressC$addr")) {
      moteMem.setIntValueOf("ActiveMessageAddressC$addr", id);
    }
  }

  public void removed() {
    super.removed();
    if (memoryMonitor != null) {
      removeMonitor("node_id", memoryMonitor);
      removeMonitor("TOS_NODE_ID", memoryMonitor);
      removeMonitor("ActiveMessageAddressC__addr", memoryMonitor);
      removeMonitor("ActiveMessageAddressC$addr", memoryMonitor);
      memoryMonitor = null;
    }
  }

  private void addMonitor(String variable, org.contikios.cooja.MoteMemory.MemoryMonitor monitor) {
    if (!moteMem.variableExists(variable)) {
      return;
    }
    int address = moteMem.getVariableAddress(variable);
    moteMem.addMemoryMonitor(address, moteMem.getIntegerLength(), monitor);
  }

  private void removeMonitor(String variable, org.contikios.cooja.MoteMemory.MemoryMonitor monitor) {
    if (!moteMem.variableExists(variable)) {
      return;
    }
    int address = moteMem.getVariableAddress(variable);
    moteMem.removeMemoryMonitor(address, moteMem.getIntegerLength(), monitor);
  }

  @Override
  public JPanel getInterfaceVisualizer() {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }

  @Override
  public void releaseInterfaceVisualizer(JPanel panel) {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
  }
}
