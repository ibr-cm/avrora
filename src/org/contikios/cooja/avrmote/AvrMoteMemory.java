/*
 * Copyright (c) 2009, Swedish Institute of Computer Science. All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer. 2. Redistributions in
 * binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution. 3. Neither the name of the
 * Institute nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

package org.contikios.cooja.avrmote;

import java.util.ArrayList;
import java.util.Iterator;

import org.apache.log4j.Logger;

import org.contikios.cooja.AddressMemory;
import org.contikios.cooja.MoteMemory;
import avrora.arch.avr.AVRProperties;
import avrora.core.SourceMapping;
import avrora.core.SourceMapping.Location;
import avrora.sim.AtmelInterpreter;
import avrora.sim.Simulator.Watch;
import avrora.sim.State;
/**
 * @author Joakim Eriksson, Fredrik Osterlind, David Kopf
 */
public class AvrMoteMemory implements MoteMemory, AddressMemory {
  private static Logger logger = Logger.getLogger(AvrMoteMemory.class);

  private SourceMapping memoryMap;
  private AtmelInterpreter interpreter;

  private boolean coojaIsAccessingMemory;

  public AvrMoteMemory(SourceMapping map, AVRProperties avrProperties, AtmelInterpreter interpreter) {
    memoryMap = map;
    this.interpreter = interpreter;
  }

  public void clearMemory() {
    logger.fatal("clearMemory() not implemented");
  }

  public byte[] getMemorySegment(int address, int size) {
    /*logger.info("getMemorySegment(" + String.format("0x%04x", address) +
        ", " + size + ")");*/

    /* XXX Unsure whether this is the appropriate method to use, as it
     * triggers memoryRead monitor. Right now I'm using a flag to indicate
     * that Cooja (as opposed to Contiki) read the memory, to avoid infinite
     * recursion. */
    coojaIsAccessingMemory = true;
    byte[] data = new byte[size];
    for (int i=0; i < size; i++) {
      data[i] = (byte) (interpreter.getDataByte(address + i) & 0xff);
    }
    coojaIsAccessingMemory = false;
    return data;
  }

  public int getTotalSize() {
    logger.warn("getTotalSize() not implemented");
    return -1;
  }

  public void setMemorySegment(int address, byte[] data) {
    coojaIsAccessingMemory = true;
    /*logger.info("setMemorySegment(" + String.format("0x%04x", address) +
        ", arr:" + data.length + ")");*/

    /* XXX See comment in getMemorySegment. */
    coojaIsAccessingMemory = true;
    for (int i=0; i < data.length; i++) {
      interpreter.writeDataByte(address + i, data[i]);
    }
    coojaIsAccessingMemory = false;
  }

  public byte getByteValueOf(String varName) throws UnknownVariableException {
    return getMemorySegment(getVariableAddress(varName), 1)[0];
  }

  public int getIntValueOf(String varName) throws UnknownVariableException {
    byte[] arr = getMemorySegment(getVariableAddress(varName), getIntegerLength());
    return parseInt(arr);
  }

  public int getIntegerLength() {
    return 2; /* XXX */
  }

  public int getVariableAddress(String varName)
  throws UnknownVariableException {
    /* RAM addresses start at 0x800000 in Avrora */
    int vma = memoryMap.getLocation(varName).vma_addr;
    int ret = vma & 0x7fffff;
    /*logger.info("Symbol '" + memoryMap.getLocation(varName).name +
        "': vma_addr: " + String.format("0x%04x", vma) +
        ", returned address: " + String.format("0x%04x", ret));*/
    return ret;
  }

  public String[] getVariableNames() {
    ArrayList<String> symbols = new ArrayList<String>();
    for (Iterator i = memoryMap.getIterator(); i.hasNext();) {
      symbols.add(((Location) i.next()).name);
    }
    return symbols.toArray(new String[0]);
  }

  public void setByteArray(String varName, byte[] data)
  throws UnknownVariableException {
    setMemorySegment(getVariableAddress(varName), data);
  }

  public void setByteValueOf(String varName, byte newVal)
  throws UnknownVariableException {
    setMemorySegment(getVariableAddress(varName), new byte[] {newVal});
  }

  public void setIntValueOf(String varName, int newVal)
  throws UnknownVariableException {
    int varAddr = getVariableAddress(varName);
    int newValToSet = Integer.reverseBytes(newVal);
    int pos = 0;
    byte[] varData = new byte[2];
    varData[pos++] = (byte) ((newValToSet & 0xFF000000) >> 24);
    varData[pos++] = (byte) ((newValToSet & 0xFF0000) >> 16);
    setMemorySegment(varAddr, varData);
  }

  public boolean variableExists(String varName) {
    return memoryMap.getLocation(varName) != null;
  }

  class AvrMemoryMonitor {
    int address;
    int size;
    MemoryMonitor mm;
    Watch watch;
  }
  private ArrayList<AvrMemoryMonitor> memoryMonitors = new ArrayList<AvrMemoryMonitor>();

  public boolean addMemoryMonitor(int address, int size, MemoryMonitor mm) {
    final AvrMemoryMonitor mon = new AvrMemoryMonitor();
    mon.address = address;
    mon.size = size;
    mon.mm = mm;
    mon.watch = new Watch.Empty() {
      public void fireAfterRead(State state, int data_addr, byte value) {
        if (coojaIsAccessingMemory) {
          return;
        }
        mon.mm.memoryChanged(AvrMoteMemory.this, MemoryEventType.READ, data_addr);
      }
      public void fireAfterWrite(State state, int data_addr, byte value) {
        if (coojaIsAccessingMemory) {
          return;
        }
        mon.mm.memoryChanged(AvrMoteMemory.this, MemoryEventType.WRITE, data_addr);
      }
    };

    interpreter.getSimulator().insertWatch(mon.watch, mon.address);
    memoryMonitors.add(mon);
    return true;
  }

  public void removeMemoryMonitor(int address, int size, MemoryMonitor mm) {
    for (AvrMemoryMonitor mcm: memoryMonitors) {
      if (mcm.mm != mm || mcm.address != address || mcm.size != size) {
        continue;
      }
      memoryMonitors.remove(mcm);
      break;
    }
  }

  public int parseInt(byte[] memorySegment) {
    if (memorySegment.length < 2) {
      return -1;
    }

    int retVal = 0;
    int pos = 0;
    retVal += ((memorySegment[pos++] & 0xFF)) << 8;
    retVal += ((memorySegment[pos++] & 0xFF)) << 0;

    return Integer.reverseBytes(retVal) >> 16;
  }

  public byte[] getByteArray(String varName, int length)
      throws UnknownVariableException {
    return getMemorySegment(getVariableAddress(varName), length);
  }
}
