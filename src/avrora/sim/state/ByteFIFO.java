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
 * Created Oct 10, 2007
 */
package avrora.sim.state;

import java.util.Arrays;

/**
 * The <code>FIFO</code> class implements a FIFO (queue) of bytes of a fixed size.
 * It detects overflow and underflow and has sticky flags that can be queried for
 * these conditions and cleared.
 *
 * @author Ben L. Titzer
 */
public class ByteFIFO {

    protected final byte[] data;

    protected int head;
    protected int used;

    protected int save_head;
    protected int save_used;
  
    protected boolean underflow;
    protected boolean overflow;

    /**
     * The default constructor for this class creates a new FIFO with the specified size.
     *
     * @param sz the size of the FIFO queue, in bytes
     */
    public ByteFIFO(int sz) {
        data = new byte[sz];
    }

    /**
     * The <code>add()</code> method adds a byte to this FIFO at the end. If the FIFO is
     * not yet full, the byte will be successfully added. If the FIFO is already full,
     * then this method will set the overflow flag which can be queried with the
     * <code>overflow()</code> method.
     *
     * @param c the byte to add to this FIFO
     */
    public void add(byte c) {
        if (full()) {
            overflow = true;
        } else data[wrap(head + used++)] = c;
    }

    /**
     * The <code>remove()</code> method removes a byte from this FIFO, if one exists. If
     * the FIFO contains elements, the first element entered into the queue is removed
     * and returned. If the queue is empty, this this method sets the underflow flag
     * which can be queried with the <code>underflow()</code> method.
     *
     * @return the next byte in the FIFO if it exists; <code>0</code> otherwise
     */
    public byte remove() {
        byte b = 0;
        if (empty()) {
            underflow = true;
        } else {
            b = data[head];
            head = wrap(head + 1);
            used--;
        }
        return b;
    }

    /**
     * Gets the byte at absolute position in FIFO
     * @param ind Absolute position
     * @return byte at position ind from beginning of FIFO
     */
    public byte getAbsoluteByte(int ind) {
        return data[ind];
    }
    
    /**
     * Sets the byte at absolute position in FIFO and returns old value
     * @param ind Absolute position
     * @param val New value
     * @return Old values
     */
    public byte setAbsoluteByte(int ind, byte val) {
    	byte prev = data[ind];
        data[ind] = val;
        return prev;
    }
    
    /**
     * Get the byte relative to a packet, which starts at the saved head.
     * Thus, clear() or saveState() has to be used at the beginning of a packet.
     * @param ind Position relative to saved head
     * @return byte at the given position
     */
    public byte getRelativeByte(int ind) {
        return data[wrap(head + ind)];
    }

    /**
     * Get a series of bytes relative to a packet, which starts at the saved head.
     * Thus, clear() or saveState() has to be used at the beginning of a packet.
     * @param from Start position relative to saved head
     * @param to End position relative to saved head
     * @return byte array with the values in the given range
     */
    public byte[] getRelativeField(int from, int to) {
        if (((head + from) < data.length) && ((head + to) == (data.length)))
            return copyOfRange(data, wrap(head + from), (head + to));
        else if (((head + from) < data.length) && ((head + to) > (data.length))) {
            byte[] r = new byte[to - from];
            System.arraycopy(data, wrap(head + from), r, 0, (data.length - (head + from)));
            System.arraycopy(data, 0, r, (data.length - (head + from)), wrap(head + to));
            return r;
        } else return copyOfRange(data, wrap(head + from), wrap(head + to));
    }

    /**
     * The <code>underFlow()</code> method returns the status of the underflow flag,
     * which occurs when a <code>remove()</code> occurs on an empty FIFO.
     * The underflow flag is cleared by the <code>clear()</code> and <code>clearFlags()</code>
     * methods.
     *
     * @return true if this FIFO has experienced an underflow condition in a previous call;
     *         false otherwise
     */
    public boolean underFlow() {
        return underflow;
    }

    /**
     * The <code>overFlow()</code> method returns the status of the overflow flag, which
     * is set when an <code>add()</code> call occurs on a full FIFO.
     * The overflow flag is cleared by the <code>clear()</code> and <code>clearFlags()</code>
     * methods.
     *
     * @return true if this FIFO has experienced an underflow condition in a previous call;
     *         false otherwise
     */
    public boolean overFlow() {
        return overflow;
    }

    public int capacity() {
        return data.length;
    }

    public int size() {
        return used;
    }

    public boolean full() {
        return used == data.length;
    }

    public boolean empty() {
        return used == 0;
    }

    public void clearFlags() {
        overflow = false;
        underflow = false;
    }

    public void saveState() {
      save_head = head;
      save_used = used;
    }
    
    public void refill() {
        overflow = false;
        underflow = false;
        head = save_head;
        used = save_used;        
    }
    
    public void dropLast() {
        // how many bytes have been read since saveState()?
        int readBytes = head > save_head ? head - save_head : data.length-save_head+head;
        if (readBytes > save_used) {  // more bytes read than were available at saveState() time
            used = 0;  // now it's empty
        }
        else {
            used = save_used - readBytes;  // drop those that have been added since saveState()
        }
    }

    public void clear() {
        overflow = false;
        underflow = false;
        save_head = head = 0;
        save_used = used = 0;
        Arrays.fill(data, (byte) 0);
    }

    private int wrap(int i) {
        if (i >= data.length) return i - data.length;
        return i;
    }

    public static byte[] copyOfRange(byte[] origin, int from, int to) {
        byte[] narray = new byte[to - from];
        for (int i = 0; i < narray.length; i++) {
            narray[i] = origin[from + i];
        }
        return narray;
    }
}
