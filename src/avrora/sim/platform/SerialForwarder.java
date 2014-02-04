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
import avrora.sim.clock.Clock;
import avrora.sim.mcu.USART;
import cck.util.Util;
import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * The <code>SerialForwarder</code> class implements a serial forwarder that takes traffic to and from a socket and
 * directs it into the UART chip of a simulated device.
 *
 * @author Olaf Landsiedel
 * @author Ben L. Titzer
 */
public class SerialForwarder implements USART.USARTDevice {

    public static final int BPS = 57600;

    private ServerSocket serverSocket = null;
    private Socket socket = null;
    private Socket newSocket = null;
    private OutputStream out = null;
    private InputStream in = null;
    private USART usart;
    private SFTicker ticker;
    private byte[] data;
    protected int portNumber;
    private final Simulator simulator;
    
    /**
     * Connects the traffic to and from a socket and directs it into the UART chip of a simulated device.
     *
     * @param usart USART device to redirect
     * @param pn socket port number
     */

    public SerialForwarder(USART usart, int pn, Simulator sim, boolean waitForConnection) {
        usart.connect(this);

        this.usart = usart;
        this.portNumber = pn;
        this.simulator = sim;
        ticker = new SFTicker(usart.getClock(), BPS);
        data = new byte[1];
        try {
            serverSocket = new ServerSocket(portNumber);
        } catch (IOException e) {
            throw Util.unexpected(e);
        }
        simulator.getPrinter().println("Waiting for serial connection on port " + portNumber + "...");
        
        if (waitForConnection) {
            try {
                newSocket = serverSocket.accept();
            } catch (IOException e) {
                throw Util.unexpected(e);
            }
            simulator.getPrinter().println("connected to " + newSocket.getRemoteSocketAddress());
            ticker.start();
        }
        
        new Thread(new Runnable() {
            public void run() {
                try {
                    do {
                        Socket mySocket = serverSocket.accept();
                        simulator.getPrinter().println("connected to " + mySocket.getRemoteSocketAddress());
                        // we assign the streams in checkReconnection called from the ticker 
                        synchronized (this) {
                            newSocket = mySocket;
                            if (socket == null) {
                                ticker.start();
                            }
                        }
                    } while(simulator.getSimulation().isRunning());
                    serverSocket.close();
                } catch (IOException e) {
                    // ignore - we have shut down the server socket
                }
            }
        }).start();
    }

    synchronized private void checkReconnection() {
        if (newSocket != null) {
            if (socket != null) {
                try {
                    socket.close();
                } catch (IOException e) {
                    // ignore
                }
            }
            socket = newSocket;
            newSocket = null;
            try {
                out = socket.getOutputStream();
                in = socket.getInputStream();
            } catch (IOException e) {
                throw Util.unexpected(e);
            }
        }
    }

    synchronized private void closeSocketInOut() {
        out = null;
        in = null;
        if (socket != null) {
            try {
                socket.close();
            } catch (IOException e) {
                // ignore
            }
            socket = null;
        }
    }

    /**
     * Connect the traffic of an USART device to a device in the host system.
     *
     * @param usdv  USART device to redirect
     * @param infile the name of the file (which could be a UNIX device) that specifies the input file
     * @param outfile the name of the file (which could be a UNIX device) taht specifies the output file
     */
    public SerialForwarder(USART usdv, String infile, String outfile) {
        usart = usdv;
        portNumber = 0;
        simulator = null;
        data = new byte[1];

        try {
            if (!infile.equals(outfile) ) {
                in = new FileInputStream(infile);
                out = new FileOutputStream(outfile);
            } else {
                RandomAccessFile handle = new RandomAccessFile(infile, "rw");
                in = new FileInputStream(handle.getFD());
                out = new FileOutputStream(handle.getFD());
            }
        } catch (IOException e) {
            throw Util.unexpected(e);
        }

        ticker = new SFTicker(usdv.getClock(), BPS);
        ticker.start();
        usdv.connect(this);
    }

    /**
     * Connect the traffic of an USART device to an external process.
     *
     * @param usdv   USART device to redirect
     * @param command Command line tokens
     */
    public SerialForwarder(USART usdv, String[] command) {
        usart = usdv;
        portNumber = 0;
        simulator = null;
        data = new byte[1];

        try {
            Process p = Runtime.getRuntime().exec(command);
            in = p.getInputStream();
            out = p.getOutputStream();
        } catch (IOException e) {
            throw Util.unexpected(e);
        }

        ticker = new SFTicker(usdv.getClock(), BPS);
        ticker.start();
        usdv.connect(this);
    }
    
    public USART.Frame transmitFrame() {
        if (in != null) {
            try {
                in.read(data, 0, 1);
                return new USART.Frame(data[0], false, 8);
            } catch (IOException e) {
                // socket has probably gone away
                closeSocketInOut();
            }
        }
        return new USART.Frame((byte)0, false, 8);
    }


    public void receiveFrame(USART.Frame frame) {
        try {
            if (out != null)
                out.write((byte)frame.value);
        } catch (IOException e) {
            // socket has probably gone away
            closeSocketInOut();
        }
    }

    public void stop() {
        if (serverSocket != null)
            try {
                closeSocketInOut();
                serverSocket.close();
            } catch (IOException e) {
                throw Util.unexpected(e);
            }
    }
    
    private class SFTicker implements Simulator.Event {

        private final long delta;
        private final Clock clock;

        SFTicker(Clock c, int bps) {
            delta = c.getHZ() / bps;
            clock = c;
        }

        public void fire() {
            checkReconnection();
            try {
                if (in != null && in.available() >= 1) {
                    usart.startReceive();
                }
            } catch (IOException e) {
                // socket has probably gone away
                closeSocketInOut();
            }
            if (in != null) {
                clock.insertEvent(this, delta);
            }
        }

        void start() {
            clock.insertEvent(this, delta);
        }
    }
}
