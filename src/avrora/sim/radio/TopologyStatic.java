/**
 * Created on 02.11.2004
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

package avrora.sim.radio;

import cck.util.Option;
import cck.util.Util;

import java.io.*;
import java.util.*;

import cck.text.Terminal;

/**
 * handles node positions.
 *
 * @author Olaf Landsiedel
 * @author Rodolfo de Paz
 */
public class TopologyStatic extends Topology {
    public final Option.Str TOPOLOGY_FILE = newOption("topology-file", "",
        "This option can be used to specify the name of " +
        "a file that contains information about the topology of the network.");

    /**
     * new topology
     */
    public TopologyStatic() {
        super("Static topology. topology-file must be given!");
    }
    
    /**
     * new topology
     *
     * @param h the help item for the topology as string
     * @param o the options representing the known and unknown options from the command line
     */
    public TopologyStatic(String h) {
        super (h);
    }
    
    public void start() {
        if (TOPOLOGY_FILE.isBlank()) {
            Util.userError("topology-file must be given!");
        }
        parseFile();
    }

    protected void parseFile() {
        try {
            String line;
            BufferedReader f = new BufferedReader(new FileReader(TOPOLOGY_FILE.get()));
            while ((line = f.readLine()) != null) {
                parseLine(line);
            }
            f.close();
        } catch (IOException e) {
            throw Util.unexpected(e);
        }
    }

    /**
     * parse one line of the file
     *
     * @param line
     */
    private void parseLine(String line) {
        String nodeName = "";
        double[] newpos = new double[4];
        //check for comment
        if (!line.startsWith("#")) {
            StringTokenizer tokenizer = new StringTokenizer(line, " ");
            int count = 0;
            while (tokenizer.hasMoreTokens() && count < 5) {
                try {
                    if (count == 0)
                        nodeName = tokenizer.nextToken();
                    else {
                        newpos[count - 1] = Double.parseDouble(tokenizer.nextToken());
                    }
                    count++;
                } catch (NoSuchElementException e) {
                    throw Util.failure("Error reading topology tokens");
                }
            }
            if (count == 4) {
                //found 4 tokens so we won't have obstacles (rho = 0)
                positions.add(new Position(newpos[0], newpos[1], newpos[2], 0.0));
            }else if (count == 5){
                //found 5 tokens so density of obstacles is included in topology file
                positions.add(new Position(newpos[0], newpos[1], newpos[2], newpos[3]));
            }
        }
    }

}
