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

import avrora.sim.Simulation;

import cck.help.HelpCategory;
import cck.util.Options;

import java.util.*;

/**
 * handles node positions.
 *
 * @author Olaf Landsiedel
 * @author Rodolfo de Paz
 * @author Daniel Minder
 */
public abstract class Topology extends HelpCategory {

    public static class Position {
        public double x,y,z,rho;

        public Position(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.rho = 0;
        }
        
        public Position(double x, double y, double z, double rho) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.rho = rho;
        }
    }

    //structure of the node positions
    protected final ArrayList<Position> positions;
    protected final ArrayList<Simulation.Node> nodes;

    /**
     * new topology
     *
     * @param h the help item for the topology as string
     * @param o the options representing the known and unknown options from the command line
     */
    protected Topology(String h) {
        super("topology", h);
        addSection("TOPOLOGY OVERVIEW", help);
        addOptionSection("Help for the options accepted by this topology is below.", options);
      
        positions = new ArrayList<Position>();
        nodes = new ArrayList<Simulation.Node>();
    }

    public Position getPosition(int id) {
        return positions.get(id);
    }
    
    public void addNode(Simulation.Node node) {
        nodes.add(node);
    }
    
    public void processOptions(Options o) {
        options.process(o);
    }
    
    public abstract void start();
}
