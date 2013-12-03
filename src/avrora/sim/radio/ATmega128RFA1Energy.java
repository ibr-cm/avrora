
package avrora.sim.radio;

import cck.text.StringUtil;
/**
 * Constants for Atmel ATmega128RFA1 radio energy consumption
 * 
 * @author David Kopf
 */
public abstract class ATmega128RFA1Energy {  
   
    /**
     * <code>startMode</code> the default operating mode after turning on / reset
     */
    public static final int startMode = 0;

    /**
     * <code>modeName</code> names of the operating modes
     * We include the Receive HL state from the AT85RF231, not present in the current RFA1
     */
    public static final String[] modeName = {
        "Power Off:",
        "Power Down:",
        "Idle:",
        "Receive (Rx):",
        "Receive HL (Rx):",
        "Receive LS (Rx):",
        "Transmit (Tx):"
    };


    /**
     * <code>modeAmpere</code> power consumption of the operating modes
     */
    //3.3 volts @ 25C, Values other than 3,1,-3,-17 are guesses.
    //TODO:interpolate from data sheet values
    public static final double[] modeAmpere = {                                             
        0.00000002,//Sleep 20nA
        0.000400,  //TRX_OFF
        0.0057,    //P_ON, PLL_ON
        0.0125,    //RX_ON High sensitivity
        0.0120,    //RX_ON High input level
        0.0120,    //RX_ON Low sensitivity
        0.0080,    //TX_BUSY, TX_PWR=15 (-17 dBm)
        0.0083,    // TX_PWR=14 (-12 dBm)
        0.0085,    // TX_PWR=13 (- 9 dBm)
        0.0087,    // TX_PWR=12 (- 7 dBm)
        0.0088,    // TX_PWR=11 (- 5 dBm)
        0.0089,    // TX_PWR=10 (- 4 dBm)
        0.0090,    // TX_PWR= 9 (- 3 dBm)
        0.0092,    // TX_PWR= 8 (- 2 dBm)
        0.0094,    // TX_PWR= 7 (- 1 dBm)
        0.0960,    // TX_PWR= 6 (  0 dBm)
        0.0980,    // TX_PWR= 5 (0.7 dBm)
 //     0.0100,    //           (1.0 dBm)
        0.0105,    // TX_PWR= 4 (1.3 dBm)
        0.0110,    // TX_PWR= 3 (1.8 dBm)
        0.0130,    // TX_PWR= 2 (2.3 dBm)
        0.0140,    // TX_PWR= 1 (2.8 dBm)
        0.0145     // TX_PWR= 0 (3.0 dBm)      
    };

    public static String[] allModeNames() {
        String[] modeName = new String[16+6];

        System.arraycopy(ATmega128RFA1Energy.modeName, 0, modeName, 0, 6);
        for (int i = 0; i < 16; i++) {
            modeName[i + 6] = ATmega128RFA1Energy.modeName[6] + StringUtil.leftJustify(i+":  ", 2);
        }
        return modeName;
    }
}
