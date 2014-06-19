/*
 * Copyright (c) 2014, TU Braunschweig.
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
 */

package avrora.sim.radio;

import java.util.Arrays;
import java.util.Random;

import avrora.sim.AtmelInterpreter;
import avrora.sim.FiniteStateMachine;
import avrora.sim.RWRegister;
import avrora.sim.Simulator;
import avrora.sim.clock.Synchronizer;
import avrora.sim.energy.Energy;
import avrora.sim.mcu.Microcontroller;
import avrora.sim.mcu.SPI;
import avrora.sim.mcu.SPIDevice;
import avrora.sim.output.SimPrinter;
import avrora.sim.state.BooleanRegister;
import avrora.sim.state.BooleanView;
import avrora.sim.state.ByteFIFO;
import avrora.sim.state.RegisterUtil;
import avrora.sim.state.RegisterView;
import cck.text.StringUtil;
import cck.util.Arithmetic;
import java.util.LinkedList;

/**
 * The <code>AT86RF231Radio</code> implements a simulation of the Atmel
 * AT86RF231 radio, and can also be used for the AT86RF230.
 * Verbose printers for this class include "radio.rf231"
 *
 * @author David A. Kopf
 * @author Enrico Jorns
 */
public class AT86RF231Radio implements Radio {
    private final static boolean DEBUG   = false;  //state changes, interrupts
    private final static boolean DEBUGV  = false;  //pin changes
    private final static boolean DEBUGRX = false;  //receiver
    private final static boolean DEBUGTX = false;  //transmitter
    private final static boolean DEBUGA  = false;  //ACKs
    private final static boolean DEBUGC  = false;  //CCA, CSMA
    private final static boolean DEBUGE  = false;  //"should not happen" debugs
    
    private static final boolean MSPSIM_SFD_COMPAT = true;

    //-- Radio states, confusingly contained in the TRX_STATUS register --------
    public static final byte STATE_P_ON               = 0x00;
    public static final byte STATE_BUSY_RX            = 0x01;
    public static final byte STATE_BUSY_TX            = 0x02;
    public static final byte STATE_RX_ON              = 0x06;
    public static final byte STATE_TRX_OFF            = 0x08;
    public static final byte STATE_PLL_ON             = 0x09;
    public static final byte STATE_SLEEP              = 0x0F;
    public static final byte STATE_BUSY_RX_AACK       = 0x11;
    public static final byte STATE_BUSY_TX_ARET       = 0x12;
    public static final byte STATE_RX_AACK_ON         = 0x16;
    public static final byte STATE_TX_ARET_ON         = 0x19;
    public static final byte STATE_RX_ON_NOCLK        = 0x1C;
    public static final byte STATE_RX_AACK_ON_NOCLK   = 0x1D;
    public static final byte STATE_BUSY_RX_AACK_NOCLK = 0x1E;
    public static final byte STATE_TRANSITION         = 0x1F;
    //-- Radio commands--------------------------------------------------------
    public static final byte CMD_NOP            = 0x00;
    public static final byte CMD_TX_START       = 0x02;
    public static final byte CMD_FORCE_TRX_OFF  = 0x03;
    public static final byte CMD_FORCE_PLL_ON   = 0x04;
    public static final byte CMD_RX_ON          = 0x06;
    public static final byte CMD_TRX_OFF        = 0x08;
    public static final byte CMD_TX_ON          = 0x09;
    public static final byte CMD_PLL_ON         = 0x09; // alias
    public static final byte CMD_RX_AACK_ON     = 0x16;
    public static final byte CMD_TX_ARET_ON     = 0x19;
    //-- Register addresses ---------------------------------------------------
    // The rf230 does not use all registers but initializes them anyway.
    public static final int TRX_STATUS   = 0x01;   //CCA_DONE, CCA_STATUS, TST_STATUS, TRX_STATUS[4:0]
    public static final int TRX_STATE    = 0x02;   //TRAC_STATUS[2:0],TRX_CMD[4:0]
    public static final int TRX_CTRL_0   = 0x03;
    public static final int TRX_CTRL_1   = 0x04;//rf231
    public static final int PHY_TX_PWR   = 0x05;
    public static final int PHY_RSSI     = 0x06;
    public static final int PHY_ED_LEVEL = 0x07;
    public static final int PHY_CC_CCA   = 0x08;
    public static final int CCA_THRES    = 0x09;
    public static final int RX_CTRL      = 0x0A;//rf231
    public static final int SFD_VALUE    = 0x0B;//rf231
    public static final int TRX_CTRL_2   = 0x0C;//rf231
    public static final int ANT_DIV      = 0x0D;//rf231
    public static final int IRQ_MASK     = 0x0E;
    public static final int IRQ_STATUS   = 0x0F;
    public static final int VREG_CTRL    = 0x10;
    public static final int BATMON       = 0x11;
    public static final int XOSC_CTRL    = 0x12;
    public static final int RX_SYN       = 0x15;//rf231
    public static final int XAH_CTRL_1   = 0x17;//rf231
    public static final int PLL_CF       = 0x1A;
    public static final int PLL_DCU      = 0x1B;
    public static final int PART_NUM     = 0x1C;
    public static final int VERSION_NUM  = 0x1D;
    public static final int MAN_ID_0     = 0x1E;
    public static final int MAN_ID_1     = 0x1F;
    public static final int SHORT_ADDR_0 = 0x20;
    public static final int SHORT_ADDR_1 = 0x21;
    public static final int PAN_ID_0     = 0x22;
    public static final int PAN_ID_1     = 0x23;
    public static final int IEEE_ADDR_0  = 0x24;
    public static final int IEEE_ADDR_1  = 0x25;
    public static final int IEEE_ADDR_2  = 0x26;
    public static final int IEEE_ADDR_3  = 0x27;
    public static final int IEEE_ADDR_4  = 0x28;
    public static final int IEEE_ADDR_5  = 0x29;
    public static final int IEEE_ADDR_6  = 0x2A;
    public static final int IEEE_ADDR_7  = 0x2B;
    public static final int XAH_CTRL_0   = 0x2C;
    public static final int CSMA_SEED_0  = 0x2D;
    public static final int CSMA_SEED_1  = 0x2E;
    public static final int CSMA_BE      = 0x2F;//rf231
    
    //-- Register implementation
    /* 0x01 */
    private class TRX_STATUS_Reg extends RWRegister {

        static final int TRX_STATUS_L   = 0;
        static final int TRX_STATUS_H   = 4;
        static final int CCA_STATUS     = 6;
        static final int CCA_DONE       = 7;

        final RegisterView _trx_status = RegisterUtil.bitRangeView(this, TRX_STATUS_L, TRX_STATUS_H);
        final RegisterView _cca_status = RegisterUtil.bitView(this, CCA_STATUS);
        final BooleanView _cca_done = RegisterUtil.booleanView(this, CCA_DONE);
    }

    /* 0x02 */
    private class TRX_STATE_Reg extends RWRegister {

        static final int TRX_CMD_L      = 0;
        static final int TRX_CMD_H      = 4;
        static final int TRAC_STATUS_L  = 5;
        static final int TRAC_STATUS_H  = 7;

        final RegisterView _trx_cmd = RegisterUtil.bitRangeView(this, TRX_CMD_L, TRX_CMD_H);
        final RegisterView _trac_status = RegisterUtil.bitRangeView(this, TRAC_STATUS_L, TRAC_STATUS_H);
    }

    /* TRAC_STATUS Transaction Status (refer to Table 7-16) */
    private static final int TRAC_STATUS_SUCCESS = 0;
    private static final int TRAC_STATUS_SUCESS_DATA_PENDING = 1;
    private static final int TRAC_STATUS_SUCCESS_WAIT_FOR_ACK = 2;
    private static final int TRAC_STATUS_CHANNEL_ACCESS_FAILURE = 3;
    private static final int TRAC_STATUS_NO_ACK = 5;
    private static final int TRAC_STATUS_INVALID = 7;
    
    /* 0x03 */
    private class TRX_CTRL_0_Reg extends RWRegister {
        
        static final int CLKM_CTRL_L    = 0;
        static final int CLKM_CTRL_H    = 2;
        static final int CLKM_SHA_SEL   = 3;
        static final int PAD_IO_CLKM_L  = 4;
        static final int PAD_IO_CLKM_H  = 5;
        static final int PAD_IO_L       = 6;
        static final int PAD_IO_H       = 7;

        final RegisterView _clkm_ctrl = RegisterUtil.bitRangeView(this, CLKM_CTRL_L, CLKM_CTRL_H);
        final RegisterView _clkm_sha_sel = RegisterUtil.bitView(this, CLKM_SHA_SEL);
        final RegisterView _pad_io_clkm = RegisterUtil.bitRangeView(this, PAD_IO_CLKM_L, PAD_IO_CLKM_H);
        final RegisterView _pad_io = RegisterUtil.bitRangeView(this, PAD_IO_L, PAD_IO_H);
    }
    
    /* 0x04 */
    private class TRX_CTRL_1_Reg extends RWRegister {
        
        static final int IRQ_POLARITY   = 0;
        static final int IRQ_MASK_MODE  = 1;
        static final int SPI_CMD_MODE_L = 2;
        static final int SPI_CMD_MODE_H = 3;
        static final int RX_BL_CTRL     = 4;
        static final int TX_AUTO_CRC_ON = 5;
        static final int IRQ_2_EXT_EN   = 6;
        static final int PA_EXT_EN      = 7;

        final RegisterView _irq_polarity = RegisterUtil.bitView(this, IRQ_POLARITY);
        final RegisterView _irq_mask_mode = RegisterUtil.bitView(this, IRQ_MASK_MODE);
        final RegisterView _spi_cmd_mode = RegisterUtil.bitRangeView(this, SPI_CMD_MODE_L, SPI_CMD_MODE_H);
        final RegisterView _rx_bl_ctrl = RegisterUtil.bitView(this, RX_BL_CTRL);
        final BooleanView _tx_auto_crc_on = RegisterUtil.booleanView(this, TX_AUTO_CRC_ON);
        final RegisterView _irq_2_ext_en = RegisterUtil.bitView(this, IRQ_2_EXT_EN);
        final RegisterView _pa_ext_en = RegisterUtil.bitView(this, PA_EXT_EN);
    }

    /* 0x05 */
    private class PHY_TX_PWR_Reg extends RWRegister {
        static final int TX_PWR_L    = 0;
        static final int TX_PWR_H    = 3;
        static final int PA_LT_L     = 4;
        static final int PA_LT_H     = 5;
        static final int PA_BUF_LT_L = 6;
        static final int PA_BUF_LT_H = 7;

        final RegisterView _tx_pwr = RegisterUtil.bitRangeView(this, TX_PWR_L, TX_PWR_H);
        final RegisterView _pa_lt = RegisterUtil.bitRangeView(this, PA_LT_L, PA_LT_H);
        final RegisterView _pa_buf_lt = RegisterUtil.bitRangeView(this, PA_BUF_LT_L, PA_BUF_LT_H);
    }

    /* 0x06 */
    private class PHY_RSSI_Reg extends RWRegister {
        static final int RSSI_L       = 0;
        static final int RSSI_H       = 4;
        static final int RND_VALUE_L  = 5;
        static final int RND_VALUE_H  = 6;
        static final int RX_CRC_VALID = 7;

        final RegisterView _rssi = RegisterUtil.bitRangeView(this, RSSI_L, RSSI_H);
        final RegisterView _rnd_value = RegisterUtil.bitRangeView(this, RND_VALUE_L, RND_VALUE_H);
        final BooleanView _rx_crc_valid = RegisterUtil.booleanView(this, RX_CRC_VALID);
    }

    /* 0x07 */
    private class PHY_ED_LEVEL_Reg extends RWRegister {
    }

    /* 0x08 */
    private class PHY_CC_CCA_Reg extends RWRegister {
        static final int CHANNEL_L   = 0;
        static final int CHANNEL_H   = 4;
        static final int CCA_MODE_L  = 5;
        static final int CCA_MODE_H  = 6;
        static final int CCA_REQUEST = 7;

        final RegisterView _channel = RegisterUtil.bitRangeView(this, CHANNEL_L, CHANNEL_H);
        final RegisterView _cca_mode = RegisterUtil.bitRangeView(this, CCA_MODE_L, CCA_MODE_H);
        final RegisterView _cca_request = RegisterUtil.bitView(this, CCA_REQUEST);
    }

    /* 0x09 */
    private class CCA_THRES_Reg extends RWRegister {
        static final int CCA_ED_THRES_L = 0;
        static final int CCA_ED_THRES_H = 3;

        final RegisterView _cca_ed_thres = RegisterUtil.bitRangeView(this, CCA_ED_THRES_L, CCA_ED_THRES_H);
    }

    /* 0x0A */
    private class RX_CTRL_Reg extends RWRegister {
        static final int PDT_THRES_L = 0;
        static final int PDT_THRES_H = 3;

        final RegisterView _pdt_thres = RegisterUtil.bitRangeView(this, PDT_THRES_L, PDT_THRES_H);
    }

    /* 0x0B */
    private class SFD_VALUE_Reg extends RWRegister {
    }

    /* 0x0C */
    private class TRX_CTRL_2_Reg extends RWRegister {
        static final int OQPSK_DATA_RATE_L  = 0;
        static final int OQPSK_DATA_RATE_H  = 1;
        static final int RX_SAFE_MODE       = 7;

        final RegisterView _oqpsk_data_rate = RegisterUtil.bitRangeView(this, OQPSK_DATA_RATE_L, OQPSK_DATA_RATE_H);
        final RegisterView _rx_safe_mode = RegisterUtil.bitView(this, RX_SAFE_MODE);
    }

    /* 0x0D */
    private class ANT_DIV_Reg extends RWRegister {
    }

    /* 0x0E */
    private class IRQ_MASK_Reg extends RWRegister {
        static final int MASK_PLL_LOCK    = 0;
        static final int MASK_PLL_UNLOCK  = 1;
        static final int MASK_RX_START    = 2;
        static final int MASK_TRX_END     = 3;
        static final int MASK_CCA_ED_DONE = 4;
        static final int MASK_AMI         = 5;
        static final int MASK_TRX_UR      = 6;
        static final int MASK_BAT_LOW     = 7;

        final BooleanView _mask_pll_lock = RegisterUtil.booleanView(this, MASK_PLL_LOCK);
        final BooleanView _mask_pll_unlock = RegisterUtil.booleanView(this, MASK_PLL_UNLOCK);
        final BooleanView _mask_rx_start = RegisterUtil.booleanView(this, MASK_RX_START);
        final BooleanView _mask_trx_end = RegisterUtil.booleanView(this, MASK_TRX_END);
        final BooleanView _mask_cca_ed_done = RegisterUtil.booleanView(this, MASK_CCA_ED_DONE);
        final BooleanView _mask_ami = RegisterUtil.booleanView(this, MASK_AMI);
        final BooleanView _mask_trx_ur = RegisterUtil.booleanView(this, MASK_TRX_UR);
        final BooleanView _mask_bat_low = RegisterUtil.booleanView(this, MASK_BAT_LOW);
    }

    /* 0x0F */
    private class IRQ_STATUS_Reg extends RWRegister {
        static final int PLL_LOCK     = 0;
        static final int PLL_UNLOCK   = 1;
        static final int RX_START     = 2;
        static final int TRX_END      = 3;
        static final int CCA_ED_DONE  = 4;
        static final int AMI          = 5;
        static final int TRX_UR       = 6;
        static final int BAT_LOW      = 7;

        final RegisterView _pll_lock = RegisterUtil.bitView(this, PLL_LOCK);
        final RegisterView _pll_unlock = RegisterUtil.bitView(this, PLL_UNLOCK);
        final RegisterView _rx_start = RegisterUtil.bitView(this, RX_START);
        final RegisterView _trx_end = RegisterUtil.bitView(this, TRX_END);
        final RegisterView _cca_ed_done = RegisterUtil.bitView(this, CCA_ED_DONE);
        final RegisterView _ami = RegisterUtil.bitView(this, AMI);
        final RegisterView _trx_ur = RegisterUtil.bitView(this, TRX_UR);
        final RegisterView _bat_low = RegisterUtil.bitView(this, BAT_LOW);
    }

    /* 0x10 */
    private class VREG_CTRL_Reg extends RWRegister {
        static final int DVDD_OK    = 2;
        static final int DVREG_EXT  = 3;
        static final int AVDD_OK    = 6;
        static final int AVREG_EXT  = 7;

        final RegisterView _dvdd_ok = RegisterUtil.bitView(this, DVDD_OK);
        final RegisterView _dvreg_ext = RegisterUtil.bitView(this, DVREG_EXT);
        final RegisterView _avdd_ok = RegisterUtil.bitView(this, AVDD_OK);
        final RegisterView _avreg_ext = RegisterUtil.bitView(this, AVREG_EXT);
    }

    /* 0x11 */
    private class BATMON_Reg extends RWRegister {
        static final int BATMON_VTH_L = 0;
        static final int BATMON_VTH_H = 3;
        static final int BATMON_HR    = 4;
        static final int BATMON_OK    = 5;

        final RegisterView _batmon = RegisterUtil.bitRangeView(this, BATMON_VTH_L, BATMON_VTH_H);
        final RegisterView _batmon_hr = RegisterUtil.bitView(this, BATMON_HR);
        final RegisterView _batmon_ok = RegisterUtil.bitView(this, BATMON_OK);
    }

    /* 0x12 */
    private class XOSC_CTRL_Reg extends RWRegister {
        static final int XTAL_TRIM_L = 0;
        static final int XTAL_TRIM_H = 3;
        static final int XTAL_MODE_L = 4;
        static final int XTAL_MODE_H = 7;

        final RegisterView _xtal_trim = RegisterUtil.bitRangeView(this, XTAL_TRIM_L, XTAL_TRIM_H);
        final RegisterView _xtal_mode = RegisterUtil.bitRangeView(this, XTAL_MODE_L, XTAL_MODE_H);
    }

    /* 0x13 */
    /* 0x14 */

    /* 0x15 */
    private class RX_SYN_Reg extends RWRegister {
        static final int RX_PDT_LEVEL_L = 0;
        static final int RX_PDT_LEVEL_H = 3;
        static final int RX_PDT_DIS     = 7;

        final RegisterView _rx_pdt_level = RegisterUtil.bitRangeView(this, RX_PDT_LEVEL_L, RX_PDT_LEVEL_H);
        final RegisterView _rx_pdt_dis = RegisterUtil.bitView(this, RX_PDT_DIS);
    }

    /* 0x16 */

    /* 0x17 */
    private class XAH_CTRL_1_Reg extends RWRegister {
        static final int AACK_PROM_MODE   = 1;
        static final int AACK_ACK_TIME    = 2;
        static final int AACK_UPLD_RES_FT = 4;
        static final int AACK_FLTR_RES_FT = 5;

        final BooleanView _aack_prom_mode = RegisterUtil.booleanView(this, AACK_PROM_MODE);
        final BooleanView _aack_ack_time = RegisterUtil.booleanView(this, AACK_ACK_TIME);
        final BooleanView _aack_upld_res_ft = RegisterUtil.booleanView(this, AACK_UPLD_RES_FT);
        final BooleanView _aack_fltr_res_ft = RegisterUtil.booleanView(this, AACK_FLTR_RES_FT);
    }

    /* 0x18 */
    private class FTN_CTRL_Reg extends RWRegister {
    }

    /* 0x19 */

    /* 0x1A */
    private class PLL_CF_Reg extends RWRegister {
    }

    /* 0x1B */
    private class PLL_DCU_Reg extends RWRegister {
    }

    /* 0x1C */
    private class PART_NUM_Reg extends RWRegister {
    }

    /* 0x1D */
    private class VERSION_NUM_Reg extends RWRegister {
    }

    /* 0x1E */
    private class MAN_ID_0_Reg extends RWRegister {
    }

    /* 0x1F */
    private class MAN_ID_1_Reg extends RWRegister {
    }

    /* 0x20 */
    private class SHORT_ADDR_0_Reg extends RWRegister {
    }

    /* 0x21 */
    private class SHORT_ADDR_1_Reg extends RWRegister {
    }

    /* 0x22 */
    private class PAN_ID_0_Reg extends RWRegister {
    }

    /* 0x23 */
    private class PAN_ID_1_Reg extends RWRegister {
    }

    /* 0x24 */
    private class IEEE_ADDR_0_Reg extends RWRegister {
    }

    /* 0x25 */
    private class IEEE_ADDR_1_Reg extends RWRegister {
    }

    /* 0x26 */
    private class IEEE_ADDR_2_Reg extends RWRegister {
    }

    /* 0x27 */
    private class IEEE_ADDR_3_Reg extends RWRegister {
    }

    /* 0x28 */
    private class IEEE_ADDR_4_Reg extends RWRegister {
    }

    /* 0x29 */
    private class IEEE_ADDR_5_Reg extends RWRegister {
    }

    /* 0x2A */
    private class IEEE_ADDR_6_Reg extends RWRegister {
    }

    /* 0x2B */
    private class IEEE_ADDR_7_Reg extends RWRegister {
    }

    /* 0x2C */
    private class XAH_CTRL_0_Reg extends RWRegister {
        static final int SLOTTED_OPERATION    = 0;
        static final int MAX_CSMA_RETRIES_L   = 1;
        static final int MAX_CSMA_RETRIES_H   = 3;
        static final int MAX_FRAME_RETRIES_L  = 4;
        static final int MAX_FRAME_RETRIES_H  = 7;
        
        final BooleanView _slotted_operation = RegisterUtil.booleanView(this, SLOTTED_OPERATION);
        final RegisterView _max_csma_retries = RegisterUtil.bitRangeView(this, MAX_CSMA_RETRIES_L, MAX_CSMA_RETRIES_H);
        final RegisterView _max_frame_retries = RegisterUtil.bitRangeView(this, MAX_FRAME_RETRIES_L, MAX_FRAME_RETRIES_H);
    }

    /* 0x2F */
    private class CSMA_SEED_0_Reg extends RWRegister {
    }

    /* 0x2E */
    private class CSMA_SEED_1_Reg extends RWRegister {
        static final int CSMA_SEED_1_L    = 0;
        static final int CSMA_SEED_1_H    = 2;
        static final int AACK_I_AM_COORD  = 3;
        static final int AACK_DIS_ACK     = 4;
        static final int AACK_SET_PD      = 5;
        static final int AACK_FVN_MODE_L  = 6;
        static final int AACK_FVN_MODE_H  = 7;
        
        final RegisterView _csma_seed_1 = RegisterUtil.bitRangeView(this, CSMA_SEED_1_L, CSMA_SEED_1_H);
        final BooleanView  _aack_i_am_coord = RegisterUtil.booleanView(this, AACK_I_AM_COORD);
        final BooleanView _aack_dis_ack = RegisterUtil.booleanView(this, AACK_DIS_ACK);
        final BooleanView _aack_set_pd = RegisterUtil.booleanView(this, AACK_SET_PD);
        final RegisterView _aack_fvn_mode = RegisterUtil.bitRangeView(this, AACK_FVN_MODE_L, AACK_FVN_MODE_H);
    }

    /* 0x2F */
    private class CSMA_BE_Reg extends RWRegister {
        static final int MIN_BE_L = 0;
        static final int MIN_BE_H = 3;
        static final int MAX_BE_L = 4;
        static final int MAX_BE_H = 7;
        
        final RegisterView _min_be = RegisterUtil.bitRangeView(this, MIN_BE_L, MIN_BE_H);
        final RegisterView _max_be = RegisterUtil.bitRangeView(this, MAX_BE_L, MAX_BE_H);
    }


    //-- Registers
    final TRX_STATUS_Reg TRX_STATUS_reg = new TRX_STATUS_Reg();
    final TRX_STATE_Reg TRX_STATE_reg = new TRX_STATE_Reg();
    final TRX_CTRL_0_Reg TRX_CTRL_0_reg = new TRX_CTRL_0_Reg();
    final TRX_CTRL_1_Reg TRX_CTRL_1_reg = new TRX_CTRL_1_Reg();
    final PHY_TX_PWR_Reg PHY_TX_PWR_reg = new PHY_TX_PWR_Reg();
    final PHY_RSSI_Reg PHY_RSSI_reg = new PHY_RSSI_Reg();
    final PHY_ED_LEVEL_Reg PHY_ED_LEVEL_reg = new PHY_ED_LEVEL_Reg();
    final PHY_CC_CCA_Reg PHY_CC_CCA_reg = new PHY_CC_CCA_Reg();
    final CCA_THRES_Reg CCA_THRES_reg = new CCA_THRES_Reg();
    final RX_CTRL_Reg RX_CTRL_reg = new RX_CTRL_Reg();
    final SFD_VALUE_Reg SFD_VALUE_reg = new SFD_VALUE_Reg();
    final TRX_CTRL_2_Reg TRX_CTRL_2_reg = new TRX_CTRL_2_Reg();
    final ANT_DIV_Reg ANT_DIV_reg = new ANT_DIV_Reg();
    final IRQ_MASK_Reg IRQ_MASK_reg = new IRQ_MASK_Reg();
    final IRQ_STATUS_Reg IRQ_STATUS_reg = new IRQ_STATUS_Reg();
    final VREG_CTRL_Reg VREG_CTRL_reg = new VREG_CTRL_Reg();
    final BATMON_Reg BATMON_reg = new BATMON_Reg();
    final XOSC_CTRL_Reg XOSC_CTRL_reg = new XOSC_CTRL_Reg();
    final RX_SYN_Reg RX_SYN_reg = new RX_SYN_Reg();
    final XAH_CTRL_1_Reg XAH_CTRL_1_reg = new XAH_CTRL_1_Reg();
    final FTN_CTRL_Reg FTN_CTRL_reg = new FTN_CTRL_Reg();
    final PLL_CF_Reg PLL_CF_reg = new PLL_CF_Reg();
    final PLL_DCU_Reg PLL_DCU_reg = new PLL_DCU_Reg();
    final PART_NUM_Reg PART_NUM_reg = new PART_NUM_Reg();
    final VERSION_NUM_Reg VERSION_NUM_reg = new VERSION_NUM_Reg();
    final MAN_ID_0_Reg MAN_ID_0_reg = new MAN_ID_0_Reg();
    final MAN_ID_1_Reg MAN_ID_1_reg = new MAN_ID_1_Reg();
    final SHORT_ADDR_0_Reg SHORT_ADDR_0_reg = new SHORT_ADDR_0_Reg();
    final SHORT_ADDR_1_Reg SHORT_ADDR_1_reg = new SHORT_ADDR_1_Reg();
    final PAN_ID_0_Reg PAN_ID_0_reg = new PAN_ID_0_Reg();
    final PAN_ID_1_Reg PAN_ID_1_reg = new PAN_ID_1_Reg();
    final IEEE_ADDR_0_Reg IEEE_ADDR_0_reg = new IEEE_ADDR_0_Reg();
    final IEEE_ADDR_1_Reg IEEE_ADDR_1_reg = new IEEE_ADDR_1_Reg();
    final IEEE_ADDR_2_Reg IEEE_ADDR_2_reg = new IEEE_ADDR_2_Reg();
    final IEEE_ADDR_3_Reg IEEE_ADDR_3_reg = new IEEE_ADDR_3_Reg();
    final IEEE_ADDR_4_Reg IEEE_ADDR_4_reg = new IEEE_ADDR_4_Reg();
    final IEEE_ADDR_5_Reg IEEE_ADDR_5_reg = new IEEE_ADDR_5_Reg();
    final IEEE_ADDR_6_Reg IEEE_ADDR_6_reg = new IEEE_ADDR_6_Reg();
    final IEEE_ADDR_7_Reg IEEE_ADDR_7_reg = new IEEE_ADDR_7_Reg();
    final XAH_CTRL_0_Reg XAH_CTRL_0_reg = new XAH_CTRL_0_Reg();
    final CSMA_SEED_0_Reg CSMA_SEED_0_reg = new CSMA_SEED_0_Reg();
    final CSMA_SEED_1_Reg CSMA_SEED_1_reg = new CSMA_SEED_1_Reg();
    final CSMA_BE_Reg CSMA_BA_reg = new CSMA_BE_Reg();

    final RWRegister[] regMap = {
        null, // 0x00
        TRX_STATUS_reg,
        TRX_STATE_reg,
        TRX_CTRL_0_reg,
        TRX_CTRL_1_reg,
        PHY_TX_PWR_reg,
        PHY_RSSI_reg,
        PHY_ED_LEVEL_reg,
        PHY_CC_CCA_reg,
        CCA_THRES_reg,
        RX_CTRL_reg,
        SFD_VALUE_reg,
        TRX_CTRL_2_reg,
        ANT_DIV_reg,
        IRQ_MASK_reg,
        IRQ_STATUS_reg,
        VREG_CTRL_reg, // 0x10
        BATMON_reg,
        XOSC_CTRL_reg,
        null,
        null,
        RX_SYN_reg,
        null,
        XAH_CTRL_1_reg,
        FTN_CTRL_reg,
        null,
        PLL_CF_reg,
        PLL_DCU_reg,
        PART_NUM_reg,
        VERSION_NUM_reg,
        MAN_ID_0_reg,
        MAN_ID_1_reg,
        SHORT_ADDR_0_reg, // 0x20
        SHORT_ADDR_1_reg,
        PAN_ID_0_reg,
        PAN_ID_1_reg,
        IEEE_ADDR_0_reg,
        IEEE_ADDR_1_reg,
        IEEE_ADDR_2_reg,
        IEEE_ADDR_3_reg,
        IEEE_ADDR_4_reg,
        IEEE_ADDR_5_reg,
        IEEE_ADDR_6_reg,
        IEEE_ADDR_7_reg,
        XAH_CTRL_0_reg,
        CSMA_SEED_0_reg,
        CSMA_SEED_1_reg,
        CSMA_BA_reg,
        null, // 0x30
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
        null,
    };

    //-- Other constants --------------------------------------------------
    private static final int NUM_REGISTERS = 0x3F;
    private static final int FIFO_SIZE     = 128;
    private static final int XOSC_START_TIME = 1000;// oscillator start time

    //-- Simulation objects -----------------------------------------------
    protected final AtmelInterpreter interpreter;
    protected final Microcontroller mcu;
    protected final Simulator sim;

    //-- Radio state ------------------------------------------------------
    protected final int xfreq;
    protected final ByteFIFO frameBuffer = new ByteFIFO(FIFO_SIZE);
    protected double BERtotal = 0.0D;
    protected int BERcount = 0;
    protected boolean txactive = false,rxactive = false;
    // Indicates that an ACK needs to be sent as reply on latest received Frame
    protected boolean sendAck;
    // Indicates that we are waiting for an incoming ACK for our latest Frame sent
    protected boolean waitingForAck;
//    protected boolean handledAck;
    protected int frame_rctr, csma_rctr;
    // The sequence number of the last received packet (required for ACK generation)
    protected byte lastRxSeqNo;
    // The sequence number of the last transmitted packet (required for ACK validation)
    protected byte lastTxSeqNo;
    protected Random random = new Random();

    protected Medium medium;
    protected Transmitter transmitter;
    protected Receiver receiver;

    //-- Pins ------------------------------------------------------------
    public final RF231Pin SCLK_pin   = new RF231Pin("SCLK");
    public final RF231Pin MISO_pin   = new RF231Pin("MISO");
    public final RF231Pin MOSI_pin   = new RF231Pin("MOSI");
    public final RF231Pin CS_pin     = new RF231Pin("SELN");
    public final RF231Pin SLPTR_pin  = new RF231Pin("SLP_TR");
    public final RF231Pin RSTN_pin   = new RF231Pin("RSTN");
    public final RF231Output IRQ_pin = new RF231Output("IRQ", new BooleanRegister());

    public final SPIInterface spiInterface = new SPIInterface();

    public int RF231_interrupt = -1;  //platform sets this to the correct interrupt number

    protected final SimPrinter printer;

    // RF231 energy
    protected final FiniteStateMachine energyStateMachine;

    /**
     * The constructor for the AT86RF231Radio class creates a new instance of an
     * AT86RF231 radio connected to a microcontroller via SPI.
     * to the specified microcontroller with the given clock frequency.
     *
     * @param mcu   the microcontroller unit
     * @param xfreq the clock frequency of this microcontroller
     */
    public AT86RF231Radio(Microcontroller mcu, int xfreq) {
        // set up references to MCU and simulator
        this.mcu = mcu;
        this.sim = mcu.getSimulator();
        this.interpreter = (AtmelInterpreter)sim.getInterpreter();

        this.xfreq = xfreq;

        // create a private medium for this radio
        // the simulation may replace this later with a new one.
        setMedium(createMedium(null, null));

        //setup energy recording
        energyStateMachine = new FiniteStateMachine(
                sim.getClock(), 
                AT86RF231Energy.startMode, 
                AT86RF231Energy.allModeNames(), 
                FiniteStateMachine.buildSparseTTM(AT86RF231Energy.allModeNames().length, 0));
        new Energy("Radio", AT86RF231Energy.modeAmpere, energyStateMachine, sim.getEnergyControl());

        // get debugging channel.
        printer = sim.getPrinter("radio.rf231");

        // set all registers to reset values, and clear FIFO
        resetDevice();
        

        // register state debug printout
        if (DEBUG && printer != null) {
            TRX_STATUS_reg._trx_status.registerValueSetListener(new RegisterView.RegisterValueSetListener() {

                @Override
                public void onValueSet(RegisterView view, int oldValue, int newValue) {
                    printer.println("RF231 -> " + getStateName(newValue));
                }
            });
            
            TRX_STATE_reg._trx_cmd.registerValueSetListener(new RegisterView.RegisterValueSetListener() {

                @Override
                public void onValueSet(RegisterView view, int oldValue, int newValue) {
                    printer.println("RF231 cmd: " + getCMDName(newValue));
                }
            });
        }
    }

    /**
     * Returns string representation of TRX_STATUS state.
     *
     * @param state state value
     * @return String representation
     */
    private String getStateName(int state) {
        switch (state) {
            case STATE_P_ON:
                return "P_ON";
            case STATE_BUSY_RX:
                return "BUSY_RX";
            case STATE_BUSY_TX:
                return "BUSY_TX";
            case STATE_RX_ON:
                return "RX_ON";
            case STATE_TRX_OFF:
                return "TRX_OFF";
            case STATE_PLL_ON:
                return "PLL_ON";
            case STATE_SLEEP:
                return "SLEEP";
            case STATE_BUSY_RX_AACK:
                return "BUSY_RX_AACK";
            case STATE_BUSY_TX_ARET:
                return "BUSY_TX_ARET";
            case STATE_RX_AACK_ON:
                return "RX_AACK_ON";
            case STATE_TX_ARET_ON:
                return "TX_ARET_ON";
            case STATE_RX_ON_NOCLK:
                return "RX_ON_NOCLK";
            case STATE_RX_AACK_ON_NOCLK:
                return "RX_AACK_ON_NOCLK";
            case STATE_BUSY_RX_AACK_NOCLK:
                return "BUSY_RX_AACK_NOCLK";
            case STATE_TRANSITION:
                return "TRANSITION";
            default:
                return "<invalid>";
        }
    }

    /**
     * Return String representation of TRX_CMD
     * 
     * @param cmd
     * @return 
     */
    private String getCMDName(int cmd) {
        switch (cmd) {
            case CMD_NOP:
                return "NOP";
            case CMD_TX_START:
                return "TX_START";
            case CMD_FORCE_TRX_OFF:
                return "FORCE_TRX_OFF";
            case CMD_FORCE_PLL_ON:
                return "FORCE_PLL_ON";
            case CMD_RX_ON:
                return "RX_ON";
            case CMD_TRX_OFF:
                return "TRX_OFF";
            case CMD_PLL_ON:
                return "PLL_ON";
            case CMD_RX_AACK_ON:
                return "RX_AACK_ON";
            case CMD_TX_ARET_ON:
                return "TX_ARET_ON";
            default:
                return "<invalid>";
        }
    }

    
    /**
     * The <code>getFiniteStateMachine()</code> method gets a reference to the finite state
     * machine that represents this radio's state. For example, there are states corresponding
     * to "on", "off", "transmitting", and "receiving". The state names and numbers will vary
     * by radio implementation. The <code>FiniteStateMachine</code> instance allows the user
     * to instrument the state transitions in order to gather information during simulation.
     * @return a reference to the finite state machine for this radio
     */
    public FiniteStateMachine getFiniteStateMachine() {
        return energyStateMachine;
    }

    /**
     * Resets the device.
     * 
     * This resets registers to their default value, clears FIFO, etc.
     */
    public void resetDevice() {
        if (DEBUG && printer!= null) printer.println("RF231: RESET");
        for (int cntr = 0; cntr < NUM_REGISTERS; cntr++) {
           resetRegister(cntr);
        }
        frameBuffer.clear();
        TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
        printer.println("RF231: STATE_TRX_OFF");
        txactive = rxactive = true;
        transmitter.shutdown();
        receiver.shutdown();
    }
    
    public void printRegisters() {
        for (int cntr = 0; cntr < NUM_REGISTERS; cntr++) {
           if (cntr % 16 == 0) {
               System.out.println();
           }
           if (regMap[cntr] == null) {
               System.out.print("null ");
           } else {
               System.out.print(String.format("0x%02x ", regMap[cntr].read()));
           }
        }
    }

    protected class BackOffDelayEvent implements Simulator.Event {

        private int BE;

        public void init() {
            // init BE to minBE
            BE = CSMA_BA_reg._min_be.getValue();
        }

        @Override
        public void fire() {
            // start CCA, delayed about 140µs
            TRX_STATUS_reg._cca_done.setValue(false);
            receiver.clock.insertEvent(ccaDelayEvent, toCycles(CCADelayEvent.DELAY_US));
        }

        /** 
         * Backoff is random(2^BE - 1) where BE is initalized to BEmin 
         * and increased up to BEmax.
         * 
         * BE is initialized by <code>init()</code>
         * @return Number of backoff unit periods
         */
        private int calcNextBackoff() {
            // calc backoff
            int backoff;
            if (CSMA_BA_reg._min_be.getValue() == 0 && CSMA_BA_reg._max_be.getValue() == 0) {
                // in this case backoff is always 0
                backoff = 0;
            } else {
                // random(2^BE - 1)
                backoff = (int) Math.round(((1 << BE) - 1) * random.nextDouble());
            }
            // increase BE up to maxBE
            BE = Math.min(BE + 1, CSMA_BA_reg._max_be.getValue());

            return backoff;
        }
    }
    protected BackOffDelayEvent backOffDelay = new BackOffDelayEvent();


    // ccaDelay fires after the cca or energy detect delay.
    protected class CCADelayEvent implements Simulator.Event {
        
        public static final int DELAY_US = 140;
        
        @Override
        public void fire() {
            //Construct PHY_ED_LEVEL from PHY_RSSI
            PHY_ED_LEVEL_reg.setValue(PHY_RSSI_reg._rssi.getValue() * 3);

            boolean ccaBusy = (PHY_RSSI_reg._rssi.getValue()) > (CCA_THRES_reg._cca_ed_thres.getValue() << 1);
            // update CCA_DONE and CCA_STATUS
            TRX_STATUS_reg._cca_status.setValue(ccaBusy ? 0 : 1);
            TRX_STATUS_reg._cca_done.setValue(true);
 
            //TODO: Carrier sense
            switch (PHY_CC_CCA_reg._cca_mode.getValue()) {
                case 0: //Carrier sense OR energy above threshold
                case 1: //Energy above threshold
                case 2: //Carrier sense only
                case 3: //Carrier sense AND energy above threshold
                break;
            }
            
            // CCA Result
            if (ccaBusy) {
                // if csma counter exceeds max retry value, abort
                if (csma_rctr > XAH_CTRL_0_reg._max_csma_retries.getValue()) {
                    transmitter.endTransmit(); //this seems to be necessary...
                    //Set TRAC_STATUS to no CHANNEL_ACCESS_FAILURE, issue TRX_END IRD and return to TX_ARET_ON
                    TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_CHANNEL_ACCESS_FAILURE);
                    postInterrupt(INT_TRX_END);
                    TRX_STATUS_reg._trx_status.setValue(STATE_TX_ARET_ON);

                    if (DEBUGC & printer != null) printer.println("RF231: TRX_END interrupt, csma failure");
                } else {
                    // Wait for a random number of 20 symbol periods, between 0 and 2**backoff-1
                    // XXX TODO: slotted transmissions start the wait at the next slot time
                    receiver.clock.insertEvent(backOffDelay, backOffDelay.calcNextBackoff() * 10 * receiver.cyclesPerByte);
                }
                return;
            }

            // Start Frame transmit
            receiver.shutdown();
            transmitter.startup();
            frame_rctr += 1;

            // XXX only for manual initiated?
            // postInterrupt(INT_CCA_ED_DONE);
        }
    }
    protected CCADelayEvent ccaDelayEvent = new CCADelayEvent();

    /**
     * The <code>readRegister()</code> method reads the value from the specified register
     * and takes any action(s) that are necessary for the specific register.
     *
     * @param addr the address of the register
     * @return an integer value representing the result of reading the register
     */
    public byte readRegister(int addr) {

        if (regMap[addr] == null) {
            printer.println("Invalid read access to addr " + addr);
            return 0x00;
        }

        byte val = regMap[addr].read();

        // custom handling for specific side effects
        switch (addr) {
            case IRQ_STATUS:
                // reading IRQ_STATUS clears all interrupts
                regMap[addr].setValue(0);
                break;
            case PHY_RSSI:
                // TODO: Add random bits 5 and 6
                break;
        }

        return val;
    }

    /**
     * The <code>writeRegister()</code> method writes the specified value to the specified
     * register, taking any action(s) necessary and activating any command strobes as
     * required.
     *
     * @param addr the address of the register
     * @param val  the value to write to the specified register
     */
    void writeRegister(int addr, byte val) {
        
        if (regMap[addr] == null) {
            printer.println("Invalid write access to addr " + addr);
            return;
        }
        
        if (DEBUGV && printer!=null) printer.println("RF231 " + regName(addr) + " <= " + StringUtil.to0xHex(val, 2));
        
        regMap[addr].setValue(val);

        switch (addr) {
            // XXX replace by listener for _trx_cmd?
            case TRX_STATE:
                trxStatusUpdate(TRX_STATE_reg._trx_cmd.getValue());
                break;
            case CSMA_SEED_0:
            case CSMA_SEED_1:
                // update random seed
                random.setSeed(CSMA_SEED_1_reg._csma_seed_1.getValue() << 8 + CSMA_SEED_0_reg.getValue());
                break;
            //In basic operation a CCA is triggered by a write to the top bit of PHY_CC_CCA
            //In extended operation it is triggered by a write to the PHY_ED_LEVEL register.
            //See data sheet for the various possibilities. In extended mode rx done is set immediately with no CCA.
            //In BUSY_RX carrier sense only the result is available immediately.
            case PHY_ED_LEVEL:
                val = (byte) 0x80; //fall through with the upper bit set?
            case PHY_CC_CCA:
                //upper bit is cca request, is cleared automatically
                 if ((val & 0x80) != 0) {
                    if (TRX_STATUS_reg._trx_status.getValue() == STATE_RX_AACK_ON) {  //or BUSY?
                        byte mode = (byte) ((val & 0x60) >> 5);
                        if (PHY_RSSI_reg._rssi.getValue() != 0) {
                            if (DEBUGC & printer!=null) printer.println("RF231: CCA_ED_REQUEST, mode = " + mode + " PHYRSSI = " +(PHY_RSSI_reg._rssi.getValue()) + " CCATHRES " + CCA_THRES_reg._cca_ed_thres.getValue());
                        }
                        boolean tbusy = (PHY_RSSI_reg._rssi.getValue()) >= (CCA_THRES_reg._cca_ed_thres.getValue() << 1);
                        //TODO: Carrier sense
                        switch (mode) {
                            case 0: //Carrier sense OR energy above threshold
                            case 1: //Energy above threshold
                            case 2: //Carrier sense only
                            case 3: //Carrier sense AND energy above threshold
                            break;
                        }
                        //clear status and done bit
                        TRX_STATUS_reg._cca_done.setValue(true);
                        TRX_STATUS_reg._cca_status.setValue(0);
                        
                        if (!tbusy) TRX_STATUS_reg._cca_status.setValue(1);
                        postInterrupt(INT_CCA_ED_DONE);
                        break;
                    }
                    if (DEBUGC & printer!=null) printer.println("RF231: CCA_REQUEST, mode = " + " PHYRSSI = " +PHY_RSSI_reg._rssi.getValue() + " CCATHRESH " + CCA_THRES_reg._cca_ed_thres.getValue());
                    regMap[addr].setValue((byte) (val & 0x7f)); // XXX ?
                    //clear status and done bit
                    TRX_STATUS_reg._cca_done.setValue(false);
                    TRX_STATUS_reg._cca_status.setValue(0);

                    //wait 140 usec (8.75 symbol periods)
                    receiver.clock.insertEvent(ccaDelayEvent, 875*receiver.cyclesPerByte/200);
                }
                break;
        }
    }

    /**
     * The <code>newCommand()</code> method alters the radio state according to
     * the rules for possible state transitions.
     *
     * @param byte the command
     */
    void trxStatusUpdate(int cmd) {
        
        int state = TRX_STATUS_reg._trx_status.getValue();

//System.out.println("*** trxStatusUpdate(" + getCMDName(cmd) + ") in state: " + getStateName(state));
        
        /* Handle forst transactions first */
        if (cmd == CMD_FORCE_TRX_OFF) {
            // Forces to TRX_OFF except when in SLEEP state
            if (state != STATE_SLEEP) {
                transmitter.shutdown();
                receiver.shutdown();
                TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
            }
            return;
        }
        
        if (cmd == CMD_FORCE_PLL_ON) {
            if ((state != STATE_SLEEP)
                    && (state != STATE_P_ON)
                    && (state != STATE_TRX_OFF)
                    && (state != STATE_BUSY_RX_AACK_NOCLK)
                    && (state != STATE_RX_AACK_ON_NOCLK)
                    && (state != STATE_RX_ON_NOCLK)) {
                TRX_STATUS_reg._trx_status.setValue(STATE_PLL_ON);
                // XXX what to perform here?
            }
            return;
        }
        
        /* Handle default transitions */
        switch (state) {
            case STATE_P_ON:
                if (cmd == CMD_TRX_OFF) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
                    postInterrupt(INT_AWAKE_END);
                    // XXX
                } else {
                    System.out.println("Rejecting invalid transition");
                }
                break;

            case STATE_TRX_OFF:
                if (cmd == CMD_PLL_ON) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRANSITION);
                    energyStateMachine.transition(AT86RF231Energy.PLL_ON);// XXX handle for all transitions?
                    /* Transition takes 110µs (typ.). See Table 7-1 */
                    transmitter.clock.insertEvent(new Simulator.Event() {
                        @Override
                        public void fire() {
                            TRX_STATUS_reg._trx_status.setValue(STATE_PLL_ON);
                        }
                    }, toCycles(110));// XXX replace by constant?
                    // XXX ?
                } else if (cmd == CMD_RX_ON) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRANSITION);
                    /* Transition takes 110µs (typ.). See Table 7-1 */
                    transmitter.clock.insertEvent(new Simulator.Event() {
                        @Override
                        public void fire() {
                            TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON);
                        }
                    }, toCycles(110));
                    // XXX receiver on?
                } else if (cmd == CMD_RX_AACK_ON) {
                    /* A state change request from TRX_OFF to RX_AACK_ON or TX_ARET_ON
                        internally passes the state PLL_ON to initiate the radio transceiver. */
                    TRX_STATE_reg._trx_cmd.setValue(CMD_PLL_ON);
                    trxStatusUpdate(TRX_STATE_reg._trx_cmd.getValue());
                    transmitter.clock.insertEvent(new Simulator.Event() {
                        @Override
                        public void fire() {
                            TRX_STATE_reg._trx_cmd.setValue(CMD_RX_AACK_ON);
                            trxStatusUpdate(TRX_STATE_reg._trx_cmd.getValue());
                        }
                    }, toCycles(110) + 1);// XXX replace by constant?
                    // XXX
                } else if (cmd == CMD_TX_ARET_ON) {
                    /* A state change request from TRX_OFF to RX_AACK_ON or TX_ARET_ON
                        internally passes the state PLL_ON to initiate the radio transceiver. */
                    TRX_STATE_reg._trx_cmd.setValue(CMD_PLL_ON);
                    trxStatusUpdate(TRX_STATE_reg._trx_cmd.getValue()); // XXX hack to trigger status updates, ok?
                    transmitter.clock.insertEvent(new Simulator.Event() {
                        @Override
                        public void fire() {
                            TRX_STATE_reg._trx_cmd.setValue(CMD_TX_ARET_ON);
                            trxStatusUpdate(TRX_STATE_reg._trx_cmd.getValue());
                        }
                    }, toCycles(110) + 1);// XXX replace by constant?
                    // XXX
                } else {
                    // invalid transition
                }
                break;

            case STATE_PLL_ON:
                if (cmd == CMD_RX_ON) {
                    transmitter.shutdown(); 
                    receiver.startup();
                    TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON);
                    // XXX
                } else if (cmd == CMD_RX_AACK_ON) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_RX_AACK_ON);
                    /* TRAC_STATUS is initialized to INVALID */
                    TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_INVALID);
                    waitingForAck = false;
                    transmitter.shutdown();
                    receiver.startup();
                    // XXX
                } else if (cmd == CMD_TX_ARET_ON) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_TX_ARET_ON);
                    frame_rctr = 0;
                    // XXX
                } else if (cmd == CMD_TRX_OFF) {
                    transmitter.shutdown();
                    receiver.shutdown();
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
                    // XXX
                } else if (cmd == CMD_TX_START) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_BUSY_TX);
                    // XXX
                } else {
                    // invalid transition
                }
                break;

            case STATE_RX_AACK_ON:
                if (cmd == CMD_TRX_OFF) {
                    transmitter.shutdown();
                    receiver.shutdown();
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
                    // XXX
                } else if (cmd == CMD_PLL_ON) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_PLL_ON);
                    // XXX
                } else {
                    // invalid transition
                }
                break;

            case STATE_TX_ARET_ON:
                if (cmd == CMD_PLL_ON) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_PLL_ON);
                    // XXX
                } else if (cmd == CMD_TRX_OFF) {
                    transmitter.shutdown();
                    receiver.shutdown();
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
                    // XXX
                } else if (cmd == CMD_TX_START) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_BUSY_TX_ARET);
                    /* TRAC_STATUS is initialized to INVALID */
                    TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_INVALID);
                    triggerARETTransmission();
                    // XXX
                } else {
                    // invalid transition
                }
                break;

            default:
                if (DEBUG && printer != null) {
                    printer.println("No Transition from State " + getStateName(state));
                }
                break;
        }
    }

    void triggerARETTransmission() {
        if (XAH_CTRL_0_reg._max_csma_retries.getValue() < 7) {
            csma_rctr = 0;
            // XXX random back-off csma_rctr = csma_rctr + 1 CCA
            //TODO:should csma should wait for rising edge of slptr?
            receiver.startup();
            // Init and wait random back-off time
            backOffDelay.init();
            receiver.clock.insertEvent(backOffDelay, backOffDelay.calcNextBackoff() * 10 * receiver.cyclesPerByte);
        } else {
            //A value of 7 initiates an immediate transmission with no CSMA
            receiver.shutdown();
            transmitter.startup();
            frame_rctr += 1;
            // XXX ?
        }
    }

    /**
     * The <code>resetRegister()</code> method resets the specified register's value
     * to its default.
     *
     * @param addr the address of the register to reset
     */
    void resetRegister(int addr) {

        if (DEBUG && printer!=null) printer.println("RF231: Reset registers");
        byte val = 0x00;
        switch (addr) {
            case TRX_CTRL_0:
                val = (byte) 0x19;
                break;
            case TRX_CTRL_1:
                val = (byte) 0x20;//rf230 sets to zero
                break;
            case PHY_TX_PWR:
                val = (byte) 0xC0;//rf230 sets to zero
                break;
            case PHY_ED_LEVEL:
                val = (byte) 0xFF;//rf230 sets to zero
                break;
            case PHY_CC_CCA:
                val = (byte) 0x2B;
                break;
            case CCA_THRES:
                val = (byte) 0xC7;
                break;
            case RX_CTRL:
                val = (byte) 0xB7;//rf230 sets to 0xBC
                break;
            case SFD_VALUE:
                if (MSPSIM_SFD_COMPAT) {
                    val = (byte) 0x7A;
                } else {
                    val = (byte) 0xA7;
                }
                break;
            case TRX_CTRL_2:
                val = (byte) 0x00;//rf230 sets to 0x04
                break;
            case ANT_DIV:
                val = (byte) 0x03;//rf230 sets to zero
                break;
            case IRQ_MASK:
                val = (byte) 0x00;//NB: rf230 sets to 0xFF
                break;
            case VREG_CTRL:
                //val = (byte) 0x00;//reads as 4 when access is possible
                break;
            case BATMON:
                val = (byte) 0x02;//reads as 0x22 when access is possible
                break;
            case XOSC_CTRL:
                val = (byte) 0xF0;
                break;
            case 0x18:
                val = (byte) 0x58;
                break;
            case 0x19:
                val = (byte) 0x55;
                break;
            case PLL_CF:
                val = (byte) 0x57;//rf230 sets to 0x5F
                break;
            case PLL_DCU:
                val = (byte) 0x20;
                break;
            case PART_NUM:
                val = (byte) 0x03;//rf230 sets of 0x02
                break;
            case VERSION_NUM:
                val = (byte) 0x02;
                break;
            case MAN_ID_0:
                val = (byte) 0x1F;
                break;
            case SHORT_ADDR_0:
            case SHORT_ADDR_1:
            case PAN_ID_0:
            case PAN_ID_1:
                val = (byte) 0xFF;//rf230 sets to zeros
                break;
            case XAH_CTRL_0:
                val = (byte) 0x38;
                break;
            case CSMA_SEED_0:
                val = (byte) 0xEA;
                break;
            case CSMA_SEED_1:
                val = (byte) 0x42;//rf230 sets to 0xC2
                break;
            case CSMA_BE:
                val = (byte) 0x53;//rf230 sets to zero
                break;
            case 0x39:
               val = (byte) 0x40;
               break;
        }
        
        if (regMap[addr] != null) {
            regMap[addr].write(val);
        }
    }

     //The same bit numbering is used in the IRQ_MASK and IRQ_STATUS registers
    protected static final byte INT_PLL_LOCK    = 0;
    protected static final byte INT_PLL_UNLOCK  = 1;
    protected static final byte INT_RX_START    = 2;
    protected static final byte INT_TRX_END     = 3;
    protected static final byte INT_CCA_ED_DONE = 4;
    protected static final byte INT_AWAKE_END   = 4; // alias
    protected static final byte INT_AMI         = 5;
    protected static final byte INT_TRX_UR      = 6;
    protected static final byte INT_BAT_LOW     = 7;
    
    /**
     * The <code>postInterrupt()</code> method posts the single RF231 interrupt if the cause
     * is enabled in the IRQ_MASK, or if interrupt polling is enabled.
     *
     * @param bitNum A byte with bit set corresponding to the cause
     */
    protected void postInterrupt(int bitNum) {
        if (DEBUG & printer != null) {
            printer.println("RF231 IRQ!: " + getInterruptName(bitNum));
        }

        // is interrupt enabled in mask or polling enabled?
        if (IRQ_MASK_reg.readBit(bitNum) || (TRX_CTRL_1_reg._irq_mask_mode.getValue() == 1)) {
            // update status register and post interrupt to controller
            IRQ_STATUS_reg.writeBit(bitNum, true);
            // PLL_LOCK auto-clears PLL_UNLOCK and vice versa
            if (bitNum == IRQ_STATUS_Reg.PLL_LOCK) {
                IRQ_STATUS_reg._pll_unlock.setValue(0);
            } else if (bitNum == IRQ_STATUS_Reg.PLL_UNLOCK) {
                IRQ_STATUS_reg._pll_lock.setValue(0);
            }
        }
      
        // interrupt is posted only if enabled in mask and connected
        if (IRQ_MASK_reg.readBit(bitNum) && (RF231_interrupt > 0)) {
            interpreter.setPosted(RF231_interrupt, true);
            if (DEBUGV && printer != null) printer.println("RF231: interrupt posted");
        }
    }
    
    private String getInterruptName(int intNum) {
        switch (intNum) {
            case INT_PLL_LOCK:
                return "PLL_LOCK";
            case INT_PLL_UNLOCK:
                return "PLL_UNLOCK";
            case INT_RX_START:
                return "RX_START";
            case INT_TRX_END:
                return "TRX_END";
            case INT_CCA_ED_DONE:
                return "CCA_ED_DONE";
            case INT_AMI:
                return "AMI";
            case INT_TRX_UR:
                return "TRX_UR";
            case INT_BAT_LOW:
                return "BAT_LOW";
            default:
                return "<invalid>";
        }
    }

    /* ------------------------------------------SPI Transfers --------------------------------*/
    protected static final int CMD_R_REG = 0;
    protected static final int CMD_W_REG = 1;
    protected static final int CMD_R_BUF = 2;
    protected static final int CMD_W_BUF = 3;
    protected static final int CMD_R_RAM = 4;
    protected static final int CMD_W_RAM = 5;

    //-- state for managing configuration information
    protected int configCommand;
    protected int configByteCnt;
    protected int configRegAddr;
    protected byte configByteHigh;
    protected int configRAMAddr;
    protected int configRAMBank;

    protected byte receiveConfigByte(byte val) {
        configByteCnt++;
        if (configByteCnt == 1) {
          // the first byte is the command byte
          //  byte status = getStatus();
            if (Arithmetic.getBit(val, 7)) {
                // register bit is set, bit2 determines write or read
                configCommand = Arithmetic.getBit(val,6) ? CMD_W_REG : CMD_R_REG;
                configRegAddr = val & 0x3f;
            }
            else if (Arithmetic.getBit(val,5)) {
                // frame buffer access
                configCommand = Arithmetic.getBit(val,6) ? CMD_W_BUF : CMD_R_BUF;
            } else {
                // SRAM access
                configCommand = Arithmetic.getBit(val,6) ? CMD_W_RAM : CMD_R_RAM;
            }
            return 0;
        } else if (configByteCnt == 2) {
       //    if (!oscStable.getValue() && configCommand != CMD_R_REG && configCommand != CMD_W_REG) {
                // with crystal oscillator disabled only register access is possible
          //      return 0;
         //   }
            // the second byte is the value to read or write, or the starting SRAM address
            switch (configCommand) {
                case CMD_R_REG:
                    return readRegister(configRegAddr);
                case CMD_W_REG:
                    writeRegister(configRegAddr, val);
                    return 0;
                case CMD_R_BUF:
                    //TODO:diagnostics if FIFO is empty
                    byte length = readFIFO(frameBuffer);
                    if (DEBUGRX && printer!=null) {
                        printer.println("RF231: Sending rx length of " + length);
                    }
                    return length;
                case CMD_W_BUF:
                    //The first write clears the FIFO
                    frameBuffer.clear();
                    return writeFIFO(frameBuffer, val, true);
                case CMD_R_RAM:
                case CMD_W_RAM:
                    configRAMAddr = val & 0x7F;
                    return 0;
            }
        } else {
         //   if (!oscStable.getValue() && configCommand != CMD_R_REG && configCommand != CMD_W_REG) {
                // with crystal oscillator disabled only register access is possible
           //     return 0;
          //  }
            // subsequent bytes are valid for fifo and RAM accesses
            switch (configCommand) {
                case CMD_R_BUF:
                    return readFIFO(frameBuffer);
                case CMD_W_BUF:
                    return writeFIFO(frameBuffer, val, true);
                case CMD_R_RAM:
                    return frameBuffer.getAbsoluteByte(configRAMAddr++);
                case CMD_W_RAM:
                     return frameBuffer.setAbsoluteByte(configRAMAddr++, val);
            }
        }
        return 0;
    }

    protected byte readFIFO(ByteFIFO fifo) {
        byte val = fifo.remove();
        if (DEBUGV && printer!=null) printer.println("RF231 Read FIFO -> " + StringUtil.toMultirepString(val, 8));
        /*
        if (fifo == rxFIFO) {
            if (fifo.empty()) {
                // reset the FIFO pin when the read FIFO is empty.
                FIFO_pin.level.setValue(!FIFO_active);
            }
            if (fifo.size() < getFIFOThreshold()) {
                // reset FIFOP pin when the number of bytes in the FIFO is below threshold
                FIFOP_pin.level.setValue(!FIFOP_active);
            }
        }
        */
        return val;
    }

    protected byte writeFIFO(ByteFIFO fifo, byte val, boolean st) {
        if (DEBUGV && printer!=null) printer.println("rf231 Write FIFO <= " + StringUtil.toMultirepString(val, 8));
        fifo.add(val);
        return 0;
    }

    @Override
    public Simulator getSimulator() {
        return sim;
    }

    public double getPower() {
        //return power in dBm
        double power=0;
        switch (PHY_TX_PWR_reg._tx_pwr.getValue()) {
            case  0:power=  3.0;break;
            case  1:power=  2.8;break;
            case  2:power=  2.3;break;
            case  3:power=  1.8;break;
            case  4:power=  1.3;break;
            case  5:power=  0.7;break;
//          case  6:power=  0.0;break;
            case  7:power= -1.0;break;
            case  8:power= -2.0;break;
            case  9:power= -3.0;break;
            case 10:power= -4.0;break;
            case 11:power= -5.0;break;
            case 12:power= -7.0;break;
            case 13:power= -9.0;break;
            case 14:power=-12.0;break;
            case 15:power=-17.0;break;
        }
        if (DEBUGV && printer!=null) printer.println("RF231: getPower returns "+ power + " dBm");
        return power;
    }

    public int getChannel() {
        if (DEBUGV && printer!=null) printer.println("RF231: getChannel returns "+ PHY_CC_CCA_reg._channel.getValue());
//        return registers[PHY_CC_CCA] & 0x1F;
        return PHY_CC_CCA_reg._channel.getValue();
    }

    public double getFrequency() {
        double frequency = 2405 + 5*(getChannel() - 11 );
        if (DEBUGV && printer!=null) printer.println("RF231: getFrequency returns "+frequency+" MHz (channel " + getChannel() + ")");
        return frequency;
    }

    public class ClearChannelAssessor implements BooleanView {
        @Override
        public void setValue(boolean val) {
             if (DEBUGV && printer!=null) printer.println("RF231: set clearchannel");
             // ignore writes.
        }

        @Override
        public boolean getValue() {
          if (DEBUGV && printer!=null) printer.println("RF231: CleanChannelAssesor.getValue");
          return true;
        }

        @Override
        public void setValueSetListener(ValueSetListener listener) {
            if (printer != null) {
                // TODO: Implement value-change events for this view
                printer.println("AT86RF231 WARN: ValueSet events for the ClearChannelAssessor are not implemented.");
            }
        }
    }

    public class SPIInterface implements SPIDevice {

        @Override
        public SPI.Frame exchange(SPI.Frame frame) {
            if (DEBUGV && printer!=null) printer.println("RF231 new SPI frame exchange " + StringUtil.toMultirepString(frame.data, 8));
            if (!CS_pin.level && RSTN_pin.level) {
                // configuration requires CS pin to be held low and RSTN pin to be held high
                return SPI.newFrame(receiveConfigByte(frame.data));
            } else {
                return SPI.newFrame((byte) 0);
            }
        }

        @Override
        public void connect(SPIDevice d) {
            // do nothing.
        }
    }


    private void pinChange_CS(boolean level) {
        // a change in the CS level always restarts a config command.
        configByteCnt = 0;
    }

    //wakupDelay fires after transition from SLEEP or RESET to TRX_OFF. The nominal delay is 240 usec from SLEEP or
    //32 usec after RESET, but board capacitance can delay the oscillator startup to as much as 1000 usec.
    protected class wakeupDelay implements Simulator.Event {
        @Override
        public void fire() {
            TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
            postInterrupt(INT_AWAKE_END);
        }
    }
    protected wakeupDelay wakeupDelayEvent = new wakeupDelay();

    /** Handles pin change on Sleep/Wake-up and Transmit Signal pin (SLP_TR).
     * 
     * @param pinHigh 
     */
    private void pinChange_SLP_TR(boolean pinHigh) {
        
        // debug output of pin change
        if (DEBUG && printer != null) {
            if (pinHigh) {
                printer.println("RF231 SLP_TR pin raised");
            } else {
                printer.println("RF231 SLP_TR pin lowered");
            }
        }
        
//        if () {
//            
//        }
        
        
        switch (TRX_STATUS_reg._trx_status.getValue()) {
            case STATE_PLL_ON:
                if (pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_BUSY_TX);
                    // XXX
                }
                break;
            case STATE_RX_ON:
                if (pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON_NOCLK);
                    // XXX
                }
                break;
            case STATE_TRX_OFF:
                if (pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_SLEEP);
                    /* In SLEEP state, the entire radio transceiver is disabled. */
                    receiver.shutdown();
                    transmitter.shutdown();
                    /* During SLEEP the register contents remains valid
                       while the content of the Frame Buffer
                       and the security engine (AES) are cleared. */
                    frameBuffer.clear();
                    energyStateMachine.transition(AT86RF231Energy.SLEEP);
                    // XXX
                    /* If CLKM is enabled, the SLEEP state is entered 35 CLKM cycles
                    after the rising edge at pin 11 (SLP_TR).
                    At that time CLKM is turned off.
                    If the CLKM output is already turned off (bits CLKM_CTRL = 0 in register 0x03),
                    the SLEEP state is entered immediately.
                    At clock rates 250 kHz and 62.5 kHz, 
                    the main clock at pin 17 (CLKM) is turned off immediately. */
                    /* 'When the radio transceiver is in TRX_OFF state
                    the microcontroller forces the AT86RF231 to SLEEP by setting SLP_TR = H.
                    If pin 17 (CLKM) provides a clock to the microcontroller
                    this clock is switched off after 35 clock cycles. */
                }
                break;
            case STATE_SLEEP:
                if (!pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
                    energyStateMachine.transition(AT86RF231Energy.TRX_OFF);
                    
                    TRX_STATUS_reg._trx_status.setValue(STATE_TRANSITION);
                    // initialize random seed base on CSMA_SEED
                    random.setSeed(CSMA_SEED_1_reg._csma_seed_1.getValue() << 8 + CSMA_SEED_0_reg.getValue());
                    
                    /* After t = 380 μs (typ.) the radio transceiver enters TRX_OFF state. */
                    transmitter.clock.insertEvent(new Simulator.Event() {
                        @Override
                        public void fire() {
                            TRX_STATUS_reg._trx_status.setValue(STATE_TRX_OFF);
                            postInterrupt(INT_AWAKE_END);
                        }
                    }, toCycles(380));
                    // XXX
                    /* 'If the radio transceiver was in SLEEP state, 
                    the XOSC and DVREG are enabled before entering
                    TRX_OFF state. */

                }
                break;
            case STATE_TX_ARET_ON:
                if (pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_BUSY_TX_ARET);
                    /* TRAC_STATUS is initialized to INVALID */
                    TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_INVALID);
                    triggerARETTransmission();
                    // XXX
                }
                break;
            case STATE_RX_ON_NOCLK:
                if (!pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON);
                    // XXX
                }
                break;
            case STATE_RX_AACK_ON_NOCLK:
                if (!pinHigh) {
                    TRX_STATUS_reg._trx_status.setValue(STATE_RX_AACK_ON);
                    // XXX
                }
                break;
            default:
//                if (DEBUG && printer != null) {
//                    printer.println("RF231 Inavlid state for SLP_TR pin change (not handled)");
//                }
                break;
        }
    }

    /** Handles pin change on /reset pin.
     * 
     * @param pinHigh True if pin is changed to high
     */
    private void pinChange_RST(boolean pinHigh) {
                // debug output of pin change
        if (DEBUG && printer != null) {
            if (pinHigh) {
                printer.println("RF231 /RST pin raised");
            } else {
                printer.println("RF231 /RST pin lowered");
            }
        }

        // XXX should be low first, high then to reach TRX_OFF ...
        if (!pinHigh) {
            // high->low indicates reset
            resetDevice();
            energyStateMachine.transition(AT86RF231Energy.TRX_OFF);//change to power down state
            if (DEBUGE && printer!=null) printer.println("RF231 reset by RSTN pin");
        }
    }

    /* ------------------------------------------CRC Computation --------------------------------*/
    protected static final int[] reverse_bits = {
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8, 0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4, 0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec, 0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2, 0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea, 0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6, 0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee, 0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1, 0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9, 0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5, 0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed, 0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, 0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb, 0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7, 0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef, 0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff};
    //TODO: which is faster
/*
    int crcAccumulatet(int crc, byte val) {
	    crc = ((crc >> 8) & 0xff) | (crc << 8) & 0xffff;
	    crc ^= (val & 0xff);
	    crc ^= (crc & 0xff) >> 4;
	    crc ^= (crc << 12) & 0xffff;
	    crc ^= (crc & 0xff) << 5;
	    crc = crc & 0xffff;
        return crc;
    }
*/
    short crcAccumulate(short crc, byte val) {
        int i = 8;
        crc = (short) (crc ^ val << 8);
        do {
            if ((crc & 0x8000) != 0) crc = (short) (crc << 1 ^ 0x1021);
            else crc = (short) (crc << 1);
        } while (--i > 0);
        return crc;
    }
    
    //ackTimeOut fires 54 symbol periods (864 usec) after tx transition to rx to receive an ack
    protected class AAckTimeOutEvent implements Simulator.Event {

        @Override
        public void fire() {
            waitingForAck = false;
            if (DEBUG && printer != null) printer.println("RF2311: Timeout while waiting for AACK");

            if (frame_rctr > XAH_CTRL_0_reg._max_frame_retries.getValue()) {
                receiver.shutdown();

                // Set TRAC_STATUS to no ack and return to TX_ARET_ON
                TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_NO_ACK);
                postInterrupt(INT_TRX_END);
                TRX_STATUS_reg._trx_status.setValue(STATE_TX_ARET_ON);
                transmitter.state = Transmitter.TX_WAIT;//???
                return;
            }
                // try again ...
printer.println("Auto retry transmission: #" + frame_rctr);
            frameBuffer.refill();
            triggerARETTransmission();
        }
    }
    protected AAckTimeOutEvent ackTimeOutEvent = new AAckTimeOutEvent(); 
    
    /* ------------------------------------------Transmitter-------------------------------------*/
    public class Transmitter extends Medium.Transmitter {
        private static final int TX_IN_PREAMBLE = 0;
        private static final int TX_SFD = 1;
        private static final int TX_LENGTH = 3;
        private static final int TX_IN_PACKET = 4;
        private static final int TX_FCS_1 = 5;
        private static final int TX_FCS_2 = 6;
        private static final int TX_END = 7;
        private static final int TX_WAIT = 8;

        protected int state, trPacketCounter, length;
        protected short crc;
        // indicates that transmitted frame had ACKRequest bit set and we are waiting for an ACK
        protected boolean waitForAck;

        public Transmitter(Medium m) {
            super(m, sim.getClock());
        }

        @Override
        public byte nextByte() {
            if (rxactive) printer.println("rx active while transmitting");
            if (!txactive) printer.println("tx not active while transmitting");
            byte val = 0;
            switch (state) {
                case TX_IN_PREAMBLE:
                    //always 4 octects of zero
                    if (++trPacketCounter >= 4) state = TX_SFD;
                    break;
                case TX_SFD:
                    val = SFD_VALUE_reg.read();
                    state = TX_LENGTH;
                    break;
                case TX_LENGTH:
                    if (sendAck) {
                        // ACK frames are always of length 5
                        length = 5;
                    } else {//data frame
                        // Save current position in frame buffer to be able to restore it for retransmissions
                        frameBuffer.saveState();
                        // length is the first byte in the FIFO buffer
                        length = frameBuffer.remove(); // XXX remove ok here???
                    }
                    if (DEBUGTX && printer!=null) printer.println("RF231: Tx frame length " + length);
                    trPacketCounter = 0;
                    crc = 0;
                    val = (byte) length;
                    state = TX_IN_PACKET;
                    break;
                case TX_IN_PACKET:
                    if (sendAck) {
                        // ACK package assembly
                        switch (trPacketCounter) {
                            case 0://FCF_low
                                val = FCFDecoder.FRAME_TYPE_ACK;
                                // set pending flag if enabled
                                if (CSMA_SEED_1_reg._aack_set_pd.getValue()) {
                                    val |= 0x10;
                                }
                                break;
                            case 1://FCF_hi
                                val = 0;
                                waitForAck = false;
                                break;
                            case 2://Sequence number
                                val = lastRxSeqNo;
                                state = TX_END;
                                break;
                            default:
                                if (DEBUG && printer!=null) printer.println("RF231: Unhandled counter value " + trPacketCounter);
                                break;
                        }
                        trPacketCounter++;
                    } else {
                        val = frameBuffer.remove();
                        trPacketCounter++;
                        if (trPacketCounter == 1) {
                            //check the ack request bit in the FCF
                            waitForAck = (val & 0x20) != 0;
                        } else if (trPacketCounter == 3) {
                            // save sequence number to compare with those of possible received ACK
                            lastTxSeqNo = val;
                        }
                    }
                    
                    // Calculate CRC and switch state if necessary
                    if (TRX_CTRL_1_reg._tx_auto_crc_on.getValue()) {
                        crc = crcAccumulate(crc, (byte) reverse_bits[(val) & 0xff]);
                        if (trPacketCounter >= length - 2) {
                            // switch to CRC state if when 2 bytes remain.
                            state = TX_FCS_1;
                        }
                    } else if (trPacketCounter >= length) {
                        // AUTOCRC not enabled, switch to packet end mode when done.
                        state = TX_END;
                    }
                    break;
                    
                case TX_FCS_1:
                    val = Arithmetic.high(crc);
                    val = (byte) reverse_bits[(val) & 0xff];
                    state = TX_FCS_2;
                    break;
                    
                case TX_FCS_2:
                    val = Arithmetic.low(crc);
                    val = (byte) reverse_bits[(val) & 0xff];
                    state = TX_END;
                    break;
            }

            if (DEBUGTX && printer!=null) printer.println("RF231 " + StringUtil.to0xHex(val, 2) + " --------> ");
            // common handling of end of transmission
            if (state == TX_END) {
                if (DEBUG && printer!=null) printer.println("state @TX_END"
                        + ", lenght: " + length
                        + ", sendAck: " + sendAck
                        + ", waitForAck: " + waitForAck);
                if ((TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_RX_AACK) && sendAck) {
                    if (DEBUG && printer!=null) printer.println("****** Auto ack was sent and we change back to STATE_RX_AACK_ON now");
                    // if ack was just send, just stop transmitter, no interrupts etc.
                    sendAck = false;
                    transmitter.shutdown();
                    receiver.startup();
                    TRX_STATUS_reg._trx_status.setValue(STATE_RX_AACK_ON);
                    // XXX always write to register!?!
//                    if (DEBUG && printer!=null) printer.println("RF231: STATE_RX_AACK_ON");
                } else {
                  //Set the TRAC status bits in the TRX_STATE register
                  //0 success 1 pending 2 waitforack 3 accessfail 5 noack 7 invalid
                   // XXX handling for normal operating mode missing
                  if ((TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_TX_ARET) && waitForAck) {
                      //Show waiting for ack, and switch to rx mode
                      waitingForAck = true;
//                      handledAck = false;
//                      registers[TRX_STATE] = (byte) (0x40 | (registers[TRX_STATE] & 0x1F));
                      TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_SUCCESS_WAIT_FOR_ACK);
                      //wait 54 symbol periods (864 usec, 27 bytes) to receive the ack
                      clock.insertEvent(ackTimeOutEvent, 10*27*cyclesPerByte); // XXX testing
                      if (!txactive) printer.println("tx not active at txend");
                      transmitter.shutdown();
                      if (rxactive) printer.println("rx active at txend");
                      if (DEBUG && printer!=null) printer.println("RF231: receiver.startup()@2");
                      receiver.startup();
                      state = TX_WAIT;
                      return val;
                  } else {
                      // set TRAC_STATUS to SUCCESS
                      TRX_STATUS_reg._trx_status.setValue(STATE_TX_ARET_ON);
                      TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_SUCCESS);
                      transmitter.shutdown();
                  }
                  // XXX not for sendingAck
                  if (sendAck) printer.println("ooops, INT_TRX_END issued after sending Ack");
                  postInterrupt(INT_TRX_END);
                }
                // transmitter stays in this state until it is really switched off
                state = TX_WAIT;
            }
            
            return val;
        }

        void startup() {
            // do not enable if active already
            if (txactive) {
                printer.println("tx startup while active");
                return;
            }
            
            txactive = true;
            // the PLL lock time is implemented as the leadCycles in Medium
            // Map TX_PWR value to AT86RF231Energy state
            energyStateMachine.transition(AT86RF231Energy.TX_PWR_15 + 15 - (PHY_TX_PWR_reg._tx_pwr.getValue()));
            state = TX_IN_PREAMBLE;
            trPacketCounter = 0;
            beginTransmit(getPower(), getFrequency());
            if (DEBUGTX && printer!=null) printer.println("RF231: TX Startup");
        }

        void shutdown() {
            // do not disable if inactive already
            if (!txactive) {
                printer.println("tx shutdown while not active");
                return;
            }
            
            // note that the stateMachine.transition() is not called here any more!
            txactive = false;
            endTransmit();
                     //           trxFIFO.clear();
            if (DEBUGTX && printer!=null) printer.println("RF231: TX shutdown");
            energyStateMachine.transition(3);//change to RX TODO:FIX this
        }
    }

    /* ------------------------------------------Receiver------------------------------------*/

    //Address recognition variables
    protected byte[] PANId;
    protected byte[] macPANId = new byte[2];
    protected byte[] shortDestAddr, shortSrcAddr;
    protected byte[] macShortAddr = new byte[2];
    protected static final byte[] SHORT_BROADCAST_ADDR = {-1, -1};
    protected byte[] extDestAddr;
    protected byte[] aExtendedAddress = new byte[8];
    protected static final byte[] LONG_BROADCAST_ADDR = {-1, -1, -1, -1, -1, -1, -1, -1};

    public class Receiver extends Medium.Receiver {

        private static final int RECV_SFD_SCAN = 0;
        private static final int RECV_SHR_DETECTED = 2;
        private static final int RECV_IN_PACKET = 3;
        private static final int RECV_FCS_1 = 4;
        private static final int RECV_FCS_2 = 5;
        private static final int RECV_END_STATE = 6;
        private static final int RECV_OVERFLOW = 7;
        private static final int RECV_WAIT = 8;

        protected int rxstate;
        // counts number received of byte in packet
        protected int rxByteCounter;
        protected int packetLength;
        protected short crc;
        protected byte crcLow;
        protected FCFDecoder fcfDecoder;

        public Receiver(Medium m) {
            super(m, sim.getClock());
        }

        @Override
        public byte nextByte(boolean lock, byte b) {
            boolean invalidAck = false;

            //The receiver will continue to get some byte runon after the force TRX_OFF command
            if (!rxactive) {
                printer.println("rx not active while receiving " +  StringUtil.to0xHex(b, 2));
                return 0;
            }
            if (txactive) printer.println("txactive while receiving");

            if (rxstate == RECV_END_STATE) {
                if (DEBUGRX && printer!=null) printer.println("RF231 <==END=== " + StringUtil.to0xHex(b, 2));
                // XXX 
                rxstate = RECV_SFD_SCAN; // to prevent loops when calling shutdown/endReceive
                // packet ended before
                if (sendAck) { //send ack?
                    if (DEBUGRX && printer!=null) printer.println("RF231: sendack");
                    receiver.shutdown();
                    // Enable transmitter to send ACK after 2 or 12 symbols delay
                    clock.insertEvent(new Simulator.Event() {
                        @Override
                        public void fire() {
                            transmitter.startup();
                        }
                    }, (XAH_CTRL_1_reg._aack_ack_time.getValue() ? 1 : 6) * cyclesPerByte);
                } else {
                    if (lock) {
                        // the medium is still locked, so there could be more packets!
                        if (DEBUGE && printer!=null) printer.println("RF231: still locked");
                        // fire the probes manually
                        if (probeList != null) {
                            System.out.println("probeList is not null!");
                            probeList.fireAfterReceiveEnd(Receiver.this);
                        }
                    }
                }
                return b;
            }


            if (!lock) {
                if (DEBUGRX && printer!=null) printer.println("RF231 notlock, rxstate = " + rxstate);
                // the reception lock has been lost
                switch (rxstate) {
                    case RECV_SHR_DETECTED:
                    case RECV_IN_PACKET:
                    case RECV_FCS_1:
                    case RECV_FCS_2:
                        // Post PLL_IRQ to indicate an unexpected unlock condition
                        postInterrupt(INT_PLL_UNLOCK);
                        //packet lost in middle -> drop frame
                        // fall through
                    case RECV_SFD_SCAN:
                        if (DEBUGRX && printer != null) printer.println("RF231: lock lost");
                        rxstate = RECV_SFD_SCAN;
                        switch (TRX_STATUS_reg._trx_status.getValue()) {
                            case STATE_BUSY_RX_AACK:
                                TRX_STATUS_reg._trx_status.setValue(STATE_RX_AACK_ON);
                                // XXX ?
                                break;
                            case STATE_BUSY_RX:
                                TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON);
                                break;
                            case STATE_BUSY_TX_ARET:
                                //that's one ack were not going to get
                                break;
                            case STATE_RX_AACK_ON:
                                //we are getting jammed
                                break;
                            case STATE_RX_ON:
                                //we are getting jammed
                                break;
                            case STATE_PLL_ON:
                                //whats up with that
                                break;
                            default:
                                if (DEBUGE && printer != null) {
                                    printer.println("RF231: Bad state when lock lost " + TRX_STATUS_reg._trx_status.getValue());
                                }
                                break;
                        }
                        break;
                    default:
                        if (DEBUGE && printer!=null) printer.println("RF231: RX badstate " + rxstate);
                        rxstate = RECV_SFD_SCAN;
                        break;

                }
                return b;
            }

            if (DEBUGRX && printer != null) printer.println("RF231 <======== " + StringUtil.to0xHex(b, 2));

            /* Receiver state machine */
            switch (rxstate) {

                // scan for SFD that indicates packet start
                case RECV_SFD_SCAN:
                    /* If zero bits received, continue scanning */
                    if (b == 0x00) {
                        break;
                    }
                    
                    /* compare byte with (configurable) SFD value to detect end of SHR
                       and start of packet data.
                       Also check for value 0x7A which is falsely used in MSPSim radios
                       as the default SFD value. */
                    if ((b == SFD_VALUE_reg.read()) || (b == (byte) 0x7A)) {
                        rxstate = RECV_SHR_DETECTED;
                        // If waiting on ack in BUSY_TX_ARET don't make any status changes
                        if (waitingForAck) break;

                        //rx start invalidates CRC and ED register
                        PHY_RSSI_reg._rx_crc_valid.setValue(false); // XXX check
                        PHY_ED_LEVEL_reg.setValue(0xFF); // XXX check

                        // SHR detected status udates
                        switch (TRX_STATUS_reg._trx_status.getValue()) {
                            case STATE_RX_AACK_ON:
                                TRX_STATUS_reg._trx_status.setValue(STATE_BUSY_RX_AACK);
                                break;
                            case STATE_RX_ON:
                                TRX_STATUS_reg._trx_status.setValue(STATE_BUSY_RX);
                                break;
                            default:
                                if (DEBUG && printer != null) {
                                    printer.println("Warn: In State " + getStateName(TRX_STATUS_reg._trx_status.getValue()) + " while SHR detected. Ignoring.");
                                }
                                return 0;
                        }

                        postInterrupt(INT_RX_START);
                        break;
                    }

                    printer.println(String.format("RF231: Expected SFD, got 0x%02x%n", b));
                    break;

                // SFD matched. read the length from the next byte.
                case RECV_SHR_DETECTED:
                    // AT86Rf231 allows packet lengths > 127
                    packetLength = b;
                    if (DEBUGRX && printer != null) printer.println("Awaiting packet with length " + packetLength);

                    // ignore frame with zero length
                    if (packetLength == 0) {
                        if (DEBUG && printer != null) printer.println("Ignoring zero length packet!");
                        rxstate = RECV_SFD_SCAN;
                        break;
                    }

                    rxByteCounter = 0;
                    crc = 0;
                    rxstate = RECV_IN_PACKET;
                    
                    if (TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_TX_ARET && waitingForAck) {
                        // Check if packet length matches ACK MPDU length (5)
                        if (packetLength != 5) {
                            invalidAck = true;
                            break;
                        }
                    } else {
                        // Start transferring bytes to FIFO
                        frameBuffer.clear();
                        frameBuffer.add((byte) packetLength);
                        //Update the energy detect register
                        //This should have been an average rssi over the previous 8 symbols
                        //but basically is 3 times PHY_RSSI
                        PHY_ED_LEVEL_reg.setValue(PHY_RSSI_reg._rssi.getValue() * 3);
                    }
                    break;

                case RECV_IN_PACKET:
                    // we are in the body of the packet.
                    rxByteCounter++;
                    crc = crcAccumulate(crc, (byte) reverse_bits[(b) & 0xff]);
                    
                    // first two bytes are the FCF field that is decoded here
                    if (rxByteCounter == 1) {
                        fcfDecoder = new FCFDecoder();
                        fcfDecoder.decodeFirstByte(b);
                    } else if (rxByteCounter == 2) {
                        fcfDecoder.decodeSecondByte(b);
                        if (DEBUG && printer != null) printer.println(fcfDecoder.toString());
                    }
                    // the last two bits should be the FCS
                    if (rxByteCounter == packetLength - 2) {
                        // transition to receiving the FCS CRC.
                        rxstate = RECV_FCS_1;
                    }
                    
                    // ACK handling
                    if (waitingForAck) {
                        if (rxByteCounter == 1 && fcfDecoder.getFrameType() != FCFDecoder.FRAME_TYPE_ACK) {
                            // Expected ACK but got something else --> abort
                            printer.println("*** Error: Expected ACK but received something else...");
                            invalidAck = true;
                            break;
                        } else if (rxByteCounter == 3) {
                            // Check if received sequence number matches expected sequence number
                            if (b != lastTxSeqNo) {
                                printer.println("*** ERROR: Received invalid ACK! Expected " + (lastTxSeqNo & 0xFF) + " but got " + (b & 0xFF));
                                invalidAck = true;
                                break;
                            }
                        }
                        break;
                    }
                    
                    // ACKs are not put in the trx fifo
                    frameBuffer.add(b);
                    
                    // Store sequence number of packet to allow generating valid ACK reply
                    if (rxByteCounter == 3) {
                        // store sequence number required to generate valid ACK for this frame
                        lastRxSeqNo = b;
                    }
                    
                    // while in MAC Header, call frame filtering
                    if (rxByteCounter <= fcfDecoder.getMHRLength()) {
                        // If filter rejects frame, wait for end of transmission
                        if (!frameFilter(b, rxByteCounter)) {
                            rxstate = RECV_WAIT;
                            break;
                        }
                        
                        // If packet passed frame filter procedue, post AMI IRQ
                        if (rxByteCounter == fcfDecoder.getMHRLength()) {
                            postInterrupt(INT_AMI);
                        }
                    }
                    break;

                case RECV_FCS_1:
                    rxstate = RECV_FCS_2;
                    frameBuffer.add(b);
                    crcLow = b;
                    break;

                case RECV_FCS_2:
                    rxstate = RECV_END_STATE;
                    frameBuffer.add(b);
                    crcLow = (byte)reverse_bits[(crcLow) & 0xff];
                    b = (byte)reverse_bits[(b) & 0xff];
                    short crcResult = Arithmetic.word(b, crcLow);

                    // LQI is written in this position
                    byte lqi = (byte) ((byte)getCorrelation() & 0x7f);
                    if (crcResult == crc) lqi |= 0x80;// TODO: LQI increases when CRC valid?
                    frameBuffer.add(lqi);
                    
                    PHY_RSSI_reg._rx_crc_valid.setValue(crcResult == crc);
                    // If FCS is invalid
                    // XXX handle ACK FCS check
                    if (crcResult != crc) {
                        if (DEBUGRX && printer != null) printer.println("RF231: FCS invalid");
                        
                        if (waitingForAck) {
                            invalidAck = true;
                            break;
                        }
                        
                        // If not im promiscuous mode, a wrong FCS always leads to frame rejection
                        if (!XAH_CTRL_1_reg._aack_prom_mode.getValue()) {
                            if (TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_RX_AACK) {
                                TRX_STATUS_reg._trx_status.setValue(STATE_RX_AACK_ON);
                            } else if (TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_RX) {
                                TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON);
                            }
                            break;
                        }
                    }
                    
                    // A valid ACK was received
                    if (waitingForAck) {// XXX place after IRQ?
                        printer.println("Received valid ACK!");
                        clock.removeEvent(ackTimeOutEvent);
                        TRX_STATUS_reg._trx_status.setValue(STATE_TX_ARET_ON);
                        TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_SUCCESS);
                        receiver.shutdown();
                        //  transmitter.startup();
                        postInterrupt(INT_TRX_END);
                        return (b);
                    }

                    postInterrupt(INT_TRX_END);

                    // XXX do not handle ACK if in RX_ON mode!?
                    // If ACK was requested and is not disabled and AACK_FVN_MODE ok, prepare to send ACK
                    if (fcfDecoder.getAckRequested()
                            && !CSMA_SEED_1_reg._aack_dis_ack.getValue()
                            && fcfDecoder.getFrameVersion() <= CSMA_SEED_1_reg._aack_fvn_mode.getValue()) {
                        
                        // indicates waiting for rising edge on SLP_TR to transmit ack (slotted operation)
                        boolean awaitACKTrigger = false; // XXX for SLP_TR
                        if (XAH_CTRL_0_reg._slotted_operation.getValue()) {
                            awaitACKTrigger = true;
                            // XXX wait for SLP_TR rising edge...
                            throw new UnsupportedOperationException("ACK slotted Operation not implemented yet!");
                        } else {
                            sendAck = true;
                            // XXX Start ACK Tranmission here or at END_STATE ?
                        }
                    } else {
                        // End receiption of no Ack needs to be sent
                        if (TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_RX_AACK) {
                            TRX_STATUS_reg._trx_status.setValue(STATE_RX_AACK_ON);
                        } else if (TRX_STATUS_reg._trx_status.getValue() == STATE_BUSY_RX) {
                            TRX_STATUS_reg._trx_status.setValue(STATE_RX_ON);
                        }
                    }
                    break;
                    
                case RECV_OVERFLOW:
                    // do nothing. we have encountered an overflow.
                    break;
                    
                case RECV_WAIT:
                    // just wait for the end of the packet
                    if (++rxByteCounter == packetLength) {
                        clearBER();  // will otherwise be done by getCorrelation()
                        rxstate = RECV_SFD_SCAN;
                    }
                    break;
                    
                default:
                    if (DEBUGRX && printer!=null) printer.println("RF231: Unknown state in receiver");
                    break;
            }

            // finally handle inalidAck
            if (waitingForAck && invalidAck) {
                            // XXX Invalid ACK, retry

                /* XXX Check if this all is valid...
                 - Does this handle frame retry handling?
                 - Is INT_TRX_END ok?
                 */
                // not an ack, show failure and abort rx
                if (DEBUGA && printer != null) {
                    printer.println("RF231: Expecting ack, got packet of length " + String.valueOf(packetLength));
                }
                //
                clock.removeEvent(ackTimeOutEvent);
                waitingForAck = false;

                //Set TRAC_STATUS bits to failure
                TRX_STATE_reg._trac_status.setValue(TRAC_STATUS_NO_ACK);
                /* Issue TRX_END interrupt */
                postInterrupt(INT_TRX_END);
                TRX_STATUS_reg._trx_status.setValue(STATE_TX_ARET_ON);

                rxstate = RECV_SFD_SCAN;

                receiver.shutdown();
            }

            
            return b;
        }

        /** 
         * Filters frames according to rule numbers specified in AT86RF231 datasheet.
         * 
         * @param b currently read byte
         * @param counter Number of this byte in packet
         * @return True if accepted, False if Rejected
         */
        private boolean frameFilter(byte b, int counter) {// XXX bool rx_aack?
            
            //
            // AACK_FLTR_RES_FT = 1 -> 802.15.4 filtering
            //                  = 0 -> no filtering (only FCS check)
            // AACK_UPLD_RES_FT = 1 -> process reserved frames further
//            //
//            if (counter > 1 && (trxFIFO.getRelativeByte(1) & 0x04) == 4) {
//                // no further address decoding is done for reserved frames
//                return true;
//            }
            switch (counter) {
                // FCF first Byte
                case 1:
                    /* 1. The Frame Type subfield shall not contain a reserved frame type. */
                    if (fcfDecoder.getFrameType() > FCFDecoder.FRAME_TYPE_MAC_CMD) {
                        if (DEBUGRX && printer!=null) printer.println("RF231: Frame rejected by filter rule 1 (reserved)");
                        return false;
                    }
                    /* 7. The frame type indicates that the frame is not an ACK frame. */
                    if (fcfDecoder.getFrameType() == FCFDecoder.FRAME_TYPE_ACK) {
                        if (DEBUGRX && printer!=null) printer.println("RF231: Frame rejected by filter rule 7 (ACK)");
                        return false;
                    }
                    return true;
                // FCF second byte
                case 2:
                    /* 2. The Frame Version subfield shall not contain a reserved value. */
                    if (fcfDecoder.getFrameVersion() > FCFDecoder.FRAME_VERSION_2006) {
                        return false;
                    }
                    /* 8. At least one address field must be configured */
                    if (fcfDecoder.getDestAddrMode() == FCFDecoder.ADDR_MODE_NONE && fcfDecoder.getSrcAddrMode() == FCFDecoder.ADDR_MODE_NONE) {
                        return false;
                    }
                    return true;
                // End of Destination or source PAN ID 
                case 5:
                    macPANId[0] = PAN_ID_0_reg.read();
                    macPANId[1] = PAN_ID_1_reg.read();
                    PANId = frameBuffer.getRelativeField(4, 6);
                    /* 3. If a destination PAN identifier is included in the frame, 
                       it shall match macPANId or shall be the broadcast PAN identifier (0xFFFF).*/
                    if (fcfDecoder.getDestAddrMode() == FCFDecoder.ADDR_MODE_SHORT
                            || fcfDecoder.getDestAddrMode() == FCFDecoder.ADDR_MODE_EXTENDED) {
                        if (DEBUGRX && printer != null) {
                            printer.println(String.format("Packet PANId:  0x%02x%02x", PANId[0], PANId[1]));
                            printer.println(String.format("Node macPANId: 0x%02x%02x", macPANId[0], macPANId[1]));
                        }
                        if (Arrays.equals(PANId, macPANId) || Arrays.equals(PANId, SHORT_BROADCAST_ADDR)) {
                            return true;
                        }
                        
                        if (DEBUGRX && printer!=null) printer.println("RF231: Frame rejected by filter rule 3 (destPanID)");
                        return false;
                    }
                    
                    /* 5. If the frame type indicates that the frame is a beacon frame,
                       the source PAN identifier hall match macPANId.*/
                    if (fcfDecoder.getFrameType() == FCFDecoder.FRAME_TYPE_BEACON) {
                        /* If  macPANId is equal to 0xFFF, frame shall be accepted
                           regardless of the source PAN identifier. */
                        if (Arrays.equals(macPANId, SHORT_BROADCAST_ADDR)) {
                            return true;
                        }
                        if (Arrays.equals(PANId, macPANId)) {
                            return true;
                        }
                        if (DEBUGRX && printer!=null) printer.println("RF231: Frame rejected by filter rule 5 (beacon)");
                        return false;
                    }
                    break;
                // End of destination/source short address
                case 7:
                    /* 4. If a short destination address is included in the frame,
                       it shall match either macShortAddress or the broadcast address (0xFFFF). */
                    if (fcfDecoder.getDestAddrMode() == FCFDecoder.ADDR_MODE_SHORT) {
                        shortDestAddr = frameBuffer.getRelativeField(6, 8);
                        macShortAddr[0] = SHORT_ADDR_0_reg.read();
                        macShortAddr[1] = SHORT_ADDR_1_reg.read();
                        if (DEBUGRX && printer != null) {
                            printer.println(String.format("Packet shortaddr: 0x%02x%02x", shortDestAddr[0], shortDestAddr[1]));
                            printer.println(String.format("Node shortaddr:   0x%02x%02x", macShortAddr[0], macShortAddr[1]));
                        }
                        if (Arrays.equals(shortDestAddr, macShortAddr) || Arrays.equals(shortDestAddr, SHORT_BROADCAST_ADDR)) {
                            return true;
                        }
                        if (DEBUGRX && printer!=null) printer.println("RF231: Frame rejected by filter rule 4 (destAddr)");
                        return false;
                    }
                    /* 6. If only source addressing fields are included in a data or MAC command frame,
                       the frame shall be accepted only if the device is the PAN coordinator
                       and the source PAN identifier matches macPANId.*/
                    if (fcfDecoder.getDestAddrMode() == FCFDecoder.ADDR_MODE_NONE
                            && (fcfDecoder.getFrameType() == FCFDecoder.FRAME_TYPE_DATA
                            || fcfDecoder.getFrameType() == FCFDecoder.FRAME_TYPE_BEACON)) {
                        shortSrcAddr = frameBuffer.getRelativeField(6, 8);
                        if (CSMA_SEED_1_reg._aack_i_am_coord.getValue() && Arrays.equals(shortSrcAddr, macPANId)) {
                            return true;
                        }
                        if (DEBUGRX && printer!=null) printer.println("RF231: Frame rejected by filter rule 6 (srcAddr)");
                        return false;
                    }
                        
                    break;
                // End of Destination/Source extended address
                case 13:
                    /* 4. If an extended destination address is included in the frame, 
                       it shall match aExtendedAddress. */
                    if (fcfDecoder.getDestAddrMode() == FCFDecoder.ADDR_MODE_EXTENDED) {
                        extDestAddr = frameBuffer.getRelativeField(6, 14);
                        aExtendedAddress = new byte[]{
                            IEEE_ADDR_0_reg.read(),
                            IEEE_ADDR_1_reg.read(),
                            IEEE_ADDR_2_reg.read(),
                            IEEE_ADDR_3_reg.read(),
                            IEEE_ADDR_4_reg.read(),
                            IEEE_ADDR_5_reg.read(),
                            IEEE_ADDR_6_reg.read(),
                            IEEE_ADDR_7_reg.read()
                        };
                        if (DEBUGRX && printer != null) {
                            printer.println(String.format("Packet longadr: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                                    extDestAddr[0], extDestAddr[1], extDestAddr[2], extDestAddr[3], extDestAddr[4], extDestAddr[5], extDestAddr[6], extDestAddr[7]));
                            printer.println(String.format("Node IEEEAddr:  %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                                    aExtendedAddress[0], aExtendedAddress[1], aExtendedAddress[2], aExtendedAddress[3], aExtendedAddress[4], aExtendedAddress[5], aExtendedAddress[6], aExtendedAddress[7]));
                        }
                        if (Arrays.equals(extDestAddr, aExtendedAddress) || Arrays.equals(extDestAddr, LONG_BROADCAST_ADDR)) {
                            return true;
                        }
                        if (DEBUGRX && printer != null) printer.println("RF231: Frame rejected by filter rule 4 (destAddr)");
                        return false;
                    }
                    break;
            }
            return true;
        }

        void startup() {
            // do not activate if already active
            if (rxactive) {
                printer.println("receiver startup while active");
                return;
            }
            
            rxactive = true;
            energyStateMachine.transition(3);//change to receive state TODO:low sensitivity state
            rxstate = RECV_SFD_SCAN;
            clearBER();
            beginReceive(getFrequency());
             //   clock.insertEvent(rssiValidEvent, 4*cyclesPerByte);  // 8 symbols = 4 bytes
            if (DEBUGRX && printer!=null) printer.println("RF231: RX startup");
        }

        void shutdown() {
            // do not deactivate if already inactive
            if (!rxactive) {
                printer.println("receiver shutdown while not active");
                return;
            }
            
            rxactive = false;
            endReceive();
            rxstate = RECV_SFD_SCAN;
            energyStateMachine.transition(AT86RF231Energy.TRX_OFF);//change to idle state
             //   setRssiValid(false);
            if (DEBUGRX && printer!=null) printer.println("RF231: RX shutdown");
        }

    /* -----------------------------------RSSI, LQI, PER------------------------------------*/
        //LUT for max and min correlation values depending on PER
        private final int [] Corr_MAX = {110,109,109,109,107,107,107,107,107,
        107,107,107,103,102,102,102,101,101,101,101,99,94,92,94,101,97,98,97,97,97,97,97,
        94,94,94,94,94,94,94,94,94,94,94,94,92,89,89,89,89,89,88,88,88,88,88,86,86,86,
        86,86,86,86,86,86,85,85,85,85,85,85,83,83,83,83,83,83,83,83,79,78,78,78,78,78,
        76,76,76,74,74,74,74,74,74,74,74,74,74,66,65,65,65};
        private final int [] Corr_MIN = {95,95,94,91,90,90,89,89,89,88,88,88,82,
        82,82,82,76,76,76,76,76,76,74,74,74,74,74,74,72,72,72,72,72,72,72,72,69,69,69,69,
        69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,67,67,67,67,67,67,65,65,65,65,65,
        65,65,64,64,63,63,63,63,63,63,63,63,63,61,61,61,60,60,60,58,58,56,56,56,55,55,55,
        50,50,50,50,50,50,50};
        protected double Correlation;
/*
        private void setRssiValid (boolean v){
            if (DEBUGA && printer!=null) printer.println("RF231: setrssivalid "+ rssi_val);
            registers[PHY_RSSI] |= 0x80; //Set RX_CRC_VALID
        }

        private boolean getRssiValid (){
            if (DEBUGA && printer!=null) printer.println("RF231: getRssivalid");
            //RF231 RSSI is always valid in non-extended mode
            return true;
        }
 */
        public double getCorrelation (){
            int PERindex = (int)(getPER()*100);
            Random random = new Random();
            //get the range, casting to long to avoid overflow problems
            long range = (long)Corr_MAX[PERindex] - (long)Corr_MIN[PERindex] + 1;
            // compute a fraction of the range, 0 <= frac < range
            long fraction = (long)(range * random.nextDouble());
            double corr = fraction + Corr_MIN[PERindex];
            if (DEBUGRX && printer!=null) printer.println("RF231: returncorr " + corr);
            return corr;
        }

        @Override
        public void setRSSI (double Prec){
            //RSSI register in units of 3dBm, 0 <-90dBm, 28 >=-10 dBm
            int rssi_val = (((int) Math.rint(Prec) + 90) / 3) +1;
            if (rssi_val < 0) rssi_val = 0;
            if (rssi_val > 28) rssi_val = 28;
    //      if (DEBUGRX && printer!=null) if (rssi_val > 0) printer.println("RF231: setrssi " + rssi_val);
            PHY_RSSI_reg._rssi.setValue(rssi_val);
        }

        public double getRSSI() {
            int rssi_val = PHY_RSSI_reg._rssi.getValue();
            return -90 + 3 * (rssi_val - 1);
        }

        @Override
        public void setBER (double BER){
        //  if (DEBUGRX && printer!=null) printer.println("RF231: setBER");
            BERcount++;
            if (BERcount > 5) {
                BERtotal += BER;
            }
        }

        public double getPER (){
            double PER = 0.0D;
            if (BERcount > 5) {
                //compute average BER after SHR
                PER = BERtotal/(BERcount-5);
                //considering i.i.s errors i compute PER
                PER = 1D-Math.pow((1D-PER),(BERcount-5)*8);
            }
            clearBER();
         // if (DEBUGRX && printer!=null) printer.println("RF231: getPER " + PER);
            return PER;
        }

        public void clearBER() {
            BERcount = 0;
            BERtotal = 0.0D;
        }


 /**
         * The <code>RssiValid</code> class implements a Simulator Event
         * that is fired when the RSSI becomes valid after 8 symbols
         */
         /*
        protected class RssiValid implements Simulator.Event {

            public void fire() {
                if (activated) {
              //      setRssiValid(true);
                }
            }
        }
        protected RssiValid rssiValidEvent = new RssiValid();
*/
    }

    private long toCycles(long us) {
        return us * sim.getClock().getHZ() / 1000000;
    }

   // public static Medium createMedium(Synchronizer synch, Medium.Arbitrator arbitrator) {
    public Medium createMedium(Synchronizer synch, Medium.Arbitrator arbitrator) {
        return new Medium(synch, arbitrator, 250000, 48, 8, 8 * 128);
    }

    @Override
    public Medium.Transmitter getTransmitter() {
        return transmitter;
    }

    @Override
    public Medium.Receiver getReceiver() {
        return receiver;
    }

    @Override
    public void setMedium(Medium m) {
        medium = m;
        transmitter = new Transmitter(m);
        receiver = new Receiver(m);
    }

    @Override
    public Medium getMedium() {
        return medium;
    }

    /**
     * The <code>RF231Pin</code>() class models pins that are inputs and outputs to the RF231 chip.
     */
    public class RF231Pin implements Microcontroller.Pin.Input, Microcontroller.Pin.Output {
      
        protected LinkedList<Microcontroller.Pin.InputListener> listeners = new LinkedList<>();
        protected final String name;
        protected boolean level;

        public RF231Pin(String n) {
            name = n;
        }

        @Override
        public void write(boolean level) {
            if (this.level != level) {
                // level changed
                this.level = level;
                if (this == CS_pin) pinChange_CS(level);
                else if (this == RSTN_pin) pinChange_RST(level);
                else if (this == SLPTR_pin) pinChange_SLP_TR(level);
                if (DEBUGV && printer!=null) printer.println("RF231 Write pin " + name + " -> " + level);
            }
        }

        @Override
        public boolean read() {
            if (DEBUGV && printer!=null) printer.println("RF231 Read pin " + name + " -> " + level);
            return level;
        }

        @Override
        public void registerListener(Microcontroller.Pin.InputListener listener) {
            listeners.add(listener);
        }

        @Override
        public void unregisterListener(Microcontroller.Pin.InputListener listener) {
            listeners.remove(listener);
        }
    }

    public class RF231Output extends Microcontroller.Pin.ListenableBooleanViewInput implements Microcontroller.Pin.Input {

        //protected BooleanView level;
        protected final String name;

        public RF231Output(String n, BooleanView lvl) {
            super(lvl);
            name = n;
        }

        @Override
        public boolean read() {
            boolean val = super.read();
            if (DEBUGV && printer!=null)
              printer.println("RF231 Read (output) pin " + name + " -> " + val);
            return val;
        }
    }

    public static String regName(int reg) {
        switch (reg) {
            case TRX_STATUS:
                return "TRX_STATUS  ";
            case TRX_STATE:
                return "TRX_STATE   ";
            case TRX_CTRL_0:
                return "TRX_CTRL_0  ";
            case TRX_CTRL_1:
                return "TRX_CTRL_1  ";
            case PHY_TX_PWR:
                return "PHY_TX_PWR  ";
            case PHY_RSSI:
                return "PHY_RSSI    ";
            case PHY_ED_LEVEL:
                return "PHY_ED_LEVEL";
            case PHY_CC_CCA:
                return "PHY_CC_CCA  ";
            case CCA_THRES:
                return "CCA_THRES   ";
            case RX_CTRL:
                return "RX_CTRL     ";
            case SFD_VALUE:
                return "SFD_VALUE   ";
            case TRX_CTRL_2:
                return "TRX_CTRL_2  ";
            case ANT_DIV:
                return "ANT_DIV     ";
            case IRQ_MASK:
                return "IRQ_MASK    ";
            case IRQ_STATUS:
                return "IRQ_STATUS  ";
            case VREG_CTRL:
                return "VREG_CTRL   ";
            case BATMON:
                return "BATMON      ";
            case XOSC_CTRL:
                return "XOSC_CTRL   ";
            case RX_SYN:
                return "RX_SYN      ";
            case XAH_CTRL_1:
                return "XAH_CTRL_1  ";
            case PLL_CF:
                return "PLL_CF      ";
            case PLL_DCU:
                return "PLL_DCU     ";
            case PART_NUM:
                return "PART_NUM    ";
            case VERSION_NUM:
                return "VERSION_NUM ";
            case MAN_ID_0:
                return "MAN_ID_0    ";
            case MAN_ID_1:
                return "MAN_ID_1    ";
            case SHORT_ADDR_0:
                return "SHORT_ADDR_0";
            case SHORT_ADDR_1:
                return "SHORT_ADDR_1";
            case PAN_ID_0:
                return "PAN_ID_0    ";
            case PAN_ID_1:
                return "PAN_ID_1    ";
            case IEEE_ADDR_0:
                return "IEEE_ADDR_0 ";
            case IEEE_ADDR_1:
                return "IEEE_ADDR_1 ";
            case IEEE_ADDR_2:
                return "IEEE_ADDR_2 ";
            case IEEE_ADDR_3:
                return "IEEE_ADDR_3 ";
            case IEEE_ADDR_4:
                return "IEEE_ADDR_4 ";
            case IEEE_ADDR_5:
                return "IEEE_ADDR_5 ";
            case IEEE_ADDR_6:
                return "IEEE_ADDR_6 ";
            case IEEE_ADDR_7:
                return "IEEE_ADDR_7 ";
            case XAH_CTRL_0:
                return "XAH_CTRL_0  ";
            case CSMA_SEED_0:
                return "CSMA_SEED_0 ";
            case CSMA_SEED_1:
                return "CSMA_SEED_1 ";
            case CSMA_BE:
                return "CSMA_BE     ";
            default:
                return StringUtil.to0xHex(reg, 2) + "    ";
        }
    }
    
    /**
     * Decodes Frame Control Field (FCF) information.
     */
    protected class FCFDecoder {
        
        /* Defined values for FCF field 'Frame Type'.
           Note: All other values are 'Reserved' */
        /** Beacon */
        public static final int FRAME_TYPE_BEACON   = 0;
        /** Data */
        public static final int FRAME_TYPE_DATA     = 1;
        /** Acknowledge */
        public static final int FRAME_TYPE_ACK      = 2;
        /** MAC command */
        public static final int FRAME_TYPE_MAC_CMD  = 3;
        
        /* Defined values for FCF field 'Frame Version'.
           Note: All other values are 'Reserved' */
        /**  Frames are compatible with IEEE 802.15.4 2003 */
        public static final int FRAME_VERSION_2003  = 0;
        /**  Frames are compatible with IEEE 802.15.4 2006 */
        public static final int FRAME_VERSION_2006  = 1;
        
        /* Defined values for FCF fields 'Destination Addressing Mode'
           and 'Source Addressing Mode'.
           Note: All other values are 'Reserved' */
        /**  PAN identifier and address fields are not present */
        public static final int ADDR_MODE_NONE      = 0;
        public static final int ADDR_MODE_RESERVED  = 1;
        /** Address field contains a 16-bit short address */
        public static final int ADDR_MODE_SHORT     = 2;
        /** Address field contains a 64-bit extended address */
        public static final int ADDR_MODE_EXTENDED  = 3;

        
        private short fcfData[];
        
        private int frameType;
        private boolean secEnabled;
        private boolean framePending;
        private boolean ackRequested;
        private boolean intraPAN;
        private int destAddrMode;
        private int frameVersion;
        private int srcAddrMode;
        
        // length of MAC Header based on FCF settings. Preset to 3.
        private int MHRLenght = 3;
        

        FCFDecoder() {
        }

        FCFDecoder(short[] data) {
            fcfData = data;
        }
        
        void setFCFData(short[] data) {
            fcfData = data;
        }

        void decodeFirstByte(byte first) {
            fcfData = new short[2];
            fcfData[0] = first;
            analyzeFirst();
        }
        
        void decodeSecondByte(byte second) {
            fcfData[1] = second;
            analyzeSecond();
        }

        private void analyzeFirst() {
            if (fcfData == null && printer != null) {
                printer.println("Error: analyze() called for null data");
            }
            frameType = fcfData[0] & 0x07;
            secEnabled = (fcfData[0] & 0x08) != 0;
            framePending = (fcfData[0] & 0x10) != 0;
            ackRequested = (fcfData[0] & 0x20) != 0;
            intraPAN = (fcfData[0] & 0x40) != 0; 
        }
        
        private void analyzeSecond() {
            destAddrMode = (fcfData[1] & 0x0C) >> 2;
            frameVersion = (fcfData[1] & 0x30) >> 4;
            srcAddrMode  = (fcfData[1] & 0xC0) >> 6;
            
            // calculate expected header lenght based on FCF data
            MHRLenght = 2 + 1;
            if (destAddrMode == ADDR_MODE_SHORT) {
                MHRLenght += 2 + 2;
            } else if (destAddrMode == ADDR_MODE_EXTENDED) {
                MHRLenght += 2 + 8;
            }
            if (srcAddrMode == ADDR_MODE_SHORT) {
                if (!intraPAN) MHRLenght += 2;
                MHRLenght += 2;
            } else if (srcAddrMode == ADDR_MODE_EXTENDED) {
                if (!intraPAN) MHRLenght += 2;
                MHRLenght += 8;
            }
        }
        
        int getFrameType() {
            return frameType;
        }

        boolean getSecEnabled() {
            return secEnabled;
        }

        boolean getFramePending() {
            return framePending;
        }

        boolean getAckRequested() {
            return ackRequested;
        }

        boolean getIntraPAN() {
            return intraPAN;
        }

        int getDestAddrMode() {
            return destAddrMode;
        }

        int getFrameVersion() {
            return frameVersion;
        }

        int getSrcAddrMode() {
            return srcAddrMode;
        }
        
        int getMHRLength() {
            return MHRLenght;
        }
        
        @Override
        public String toString() {
            StringBuilder builder = new StringBuilder();
            builder.append("FCF: MHRLength: ")
                    .append(MHRLenght)
                    .append("\n    Type: ");
            switch (frameType) {
                case FRAME_TYPE_BEACON:
                    builder.append("Beacon");
                    break;
                case FRAME_TYPE_DATA:
                    builder.append("Data");
                    break;
                case FRAME_TYPE_ACK:
                    builder.append("ACK");
                    break;
                case FRAME_TYPE_MAC_CMD:
                    builder.append("Mac Command");
                    break;
                default:
                    builder.append("Reserved");
                    break;
            }
            builder.append("\n    Sec.Enabled: ")
                    .append(secEnabled)
                    .append("\n    Frame Pending: ")
                    .append(framePending)
                    .append("\n    ACK requested: ")
                    .append(ackRequested)
                    .append("\n    Intra Pan: ")
                    .append(intraPAN)
                    .append("\n    Dest addr mode: ");
            switch (destAddrMode) {
                case ADDR_MODE_NONE:
                    builder.append("none");
                    break;
                case ADDR_MODE_SHORT:
                    builder.append("short");
                    break;
                case ADDR_MODE_EXTENDED:
                    builder.append("extended");
                    break;
            }
            builder.append("\n    Frame Version: ")
                    .append(frameVersion)
                    .append("\n    Src addr mode: ");
            switch (srcAddrMode) {
                case ADDR_MODE_NONE:
                    builder.append("none");
                    break;
                case ADDR_MODE_SHORT:
                    builder.append("short");
                    break;
                case ADDR_MODE_EXTENDED:
                    builder.append("extended");
                    break;
            }
            return builder.toString();
        }

    }
}
