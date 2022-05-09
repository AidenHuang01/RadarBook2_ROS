# Radarbook2.py -- Radarbook2 class
#
# Copyright (C) 2017-04 Inras GmbH Haderer Andreas
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE file for details.

# Version 1.0.0
# Version 1.1.0
# - Add DmaMult value to que multiple transfers used for framed mode of operateion
# - Add Command to set role and Adc sampling to 40 MHz for range-Doppler processing

# Version 1.1.0
# - Correct Fpga_DispRfChipSts function

# Revison R 1.2.0
#   - Add UBLOX commands and commands to parse GPS

# Revision R 1.2.1
#   - Add Get cmd to read back int dc couplin
#   - Allow CIC filter R > 2^10
#  Revision R 1.2.2
#   - Change output string of Version number
# Revision R 1.3.0
#   - Add Set File param, add function for clock selection
# Revision R 1.4.0
#   - Add function to read clock and trigger status
# Revison R 1.5.0
#   - Add UBLOX commands and commands to parse GPS


import  sys
import  os
import  socket
import  struct
from    numpy import *
import  time
import  src.cmd_modules.Connection as Connection
import  src.cmd_modules.SeqTrig as SeqTrig

class Rbk2(Connection.Connection):
    """ Radarbook2 class object:
        (c) Haderer Andreas Inras GmbH

        Communication and access of Radarbook2 device with TCPIP Connection
    """
    def __init__(self, stConType='PNet', *args):
        super(Rbk2, self).__init__('Radarbook2', stConType, *args)

        self.stVers         =   "1.5.0"
        self.Sts            =   "Not connected"                 # Status of object
        #self.stIpAdr        =   stIpAdr                        # Ip Address
        #self.CfgPort        =   8001
        #self.DatPort        =   6000
        self.HUid           =   0                               # Hardware UID of radarbook
        self.SUid           =   0                               # Software UID of radarbook FPGA framework
        self.SVers          =   0
        self.DictModuleId   =   {   11: "AD7622    ",   12: "PiPoAv",       13: "SaCon",
                                    14: "Cic1      ",   15: "AD7991",       16: "RefC",
                                    17: "AD5660    ",   18: "AltSpi",       19: "USpi",
                                    20: "MCP2515   ",   21: "MT29F4G08",    22: "FArmSpi",
                                    23: "FArmMem   ",   24: "AD8283DatInt", 25: "AD8283",
                                    26: "AD9512    ",   27: "SFtdi",        28: "TimUnit",
                                    29: "AltTim    ",   30: "AltOut",       31: "AltIn",
                                    32: "VersContr ",   33: "Cmd",          34: "FrameGen",
                                    35: "BGT24     ",   36: "HFtdi",        37: "FArmSysMon",
                                    38: "SysChn    ",   39: "Hmc703e",      40: "FrmCtrl",
                                    41: "SeqTrig   ",   42: "BLX",          43: "RCC",
                                    44: "ChnMap6   ",   47: "AD5160",       48: "FArmHPPwr",
                                    49: "SRamFifo  ",   54: "ICTA",         57: "ZCFilt",
                                    58: "TarList   ",   59: "DPA",          60: "ChnMap8",
                                    61: "FrmCtrl8  ",   62: "KorTim",       63: "RDMap",
                                    64: "AFE5801DatInt",65: "FArmWDG",      67: "ArArmPwr",
                                    68: "USpi8     ",   69: "AD9517_4",     72: "AFE5801",
                                    76: "USPIF     ",
                                    77: "USpiF     ",   78: "USpi16F",      79: "IFRX",
                                    80: "I2CCtrl   ",   81: "RfDevCfg",     89: "ADF5901",
                                    91: "DigIn"}
        # define Brd constants
        self.DefBrdConst()

        #self.cDebugInf                           =   0
        self.stName                             =   "Radarbook2"
        #self.hCon                               =   -1
        #self.hConDat                            =   -1

        #------------------------------------------------------------------
        # Configure Sampling
        #------------------------------------------------------------------
        self.Rad_N                              =   1248
        self.Rad_NMult                          =   1
        self.Rad_Rst                            =   1

        self.Rad_SampCfg_Nr                     =   1248-1
        self.Rad_SampCfg_Rst                    =   1

        self.Rad_MaskChn                        =   2**8-1
        self.Rad_MaskChnEna                     =   2**8-1
        self.Rad_MaskChnRst                     =   0

        self.Rad_Afe5801_Mask                   =   3
        self.Rad_Afe5801_RegConfig              =   0
        self.Rad_Afe5801_RegPower               =   0
        self.Rad_Afe5801_RegLvds                =   int('0x0', 0)       # AFE5801_REG_LVDS_MODE_NO_PATTERN
        self.Rad_Afe5801_RegData                =   int('0x0', 0)       # AFE5801_REG_DATA_SERIAL_RATE12
        self.Rad_Afe5801_RegDfs                 =   0
        self.Rad_Afe5801_RegLvdsPattern         =   8
        self.Rad_Afe5801_RegFilter              =   int('0x08', 0)      # AFE5801_REG_FILTER_BW_7_5MHZ
        self.Rad_Afe5801_RegOffsetGainCh1       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh2       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh3       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh4       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegHighpass14          =   0
        self.Rad_Afe5801_RegOffsetGainCh5       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh6       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh7       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh8       =   (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegHighpass58          =   0
        self.Rad_Afe5801_RegClamp               =   0
        self.Rad_Afe5801_RegTgcStartIndex       =   1
        self.Rad_Afe5801_RegTgcStopIndex        =   1
        self.Rad_Afe5801_RegTgcStartGain        =   int('0x19', 0)     # AFE5801_REG_TGC_GAIN_20DB
        self.Rad_Afe5801_RegTgcHoldGainTime     =   0
        self.Rad_Afe5801_RegTgcGainMode         =   int('0x08', 0)     # AFE5801_REG_TGC_GAIN_MODE_STATIC_PGA
        self.Rad_Afe5801_RegTgcGainCoarse       =   int('0x05', 0)     # AFE5801_REG_TGC_GAIN_10DB
        self.Rad_Afe5801_RegTgcUniformSlope     =   0


        self.Rad_ArmMemCfg_Mask                 =   3
        self.Rad_ArmMemCfg_Ctrl                 =   16                   # 16 realease reset on EoP
        self.Rad_ArmMemCfg_Chn                  =   2**8-1
        self.Rad_ArmMemCfg_FrmSiz               =   1024
        self.Rad_ArmMemCfg_WaitLev              =   32768 - 4096
        self.Rad_ArmMemCfg_FifoSiz              =   32768

        self.Rad_fAdc                           =   20e6
        self.Rad_NrChn                          =   4
        self.Rad_FrmSiz                         =   1
        self.Rad_Role = 0

        #------------------------------------------------------------------
        # Configure Cic filter
        #------------------------------------------------------------------
        self.Rad_CicCfg_FiltSel                 =   2**8-1                                          # Select Cic filters
        self.Rad_CicCfg_CombDel                 =   0                                               # Comb delay
        self.Rad_CicCfg_OutSel                  =   0                                               # Output select
        self.Rad_CicCfg_SampPhs                 =   0                                               # Sampling phase
        self.Rad_CicCfg_SampRed                 =   1                                               # Sampling rate reduction
        self.Rad_CicCfg_RegCtrl                 =   int('0x800',0) + int('0x400',0) + int('0x1',0)  # Cic filter control register


        #------------------------------------------------------------------
        # Configure Frame Control MMP
        #------------------------------------------------------------------
        self.Rad_FrmCtrlCfg_ChnSel                  =   2**8 - 1                                            # Select Channels to be configured
        self.Rad_FrmCtrlCfg_Rst                     =   1                                                   # Reset MMP
        self.Rad_FrmCtrlCfg_RegChnCtrl              =   int('0x2',0) + int('0x8',0) + int('0x10',0)         # Channel control register


        self.Rad_DmaMult                            =   1

        self.SeqCfg                                 =   { "Mask" : 1}

        self.RAD_ROLE_MS = 0
        self.RAD_ROLE_MSCLKOUT = 1
        self.RAD_ROLE_SL = 128
        self.RAD_ROLE_SLCLKIN = 129
        self.RAD_ROLE_INTGPS = 130

        self.Gps = 0
        self.GpsDat = []
        self.GpsFrmDat = []
        self.GpsFrmCntr = []
        self.GpsFrmCtrl = 2 + 4
        self.GpsCntr = 0
        self.GpsRstFifoOnTimStamp = 8


        self.FuSca      =   0.25/2**12

        fSeqTrig        =   20e6
        objSeqTrig      =   SeqTrig.SeqTrig(fSeqTrig)
        self.SeqCfg     =   objSeqTrig.ContUnifM1(1e-3)

    def __str__(self):
        return "Radarbook2 " + " (" + self.stIpAdr + ", " + str(self.PortNr) + ") " + self.Sts

    def __del__(self):
        return;

    # DOXYGEN ------------------------------------------------------
    #> @brief Get Version information of Radarbook class
    #>
    #> Get version of class
    #>      - Version string is returned as string
    #>
    #> @return  Returns the version string of the class (e.g. 0.5.0)
    def GetVers(self):
        return self.stVers

    # DOXYGEN -------------------------------------------------
    #> @brief Displays Version information in Matlab command window
    #>
    #> Display version of class in Matlab command window
    def DispVers(self):
        print("Radarbook2 Class Version: ", self.stVers)

    def BrdChkSocSysId(self):
        Vals = self.Fpga_GetBrdSts(0)
        if Vals[1] == 545404223:
            print('FFT framework installed on SoC')
            print('Raw sampling framework not installed on Soc')
            sys.exit()
        elif Vals[1] == 1884392676:
            print('Raw sampling framework installed on SoC')
        else:
            print('Update of SoC Firmware with webinterface required')
            print('Raw sampling framework not installed on Soc')
            sys.exit()

    def SetFileParam(self, stKey, Val, DataType='STRING'):
        if isinstance(stKey,str):
            self.ConSetFileParam(stKey, Val, DataType);
        else:
            print('Key is not of type string');

    def GetFileParam(self, stKey, stType='UNKNOWN'):
        if isinstance(stKey,str):
            return self.ConGetFileParam(stKey, stType);
        else:
            print('Key is not of type string')
            return [];

    def Set(self, stVal, *varargin):
        #   @function       Set
        #   @author         Haderer Andreas (HaAn)
        #   @date           2015-07-27
        #   @brief          Set Parameters of Radarbook Baseboard
        #   @paramin[in]    stSelect: String to select parameter to change
        #
        #                   Known stSelect Parameter Strings:
        #                       DebugInf:       Debug Level (>10 Debug Information is output)
        #                           ->          Debug Level number 0 - ...
        #                       Name:           Name of Board
        #                           ->          string containing desired name
        #                       N:              Number of Samples
        #                           ->          Value with number of samples
        #                       CicDi:          Disable CIC filters
        #                       CicEna:         Enable CIC filters
        #                       CicStages:      Number of stages of CIC filter
        #                           ->          Value 1 - 4
        #                       CicDelay:       Comb delay of CIC filter
        #                           ->          Comb delay: 2,4,8,16 are supported values
        #                       CicR:           Sample rate reduction values of CIC filter
        #                           ->          Sample rate reduction 1- ...
        #                       ClkDiv:         AD9512 Clock Divider
        #                           ->          Clock divider value 1 - 32
        #                       NrFrms:         Number of frames to sample
        #                           ->          Value containing number of frames
        #                       ClkSrc:         Clock Source
        #                           ->          0: Clock from frontend; 1 (default) from baseboard
        #                       AdcImp:         Impdeance of AD8283
        #                           ->          < 100.1k Set to 200E; > 100.1k set to 200k
        #                       NrChn:          Number of channels for processing and dat transfer
        #                           ->          1 to 12
        #                       Fifo:           SRamFifo: Enable or Disable Fifo
        #                           ->          On or Off
        #                       AdcChn:         Set number of ADC channels
        #                           ->          2 - 6
        #                       NMult:          Combine multiple frames for data transfer (Transfer size: N*NMult)
        #                           ->          integer value 1 ..
        #                       AdcGain:
        #                           ->
        if stVal == 'DebugInf':
            if len(varargin) > 0:
                self.cDebugInf    =   varargin[0]
        elif stVal == 'Name':
            if len(varargin) > 0:
                if isinstance(varargin[0], str):
                    self.Name       =   varargin[0]
                self.ConSetFileParam(stVal, varargin[0], 'STRING');
        elif stVal == 'N':
            if len(varargin) > 0:
                Val         =   varargin[0]
                Val         =   floor(Val)
                Val         =   int(ceil(Val/8)*8)
                self.Rad_N  =   Val
                self.ConSetFileParam('N', Val, 'INT');
        elif stVal == 'Samples':
            if len(varargin) > 0:
                Val         =   varargin[0]
                Val         =   floor(Val)
                Val         =   int(ceil(Val/8)*8)
                self.Rad_N  =   Val
                self.ConSetFileParam('N', Val, 'INT');
        elif stVal  == 'CicDi':
            self.Rad_CicCfg_SampRed = 1
            self.Rad_CicCfg_RegCtrl = self.cCIC1_REG_CONTROL_BYPASS + self.cCIC1_REG_CONTROL_EN
            self.ConSetFileParam('CicEna', 'false', 'STRING')
            self.Setfs()
        elif stVal  == 'CicEna':
            self.Rad_CicCfg_SampRed = 1
            self.Rad_CicCfg_RegCtrl = self.cCIC1_REG_CONTROL_EN + self.cCIC1_REG_CONTROL_RSTCNTREOP  + self.cCIC1_REG_CONTROL_RSTFILTEOP
            self.ConSetFileParam('CicEna', 'true', 'STRING')
            self.Setfs()
        elif stVal == 'CicStages':
            if len(varargin) > 0:
                Val             =   floor(varargin[0] - 1)
                if Val > 3:
                    Val     = 3
                if Val < 0:
                   Val      = 0
                self.Rad_CicCfg_OutSel      =   Val
                self.ConSetFileParam('CicStages', Val, 'DOUBLE')
        elif stVal ==  'CicDelay':
            if len(varargin) > 0:
                Val         =   varargin[0]
                Val         =   floor(log2(Val)-1)
                if Val > 3:
                    Val     = 3
                if Val < 0:
                    Val     = 0
                self.Rad_CicCfg_CombDel = Val
                self.ConSetFileParam('CicDelay', Val, 'DOUBLE')
        elif stVal ==  'CicR':
            if len(varargin) > 0:
                Val         =   floor(varargin[0])
                if Val > 2**10:
                    Val     = 2**10
                if Val < 1:
                    Val     = 1
                self.Rad_CicCfg_SampRed = Val
                self.ConSetFileParam('CicR', Val, 'DOUBLE')
                self.Setfs()
        elif stVal ==  'DmaMult':
            if len(varargin) > 0:
                Val         =   floor(varargin[0])
                if Val > 2048:
                    Val     = 2048
                if Val < 1:
                    Val     = 1
                self.Rad_DmaMult     =   Val
        elif stVal == 'AfeFilt':
            if len(varargin) > 0:
                stFilt          =   varargin[0]
                if isinstance(stFilt, str):
                    if stFilt == '14MHz':
                        Val     =   self.Rad_Afe5801_RegFilter
                        Cmp     =   ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val     =   Val & Cmp
                        Val     =   Val + self.AFE5801_REG_FILTER_BW_14MHZ
                        self.Rad_Afe5801_RegFilter   =   Val
                    if stFilt == '10MHz':
                        Val     =   self.Rad_Afe5801_RegFilter
                        Cmp     =   ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val     =   Val & Cmp
                        Val     =   Val + self.AFE5801_REG_FILTER_BW_10MHZ
                        self.Rad_Afe5801_RegFilter   =   Val
                    if stFilt == '7.5MHz':
                        Val     =   self.Rad_Afe5801_RegFilter
                        Cmp     =   ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val     =   Val & Cmp
                        Val     =   Val + self.AFE5801_REG_FILTER_BW_7_5MHZ
                        self.Rad_Afe5801_RegFilter = Val
                    self.ConSetFileParam('AfeFilt', stFilt, 'STRING')
                else:
                    if stFilt > 12e6:
                        Val     =   self.Rad_Afe5801_RegFilter
                        Cmp     =   ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val     =   Val & Cmp
                        Val     =   Val + self.AFE5801_REG_FILTER_BW_14MHZ
                        self.Rad_Afe5801_RegFilter   =   Val
                    else:
                        if stFilt > 8.75e6:
                            Val     =   self.Rad_Afe5801_RegFilter
                            Cmp     =   ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                            Val     =   Val & Cmp
                            Val     =   Val + self.AFE5801_REG_FILTER_BW_10MHZ
                            self.Rad_Afe5801_RegFilter   =   Val
                        else:
                            Val     =   self.Rad_Afe5801_RegFilter
                            Cmp     =   ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                            Val     =   Val & Cmp
                            Val     =   Val + self.AFE5801_REG_FILTER_BW_7_5MHZ
                            self.Rad_Afe5801_RegFilter   =   Val
                    self.ConSetFileParam('AfeFilt', stFilt, 'DOUBLE')
            else:
                print('RadarLog Err: Set(AfeFilt) bandwidth not specified!')
        elif stVal  == 'AfeGaindB':
            if len(varargin) > 0:
                Val = varargin[0]
                self.SetAfe5801GainCoarse(Val);
                self.ConSetFileParam('AfeGaindB', Val, 'DOUBLE')
            else:
                print('RadarLog Err: Set(AfeGaindB) gain not specified!')
        elif stVal == 'AfeHighPass':
            if len(varargin) > 0:
                Val             =   varargin[0]
                self.SetAfe5801HighPass(Val)
            else:
                print('RadarLog Err: Set(AfeHighPass) K not specified!')
        elif stVal == 'AfeLowNoise':
            if len(varargin) > 0:
                stVal = varargin[0]
                if isinstance(stVal, str):
                    if stVal == 'On':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_VCA_LOW_NOISE
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_VCA_LOW_NOISE
                        self.Rad_Afe5801_RegFilter = Val
                    if stVal == 'Off':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_VCA_LOW_NOISE
                        Val = Val & Cmp
                        self.Rad_Afe5801_RegFilter = Val
                    self.ConSetFileParam('AfeLowNoise', stVal, 'STRING')
                else:
                    if stVal > 0:
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_VCA_LOW_NOISE
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_VCA_LOW_NOISE
                        self.Rad_Afe5801_RegFilter = Val
                    else:
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_VCA_LOW_NOISE
                        Val = Val & Cmp
                        self.Rad_Afe5801_RegFilter = Val
                    self.ConSetFileParam('AfeLowNoise', stVal, 'DOUBLE')
            else:
                print('RadarLog Err: Set(AfeLowNoise) no value specified!')

        elif stVal == 'AfeIntDcCoupling':
            if len(varargin) > 0:
                stVal = varargin[0]
                if isinstance(stVal, str):
                    if stVal == 'On':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING
                        self.Rad_Afe5801_RegFilter = Val
                    if stVal == 'Off':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING
                        Val = Val & Cmp
                        self.Rad_Afe5801_RegFilter = Val
                    self.ConSetFileParam('AfeIntDcCoupling', stVal, 'STRING')
                else:
                    if stVal > 0:
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING
                        self.Rad_Afe5801_RegFilter = Val
                    else:
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING
                        Val = Val & Cmp
                        self.Rad_Afe5801_RegFilter = Val
                    self.ConSetFileParam('AfeIntDcCoupling', stVal, 'DOUBLE')
            else:
                print('RadarLog Err: Set(AfeIntDcCoupling) no value specified!')
        elif stVal == 'AfePatRamp':
            if len(varargin) > 0:
                stVal = varargin[0]
                if isinstance(stVal, str):
                    if stVal == 'On':
                        self.Rad_Afe5801_RegLvds = self.AFE5801_REG_LVDS_MODE_PATTERN_RAMP
                        self.ConSetFileParam('AfePatRamp', 'On', 'STRING')
                    if stVal == 'Off':
                        self.Rad_Afe5801_RegLvds = self.AFE5801_REG_LVDS_MODE_NO_PATTERN
                        self.ConSetFileParam('AfePatRamp', 'Off')
                else:
                    if stVal > 0:
                        self.Rad_Afe5801_RegLvds = self.AFE5801_REG_LVDS_MODE_PATTERN_RAMP
                        self.ConSetFileParam('AfePatRamp', 'On')
                    else:
                        self.Rad_Afe5801_RegLvds = self.AFE5801_REG_LVDS_MODE_NO_PATTERN
                        self.ConSetFileParam('AfePatRamp', 'Off')
            else:
                print('RadarLog Err: Set(AfePatRamp) no value specified!')
        elif stVal == 'NrChn':
            if len(varargin) > 0:
                NrChn       =   floor(varargin[0])
                if NrChn < 1:
                    NrChn   =   1
                if NrChn > 16:
                    NrChn   =   16
                print('Set NrChn to: ', NrChn)
                if NrChn >= 8:
                    self.GpsFrmCtrl = 2 + 4
                elif NrChn >= 4:
                    self.GpsFrmCtrl = 2
                else:
                    self.GpsFrmCtrl = 0

                Mask  = 2**NrChn - 1
                self.Rad_ArmMemCfg_Chn = Mask
                self.Rad_MaskChnEna = Mask
                self.Rad_FrmCtrlCfg_ChnSel = Mask
                self.Rad_CicCfg_FiltSel = Mask
                self.Rad_NrChn = NrChn
                self.Computation.SetNrChn(NrChn)
                self.ConSetFileParam('NrChn', NrChn, 'INT')
        elif stVal == 'GpsRstFifo':

            if len(varargin) > 0:
                Val = floor(varargin[0])
                if Val > 0:
                    self.GpsRstFifoOnTimStamp = 8
                else:
                    self.GpsRstFifoOnTimStamp = 0

        elif stVal == 'FrmSiz':
            if len(varargin) > 0:
                self.Rad_FrmSiz     =   floor(varargin[0])
        else:
            print('Set: Parameter not known: ', stVal)


    def Get(self, stVal):
        #   @function       Get
        #   @author         Haderer Andreas (HaAn)
        #   @date           2015-07-27
        #   @brief          Get Parameters of Radarbook Baseboard
        #   @paramin[in]    stSelect: String to select parameter to change
        #
        #                   Known stSelect Parameter Strings:
        #                       DebugInf:       Debug Level (>10 Debug Information is output)
        #                       Name:           Name of Board
        #                       N:              Number of Samples
        #                       CicStages:      Number of stages of CIC filter
        #                       CicDelay:       Comb delay of CIC filter
        #                       CicR:           Sample rate reduction values of CIC filter
        #                       ClkDiv:         AD9512 Clock Divider
        #                       NrFrms:         Number of frames to sample
        #                       fAdc:           AD8283 Sampling Clock
        #                       fs:             Sampling frequency after CIC filter
        #                       AdcChn:         Number of enabled ADC Channels for a single AD8283
        #                       AdcImp:         Adc Impedance
        #                       NMult:          Multi frame processing
        #   @return         Requested value

        if isinstance(stVal, str):
            if stVal == 'DebugInf':
                Ret         =   self.cDebugInf
            elif stVal == 'Name':
                Ret         =   self.Name
            elif stVal == 'N':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'INT')
                    self.Rad_N = Ret
                    self.Rad_NTx = Ret
                else:
                    Ret         =   self.Rad_N
            elif stVal == 'Samples':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'INT')
                    self.Rad_N  = Ret
                    self.Rad_NTx = Ret
                else:
                    Ret         =   self.Rad_N
            elif stVal == 'CicStages':
                Ret         =   1
            elif stVal == 'CicDelay':
                Ret         =   2**(self.Rad_CicCfg_CombDel + 1)
            elif stVal == 'CicR':
                Ret         =   self.Rad_CicCfg_SampRed
            elif stVal == 'fAdc':
                Ret         =   self.Rad_fAdc
            elif stVal == 'fs':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal,'DOUBLE')
                else:
                    Ret = self.Rad_fAdc/self.Rad_CicCfg_SampRed
            elif stVal == 'NrChn':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'INT')
                    self.Computation.SetNrChn(Ret)
                    self.Rad_NrChn  = Ret;
                else:
                    Ret         =   self.Rad_NrChn
            elif stVal == 'AfeGaindB':
                Ret         =   self.GaindB;
            elif stVal == 'AdcGaindB':
                Ret         =   self.GaindB;
            elif stVal == 'AfeIntDcCoupling':
                Val = uint32(self.Rad_Afe5801_RegFilter)
                Cmp = uint32(self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING)
                Val = (Val & Cmp)
                if Val > 0:
                    Ret = 1
                else:
                    Ret = 0
            elif stVal == 'FuSca':
                Ret         =   self.FuSca
            elif stVal == 'FileSize':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'DOUBLE')
                else:
                    Ret = 0;
            elif stVal == 'MeasStart':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'DOUBLE')
                else:
                    Ret = 0;
            elif stVal == 'ExtensionSize':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'DOUBLE')
                else:
                    Ret = 0;
            else:
                print('Get:: Parameter not known: ', stVal)
                Ret         =   -1

        return Ret



    def Setfs (self):
        R = self.Rad_CicCfg_SampRed
        self.Computation.SetParam('fs', self.Rad_fAdc/R)

    def SetFuSca (self):
        self.Computation.SetParam('FuSca', self.FuSca)

    # DOXYGEN ------------------------------------------------------
    #> @brief Prnt attribute of class object in Matlab command window
    #>
    #> Prints the selected attribut in the command window of Matlab
    #>
    #> @param[in]   stSel: String to select attribute
    #>
    #> Supported parameters
    #>      -   <span style="color: #ff9900;"> 'Name': </span>Name of board <br>
    #>                          string containing desired name of board
    #>      -   <span style="color: #ff9900;"> 'N': </span>Number of samples <br>
    #>                          Value with number of samples
    #>                          Value containing number of frames
    #>      -   <span style="color: #ff9900;"> 'fAdc': </span> Clock frequency of the AD8283 <br>
    #>                          Value containing the actual clock frequency in Hz. This parameter is defined by the ClkDiv value.
    #>                          fAdc = 80e6/ClkDiv
    #>      -   <span style="color: #ff9900;"> 'fs': </span> Sampling rate for a single channel <br>
    #>                          Sampling frequency for a single channel. This value depends on the number of ADC channels, the clock divider and the
    #>                          sampling rate reduction
    #>                          fAdc = 80e6/ClkDiv; fs = fAdc/(CicR*NrAdcChn)
    #>
    #> e.g. Print sampling frequency
    #>   @code
    #>      Brd =   Radarbook('PNet','192.168.1.1')
    #>
    #>      Brd.Prnt('fs')
    #>   @endcode
    def Prnt(self,*varargin):
        if len(varargin) > 0:
            stVal       = varargin[0]
            if stVal == 'Name':
                print(' Prnt: Name: ', self.stName)
            elif stVal == 'N':
                print(' Prnt: N: ', self.Rad_N ,' Samples')
            elif stVal == 'Samples':
                print(' Prnt: N: ', self.Rad_N,' Samples')
            elif stVal == 'fAdc':
                print(' Prnt: fAdc: ', self.Rad_fAdc/1e6 , ' MHz')
            elif stVal == 'fs':
                R       =   self.Rad_CicCfg_SampRed
                fs      =   self.Rad_fAdc/R
                print(' Prnt: fs: ', (fs/1e3), ' kHz')
            else:
                print('Parameter ', stVal, ' not known')
        else:
            print('Parameter ', stVal, ' not known')

    # DOXYGEN ------------------------------------------------------
    #> @brief Display configuration of CIC filters in Matlab command window
    #>
    #> Display actual configuration in Matlab window
    #>
    #>
    #> e.g. Set sampling rate reduction to 24 and display config afterwards
    #>   @code
    #>      Brd =   Radarbook('PNet','192.168.1.1')
    #>
    #>      Brd.Set('CicR',24)
    #>      Brd.DispCic()
    #>   @endcode
    #>   After calling DispCic the configuration is plotted in the Matlab command windo
    #>   @code
    #>      RBK: CIC enabled
    #>          Delay:   2
    #>          Stages:  1
    #>          R:       24
    #>   @endcode
    def     DispCic(self):
        if floor(self.Rad_CicCfg_RegCtrl/self.CIC1_REG_CONTROL_BYPASS) > 0:
            print(self.stName, ': CIC disabled')
        else:
            print(self.stName, ': CIC enabled')
        print('  Delay:   ', (2**(self.Rad_CicCfg_CombDel+1)))
        print('  Stages:  ', (self.Rad_CicCfg_OutSel+1))
        print('  R:       ', (self.Rad_CicCfg_SampRed))

    # DOXYGEN ------------------------------------------------------
    #> @brief Access Measurement Data
    #%>
    #> This function is used to configure the interface for measurement data transfer
    #> The following functions are performed by the method:
    #>      - Reset SRAM Fifo: old data is removed
    #>      - Configure ARM or USB board for data transfer
    #>
    #> @node The function behaves different for USB and TCP/IP connections.
    #>
    def BrdAccessData(self):

        if (int(self.cDebugInf) & 8) > 0:
            print("Access Data")


        # Calculate Number of Samples to
        Cmp2                =   int(self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA + self.cFRMCTRL_REG_CH0CTRL_SOFTID)
        Cmp1                =   int(self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA)

        if (int(self.Rad_FrmCtrlCfg_RegChnCtrl) & Cmp2) == Cmp2:
            Samp    =   self.Rad_N - 2
        elif (int(self.Rad_FrmCtrlCfg_RegChnCtrl) & Cmp1) == Cmp1:
            Samp    =   self.Rad_N - 1
        else:
            Cmp1    =   self.cFRMCTRL_REG_CH0CTRL_SOFTID
            if (int(self.Rad_FrmCtrlCfg_RegChnCtrl) & Cmp1) == Cmp1:
                Samp    =   self.Rad_N - 1
            else:
                Samp    =   self.Rad_N

        if (int(self.Rad_FrmCtrlCfg_RegChnCtrl) & self.cFRMCTRL_REG_CH0CTRL_PADENA) == self.cFRMCTRL_REG_CH0CTRL_PADENA:
            print("Pad")
            Samp        =   Samp - floor(self.Rad_FrmCtrlCfg_RegChnCtrl/2**16) - 1

        dSampCfg            =   {
                                    "Nr"        :   Samp,
                                    "Rst"       :   1
                                }

        self.Fpga_SetSamp(dSampCfg)

        #--------------------------------------------------------------
        # Configure FArmMem Channels
        #--------------------------------------------------------------
        dArmMemCfg          =   {
                                    "Chn"            :   self.Rad_ArmMemCfg_Chn,                # Channel select Mask
                                    "Ctrl"           :   self.Rad_ArmMemCfg_Ctrl,               # Channel control register
                                    "FrmSiz"         :   self.Rad_N*self.Rad_NMult,
                                    "WaitLev"        :   self.Rad_ArmMemCfg_FifoSiz - self.Rad_N*self.Rad_NMult     # Frame wait level
                                }
        #--------------------------------------------------------------
        # Configure Frame Control
        #--------------------------------------------------------------
        dFrmCtrlCfg         =   {
                                    "ChnSel"              :   self.Rad_FrmCtrlCfg_ChnSel,                     # Select Channels to be configured
                                    "Rst"                 :   self.Rad_FrmCtrlCfg_Rst,                        # Reset MMP
                                    "RegChnCtrl"          :   self.Rad_FrmCtrlCfg_RegChnCtrl                  # Channel control register
                                }



        #----------------------------------------------------------
        # Start Sampling: Reset framecounter, and start sequence
        #----------------------------------------------------------
        self.Fpga_SetFrmCtrl(dFrmCtrlCfg)
        self.Fpga_CfgArmMem(dArmMemCfg)

    # DOXYGEN ------------------------------------------------------
    #> @brief Set Calibration Information
    #>
    #> This function writes the calibartion data (configuration and data) to the FPGA. The information is not stored in the EEPROM of the frontend.
    #>
    #> @param[in]   Cfg: structure containing the calibration data
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'RevNr': </span> Revision number of the calibration data <br>
    #>          Uint32 containing revision number
    #>      -   <span style="color: #ff9900;"> 'DateNr': </span> Date number of calibration <br>
    #>          Uint32 containing date number of calibration e.g. floor(now())
    #>      -   <span style="color: #ff9900;"> 'Len': </span> Number of entries in calibration table <br>
    #>          Uint32 containing the length of the calibration table
    #>      -   <span style="color: #ff9900;"> 'Data': </span> Array containing the calibration coefficients <br>
    #>          Array containing the calibration data
    #>
    #> e.g. Setup the configuration structure CalCfg and write the data to the FPGA framework.
    #>      In the current example a linear ramp is programmed to the device.
    #>  @code
    #>      %--------------------------------------------------------------------------
    #>      % Write Calibration Data
    #>      %--------------------------------------------------------------------------
    #>      CalCfg.Mask         =   1;
    #>      CalCfg.RevNr        =   2;
    #>      CalCfg.DateNr       =   floor(now);
    #>      CalCfg.Len          =   16;
    #>      CalCfg.Data         =   Brd.Num2Cal([0:63].');
    #>
    #>      Brd.BrdSetCal(CalCfg);
    #>  @endcode
    def BrdSetCal(self, dCfg):
        self.Fpga_SetCalCfg(dCfg)
        self.Fpga_SetCalData(dCfg)

    # DOXYGEN ------------------------------------------------------
    #> @brief Get Calibration Data
    #>
    #> This function reads the calibration data from the FPGA framework.
    #>
    #> @param[in]   Cfg: structure containing the calibration data
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Len': </span> Number of entries in calibration table <br>
    #>          Uint32 containing the length of the calibration table number of entries in the array. To read 16 complex values the length is 32.
    #> @return Array with complex elements containing the calibration data. If the Len fiels is set to 32, then the array contains 16 complex values.
    #>
    #> e.g. Get configuration data for 8 channels. Set length field to 16
    #>
    #>  @code
    #>      %--------------------------------------------------------------------------
    #>      % Write Calibration Data
    #>      %--------------------------------------------------------------------------
    #>      CalCfg.Mask         =   1;
    #>      CalCfg.Len          =   16;
    #>
    #>      Brd.BrdGetCal(CalCfg);
    #>  @endcode
    #>
    #>  @note The calibration data is stored in an EEPROM on the frontend. It is not required to enable the RF power supply in order to read the cal information.
    def BrdGetCalData(self, dCfg):
        #   @function       BrdGetCalData
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-28
        #   @brief          Get Calibration Data
        Ret             =   self.Fpga_GetCalData(dCfg)
        Ret             =   self.Cal2Num(Ret)
        dCfg["Type"]    =   0
        if dCfg["Type"] == 0:
            CplxData    =   Ret[::2] + 1j*Ret[1::2]
        else:
            CplxData    =   -1

        self.Computation.SetParam('CalRe', real(CplxData[0:int(self.Rad_NrChn)]));
        self.Computation.SetParam('CalIm', imag(CplxData[0:int(self.Rad_NrChn)]));
        return CplxData

    # DOXYGEN ------------------------------------------------------
    #> @brief Reset the board
    #>
    #> This function is used to reset the board. The reset function resets the timing unit SeqTrig and clears the entries in the sequencer.
    #>
    #> @note At the begin of a script the BrdRst function should be called in order to disable measurements.
    def BrdRst(self):
        self.Fpga_SeqTrigRst(self.SeqCfg["Mask"])
        Ret     =   self.Fpga_MimoSeqRst()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Set Role of Board (Master, Slave, )
    #>
    #> This function is used to reset the board. The reset function resets the timing unit SeqTrig and clears the entries in the sequencer.
    #>
    #> @note At the begin of a script the BrdRst function should be called in order to disable measurements.
    def     BrdSetRole(self, Role, AdcFreq, *varargin):

        if isinstance(Role, str):
            if (Role == "Ms"):
                Sel = 0
            elif (Role == "MsClkOut"):
                Sel = 1
            elif (Role == "Sl"):
                Sel = 128
            elif (Role == "SlClkIn"):
                Sel = 129
            elif (Role == "IntGps"):
                Sel = 130
            else:
                Sel     =   0
        else:
            Sel = int(Role)

        if len(varargin) > 0:
            AdcSel = varargin[0]
            self.Rad_fAdc = 40e6
        else:
            if AdcFreq > 30e6:
                AdcSel = 1
                self.Rad_fAdc = 40e6
            else:
                AdcSel = 0
                self.Rad_fAdc = 20e6

        self.Rad_Role = Sel
        self.Setfs()
        self.Fpga_SetRole(Sel, AdcSel)


    def BrdGetClkSts(self):
        return self.Fpga_GetBrdClkSts(0)

    def BrdDispClkSts(self):

        Ret =  self.Fpga_GetBrdSts(0)
        print('Clk/Trigger Status')
        print('  BrdRole: ', self.Rad_Role)
        print('  ClkFreq: ', (Ret[2]/1e3), ' kHz')
        print('  RfTrigCntr: ', Ret[3], ' Events')
        print('  RfTrigWidth: ', Ret[4], ' Cycles')
        print('  ExtTrigCntr: ', Ret[5], ' Events')


    # DOXYGEN ------------------------------------------------------
    #> @brief Enable RF power supply (enable all three supplies)
    #>
    #> This function enables the internal (switching regulators) and the RF linear regulators for the supply of the frontend.
    #> The EduRad has no internal power supply that can be switched on
    #> or off. The field is ignored in the Linux driver. Only the RF
    #> is required for switching the power supply on and off
    #>
    def BrdPwrEna(self):
        dPwrCfg     =   {   "IntEna"        : 0,
                            "RfEna"         : self.cPWR_RF_ON
                        }
        Ret         =   self.Fpga_SetRfPwr(dPwrCfg)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Disable RF power supply (diable all three supplies)
    #>
    #> This function disables the supply of the frontend.
    #>
    def BrdPwrDi(self):
        #   @function       BrdPwrDi
        #   @author         Haderer Andreas (HaAn)
        #   @date           2015-07-27
        #   @brief          Disable all power supplies for RF frontend
        dPwrCfg     =   {   "IntEna"        : 0,
                            "RfEna"         : self.cPWR_RF_OFF
                        }
        Ret         =   self.Fpga_SetRfPwr(dPwrCfg)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Initialize Sampling
    #>
    #> This function configures the sampling chain in the FPGA framework. The following tasks are performed.
    #>      - Configure ADC
    #>      - Configure Framecontrol MMP
    #>      - Configure CIC filters
    #>      - Enable processing
    #>
    #>
    #>  @note To change the settings for the ADC and CIC filters the Set function can be used before calling BrdSampIni.
    #>
    def BrdSampIni(self):
        if (int(self.cDebugInf) & 8) > 0:
            print('BrdSampIni')

        dFrmCtrlCfg                 =   {
                                            "ChnSel"    :   0
                                        }
        dFrmCtrlCfg["ChnSel"]       =   self.Rad_FrmCtrlCfg_ChnSel                      # Select Channels to be configured
        dFrmCtrlCfg["Rst"]          =   self.Rad_FrmCtrlCfg_Rst                         # Reset MMP
        dFrmCtrlCfg["RegChnCtrl"]   =   self.Rad_FrmCtrlCfg_RegChnCtrl                  # Channel control register


        dCicCfg                     =   {
                                            "FiltSel"    :   0
                                        }
        dCicCfg["FiltSel"]          =   self.Rad_CicCfg_FiltSel                         # Select Cic filters
        dCicCfg["CombDel"]          =   self.Rad_CicCfg_CombDel                         # Comb delay
        dCicCfg["OutSel"]           =   self.Rad_CicCfg_OutSel                          # Output select
        dCicCfg["SampPhs"]          =   self.Rad_CicCfg_SampPhs                         # Sampling phase
        dCicCfg["SampRed"]          =   self.Rad_CicCfg_SampRed                         # Sampling rate reduction
        dCicCfg["RegCtrl"]          =   self.Rad_CicCfg_RegCtrl                         # Cic filter control register

        self.Fpga_SetAfe5801()                                                          # Adc and Clock control
        self.Fpga_SetFrmCtrl(dFrmCtrlCfg)                                               # Data transmission control
        self.Fpga_SetCic(dCicCfg)                                                       # Cic filters

    # DOXYGEN ------------------------------------------------------
    #> @brief Display board information
    #>
    #> This function prints the actual status of the board to the Matlab command window. The board monitors the
    #> temperature near the ADC and the input power supply circuit. In addition, the supply voltage and supply current are shown.
    #>
    #>   After calling BrdDispInf the configuration is plotted in the Matlab command window
    #>   @code
    #>      -------------------------------------------------------------------
    #>      Board Information
    #>      Temp1 (AD8283)    = 62.1473°
    #>      Temp2 (Sup)       = 56.4602°
    #>      VSup (Sup)        = 16.4193V
    #>      ISup (Sup)        = 0.74766A
    #>      -------------------------------------------------------------------
    #>   @endcode
    #>
    #>   @node The supply current is only monitored correct, if the RF supply is enabled.
    def BrdDispInf(self):
        Ret     =   self.Fpga_DispBrdInf()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Get Status of RF frontend
    #>
    #> All RF frontends are equipped with a EEPROM device, to store the board information and the calibration data
    #> The BrdGetRfSts function reads the board information of the RF frontend
    #>
    #> @return Structure with board information. The structure contains the following fields
    #>      -   <span style="color: #9966ff;"> 'RfUid': </span> <br>
    #>          Unique identifier of RF frontend design
    #>      -   <span style="color: #9966ff;"> 'RfRevNr': </span> <br>
    #>          Revision number of the frontend; number format: Major*2**16 + Minor*2**8 + Patch
    #>      -   <span style="color: #9966ff;"> 'RfSerial': </span> <br>
    #>          Serial number of the frontend
    #>      -   <span style="color: #9966ff;"> 'RfDate': </span> <br>
    #>          Date of configuration; generated with now form Matlab. Use datestr to convert to a string with the date.
    #>      -   <span style="color: #9966ff;"> 'RfStrtup': </span> <br>
    #>          Startup counter: Counter is incremented after the FPGA boots. An enable and disable of the RF supply does not increment the counter.
    #>
    def     BrdGetRfSts(self):
        Val = self.Fpga_GetRfChipSts(1)
        dSts = dict()
        if len(Val) > 2:
            dSts["RfUid"] = Val[0]
            dSts["RfRevNr"] = Val[1]
            dSts["RfSerial"] = Val[2]
            dSts["RfDate"] = Val[3]
            dSts["RfStrtup"] = Val[4]
        return dSts

    # DOXYGEN ------------------------------------------------------
    #> @brief Disp status of RF frontend
    #>
    #> All RF frontends are equipped with a EEPROM device, to store the board information and the calibration data
    #> The BrdDispRfSts function prints the status of the board to the command window. If the ID of the frontend is knwon, the chip configuration is also displayed.
    #> Most RF chips have an unique identification number. This number is printed in the command window.
    #>
    #>   After calling BrdDispRfSts the configuration is plotted in the Matlab command window.
    #>   In case of the MIMO77 frontend the status of the chips is displayed too.
    #>   @code
    #>      -------------------------------------------------------------------
    #>      RF Board Configuration
    #>      MIMO-77-TX4RX8
    #>      RF UID            = 64
    #>      RF RevNr          = 1.1.0
    #>      RF SerialNr       = 64000E
    #>      RF Date           = 07-Jan-2016
    #>      RF Startup        = 7
    #>      --------------------
    #>      RCC               = 40091739537F
    #>      DPA1              = 1067295654D8
    #>      DPA2              = 1067284754D8
    #>      DPA3              = 1067253254D8
    #>      MRX1              = 1000000200
    #>      MRX2              = 1000000200
    #>      -------------------------------------------------------------------
    #>   @endcode
    #>   @note To display the ID of the RF chips the RF supply must be enabled (BrdPwrEna())
    def BrdDispRfSts(self):
        Ret         =   self.Fpga_DispRfChipSts(1)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Set pre-defined timing patterns
    #>
    #> This method is used to select a predefined timing pattern for the SeqTrig unit.
    #>
    #>  @param[in] stSel: string to select timing patter
    #>      -   <span style="color: #9966ff;"> 'ContUnif' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'ContUnifMx' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RccSegUnifM1' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RccSegUnifMx' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RccSegUnifM1_RccMs' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RccSegUnifMTx' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RccSegUnifMTxPaCon' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'SampWaitM1' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RccWaitM1' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'WaitM1' </span> <br>
    #>      -   <span style="color: #9966ff;"> 'RDWaitM1' </span> <br>
    #>   @note To display the ID of the RF chips the RF supply must be enabled (BrdPwrEna())
    def BrdSetTimCont(self, stSel, *varargin):
        fSeqTrig                =   self.Rad_fAdc

        if stSel == 'ContUnifM1':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 0:
                self.SeqCfg     =   objSeqTrig.ContUnifM1(varargin[0])
                Ini             =   1;
        elif stSel == 'ContUnifM1PWPT':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 2:
                self.SeqCfg     =   objSeqTrig.ContUnifM1PWPT(varargin[0],varargin[1], varargin[2])
                Ini             =   1;
        elif stSel == 'ContUnifMx':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 2:
                print('Set ContUnifMx')
                self.SeqCfg     =   objSeqTrig.ContUnifMx(varargin[0], varargin[1], varargin[2])
                Ini             =   1
            else:
                Ini             =   0
                print('Duration must be specified')
        elif stSel == 'RccSegUnifM1':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 1:
                self.SeqCfg     =   objSeqTrig.RccSeqContUnifM1(varargin[0], varargin[1])
                Ini             =   1;
            else:
                Ini             =   0;
                print('Parameters must be specified')
        elif stSel == 'RccSegUnifM1PW':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 2:
                self.SeqCfg     =   objSeqTrig.RccSeqContUnifM1PW(varargin[0], varargin[1], varargin[2])
                Ini             =   1;
            else:
                Ini             =   0;
                print('Parameters must be specified')
        elif stSel == 'RccSegUnifMx':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 3:
                # RccSeqContUnifMx(self, Tp, TCfg, TWait, Np):
                self.SeqCfg     =   objSeqTrig.RccSeqContUnifMx(varargin[0], varargin[1], varargin[2], varargin[3])
                Ini             =   1;
            else:
                Ini             =   0;
                print('Parameters must be specified')
        elif stSel == 'RccSegUnifMxPW':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 4:
                # RccSeqContUnifMx(self, Tp, TCfg, TWait, Np):
                self.SeqCfg     =   objSeqTrig.RccSeqContUnifMxPW(varargin[0], varargin[1], varargin[2], varargin[3], varargin[4])
                Ini             =   1;
            else:
                Ini             =   0;
                print('Parameters must be specified')
        elif stSel == 'RccSegUnifMTxPaCon':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig)
            if len(varargin) > 3:
                self.SeqCfg     =   objSeqTrig.RccSeqContUnifMTxPaCon(varargin[0], varargin[1], varargin[2], varargin[3])
                Ini             =   1
            else:
                Ini             =   0
                print('Parameters must be specified')
        elif stSel == 'RccSegUnifM1_RccMs':
            objSeqTrig          =   SeqTrig.SeqTrig(fSeqTrig);
            if len(varargin) > 1:
                self.SeqCfg     =   objSeqTrig.RccSeqContUnifM1_RccMs(varargin[0], varargin[1]);
                Ini             =   1
            else:
                Ini             =   0
                print('Parameters must be specified')
        else:
            Ini             =   0
            print('Duration must be specified')

        if Ini > 0:
            self.Fpga_SeqTrigRst(self.SeqCfg["Mask"]);
            NSeq        =   len(self.SeqCfg["Seq"]);
            if self.cDebugInf > 0:
                print('Run SeqTrig: ', NSeq, ' Entries')
            Tmp     =   self.SeqCfg["Seq"]
            for Idx in range (0, int(NSeq)):
                self.Fpga_SeqTrigCfgSeq(self.SeqCfg["Mask"], Idx, self.SeqCfg["Seq"][Idx])

    # DOXYGEN ------------------------------------------------------
    #> @brief Display software version of FPGA framework
    #>
    #> This function prints the software version of the FPGA framework in the command window of Matlab
    #>
    #>  After calling the function the following text is plotted:
    #>  @code
    #>      -------------------------------------------------------------------
    #>      FPGA Software UID
    #>      Sw-Rev: 4-0-3 | Sw-UID: 20 | Hw-UID: 12
    #>      -------------------------------------------------------------------
    #>  @endcode
    #>  The software identification number (Sw-UID) refers to the FPGA framework and the hardware identification number
    #>  refers to the type of baseband.
    def     BrdDispSwVers(self):
        #   @function       BrdDispSwVers
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Disp software version of the board
        self.Fpga_DispSwCfg()

    # DOXYGEN ------------------------------------------------------
    #> @brief Get software version of FPGA framework
    #>
    #> This function returns a struct with the software version of the FPGA framework
    #>
    #>  @return Software Version: the structure contains the following fields
    #>      -   <span style="color: #9966ff;"> 'HUid': </span> <br>
    #>          Unique identifier of baseband board
    #>      -   <span style="color: #9966ff;"> 'SUid': </span> <br>
    #>          Unique identifier of the FPGA software framework
    #>      -   <span style="color: #9966ff;"> 'SwMaj': </span> <br>
    #>          Major software version
    #>      -   <span style="color: #9966ff;"> 'SwMin': </span> <br>
    #>          Minor software version
    #>      -   <span style="color: #9966ff;"> 'SwPatch': </span> <br>
    #>          Patch software version
    #>
    def     BrdGetSwVers(self):
        Vers        =   self.Fpga_GetSwVers()
        dRet        =   {
                            "SwPatch"   :   -1,
                            "SwMin"     :   -1,
                            "SwMaj"     :   -1,
                            "SUid"      :   -1,
                            "HUid"      :   -1
                        }
        if len(Vers) > 2:
            dRet["SwPatch"]     =   Vers[0] % (2**8)
            TmpVal              =   floor(Vers[0]/(2**8))
            dRet["SwMin"]       =   TmpVal % (2**8)
            dRet["SwMaj"]       =   floor(TmpVal/(2**8))
            dRet["SUid"]        =   Vers[1]
            dRet["HUid"]        =   Vers[2]
        else:
            print('No version information available');

        return dRet

    def     BrdGetTemp(self):
        Ret = self.Fpga_GetTemp()
        if Ret[0] == True:
            Data = Ret[1][2]
            return Data-273
        else:
            return 0

    def     Fpga_GetTemp(self):
        FpgaCmd    = zeros(1, dtype='uint32')
        Cod        = int('0x9013', 0)
        FpgaCmd[0] = 0
        Ret        = self.CmdSend(0, Cod, FpgaCmd)
        Ret        = self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Display calibration information
    #>
    #> This function print the calibration information to the command window of Matlab
    #>
    #> After calling the method the following output results in the command window:
    #>  @code
    #>      -------------------------------------------------------------------
    #>      Calibration Table Information
    #>      Type              = 0
    #>      Date              = 07-Jan-2016
    #>      Rev               = 0.0.1
    #>      -------------------------------------------------------------------
    #>  @endcode
    #>
    def BrdDispCalInf(self, *varargin):
        if len(varargin) > 0:
            dCfg        =   varargin[0]
        else:
            dCfg        =   {
                                "Mask"    :   1
                            }
        print(' ')
        print('-------------------------------------------------------------------')
        print('Calibration Table Information')
        Val     =   self.Fpga_GetCalCfg(dCfg)
        if len(Val) > 2:
            print('Type              = ',Val[0])
            print('Date              = ',Val[1])
            print('Rev               = ',self.RevStr(Val[2]))
        print('-------------------------------------------------------------------');

    # DOXYGEN ------------------------------------------------------
    #> @brief Start sampling
    #>
    #> This function is used to start the sampling on the FPGA board (i.e. the timing unit SeqTrig is started)
    #> The method first loads the timing unit with, the programmed timing and afterwards a trigger signal is applied.
    #> The trigger signal is only required for timing patterns that wait for a trigger signal. If the timing pattern does
    #> not require a trigger, then genertion of the trigger has no effect.
    def     BrdSampStrt(self):
        self.Fpga_SeqTrigLoad(self.SeqCfg["Mask"], self.SeqCfg["Ctrl"], self.SeqCfg["ChnEna"])
        self.Fpga_SeqTrigEve(self.SeqCfg["Mask"])

    def     BrdSampStrtDelay(self, Delay):
        self.Fpga_SeqTrigLoad(self.SeqCfg["Mask"], self.SeqCfg["Ctrl"], self.SeqCfg["ChnEna"])
        time.sleep(Delay)
        self.Fpga_SeqTrigEve(self.SeqCfg["Mask"])

    # DOXYGEN ------------------------------------------------------
    #> @brief Generate trigger event for timing unit
    #>
    #> This method generates a trigger signal for the timing unit in the FPGA (SeqTrig)
    def     BrdSampTrig(self):
        self.Fpga_SeqTrigEve(self.SeqCfg["Mask"]);

    # DOXYGEN ------------------------------------------------------
    #> @brief Read measurement data
    #>
    #> After starting the measurements, the measurement data can be accessed with this function. The function returns a two-dimensional
    #> array of size N x NrChn. After reset the number of channels is set to 8. The Set('NrChn') method can be used to change the enabled input channels.
    #>
    #> @param[in] (opt) NPack: Read NPack samples simultaneously. In this case the returned array is of size (N*NPack) x NrChn
    #>
    #> @return Two-dimensional array containing the measurement data.
    #>          - Col1:     samples of channel 1
    #>          - Col2:     samples of channel 2
    #>          ...
    def BrdGetData(self,  NrPack=1, Extension=False):
        if self.cType == 'PNet':
            Data                =   zeros((int(self.Rad_N*NrPack*self.Rad_NMult), int(self.Rad_NrChn)))
            for Idx in range(0,int(NrPack)):
                ArmData             =   self.Fpga_GetData()
                DataSiz             =   ArmData.shape
                if (DataSiz[0] == self.Rad_N*self.Rad_NMult) and (DataSiz[1] == self.Rad_NrChn):
                    if len(ArmData) > 1:
                        FrmCntr         =   ArmData[0,:]
                        ArmData         =   ArmData/16
                        ArmData[0,:]    =   FrmCntr

                    Data[int((Idx)*self.Rad_N*self.Rad_NMult):int((Idx+1)*self.Rad_N*self.Rad_NMult),:]    =   ArmData
        elif self.cType == 'RadServe':
            #----------------------------------------------------------
            # Open Data Port
            #----------------------------------------------------------
            if self.hConDat < 0:
                if not Extension:
                    self.GetDataPort()
                    self.cDataIdx = 0;
                else:
                    self.GetExtensionPort(Extension)
                    self.cDataIdx = 0;
                if self.cDataOpened < 0:
                    quit();

            if (not Extension) and self.Computation.GetDataType() <= 1 and (self.cReplayExt < 1):
                UsbData             =   self.ConGetData(NrPack*self.Rad_NrChn*self.Rad_N*2)

                UsbData             =   double(UsbData);
                Data                =   zeros((int(self.Rad_N)*NrPack,int(self.Rad_NrChn)))
                for Idx in range(0, NrPack):
                    IdxStrt         =   int((Idx)*self.Rad_NrChn*self.Rad_N)
                    IdxStop         =   int((Idx+1)*self.Rad_NrChn*self.Rad_N)
                    Data1           =   UsbData[IdxStrt:IdxStop]
                    Data1           =   Data1.reshape((int(self.Rad_NrChn),int(self.Rad_N))).transpose()
                    Data1[1:,:]     =   Data1[1:,:]/16
                    Data[(Idx)*int(self.Rad_N):(Idx+1)*int(self.Rad_N),:]       =   Data1
                self.cDataIdx = self.cDataIdx + NrPack;
            elif not Extension:
                self.cDataIdx = self.cDataIdx + NrPack * self.cExtMult;

                return self.Computation.GetData(NrPack);
            else:
                Data                =   self.ConGetData(NrPack * self.cExtSize * 2);
                self.cDataIdx       =   self.cDataIdx + self.cExtMult * NrPack;

        return Data

    def     EnaGps(self, stSel):
        self.Gps = 1;
        if stSel == 'UBX':
            # Enable 3 = UBX Protocoll with Tim and Pvt enabled
            self.SetUBloxMsgMod(3)
            self.SetUBloxMsgCtrl(self.GpsFrmCtrl + self.GpsRstFifoOnTimStamp)
        else:
            pass


    def     DiGps(self):
        self.Gps = 0

    def     SetUBloxMsgMod(self, Sel):
        Cod         =   int('0x9510',0)
        FpgaCmd     =   zeros(2, dtype='uint32')
        FpgaCmd[0]  =   1
        FpgaCmd[1]  =   Sel
        self.CmdSend(0,Cod,FpgaCmd)
        return self.CmdRecv()

    def     SetUBloxMsgCtrl(self, Reg):
        Cod         =   int('0x9510',0)
        FpgaCmd     =   zeros(3, dtype='uint32')
        FpgaCmd[0]  =   1
        FpgaCmd[1]  =   5
        FpgaCmd[2]  =   Reg
        self.CmdSend(0,Cod,FpgaCmd)
        return self.CmdRecv()


    def     DispUbxTim(self, lMsg):
        for dMsg in lMsg:
            if 'stName' in dMsg:
                if dMsg['stName'] == 'Ubx-Nav-Pvt':
                    print('UbxTim @Frm ', (dMsg['FrmCntr']),' : ', int(dMsg['year']),'-',int(dMsg['month']),'-',int(dMsg['day']),' ',int(dMsg['hour']),':',int(dMsg['min']),':',int(dMsg['sec']))

    def     ParseUBloxUbx(self, Data):

        Eval = 1

        lMsg = list()
        MsgIdx = 0

        if self.Rad_NrChn >= 8:
            NrChn = 8
        else:
            NrChn = self.Rad_NrChn


        if self.Gps > 0 and Eval > 0:

            GpsDat = Data[int(self.Rad_N-1)::int(self.Rad_N),0:int(NrChn)]*16
            GpsDat = GpsDat.flatten()
            GpsChn = floor(GpsDat/256)
            IdcsFrmCntr = argwhere(GpsChn == 1)
            FrmDat = double(GpsDat[IdcsFrmCntr]  % 256)

            Idcs = argwhere(GpsChn == 2)
            self.GpsCntr = self.GpsCntr + len(Idcs)
            GpsDat = (GpsDat[Idcs] % 256)

            Dat = uint8(GpsDat.flatten())
            FrmDat = FrmDat.flatten()
            self.GpsFrmDat = concatenate((self.GpsFrmDat, FrmDat))
            if len(self.GpsFrmDat) == 2:
                self.GpsFrmCntr = self.GpsFrmDat[1]*2**8 + self.GpsFrmDat[0]
                if self.GpsFrmCntr > 2**15:
                    self.GpsFrmCntr = self.GpsFrmCntr - 2**16
                #print("Frm found: ", self.GpsFrmCntr)
                self.GpsFrmDat = [];
            else:
                if len(IdcsFrmCntr) == 1:
                    self.GpsFrmDat = FrmDat
                else:
                    self.GpsFrmDat = []

            Dat = uint8(concatenate((self.GpsDat, Dat)))
            self.GpsDat = Dat

            if len(self.GpsDat) > 1000:
                self.GpsDat = uint8([])
                disp('Clear')

            if len(Dat) > 0:


                dUbxNavPvt = dict()
                dUbxNavPvt['stName'] = 'Ubx-Nav-Pvt'
                dUbxNavPvt['Header'] = uint8(array([181, 98, 1, 7]))
                dUbxNavPvt['Len'] = int(92)

                StrtIdx = -1;
                if len(Dat) > len(dUbxNavPvt['Header']):
                    for Idx in range(len(Dat)-len(dUbxNavPvt['Header'])):
                        DatExt = Dat[Idx:Idx+len(dUbxNavPvt['Header'])]
                        if all(DatExt == dUbxNavPvt['Header']):
                            StrtIdx = Idx

                Msg = [];
                if StrtIdx >= 0:

                    MsgDat = Dat[StrtIdx:]
                    # Check if complete measage is in the data stream
                    if len(MsgDat) >= len(dUbxNavPvt['Header']) + dUbxNavPvt['Len'] + 4:
                        # Remove Message
                        self.GpsDat = Dat[StrtIdx+len(dUbxNavPvt['Header'])+ dUbxNavPvt['Len'] + 4:]
                        # Extract measage
                        MsgDat = double(MsgDat[0:len(dUbxNavPvt['Header'])+ dUbxNavPvt['Len'] + 4])
                        # Remove Header
                        MsgDat = MsgDat[len(dUbxNavPvt['Header']):]
                        # Read Len
                        MsgLen = 2**8*MsgDat[1] + MsgDat[0]
                        if dUbxNavPvt['Len'] == MsgLen:
                            dMsg = dict()
                            MsgDat = MsgDat[2:]
                            dMsg['iTow'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            MsgDat = MsgDat[4:]
                            dMsg['year'] = 2**8*MsgDat[1] + MsgDat[0]
                            MsgDat = MsgDat[2:]
                            dMsg['month'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['day'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['hour'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['min'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['sec'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            # Valid Flgas offst 11
                            dMsg['valid'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['tAcc'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            MsgDat = MsgDat[4:]
                            dMsg['nano'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['nano'] > 2**31:
                                dMsg['nano'] = dMsg['nano'] - 2**32
                            MsgDat = MsgDat[4:]
                            dMsg['fixType'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['flags'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['flags2'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            dMsg['numSV'] = MsgDat[0]
                            MsgDat = MsgDat[1:]
                            # long and lat
                            dMsg['lon'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['lon'] > 2**31:
                                dMsg['lon'] = dMsg['lon'] - 2**32
                            dMsg['lon'] = dMsg['lon']*1e-7
                            MsgDat = MsgDat[4:]
                            dMsg['lat'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['lat'] > 2**31:
                                dMsg['lat'] = dMsg['lat'] - 2**32
                            dMsg['lat'] = dMsg['lat']*1e-7
                            MsgDat = MsgDat[4:]
                            # height
                            dMsg['height'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['height'] > 2**31:
                                dMsg['height'] = dMsg['height'] - 2**32
                            MsgDat = MsgDat[4:]
                            # hMSL
                            dMsg['hMSL'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['hMSL'] > 2**31:
                                dMsg['hMSL'] = dMsg['hMSL'] - 2**32
                            MsgDat = MsgDat[4:]
                            # hACC
                            dMsg['hAcc'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            MsgDat = MsgDat[4:]
                            # vACC
                            dMsg['vAcc'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            MsgDat = MsgDat[4:]
                            # velN
                            dMsg['velN'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['velN'] > 2**31:
                                dMsg['velN'] = dMsg['velN'] - 2**32
                            MsgDat = MsgDat[4:]
                            # velE
                            dMsg['velE'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['velE'] > 2**31:
                                dMsg['velE'] = dMsg['velE'] - 2**32
                            MsgDat = MsgDat[4:]
                            # velD
                            dMsg['velD'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['velD'] > 2**31:
                                dMsg['velD'] = dMsg['velD'] - 2**32
                            MsgDat = MsgDat[4:]
                            # gSpeed
                            dMsg['gSpeed'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['gSpeed'] > 2**31:
                                dMsg['gSpeed'] = dMsg['gSpeed'] - 2**32
                            MsgDat = MsgDat[4:]
                            # headMot
                            dMsg['headMot'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['headMot'] > 2**31:
                                dMsg['headMot'] = dMsg['headMot'] - 2**32
                            dMsg['headMot'] = dMsg['headMot']*1e-5
                            MsgDat = MsgDat[4:]
                            # sACC
                            dMsg['sAcc'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            MsgDat = MsgDat[4:]
                            # headAcc
                            dMsg['headAcc'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            dMsg['headAcc'] = dMsg['headAcc']*1e-5
                            MsgDat = MsgDat[4:]
                            # pDop
                            dMsg['pDOP'] = (2**8*MsgDat[1] + MsgDat[0]);
                            dMsg['pDOP'] = dMsg['pDOP']*1e-2;
                            MsgDat = MsgDat[2:]
                            # Reserved 6
                            MsgDat = MsgDat[6:]
                            # headVeh
                            dMsg['headVeh'] = (2**24*MsgDat[3] + 2**16*MsgDat[2] + 2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['headVeh'] > 2**31:
                                dMsg['headVeh'] = dMsg['headVeh'] - 2**32
                            dMsg['headVeh'] = dMsg['headVeh']*1e-5
                            MsgDat = MsgDat[4:]
                            # magDec
                            dMsg['magDec'] = (2**8*MsgDat[1] + MsgDat[0])
                            if dMsg['headVeh'] > 2**15:
                                dMsg['magDec'] = dMsg['magDec'] - 2**16
                            dMsg['magDec'] = dMsg['magDec']*1e-2
                            MsgDat = MsgDat[2:]
                            # magAcc
                            dMsg['magAcc'] = (2**8*MsgDat[1] + MsgDat[0])
                            dMsg['magAcc'] = dMsg['magAcc']*1e-2
                            MsgDat = MsgDat[2:]

                            dMsg['FrmCntr'] = self.GpsFrmCntr
                            dMsg['stName'] = dUbxNavPvt['stName']
                            lMsg.append(dMsg)
                            MsgIdx = MsgIdx + 1

            return lMsg

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Configure AFE5801
    #>
    #> This method is used to configure the AFE5801. The values are programmed to the register of the device. A detailed
    #> information of the register values is contained in the datasheet of the device.
    #> @param[in]   Cfg
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the USPI device <br>
    #>          Commonly multiple USPI devices are present; Therefore the Mask field is important to address the desired device
    #>      -   <span style="color: #ff9900;"> 'RegConfig': </span>
    #>      -   <span style="color: #ff9900;"> 'RegPower': </span>
    #>      -   <span style="color: #ff9900;"> 'RegLvds': </span>
    #>      -   <span style="color: #ff9900;"> 'RegData': </span>
    #>      -   <span style="color: #ff9900;"> 'RegDfs': </span>
    #>      -   <span style="color: #ff9900;"> 'RegLvdsPattern': </span>
    #>      -   <span style="color: #ff9900;"> 'RegFilter': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh1': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh2': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh3': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh4': </span>
    #>      -   <span style="color: #ff9900;"> 'RegHighpass14': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh5': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh6': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh7': </span>
    #>      -   <span style="color: #ff9900;"> 'RegOffsetGainCh8': </span>
    #>      -   <span style="color: #ff9900;"> 'RegHighpass58': </span>
    #>      -   <span style="color: #ff9900;"> 'RegClamp': </span>
    #>      -   <span style="color: #ff9900;"> 'RegTgcStartIndex': </span>
    #>      -   <span style="color: #ff9900;"> 'RegTgcStopIndex': </span>
    #>      -   <span style="color: #ff9900;"> 'RegTgcStartGain': </span>
    #>      -   <span style="color: #ff9900;"> 'RegTgcHoldGainTime': </span>
    #>      -   <span style="color: #ff9900;"> 'RegTgcGainMode': </span>
    #>      -   <span style="color: #ff9900;"> 'RegTgcGainCoarse': </span>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetAfe5801(self, *varargin):

        if len(varargin) > 0:
            dCfg    =   varargin[0]
            if 'Mask' in dCfg:
                dCfg['Mask']                        =   self.Rad_Afe5801_Mask
            else:
                self.Rad_Afe5801_Mask               =   dCfg['Rad_Afe5801_Mask']
            if 'RegConfig' in dCfg:
                dCfg['RegConfig']                   =   self.Rad_Afe5801_RegConfig
            else:
                self.Rad_Afe5801_RegConfig          =   dCfg['Rad_Afe5801_RegConfig']
            if 'RegPower' in dCfg:
                dCfg['RegPower']                    =   self.Rad_Afe5801_RegPower
            else:
                self.Rad_Afe5801_RegPower           =   dCfg['Rad_Afe5801_RegPower']
            if 'RegLvds' in dCfg:
                dCfg['RegLvds']                     =   self.Rad_Afe5801_RegLvds
            else:
                self.Rad_Afe5801_RegLvds            =   dCfg['Rad_Afe5801_RegLvds']
            if 'RegData' in dCfg:
                dCfg['RegData']                     =   self.Rad_Afe5801_RegData
            else:
                self.Rad_Afe5801_RegData            =   dCfg['Rad_Afe5801_RegData']
            if 'RegDfs' in dCfg:
                dCfg['RegDfs']                      =   self.Rad_Afe5801_RegDfs
            else:
                self.Rad_Afe5801_RegDfs             =   dCfg['Rad_Afe5801_RegDfs']
            if 'RegLvdsPattern' in dCfg:
                dCfg['RegLvdsPattern']              =   self.Rad_Afe5801_RegLvdsPattern
            else:
                self.Rad_Afe5801_RegLvdsPattern     =   dCfg['Rad_Afe5801_RegLvdsPattern']
            if 'RegFilter' in dCfg:
                dCfg['RegFilter']                   =   self.Rad_Afe5801_RegFilter
            else:
                self.Rad_Afe5801_RegFilter          =   dCfg['Rad_Afe5801_RegFilter']
            if 'RegOffsetGainCh1' in dCfg:
                dCfg['RegOffsetGainCh1']            =   self.Rad_Afe5801_RegOffsetGainCh1
            else:
                self.Rad_Afe5801_RegOffsetGainCh1   =   dCfg['Rad_Afe5801_RegOffsetGainCh1']
            if 'RegOffsetGainCh2' in dCfg:
                dCfg['RegOffsetGainCh2']            =   self.Rad_Afe5801_RegOffsetGainCh2
            else:
                self.Rad_Afe5801_RegOffsetGainCh2   =   dCfg['Rad_Afe5801_RegOffsetGainCh2']
            if 'RegOffsetGainCh3' in dCfg:
                dCfg['RegOffsetGainCh3']            =   self.Rad_Afe5801_RegOffsetGainCh3
            else:
                self.Rad_Afe5801_RegOffsetGainCh3   =   dCfg['Rad_Afe5801_RegOffsetGainCh3']
            if 'RegOffsetGainCh4' in dCfg:
                dCfg['RegOffsetGainCh4']            =   self.Rad_Afe5801_RegOffsetGainCh4
            else:
                self.Rad_Afe5801_RegOffsetGainCh4   =   dCfg['Rad_Afe5801_RegOffsetGainCh4']
            if 'RegHighpass14' in dCfg:
                dCfg['RegHighpass14']               =   self.Rad_Afe5801_RegHighpass14
            else:
                self.Rad_Afe5801_RegHighpass14      =   dCfg['Rad_Afe5801_RegHighpass14']
            if 'RegOffsetGainCh5' in dCfg:
                dCfg['RegOffsetGainCh5']            =   self.Rad_Afe5801_RegOffsetGainCh5
            else:
                self.Rad_Afe5801_RegOffsetGainCh5   =   dCfg['Rad_Afe5801_RegOffsetGainCh5']
            if 'RegOffsetGainCh6' in dCfg:
                dCfg['RegOffsetGainCh6']            =   self.Rad_Afe5801_RegOffsetGainCh6
            else:
                self.Rad_Afe5801_RegOffsetGainCh6   =   dCfg['Rad_Afe5801_RegOffsetGainCh6']
            if 'RegOffsetGainCh7' in dCfg:
                dCfg['RegOffsetGainCh7']            =   self.Rad_Afe5801_RegOffsetGainCh7
            else:
                self.Rad_Afe5801_RegOffsetGainCh7   =   dCfg['Rad_Afe5801_RegOffsetGainCh7']
            if 'RegOffsetGainCh8' in dCfg:
                dCfg['RegOffsetGainCh8']            =   self.Rad_Afe5801_RegOffsetGainCh8
            else:
                self.Rad_Afe5801_RegOffsetGainCh8   =   dCfg['Rad_Afe5801_RegOffsetGainCh8']
            if 'RegHighpass58' in dCfg:
                dCfg['RegHighpass58']               =   self.Rad_Afe5801_RegHighpass58
            else:
                self.Rad_Afe5801_RegHighpass58      =   dCfg['Rad_Afe5801_RegHighpass58']
            if 'RegClamp' in dCfg:
                dCfg['RegClamp']                    =   self.Rad_Afe5801_RegClamp
            else:
                self.Rad_Afe5801_RegClamp           =   dCfg['Rad_Afe5801_RegClamp']
            if 'RegTgcStartIndex' in dCfg:
                dCfg['RegTgcStartIndex']            =   self.Rad_Afe5801_RegTgcStartIndex
            else:
                self.Rad_Afe5801_RegTgcStartIndex   =   dCfg['Rad_Afe5801_RegTgcStartIndex']
            if 'RegTgcStopIndex' in dCfg:
                dCfg['RegTgcStopIndex']             =   self.Rad_Afe5801_RegTgcStopIndex
            else:
                self.Rad_Afe5801_RegTgcStopIndex    =   dCfg['Rad_Afe5801_RegTgcStopIndex']
            if 'RegTgcStartGain' in dCfg:
                dCfg['RegTgcStartGain']             =   self.Rad_Afe5801_RegTgcStartGain
            else:
                self.Rad_Afe5801_RegTgcStartGain    =   dCfg['Rad_Afe5801_RegTgcStartGain']
            if 'RegTgcHoldGainTime' in dCfg:
                dCfg['RegTgcHoldGainTime']          =   self.Rad_Afe5801_RegTgcHoldGainTime
            else:
                self.Rad_Afe5801_RegTgcHoldGainTime =   dCfg['Rad_Afe5801_RegTgcHoldGainTime']
            if 'RegTgcGainMode' in dCfg:
                dCfg['RegTgcGainMode']              =   self.Rad_Afe5801_RegTgcGainMode
            else:
                self.Rad_Afe5801_RegTgcGainMode     =   dCfg['Rad_Afe5801_RegTgcGainMode']
            if 'RegTgcGainCoarse' in dCfg:
                dCfg['RegTgcGainCoarse']            =   self.Rad_Afe5801_RegTgcGainCoarse
            else:
                self.Rad_Afe5801_RegTgcGainCoarse   =   dCfg['Rad_Afe5801_RegTgcGainCoarse']
            if 'RegTgcUniformSlope' in dCfg:
                dCfg['RegTgcUniformSlope']          =   self.Rad_Afe5801_RegTgcUniformSlope
            else:
                self.Rad_Afe5801_RegTgcUniformSlope =   dCfg['Rad_Afe5801_RegTgcUniformSlope']
        else:
            dCfg                        =   dict()
            dCfg['Mask']                =   self.Rad_Afe5801_Mask
            dCfg['RegConfig']           =   self.Rad_Afe5801_RegConfig
            dCfg['RegPower']            =   self.Rad_Afe5801_RegPower
            dCfg['RegLvds']             =   self.Rad_Afe5801_RegLvds
            dCfg['RegData']             =   self.Rad_Afe5801_RegData
            dCfg['RegDfs']              =   self.Rad_Afe5801_RegDfs
            dCfg['RegLvdsPattern']      =   self.Rad_Afe5801_RegLvdsPattern
            dCfg['RegFilter']           =   self.Rad_Afe5801_RegFilter
            dCfg['RegOffsetGainCh1']    =   self.Rad_Afe5801_RegOffsetGainCh1
            dCfg['RegOffsetGainCh2']    =   self.Rad_Afe5801_RegOffsetGainCh2
            dCfg['RegOffsetGainCh3']    =   self.Rad_Afe5801_RegOffsetGainCh3
            dCfg['RegOffsetGainCh4']    =   self.Rad_Afe5801_RegOffsetGainCh4
            dCfg['RegHighpass14']       =   self.Rad_Afe5801_RegHighpass14
            dCfg['RegOffsetGainCh5']    =   self.Rad_Afe5801_RegOffsetGainCh5
            dCfg['RegOffsetGainCh6']    =   self.Rad_Afe5801_RegOffsetGainCh6
            dCfg['RegOffsetGainCh7']    =   self.Rad_Afe5801_RegOffsetGainCh7
            dCfg['RegOffsetGainCh8']    =   self.Rad_Afe5801_RegOffsetGainCh8
            dCfg['RegHighpass58']       =   self.Rad_Afe5801_RegHighpass58
            dCfg['RegClamp']            =   self.Rad_Afe5801_RegClamp
            dCfg['RegTgcStartIndex']    =   self.Rad_Afe5801_RegTgcStartIndex
            dCfg['RegTgcStopIndex']     =   self.Rad_Afe5801_RegTgcStopIndex
            dCfg['RegTgcStartGain']     =   self.Rad_Afe5801_RegTgcStartGain
            dCfg['RegTgcHoldGainTime']  =   self.Rad_Afe5801_RegTgcHoldGainTime
            dCfg['RegTgcGainMode']      =   self.Rad_Afe5801_RegTgcGainMode
            dCfg['RegTgcGainCoarse']    =   self.Rad_Afe5801_RegTgcGainCoarse
            dCfg['RegTgcUniformSlope']  =   self.Rad_Afe5801_RegTgcUniformSlope

        Cod             =   int('0x910D',0)
        FpgaCmd         =   zeros(26, dtype='uint32')
        FpgaCmd[0]      =   dCfg['Mask']
        FpgaCmd[1]      =   dCfg['RegConfig']
        FpgaCmd[2]      =   dCfg['RegPower']
        FpgaCmd[3]      =   dCfg['RegLvds']
        FpgaCmd[4]      =   dCfg['RegData']
        FpgaCmd[5]      =   dCfg['RegDfs']
        FpgaCmd[6]      =   dCfg['RegLvdsPattern']
        FpgaCmd[7]      =   dCfg['RegFilter']
        FpgaCmd[8]      =   dCfg['RegOffsetGainCh1']
        FpgaCmd[9]      =   dCfg['RegOffsetGainCh2']
        FpgaCmd[10]     =   dCfg['RegOffsetGainCh3']
        FpgaCmd[11]     =   dCfg['RegOffsetGainCh4']
        FpgaCmd[12]     =   dCfg['RegHighpass14']
        FpgaCmd[13]     =   dCfg['RegOffsetGainCh5']
        FpgaCmd[14]     =   dCfg['RegOffsetGainCh6']
        FpgaCmd[15]     =   dCfg['RegOffsetGainCh7']
        FpgaCmd[16]     =   dCfg['RegOffsetGainCh8']
        FpgaCmd[17]     =   dCfg['RegHighpass58']
        FpgaCmd[18]     =   dCfg['RegClamp']
        FpgaCmd[19]     =   dCfg['RegTgcStartIndex']
        FpgaCmd[20]     =   dCfg['RegTgcStopIndex']
        FpgaCmd[21]     =   dCfg['RegTgcStartGain']
        FpgaCmd[22]     =   dCfg['RegTgcHoldGainTime']
        FpgaCmd[23]     =   dCfg['RegTgcGainMode']
        FpgaCmd[24]     =   dCfg['RegTgcGainCoarse']
        FpgaCmd[25]     =   dCfg['RegTgcUniformSlope']


        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set frame control
    #>
    #> This function configures the frame control MMP. The frame control MMP is used to control and number the measured frames.
    #> The framecontrol block inserts the frame counter and the software id in case of multiple transmit antennas
    #>
    #> @param[in]   Cfg: structure containing the configuration data. The configuration structure is optional. If not specified
    #>              the local configuration of the class is use.
    #>      -   <span style="color: #ff9900;"> 'ChnSel': </span> Bitmask to identify the framecontrol block
    #>          Most frameworks support a single FrameControl block;
    #>      -   <span style="color: #ff9900;"> 'Rst': </span> Reset frame control block <br>
    #>          If 1, the counters in the MMP are set to zero.
    #>      -   <span style="color: #ff9900;"> 'RegChnCtrl': </span> MMP channel control register <br>
    #>          Value for the channel control register
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetFrmCtrl(self, *varargin):
        if len(varargin) > 0:
            dFrmCtrlCfg     =   varargin[0]
            if not ('ChnSel' in dFrmCtrlCfg):
                dFrmCtrlCfg["ChnSel"]           =   self.Rad_FrmCtrlCfg_ChnSel
            else:
                self.Rad_FrmCtrlCfg_ChnSel      =   dFrmCtrlCfg["ChnSel"]
            if not ('Rst' in dFrmCtrlCfg):
                dFrmCtrlCfg["Rst"]              =   self.Rad_FrmCtrlCfg_Rst
            else:
                self.Rad_FrmCtrlCfg_Rst         =   dFrmCtrlCfg["Rst"]
            if not('RegChnCtrl' in dFrmCtrlCfg):
                dFrmCtrlCfg["RegChnCtrl"]       =   self.Rad_FrmCtrlCfg_RegChnCtrl
            else:
                self.Rad_FrmCtrlCfg_RegChnCtrl  =   dFrmCtrlCfg["RegChnCtrl"]
        else:
            dFrmCtrlCfg             =   {
                                            ["ChnSel"]      :   self.Rad_FrmCtrlCfg_ChnSel,
                                            ["Rst"]         :   self.Rad_FrmCtrlCfg_Rst,
                                            ["RegChnCtrl"]  :   self.Rad_FrmCtrlCfg_RegChnCtrl
                                        }

        Cod             =   int('0x9012',0);
        FpgaCmd         =   zeros(3, dtype='uint32')
        FpgaCmd[0]      =   dFrmCtrlCfg["ChnSel"]
        FpgaCmd[1]      =   dFrmCtrlCfg["Rst"]
        FpgaCmd[2]      =   dFrmCtrlCfg["RegChnCtrl"]
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetFrmCtrl Cfg:", FpgaCmd)
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetFrmCtrl Ret:", Ret)
        return     Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set CIC filter configuration
    #>
    #> All FPGA sampling frameworks include CIC filters in the chain. This method can be used to set the configuration of the filters.
    #>
    #> @param[in]   Cfg: structure containing the configuration data. The configuration structure is optional. If not specified
    #>              the local configuration of the class is use.
    #>      -   <span style="color: #ff9900;"> 'FiltSel': </span> Bitmask to identify the CIC filter
    #>          If all filters are programmed with the same configuration, then the Mask is set to 255 in a eight channel framewok.
    #>          In a twelve channel framewok the mask is set to 2^12-1; it is also possible to use a different configruation for the
    #>          available channels. In this case the command must be executed multiple times with a different configuration.
    #>      -   <span style="color: #ff9900;"> 'CombDelay': </span> Comp delay register of the MMP<br>
    #>          Register value for CombDelay (0,1,2,3) -> (2,4,8,16)
    #>          The values 0,1,2,3 are supported in the CIC filter. A value of 0 corresponds to a delay of 2.
    #>      -   <span style="color: #ff9900;"> 'OutSel': </span> Output select register <br>
    #>          Output selection (0,1,2,3) -> (1,2,3,4)
    #>          The design supports up to four stages. A value of 0 selects a single output stage.
    #>      -   <span style="color: #ff9900;"> 'SampPhs': </span> Sampling phase register <br>
    #>          In case of an integer sampling rate reduction R, R-1 values are rejected. The sampling phase selects the sample taken out of
    #>          the data stream. The programmed value must be in the range from 0 to R-1.
    #>      -   <span style="color: #ff9900;"> 'SampRed': </span> Sampling rate reduction register <br>
    #>          Integer value with the sampling rate reduction value.
    #>      -   <span style="color: #ff9900;"> 'RegCtrl': </span> Control register <br>
    #>          Integer value with the sampling rate reduction value.
    #>          The design supports up to four stages. A value of 0 selects a single output stage.
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetCic(self, *varargin):
        if len(varargin) > 0:
            dCicCfg      =   varargin[0]
            if not ('FiltSel' in dCicCfg):
                dCicCfg["FiltSel"]          =   self.Rad_CicCfg_FiltSel
            else:
                self.Rad_CicCfg_FiltSel     =   dCicCfg["FiltSel"]
            if not ('CombDel' in dCicCfg):
                dCicCfg["CombDel"]          =   self.Rad_CicCfg_CombDel
            else:
                self.Rad_CicCfg_CombDel     =   dCicCfg["CombDel"]
            if not ('OutSel' in dCicCfg):
                dCicCfg["OutSel"]           =   self.Rad_CicCfg_OutSel
            else:
                self.Rad_CicCfg_OutSel      =   dCicCfg["OutSel"]
            if not ('SampPhs' in dCicCfg):
                dCicCfg["SampPhs"]          =   self.Rad_CicCfg_SampPhs
            else:
                self.Rad_CicCfg_SampPhs     =   dCicCfg["SampPhs"]
            if not ('SampRed' in dCicCfg):
                dCicCfg["SampRed"]          =   self.Rad_CicCfg_SampRed
            else:
                self.Rad_CicCfg_SampRed     =   dCicCfg["SampRed"]
                self.Setfs()
            if not ('RegCtrl' in dCicCfg):
                dCicCfg["RegCtrl"]          =   self.Rad_CicCfg_RegCtrl
            else:
                self.Rad_CicCfg_RegCtrl     =   dCicCfg["RegCtrl"]
        else:
            dCicCfg             =   {
                                        "FiltSel"   :   self.Rad_CicCfg_FiltSel,
                                        "CombDel"   :   self.Rad_CicCfg_CombDel,
                                        "OutSel"    :   self.Rad_CicCfg_OutSel,
                                        "SampPhs"   :   self.Rad_CicCfg_SampPhs,
                                        "SampRed"   :   self.Rad_CicCfg_SampRed,
                                        "RegCtrl"   :   self.Rad_CicCfg_RegCtrl
                                    }
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetCic Cfg:", dCicCfg)
        Cod             =   int('0x9101',0)
        FpgaCmd         =   zeros(6, dtype='uint32')
        FpgaCmd[0]      =   dCicCfg["FiltSel"]
        FpgaCmd[1]      =   dCicCfg["CombDel"]
        FpgaCmd[2]      =   dCicCfg["OutSel"]
        FpgaCmd[3]      =   dCicCfg["SampPhs"]
        FpgaCmd[4]      =   dCicCfg["SampRed"]
        FpgaCmd[5]      =   dCicCfg["RegCtrl"]
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetCic Ret:", Ret)

        return  Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set number of samples
    #>
    #> Set the number of samples for a single measurement phase.
    #>
    #> @param[in]   Cfg: structure containing the configuration data. The configuration structure is optional. If not specified
    #>              the local configuration of the class is use.
    #>      -   <span style="color: #ff9900;"> 'Nr': </span> Number of samples
    #>      -   <span style="color: #ff9900;"> 'Rst': </span> Reset sampling<br>
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetSamp(self, *varargin):
        if len(varargin) > 0:
            dSampCfg    =   varargin[0]
            if not ('Nr' in dSampCfg):
                dSampCfg["Nr"]          =   self.Rad_N
                self.Rad_SampCfg_Nr     =   self.Rad_N
            if not ('Rst' in dSampCfg):
                dSampCfg["Rst"]         =   self.Rad_Rst
                self.Rad_SampCfg_Rst    =   self.Rad_Rst
        else:
            dSampCfg        =   {
                                    "Nr"        :   self.Rad_SampCfg_Nr,
                                    "Rst"       :   self.Rad_SampCfg_Rst
                                }

        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetSamp Cfg", dSampCfg)

        Cod             =   int('0x9102',0)
        FpgaCmd         =   zeros(2,dtype='uint32')
        FpgaCmd[0]      =   dSampCfg["Nr"]
        FpgaCmd[1]      =   dSampCfg["Rst"]
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetSamp Ret", Ret)

        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Reset timing unit
    #>
    #> Reset the timing unit (SeqTrig) in the FPGA. All outputs are disabled and the timing generation is put into reset.
    #>
    #> @param[in]   Mask: bitmask to select the SeqTrig unit. Most of the FPGA framework only support a single SeqTrig device.
    #>              In this case the mask is set to 1.
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigRst(self, Mask):
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   1
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigRst Ret:", Ret)

        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Reset MIMO sequencer
    #>
    #> The MIMO sequencer is implemented in the softcore processor in the FPGA. The MIMO sequencer is triggered by the SeqTrig unit
    #> and can be used to configure different RF transceivers. The SeqTrig Unit generates an interrupt and the MIMO sequencer reconfigures
    #> the RF chips (commonly with SPI transfers)
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_MimoSeqRst(self):
        FpgaCmd         =   zeros(1,dtype='uint32')
        Cod             =   int('0x9212',0)
        FpgaCmd[0]      =   1
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_MimoSeqRst Ret:", Ret)

        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set number of entries for the MIMO sequencer
    #>
    #> The MIMO sequencer is organized as table, with mulitple channels and each channel is connected to a transceiver. This
    #> method set the number of states (cycles) in the table.
    #>
    #> @param[in] NrEntries: Number of entries in the MIMO sequencer table.
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_MimoSeqNrEntries(self, NrEntries):
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x9212',0)
        FpgaCmd[0]      =   2
        FpgaCmd[1]      =   NrEntries
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_MimoSeqNrEntries Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set mode of MIMO sequencer
    #>
    #> This method is used to configure the operational mode. In the normal mode the table entry is selected by the SeqId field of
    #> the SeqTrig unit.
    #>
    #> @param[in] Mod
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_MimoSeqSetMod(self, Mod):
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x9212',0)
        FpgaCmd[0]      =   0
        FpgaCmd[1]      =   Mod
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_MimoSeqSetMod Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set entries of the sequencer for the RCC channel
    #>
    #> This method is used to store the SPI transfers in the sequencer table for the RCC channel.
    #>
    #> @param[in] SeqNr: sequence number; must be smaller than number of entries
    #>
    #> @param[in] Data: Array with the SPI data. This data is transfered to the RCC on an interrupt and when the sequence is selected.
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_MimoSeqSetCrocRegs(self, SeqNr, Data):
        if len(Data) > 28:
            Data        =   Data[0:28]

        FpgaCmd         =   zeros(int(2 + len(Data)),dtype='uint32')
        Cod             =   int('0x9212',0)
        FpgaCmd[0]      =   4
        FpgaCmd[1]      =   SeqNr
        FpgaCmd[2:]     =   Data
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_MimoSeqSetCrocRegs Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set entries of the sequencer for the selected channel
    #>
    #> This method is used to store the SPI transfers in the sequencer table for the selected channel.
    #>
    #> @param[in] Chn: Channel of the sequencer; The connection of the channel to a transceiver depends on the FPGA framework.
    #>
    #> @param[in] SeqNr: sequence number; must be smaller than number of entries
    #>
    #> @param[in] Data: Array with the SPI data. This data is transfered to the RCC on an interrupt and when the sequence is selected.
    #>
    #> @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_MimoSeqSetChnRegs(self, Chn, SeqNr, Data):
        if len(Data) > 28:
            Data        =   Data[0:28]
        FpgaCmd         =   zeros(int(3 + len(Data)),dtype='uint32')
        Cod             =   int('0x9212',0)
        FpgaCmd[0]      =   3
        FpgaCmd[1]      =   Chn
        FpgaCmd[2]      =   SeqNr
        FpgaCmd[3:]     =   Data
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_MimoSeqSetChnRegs Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Display board information
    #>
    #> This function prints the actual status of the board to the Matlab command window. The board monitors the
    #> temperature near the ADC and the input power supply circuit. In addition, the supply voltage and supply current are shown.
    #>
    #>   After calling BrdDispInf the configuration is plotted in the Matlab command window
    #>   @code
    #>      -------------------------------------------------------------------
    #>      Board Information
    #>      Temp1 (AD8283)    = 62.1473°
    #>      Temp2 (Sup)       = 56.4602°
    #>      VSup (Sup)        = 16.4193V
    #>      ISup (Sup)        = 0.74766A
    #>      -------------------------------------------------------------------
    #>   @endcode
    #>
    #>  @note The supply current is only monitored correct, if the RF supply is enabled.
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_DispBrdInf(self):
        print(' ')
        print('-------------------------------------------------------------------')
        print('Radarbook2: Board Information')
        Val             =   self.Fpga_GetBrdInf()
        Ret             =   -1
        if len(Val) > 1:
            uidHigh = '%08X' % (Val[1]);
            uidLow  = '%08X' % (Val[0]);
            print('UID       = ', uidHigh, uidLow, 'h');
            if len(Val) > 2:
                Temp = Val[2] - 273;
                print('Temp      = ', Temp, 'deg');
        else:
            print('Board does not respond!')

        print('-------------------------------------------------------------------')
        return Val

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set RF power supply
    #>
    #> Low level function to enable or disable the internal (switching regulators) and RF (linear regulators) power supply.
    #>
    #> @param[in] PwrCfg: configuration structure; the following fields are supported
    #>          -   <span style="color: #ff9900;"> 'IntEna': </span> Enable/disable of internal supply <br>
    #>              Bit0: P1; Bit1: P2; Bit3: P3
    #>          -   <span style="color: #ff9900;"> 'RfEna': </span> Enable/disable of RF supply <br>
    #>              Bit0: P1; Bit1: P2; Bit3: P3
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetRfPwr(self, dCfg):
        #   @function       Fpga_SetRfPwr
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-28
        #   @brief          Set power control of FArmHP board
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x9015',0)
        if "IntEna" in dCfg:
            FpgaCmd[0]      =   dCfg["IntEna"]
        else:
            FpgaCmd[0]      =   0
        if "RfEna" in dCfg:
            FpgaCmd[1]      =   dCfg["RfEna"]
        else:
            FpgaCmd[1]      =   0
        Ret                 =   self.CmdSend(0, Cod, FpgaCmd)
        Ret                 =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetRfPwr Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Disp status of RF frontend
    #>
    #> All RF frontends are equipped with a EEPROM device, to store the board information and the calibration data
    #> The BrdDispRfSts function prints the status of the board to the command window. If the ID of the frontend is knwon, the chip configuration is also displayed.
    #> Most RF chips have an unique identification number. This number is printed in the command window.
    #>
    #>   After calling BrdDispRfSts the configuration is plotted in the Matlab command window.
    #>   In case of the MIMO77 frontend the status of the chips is displayed too.
    #>   @code
    #>      -------------------------------------------------------------------
    #>      RF Board Configuration
    #>      MIMO-77-TX4RX8
    #>      RF UID            = 64
    #>      RF RevNr          = 1.1.0
    #>      RF SerialNr       = 64000E
    #>      RF Date           = 07-Jan-2016
    #>      RF Startup        = 7
    #>      --------------------
    #>      RCC               = 40091739537F
    #>      DPA1              = 1067295654D8
    #>      DPA2              = 1067284754D8
    #>      DPA3              = 1067253254D8
    #>      MRX1              = 1000000200
    #>      MRX2              = 1000000200
    #>      -------------------------------------------------------------------
    #>   @endcode
    #>   @note To display the ID of the RF chips the RF supply must be enabled (BrdPwrEna())
    def     Fpga_DispRfChipSts(self, Mask):
        print(' ')
        print('-------------------------------------------------------------------')
        print('RF Board Configuration')
        Val     =   array([self.Fpga_GetRfChipSts(Mask)])


        if len(Val) > 2:
                print('BrdInf');
                print('RF UID            = ', Val[0])
                print('RF RevNr          = ', self.RevStr(Val[1]))
                print('RF SerialNr       = ', Val[2])
                print('RF Date           = ', Val[3])
                print('RF Startup        = ', Val[4])
        else:
            print('No status information available')

        print('-------------------------------------------------------------------')

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Configure single sequence of SeqTrig MMP
    #>
    #> This method programms a single sequence to the RAM of the SeqTrig MMP
    #>
    #>  @param[in]
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to select the MMP
    #>      -   <span style="color: #ff9900;"> 'CntrCtrl': </span> Counter control register
    #>      -   <span style="color: #ff9900;"> 'CntrPerd': </span> Counter period register
    #>      -   <span style="color: #ff9900;"> 'NextAdr': </span> Next Adress register
    #>      -   <span style="color: #ff9900;"> 'SeqId': </span> Sequence id register
    #>      -   <span style="color: #ff9900;"> 'Chn0Tim0': </span> Channel 0 time 0 register
    #>      -   <span style="color: #ff9900;"> 'Chn0Tim1': </span> Channel 0 time 1 register
    #>      -   <span style="color: #ff9900;"> 'Chn0Cfg': </span> Channel 0 cfg word register
    #>      -   <span style="color: #ff9900;"> 'Chn1Tim0': </span> Channel 1 time 0 register
    #>      -   <span style="color: #ff9900;"> 'Chn1Tim1': </span> Channel 1 time 1 register
    #>      -   <span style="color: #ff9900;"> 'Chn1Cfg': </span> Channel 1 cfg word register
    #>      -   <span style="color: #ff9900;"> 'Chn2Tim0': </span> Channel 2 time 0 register
    #>      -   <span style="color: #ff9900;"> 'Chn2Tim1': </span> Channel 2 time 1 register
    #>      -   <span style="color: #ff9900;"> 'Chn2Cfg': </span> Channel 2 cfg word register
    #>      -   <span style="color: #ff9900;"> 'Chn3Tim0': </span> Channel 3 time 0 register
    #>      -   <span style="color: #ff9900;"> 'Chn3Tim1': </span> Channel 3 time 1 register
    #>      -   <span style="color: #ff9900;"> 'Chn3Cfg': </span> Channel 3 cfg word register
    #>
    #>   @note To display the ID of the RF chips the RF supply must be enabled (BrdPwrEna())
    def     Fpga_SeqTrigCfgSeq(self, Mask, SeqNr, dSeq):
        FpgaCmd         =   zeros(19,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   2
        FpgaCmd[2]      =   SeqNr
        FpgaCmd[3]      =   dSeq["CntrCtrl"]
        FpgaCmd[4]      =   dSeq["CntrPerd"]
        FpgaCmd[5]      =   dSeq["NextAdr"]
        FpgaCmd[6]      =   dSeq["SeqId"]
        FpgaCmd[7]      =   dSeq["Chn0Tim0"]
        FpgaCmd[8]      =   dSeq["Chn0Tim1"]
        FpgaCmd[9]      =   dSeq["Chn0Cfg"]
        FpgaCmd[10]     =   dSeq["Chn1Tim0"]
        FpgaCmd[11]     =   dSeq["Chn1Tim1"]
        FpgaCmd[12]     =   dSeq["Chn1Cfg"]
        FpgaCmd[13]     =   dSeq["Chn2Tim0"]
        FpgaCmd[14]     =   dSeq["Chn2Tim1"]
        FpgaCmd[15]     =   dSeq["Chn2Cfg"]
        FpgaCmd[16]     =   dSeq["Chn3Tim0"]
        FpgaCmd[17]     =   dSeq["Chn3Tim1"]
        FpgaCmd[18]     =   dSeq["Chn3Cfg"]
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigCfgSeq: ", dSeq)
        Ret             =   self.CmdSend(0, Cod, FpgaCmd);
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigCfgSeq Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get board information
    #>
    #> This method reads the ADC values of the internal monitoring ADC. The method Fpga_DispBrdInf uses this function to read the
    #> ADC values.
    #>  @return Array with four ADC values.
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_GetBrdInf(self):
        FpgaCmd         =   zeros(1,dtype='uint32')
        Cod             =   int('0x9013',0)
        FpgaCmd[0]      =   0
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_GetBrdInf Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Load timing to SeqTrig unit
    #>
    #> This method loads the timing in the SeqTrig unit; After the function call the programmed timing is processed.
    #>
    #>  @param[in]  Mask: Bitmask to select the SeqTrig Unit in the FPGA framework; if only one unit is present set to 1
    #>
    #>  @param[in]  RegCtrl: Control register of the MMP
    #>
    #>  @param[in]      RegChnEna: Channel Enable register of MMP
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigLoad(self, Mask, RegCtrl, RegChnEna):
        FpgaCmd         =   zeros(4,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   3
        FpgaCmd[2]      =   RegCtrl
        FpgaCmd[3]      =   RegChnEna
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigLoad Cfg:", FpgaCmd)

        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()

        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigLoad Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Generate trigger event for timing unit
    #>
    #> This method generates a trigger signal for the timing unit in the FPGA (SeqTrig)
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigEve(self, Mask):
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   4
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigEve Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get value of SeqTrig internal counter
    #>
    #> This method reads the internal counter of the SeqTrig unit; The SeqTrig Unit (> 3.0.0) has an internal counter, which
    #> can be used to count marked sequencies. This function is used to readback the value of the counter.
    #>
    #>  @param[in] Mask: Bitmask to select the SeqTrig Unit in the FPGA framework; if only one unit is present set to 1
    #>
    #>  @param[in] Clr: Clear counter; If set greater than 0, then the counter is cleared after the read operation
    #>
    #>  @return Value of the internal counter in the SeqTrig MMP
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigGetCntr(self, Mask, Clr):
        FpgaCmd         =   zeros(4,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   5
        FpgaCmd[2]      =   4
        FpgaCmd[3]      =   Clr
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigEve Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set hold event for the SeqTrig Unit
    #>
    #> This method set the hold event for the SeqTrig Unit; the hold event can be used to hold the timing on pre-defined hold points.
    #> To set a hold point the HOLD flag must be set in the timing pattern.
    #>
    #>  @param[in] Mask: Bitmask to select the SeqTrig Unit in the FPGA framework; if only one unit is present set to 1
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigSetHold(self, Mask):
        FpgaCmd         =   zeros(3,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   5
        FpgaCmd[2]      =   1
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigEve Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Clear hold event for the SeqTrig Unit
    #>
    #> This method clear the hold event for the SeqTrig Unit; the hold event can be used to hold the timing on pre-defined hold points.
    #> To set a hold point the HOLD flag must be set in the timing pattern.
    #>
    #>  @param[in] Mask: Bitmask to select the SeqTrig Unit in the FPGA framework; if only one unit is present set to 1
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigClrHold(self, Mask):
        FpgaCmd         =   zeros(3,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   5
        FpgaCmd[2]      =   2
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigEve Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get hold event for the SeqTrig Unit
    #>
    #> This method tests if a hold condition is active
    #>
    #>  @param[in] Mask: Bitmask to select the SeqTrig Unit in the FPGA framework; if only one unit is present set to 1
    #>
    #>  @return Hold: 1 MMP is in hold; 0 MMP is not in hold
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SeqTrigGetHold(self, Mask):
        FpgaCmd         =   zeros(3,dtype='uint32')
        Cod             =   int('0x9213',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   5
        FpgaCmd[2]      =   3
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigEve Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get software version of FPGA framework
    #>
    #> This function returns a struct with the software version of the FPGA framework
    #>
    #>  @return HUid hardware identification number
    #>
    #>  @return SUid software identification number
    #>
    #>  @return SwVer software version struct
    #>      -   <span style="color: #9966ff;"> 'SwMaj': </span> major software version <br>
    #>      -   <span style="color: #9966ff;"> 'SwMin': </span> minor software version <br>
    #>      -   <span style="color: #9966ff;"> 'SwPatch': </span> patch software version <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_GetSwCfg(self):
        Result      =   self.Fpga_GetSwVers()
        Result      =   ((self.SVers % 256),)
        Dummy       =   self.SVers // 256
        Result      +=  ((Dummy//256),)
        Dummy       =   Dummy // 256
        Result      +=  ((Dummy,))

        return      Result

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Display software version of FPGA framework
    #>
    #> This function prints the software version of the FPGA framework in the command window of Matlab
    #>
    #>  After calling the function the following text is plotted:
    #>  @code
    #>      -------------------------------------------------------------------
    #>      FPGA Software UID
    #>      Sw-Rev: 4-0-3 | Sw-UID: 20 | Hw-UID: 12
    #>      -------------------------------------------------------------------
    #>  @endcode
    #>  The software identification number (Sw-UID) refers to the FPGA framework and the hardware identification number
    #>  refers to the type of baseband.
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_DispSwCfg(self):
        print(' ');
        print('-------------------------------------------------------------------');
        print('FPGA Software UID');
        Vers    =   self.Fpga_GetSwVers()
        if len(Vers) > 2:
            Tmp         =   Vers[0]
            SwPatch     =   Tmp % 2**8
            Tmp         =   floor(Tmp/2**8)
            SwMin       =   Tmp % 2**8
            SwMaj       =   floor(Tmp/2**8)
            print(' Sw-Rev: ',int(SwMaj),'-',int(SwMin),'-',int(SwPatch), end='')
            print(' | Sw-UID: ',int(Vers[1]), end='')
            print(' | Hw-UID: ',int(Vers[2]))
        else:
            print('No version information available')
        print('-------------------------------------------------------------------');

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get software version of FPGA framework
    #>
    #> This function returns an array with the software version of the FPGA framework
    #>
    #>  @return Array with the software version
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_GetSwVers(self):
        FpgaCmd         =   zeros(1,dtype='uint32')
        Cod             =   int('0x900E',0)
        FpgaCmd[0]      =   0
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return  Ret


    def     Fpga_CfgArmMem(self,*varargin):
        if len(varargin) > 0:
            dArmMemCfg          =   varargin[0]
            if not ('Mask' in dArmMemCfg):
                dArmMemCfg['Mask']          =   self.Rad_ArmMemCfg_Mask
            else:
                self.Rad_ArmMemCfg_Mask     =   dArmMemCfg['Mask']
            if not ('Chn' in dArmMemCfg):
                dArmMemCfg['Chn']           =   self.Rad_ArmMemCfg_Chn
            else:
                self.Rad_ArmMemCfg_Chn      =   dArmMemCfg['Chn']
            if not ('Ctrl' in dArmMemCfg):
                dArmMemCfg['Ctrl']          =   self.Rad_ArmMemCfg_Ctrl
            else:
                self.Rad_ArmMemCfg_Ctrl     =   dArmMemCfg['Ctrl']

            if not ('FrmSiz' in dArmMemCfg):
                dArmMemCfg['FrmSiz']        =   self.Rad_ArmMemCfg_FrmSiz
            else:
                self.Rad_ArmMemCfg_FrmSiz   =   dArmMemCfg['FrmSiz']

            if not ('WaitLev' in dArmMemCfg):
                dArmMemCfg['WaitLev']       =   self.Rad_ArmMemCfg_WaitLev
            else:
                self.Rad_ArmMemCfg_WaitLev  =   dArmMemCfg['WaitLev']
        else:
            dArmMemCfg      =   {
                    'Mask'          :   self.Rad_ArmMemCfg_Mask,
                    'Ctrl'          :   self.Rad_ArmMemCfg_Ctrl,
                    'Chn'           :   self.Rad_ArmMemCfg_Chn,
                    'FrmSiz'        :   self.Rad_ArmMemCfg_FrmSiz,
                    'WaitLev'       :   self.Rad_ArmMemCfg_WaitLev
                    }

        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_CfgArmMem:", dArmMemCfg)

        Cod             =   int('0x9400', 0)
        FpgaCmd         =   zeros(8, dtype='uint32')
        FpgaCmd[0]      =   dArmMemCfg['Mask']
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   dArmMemCfg['Ctrl']
        FpgaCmd[3]      =   dArmMemCfg['Chn']
        FpgaCmd[4]      =   dArmMemCfg['FrmSiz']
        FpgaCmd[5]      =   dArmMemCfg['WaitLev']
        FpgaCmd[6]      =   self.cRbkDat
        FpgaCmd[7]      =   self.Rad_DmaMult

        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return  Ret

    def     Fpga_GetArmMemSts(self):
        Cod             =   int('0x9400', 0)
        FpgaCmd         =   zeros(2, dtype='uint32')
        FpgaCmd[0]      =   0
        FpgaCmd[1]      =   2
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return  Ret

    def     Fpga_RstStAna(self, Sel):
        Cod             =   int('0x9401', 0)
        FpgaCmd         =   zeros(3, dtype='uint32')
        FpgaCmd[0]      =   0
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   Sel
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return  Ret

    def     Fpga_GetStAna(self):
        Cod             =   int('0x9401', 0)
        FpgaCmd         =   zeros(3, dtype='uint32')
        FpgaCmd[0]      =   0
        FpgaCmd[1]      =   2
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return  Ret

    def     Fpga_GetArmMem(self):
        Cod             =   int('0x8000', 0)
        FpgaCmd         =   zeros(6, dtype='uint32')
        FpgaCmd[0]      =   0
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   0
        FpgaCmd[3]      =   1
        FpgaCmd[4]      =   1024
        FpgaCmd[5]      =   4000
        Ret             =   self.CmdSend(0,Cod,FpgaCmd)
        Ret             =   self.DataRecv()
        return double(Ret)


    def Fpga_GetData(self):
        #   @function       CmdRecv
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Receive response from baseboard
        TxData  =   0
        if self.hConDat > -1:
            TxData      =   1
        else:
            self.OpenTcpIpDatCom(self.cRbkIp, int(self.cRbkDat))
            TxData      =   1

        Ret         =   zeros((int(self.Rad_N), int(self.Rad_NrChn)), dtype='int16')

        if TxData > 0:



            try:
                RxBytes         =   self.cRadDatSocket.recv(int(2*self.Rad_N*self.Rad_NrChn))
            except Exception:
                print("Error in Arm_ReadFrm recv: Exception raised")
                return Ret

            while len(RxBytes) < (2*self.Rad_N*self.Rad_NrChn):
                LenData     =   len(RxBytes)
                RxBytes     =   RxBytes + self.cRadDatSocket.recv(int(2*self.Rad_N*self.Rad_NrChn - LenData))

            Res             =   fromstring(RxBytes, dtype = 'int16')
            Ret             =   reshape(Res, (int(self.Rad_NrChn), int(self.Rad_N)))


        return Ret.transpose()

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set calibration information
    #>
    #> This function writes the calibartion configuration data to the FPGA. The information is not stored in the EEPROM of the frontend.
    #>
    #> @param[in]   Cfg: structure containing the calibration data
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Type': </span> type of calibration table <br>
    #>      -   <span style="color: #ff9900;"> 'RevNr': </span> Revision number of the calibration data <br>
    #>          Uint32 containing revision number
    #>      -   <span style="color: #ff9900;"> 'DateNr': </span> Date number of calibration <br>
    #>          Uint32 containing date number of calibration e.g. floor(now())
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def Fpga_SetCalCfg(self, dCfg):
        FpgaCmd         =   zeros(5, dtype='uint32')
        Cod             =   int('0x901F',0)
        if not ('Type' in dCfg):
            dCfg["Type"]    =   0

        FpgaCmd[0]      =   dCfg["Mask"]
        FpgaCmd[1]      =   2
        FpgaCmd[2]      =   dCfg["Type"]
        FpgaCmd[3]      =   dCfg["RevNr"]
        FpgaCmd[4]      =   dCfg["DateNr"]
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetCalCfg Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get calibration information
    #>
    #> This function reads the calibration information from the FPGA.
    #>
    #>  @param[in]  Cfg: structure with the configuration information
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def Fpga_GetCalCfg(self, dCfg):
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x901F',0)
        FpgaCmd[0]      =   dCfg["Mask"]
        FpgaCmd[1]      =   4
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_GetCalCfg Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get calibration information
    #>
    #> This function reads the calibration information from the FPGA.
    #>
    #>  @param[in]  Cfg: structure with the configuration information
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def Fpga_GetCalData(self, dCfg):
        N           =   dCfg["Len"]
        StrtIdx     =   1
        Data        =   zeros(int(N), dtype='int32')
        while(StrtIdx < N):
            if (StrtIdx + 24) < N:
                Len         =   24
            else:
                Len         =   N - StrtIdx + 1
            FpgaCmd         =   zeros(4, dtype='uint32')
            Cod             =   int('0x901F',0)
            FpgaCmd[0]      =   dCfg["Mask"]
            FpgaCmd[1]      =   5
            FpgaCmd[2]      =   StrtIdx
            FpgaCmd[3]      =   Len
            Ret             =   self.CmdSend(0, Cod, FpgaCmd)
            Ret             =   self.CmdRecv()
            Data[int(StrtIdx-1):int(StrtIdx+Len-1)]       =   Ret
            StrtIdx         =   StrtIdx + Len
        return Data

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set calibration data
    #>
    #> This function writes the calibartion data to the FPGA. The information is not stored in the EEPROM of the frontend.
    #>
    #> @param[in]   Cfg: structure containing the calibration data
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Data': </span> calibration data <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def Fpga_SetCalData(self, dCfg):
        #   @function       Fpga_SetCalData
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-28
        #   @brief          Set Calibration Data
        Data            =   dCfg["Data"]

        N               =   len(Data)
        StrtIdx         =   1
        while(StrtIdx < N):
            Cod             =   int('0x901F',0)
            if (StrtIdx + 24) < N:
                TmpData     =   Data[StrtIdx-1:StrtIdx + 24 - 1]
                Len         =   24
            else:
                TmpData     =   Data[StrtIdx-1:]
                Len         =   N - StrtIdx + 1

            FpgaCmd         =   zeros(int(Len + 4), dtype='uint32')
            FpgaCmd[0]      =   dCfg["Mask"]
            FpgaCmd[1]      =   3
            FpgaCmd[2]      =   StrtIdx
            FpgaCmd[3]      =   Len
            FpgaCmd[4:]     =   TmpData
            if (int(self.cDebugInf) & 8) > 0:
                print("Fpga_SetCalData Cfg:", Fpga_SetCalData)

            Ret             =   self.CmdSend(0, Cod, FpgaCmd)
            Ret             =   self.CmdRecv()
            if (int(self.cDebugInf) & 8) > 0:
                print("Fpga_SetCalData Ret:", Ret)
            StrtIdx         =   StrtIdx + Len



    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get table data (32 Bit, Max length is 128)
    #>
    #> This function read data from the selected table the data is stored in the EEPROM
    #>
    #> @param[in]   Cfg: structure containing the calibration data
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Len': </span> Number of entries <br>
    #> @return  Array with calibration data (uint32) values
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_GetTableData(self, dCfg):
        N               =   dCfg['Len']
        StrtIdx         =   1
        Data            =   []
        while(StrtIdx < N):
            if (StrtIdx + 24) < N:
                Len         =   24
            else:
                Len         =   N - StrtIdx + 1

            FpgaCmd         =   zeros(int(5), dtype='uint32')
            Cod             =   int('0x901F',0)
            FpgaCmd[0]      =   dCfg["Mask"]
            FpgaCmd[1]      =   15
            FpgaCmd[2]      =   dCfg["Table"]
            FpgaCmd[3]      =   StrtIdx
            FpgaCmd[4]      =   Len
            Ret             =   self.CmdSend(0, Cod, FpgaCmd);
            Ret             =   self.CmdRecv()
            StrtIdx         =   StrtIdx + Len
            if Ret[0]:
                Data            =   append(asarray(Data), asarray(Ret[1]))
        return Data

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set table data
    #>
    #> This function writes the calibartion data to the FPGA. The information is not stored in the EEPROM of the frontend.
    #>
    #> @param[in]   Cfg: structure containing the table data
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Data': </span> calibration data <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetTableData(self, dCfg):
        N           =   len(dCfg['Data'])
        StrtIdx     =   0
        while(StrtIdx < N):
            Cod         =   int('0x901F',0)
            if (StrtIdx + 24) < N:
                Data        =   dCfg['Data'][StrtIdx:StrtIdx + 24]
                Len         =   24
            else:
                Data        =   dCfg['Data'][StrtIdx:]
                Len         =   N - StrtIdx + 1

            FpgaCmd         =   zeros(int(5 + len(Data)), dtype='uint32')
            FpgaCmd[0]      =   dCfg["Mask"]
            FpgaCmd[1]      =   13
            FpgaCmd[2]      =   dCfg["Table"]
            FpgaCmd[3]      =   StrtIdx + 1
            FpgaCmd[4]      =   Len
            FpgaCmd[5:]     =   asarray(Data)
            Ret             =   self.CmdSend(0,Cod,FpgaCmd)
            Ret             =   self.CmdRecv()
            StrtIdx         =   StrtIdx + Len

        return Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set table type 1 data
    #>
    #> This function writes the table with data format 1 to the EEPROM
    #>
    #> @param[in]   Cfg: structure containing the table configuration
    #> @param[in]   Inf: structure containing the table information
    #> @param[in]   Tab1: structure containing the table 1 data
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetTableType1(self, dCfg, dInf, dTab1):
        Ret     =   []
        if dInf['Type'] == 1:
            self.Fpga_SetTableInf(dCfg, dInf)
            dCfg['Data']    =   asarray([dTab1['VcoCTune'], dTab1['VcoCoreBiasing'], dTab1['CTuneTx'], dTab1['CTuneRx']])
            Ret     =   self.Fpga_SetTableData(dCfg)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get table data
    #>
    #> This function reads the table from the EEPROM and formats the data according to the table type information
    #> Currently only type 1 is implemented
    #> @param[in]   Cfg: structure containing the table configuration
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_GetTable(self, dCfg):
        dTable              =   dict()
        self.Fpga_RdTable(dCfg)
        dTable['dInf']      =   self.Fpga_GetTableInf(dCfg)
        if dTable['dInf']['Type'] == 1:
            dCfg['Len']     =   dTable['dInf']['Entrs']
            Data            =   self.Fpga_GetTableData(dCfg)
            if len(Data) == 4:
                dTable['dTab1']                     =   dict()
                dTable['dTab1']['VcoCTune']         =   Data[0]
                dTable['dTab1']['VcoCoreBiasing']   =   Data[1]
                dTable['dTab1']['CTuneTx']          =   Data[2]
                dTable['dTab1']['CTuneRx']          =   Data[3]
        return dTable

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get dat table information
    #>
    #> This function is reset the data table
    #>
    #> @param[in]   Cfg: structure containing the calibration setup
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Type': </span> Table type <br>
    #>      -   <span style="color: #ff9900;"> 'Table': </span> Table number <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_GetTableInf(self, dCfg):
        Cod         =   int('0x901F',0)
        dInf        =   dict()
        if not ('Table' in dCfg):
            dCfg['Table']   =   0
        FpgaCmd     =   zeros(3,dtype='uint32')
        FpgaCmd[0]  =   dCfg['Mask']
        FpgaCmd[1]  =   14
        FpgaCmd[2]  =   dCfg['Table']
        Ret         =   self.CmdSend(0,Cod,FpgaCmd)
        Ret         =   self.CmdRecv()
        if Ret[0]:
            Ret             =   Ret[1]
            dInf['Entrs']   =   Ret[0]
            dInf['Chn']     =   Ret[1]
            dInf['Type']    =   Ret[2]
            dInf['Date']    =   Ret[3]
            dInf['RevNr']   =   Ret[4]
        return dInf

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set table information data
    #>
    #> This function used to set the table information data
    #>
    #> @param[in]   Cfg: structure containing the calibration setup
    #>      -   <span style="color: #ff9900;"> 'Cfg': </span> Configuration of the table <br>
    #>      -   <span style="color: #ff9900;"> 'Inf': </span> Cell array with table information data type <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetTableInf(self, dCfg, dInf):
        Cod         =   int('0x901F', 0)
        if not ('Table' in dCfg):
            dCfg['Table']   =   0
        FpgaCmd     =   zeros(8,dtype='uint32')
        FpgaCmd[0]  =   dCfg['Mask']
        FpgaCmd[1]  =   12
        FpgaCmd[2]  =   dCfg['Table']
        FpgaCmd[3]  =   dInf['Entrs']
        FpgaCmd[4]  =   dInf['Chn']
        FpgaCmd[5]  =   dInf['Type']
        FpgaCmd[6]  =   dInf['DateNr']
        FpgaCmd[7]  =   dInf['RevNr']

        Ret         =   self.CmdSend(0,Cod,FpgaCmd)
        Ret         =   self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Reset calibration table
    #>
    #> This function is reset the data table
    #>
    #> @param[in]   Cfg: structure containing the calibration setup
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Type': </span> Table type <br>
    #>      -   <span style="color: #ff9900;"> 'Table': </span> Table number <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_RstTable(self, dCfg):
        Cod         =   inf('0x901F',0)
        if not ('Table' in dCfg):
            dCfg['Table']   =   0
        FpgaCmd     =   zeros(3,dtype='uint32')
        FpgaCmd[0]  =   dCfg['Mask']
        FpgaCmd[1]  =   10
        FpgaCmd[2]  =   dCfg['Table']
        Ret         =   self.CmdSend(0,Cod,FpgaCmd)
        Ret         =   self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Reset data table
    #>
    #> This function is reset the data table
    #>
    #> @param[in]   Cfg: structure containing the calibration setup
    #>      -   <span style="color: #ff9900;"> 'Cfg': </span> cell array to identifiy the board<br>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_StoreTable(self, dCfg):
        Cod         =   int('0x901F',0)
        if not ('Table' in dCfg):
            dCfg['Table']   =   0
        FpgaCmd     =   zeros(3,dtype='uint32')
        FpgaCmd[0]  =   dCfg['Mask']
        FpgaCmd[1]  =   11
        FpgaCmd[2]  =   dCfg['Table']
        Ret         =   self.CmdSend(0,Cod,FpgaCmd)
        Ret         =   self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Copy data table from EEPROM to internal memory of the arm
    #>
    #> Read the data table
    #>
    #> @param[in]   Cfg: structure containing the calibration setup
    #>      -   <span style="color: #ff9900;"> 'Cfg': </span> cell array to identifiy the board<br>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_RdTable(self, dCfg):
        Cod         =   int('0x901F',0)
        if not ('Table' in dCfg):
            dCfg['Table']   =   0
        FpgaCmd     =   zeros(3,dtype='uint32')
        FpgaCmd[0]  =   dCfg['Mask']
        FpgaCmd[1]  =   9
        FpgaCmd[2]  =   dCfg['Table']
        Ret         =   self.CmdSend(0,Cod,FpgaCmd)
        Ret         =   self.CmdRecv()
        return Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Store calibration information
    #>
    #> This function is used to store the calibration data in the EEPROM of the frontend
    #>
    #> @param[in]   Cfg: structure containing the calibration setup
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the EEPROM <br>
    #>          Standard frontends only support one EEPROM: Mask is 1
    #>      -   <span style="color: #ff9900;"> 'Type': </span> Table type <br>
    #>      -   <span style="color: #ff9900;"> 'Table': </span> Table number <br>
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def Fpga_StoreCalTable(self, dCfg):
        #   @function       Fpga_StoreCalTable
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-28
        #   @brief          Store Calibration Data to EEPROM
        FpgaCmd         =   zeros(4,dtype='uint32')
        Cod             =   int('0x901F',0)
        if not ('Table' in dCfg):
            dCfg["Table"]    =   0

        FpgaCmd[0]      =   dCfg["Mask"]
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   dCfg["Type"]
        FpgaCmd[3]      =   dCfg["Table"]
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_StoreCalTable Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Get Status of RF frontend
    #>
    #> All RF frontends are equipped with a EEPROM device, to store the board information and the calibration data
    #> The BrdGetRfSts function reads the board information of the RF frontend
    #>
    #> @return Array with board information.
    #>      -   <span style="color: #9966ff;"> [1]: 'RfUid': </span> <br>
    #>          Unique identifier of RF frontend design
    #>      -   <span style="color: #9966ff;"> [2]: 'RfRevNr': </span> <br>
    #>          Revision number of the frontend; number format: Major*2**16 + Minor*2**8 + Patch
    #>      -   <span style="color: #9966ff;"> [3]: 'RfSerial': </span> <br>
    #>          Serial number of the frontend
    #>      -   <span style="color: #9966ff;"> [4]: 'RfDate': </span> <br>
    #>          Date of configuration; generated with now form Matlab. Use datestr to convert to a string with the date.
    #>      -   <span style="color: #9966ff;"> [5]: 'RfStrtup': </span> <br>
    #>          Startup counter: Counter is incremented after the FPGA boots. An enable and disable of the RF supply does not increment the counter.
    #>
    def     Fpga_GetRfChipSts(self, Mask):
        #   @function       Fpga_GetRfChipSts.m
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-28
        #   @brief          Get RfUId and Chip ID of RF chips
        FpgaCmd         =   zeros(2,dtype='uint32')
        Cod             =   int('0x901A',0)
        FpgaCmd[0]      =   Mask
        FpgaCmd[1]      =   1
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_GetRfChipSts Ret:", Ret)
        return Ret


    def     Fpga_MimoRstSequencer(self):
        FpgaCmd         =   zeros(1,dtype='uint32')
        Cod             =   int('0x9212',0)
        FpgaCmd[0]      =   1
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_MimoRstSequencer Ret:", Ret)
        return  Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Transmit data over USPI interface
    #>
    #> This function is used to access devices connected to the USPI interface. The specified data is written over the channel
    #> and the received data is returned.
    #>
    #> @param[in]   USpiCfg
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the USPI device <br>
    #>          Commonly multiple USPI devices are present; Therefore the Mask field is important to address the desired device
    #>      -   <span style="color: #ff9900;"> 'Chn': </span> Channel for data transfer <br>
    #>          The USPI supports up to four channels, hence the channel can be set in the range from 0 to 3
    #> @param[in]   Regs: data transmitted over the USPI interface
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetUSpiData(self, dUSpiCfg, Regs):
        # Regs        =   Regs.flatten()
        if (len(Regs) > 28):
            Regs    =   Regs[0:28];

        FpgaCmd         =   zeros(int(3 + len(Regs)), dtype='uint32')
        Cod             =   int('0x9017',0)
        FpgaCmd[0]      =   dUSpiCfg["Mask"]
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   dUSpiCfg["Chn"]
        FpgaCmd[3:]     =   Regs
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetUSpiData Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Transmit data over USPI8 interface
    #>
    #> This function is used to access devices connected to the USPI8 interface. The specified data is written over the channel
    #> and the received data is returned.
    #>
    #> @param[in]   USpiCfg
    #>      -   <span style="color: #ff9900;"> 'Mask': </span> Bitmask to identify the USPI device <br>
    #>          Commonly multiple USPI devices are present; Therefore the Mask field is important to address the desired device
    #>      -   <span style="color: #ff9900;"> 'Chn': </span> Channel for data transfer <br>
    #>          The USPI8 supports up to eight channels, hence the channel can be set in the range from 0 to 7
    #> @param[in]   Regs: data transmitted over the USPI interface
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetUSpi8Data(self, dUSpiCfg, Regs):
        Regs        =   array(Regs)
        # Regs        =   Regs.flatten()
        if (len(Regs) > 28):
            Regs    =   Regs[0:28]

        FpgaCmd         =   zeros(int(3 + len(Regs)), dtype='uint32')
        Cod             =   int('0x9018',0)
        FpgaCmd[0]      =   dUSpiCfg["Mask"]
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   dUSpiCfg["Chn"]
        FpgaCmd[3:]     =   Regs
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetUSpi8Data Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set IO
    #> Multiplexer to output
    #>
    #> This function is used to configure the bits of the Mask register
    #> to output.
    def     Fpga_IoMuxRdnSetOutput(self, Mask):
        FpgaCmd         =   zeros(3,dtype='uint32')
        Cod             =   int('0x9405',0)
        FpgaCmd[0]      =   1
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   Mask
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set IO
    #> Multiplexer to output
    #>
    #> This function is used to configure the bits of the Mask register
    #> to output.
    def     Fpga_IoMuxRdnSetInput(self, Mask):
        FpgaCmd         =   zeros(3,dtype='uint32')
        Cod             =   int('0x9405',0)
        FpgaCmd[0]      =   1
        FpgaCmd[1]      =   2
        FpgaCmd[2]      =   Mask
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set IO
    #> Multiplexer to output
    #>
    #> This function is used to configure the bits of the Mask register
    #> to output.
    def     Fpga_IoMuxRdnSetReg(self, Adr, Val):
        FpgaCmd         =   zeros(4,dtype='uint32')
        Cod             =   int('0x9405',0)
        FpgaCmd[0]      =   1
        FpgaCmd[1]      =   3
        FpgaCmd[2]      =   Adr
        FpgaCmd[3]      =   Val
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set Mux Select Port
    #> Bit 0: = 0 -> RfTrig (internal), = 1 -> ExtTrigIn (from external connector)
    #>
    #> This function is used to configure the bits of the Mask register
    #> to output.
    def     Fpga_SetMuxSel(self, Val):
        FpgaCmd         =   zeros(3,dtype='uint32')
        Cod             =   int('0x9406',0)
        FpgaCmd[0]      =   1
        FpgaCmd[1]      =   1
        FpgaCmd[2]      =   Val
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Set system data
    #>
    #> Support of function depends on FPGA software framework
    #>
    #> @param[in]   Field: number selecting field
    #>
    #> @param[in]   Val: Value for the selected field
    #>
    #>  @note This function is a low level function. Low level functions are not supported by all baseboards.
    def     Fpga_SetSysCfg(self, Field, Val):
        Cod             =   int('0x9019',0);
        FpgaCmd         =   zeros(2, dtype='uint32')
        FpgaCmd[0]      =   Field
        FpgaCmd[1]      =   Val
        Ret             =   self.CmdSend(0, Cod, FpgaCmd)
        Ret             =   self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetSysCfg Ret:", Ret)
        return Ret

    def Fpga_GetBrdSts(self, Sel):
        Cod = int('0x9005',0)
        FpgaCmd = zeros(3, dtype='uint32')
        FpgaCmd[0] = 1
        FpgaCmd[1] = Sel
        FpgaCmd[2] = 1

        Ret = self.CmdSend(0,Cod,FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def Fpga_SetRole(self, Role, AdcSel):
        Cod         =   int('0x9004', 0)
        FpgaCmd     =   zeros(3, dtype='uint32')
        FpgaCmd[0]  =   1
        FpgaCmd[1]  =   Role
        FpgaCmd[2]  =   AdcSel
        Ret         =   self.CmdSend(0, Cod, FpgaCmd)
        Ret         =   self.CmdRecv()
        return Ret


    ## @brief Calculate gain setting for AFE5801
    #
    #  The function selects the closest to the desired setting.
    #  Setting is stored to the register of the class; and FuSca constant is adjusted for the selected gain setting
    #
    #  @param[in]  Val  desired gain value in dB
    def     SetAfe5801GainCoarse(self, Val):
        if Val > 28:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_31DB
            self.GaindB = 31
        elif Val > 22.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_25DB
            self.GaindB = 25
        elif Val > 17.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_20DB
            self.GaindB = 20
        elif Val > 12.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_15DB
            self.GaindB = 15
        elif Val > 7.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_10DB
            self.GaindB = 10
        elif Val > 2.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_05DB
            self.GaindB = 5
        elif Val > -0.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_00DB
            self.GaindB = 0
        elif Val > -1.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_M1DB
            self.GaindB = -1
        elif Val > -2.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_M2DB
            self.GaindB = -2
        elif Val > -3.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_M3DB
            self.GaindB = -3
        elif Val > -4.5:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_M4DB
            self.GaindB = -4
        else:
            self.Rad_Afe5801_RegTgcGainCoarse = self.AFE5801_REG_TGC_GAIN_M5DB
            self.GaindB = -5
        Gain = 10**(self.GaindB/20)
        self.FuSca = 2/2**12/Gain

    # DOXYGEN ------------------------------------------------------
    #> @brief Calculate high pass setting for AFE5801
    #>
    #> The function selects the closest to the desired setting.
    #> Setting is stored to the register of the class; and FuSca constant is adjusted for the selected gain setting
    #>
    #> @param[in]  Val: desired gain value in dB
    def     SetAfe5801HighPass(self, Val):
        if Val >= 10:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K10 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K10 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 9:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K9 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K9 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 8:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K8 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K8 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 7:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K7 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K7 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 6:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K6 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K6 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >=5:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K5 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K5 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 4:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K4 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K4 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 3:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K3 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K3 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 2:
            self.Rad_Afe5801_RegHighpass14       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K2 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58       =   self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K2 + self.AFE5801_REG_HIGHPASS_ENABLE
        else:
            self.Rad_Afe5801_RegHighpass14       =   0
            self.Rad_Afe5801_RegHighpass58       =   0

    ## @brief Calculate actual AFE5801 sampling frequency
    #
    #  The function returns the acutal sampling frequency of the AFE5801
    #
    #  @return  fAdc  AFE5801 sampling frequency
    def     GetAfeFreq(self):
        fAdc = 20e6
        return fAdc

    def RevStr(self, Val):
        #   @function    RevStr
        #   @author      Haderer Andreas (HaAn)
        #   @date        2013-12-01
        #   @brief       Convert revision number ot revision string (Maj.Min.Patch)
        RevMaj  =   int(floor(Val/2**16))
        RevMin  =   int(floor(Val/2**8) % 2**8)
        RevPat  =   int(Val % 2**8)
        stRev   =   str(RevMaj) + '.' + str(RevMin) + '.' + str(RevPat)
        return stRev

    def Fpga_GetSwCfgString(self):
        Result      =   self.Fpga_GetSwCfg()
        return      "SwVers: " + str(Result[2]) + "-" + str(Result[1]) + "-" + str(Result[0])

    def GetAdcChn(self, *varargin):
        #   @function    GetAdcChn
        #   @author      Haderer Andreas (HaAn)
        #   @date        2013-12-01
        #   @brief       Return nr of configured channels for a single AD8283
        #   @param[in]   AdcCfg (optional): If no parameter is provided, then the field of the obj is analyzed
        #                Required field:
        #                   ChCtrl          Channel control register of ADC
        Nr          =   0
        if len(varargin) > 0:
            dAdcCfg     =   varargin[0]
            if 'ChCtrl' in dAdcCfg:
                if AdcCfg.ChCtrl == self.cAD8283_REG_MUXCNTRL_AB:
                    Nr  =   2
                elif dAdcCfg["ChCtrl"] == self.cAD8283_REG_MUXCNTRL_ABC:
                    Nr  =   3
                elif dAdcCfg["ChCtrl"] ==  self.cAD8283_REG_MUXCNTRL_ABCD:
                        Nr  =   4;
                elif dAdcCfg["ChCtrl"] ==  self.cAD8283_REG_MUXCNTRL_ABCDE:
                        Nr  =   5;
                elif dAdcCfg["ChCtrl"] ==  self.cAD8283_REG_MUXCNTRL_ABCDEF:
                        Nr  =   6;
                else:
                    print('ADC Cfg: Error channel setting!')
                    Nr  =   -1
            else:
                print('ADC Cfg: ChCtrl field missing!')
        else:
            if self.Rad_AdcCfg_ChCtrl == self.cAD8283_REG_MUXCNTRL_AB:
                Nr  =   2
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABC:
                Nr  =   3
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABCD:
                Nr  =   4
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABCDE:
                Nr  =   5
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABCDEF:
                Nr  =   6;
            else:
                print('ADC Cfg: Error channel setting!')
                Nr  =   -1;
        return Nr


    def RcsCornerCube(self, fc, aCorner):
        #   @file        RefCornerCube.m
        #   @author      Andreas Haderer
        #   @date        2014-05-05
        #   @brief       Calcuate RCS of corner cube, aCorner: length of connecting edge
        c0          =   3e8
        Lambdac     =   c0/fc
        Rcs         =   4*pi*aCorner**4/(3*Lambdac**2)
        return      Rcs

    def GetAdcImp(self, *varargin):
        #   @function    GetAdcImp
        #   @author      Haderer Andreas (HaAn)
        #   @date        2013-12-01
        #   @brief       Return AD8283 Input Impedance
        #   @param[in]   AdcCfg (optional): If no parameter is provided, then the field of the obj is analyzed
        #                Required field:
        #                   ImpCtrl          Channel impedance control register of ADC

        Imp         =   0
        if len(varargin) > 0:
            dAdcCfg  =   varargin[0]
            if 'ImpCtrl' in dAdcCfg:
                if dAdcCfg["ChCtrl"] == self.cAD8283_REG_CHIMP_200E:
                    Imp     =   200
                elif dAdcCfg["ChCtrl"] == self.cAD8283_REG_CHIMP_200K:
                    Imp     =   200e3
                else:
                    print('ADC Cfg: Error impedance setting!')
                    Imp     =   -1
            else:
                print('ADC Cfg: ImpCtrl field missing!')
        else:
            if self.Rad_AdcCfg_ImpCtrl == self.cAD8283_REG_CHIMP_200E:
                Imp     =   200
            elif self.Rad_AdcCfg_ImpCtrl == self.cAD8283_REG_CHIMP_200K:
                Imp     =   200e3
            else:
                print('ADC Cfg: Error impedance setting!')
                Imp     =   -1
        return Imp

    def Num2Cal(self, Val):
        #   @function       Num2Cal
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Convert calibration number to s8.24 number format
        #   @param[in]      Double array with cal
        #   @return         S8.24 values for eeprom
        # if isreal(Val)
        #     Val         =   round(Val*2**24);
        #     Idcs        =   find(Val < 0);
        #     Val(Idcs)   =   Val(Idcs) + 2**32;
        #     Cal         =   Val;
        # else
        #     Val1        =   round(real(Val)*2**24);
        #     Idcs        =   find(Val1 < 0);
        #     Val1(Idcs)  =   Val1(Idcs) + 2**32;
        #     CalReal     =   Val1;
        #     Val1        =   round(imag(Val)*2**24);
        #     Idcs        =   find(Val1 < 0);
        #     Val1(Idcs)  =   Val1(Idcs) + 2**32;
        #     CalImag     =   Val1;
        #     Cal         =   CalReal + i.*CalImag;
        if isreal(Val).all:
            Cal                     =   array(Val*2**24, dtype=int32)
        else:
            pass

        return Cal

    def Cal2Num(self, Cal):
        #   @function       Cal2Num
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Convert calibration data eeprom number format s8.24 to double
        #   @param[in]      Cal data (S8.24 integer values)
        #   @return         cal data as double
        Ret         =   Cal/2**24

        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Generate hanning window
    #>
    #> This function returns a hanning window.
    #>
    #>  @param[in]   M: window size
    #>
    #>  @param[In]   N (optional): Number of copies in second dimension
    def     hanning(self, M, *varargin):
        #m   =   [-(M-1)/2: (M-1)/2].';
        m       = linspace(-(M-1)/2,(M-1)/2,M)
        Win     =   0.5 + 0.5*cos(2*pi*m/M)
        if len(varargin) > 0:
            N       =   varargin[0]
            Win     =   broadcast_to(Win,(N,M)).T
        return Win


    def ConvSerialNr(self, Nr):
        UId = floor(Nr/2**20)
        Nr = Nr - UId* 2**20
        Patch = floor(Nr/2**12)
        Serial = (Nr % 2**12)

        stSerial = '%04d%04d%04d' % (UId, Patch, Serial)
        return stSerial

    def GetSerialNr(self, Nr):
        UId = floor(Nr/2**20)
        Nr = Nr - UId * 2**20
        Patch = floor(Nr/2**12)
        Serial = (Nr % 2**12)
        return (UId, Patch, Serial)

    def DefBrdConst(self):
        #--------------------------------------------------------------------------
        # AFE5801
        self.AFE5801_REG_CONFIG_TGC_WREN                 = int('0x0004', 0)    # *< Use to enable access to TGC registers
        self.AFE5801_REG_CONFIG_READOUT_EN               = int('0x0002', 0)    # *< Use to enable register readout
        self.AFE5801_REG_CONFIG_SOFT_RESET               = int('0x0001', 0)    # *< Use to reset the AFE5801

        self.AFE5801_REG_POWER_OUTRATE2X                 = int('0x4000', 0)    # *<
        self.AFE5801_REG_POWER_EXT_REF                   = int('0x2000', 0)    # *<
        self.AFE5801_REG_POWER_LOW_FREQ_NOISE_SUPPRESS   = int('0x0800', 0)    # *<
        self.AFE5801_REG_POWER_STANDBY                   = int('0x0400', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH1                   = int('0x0004', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH2                   = int('0x0008', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH3                   = int('0x0010', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH4                   = int('0x0020', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH5                   = int('0x0040', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH6                   = int('0x0080', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH7                   = int('0x0100', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH8                   = int('0x0200', 0)    # *<
        self.AFE5801_REG_POWER_OUTPUT_DISABLE            = int('0x0002', 0)    # *<
        self.AFE5801_REG_POWER_GLOBAL_PDN                = int('0x0001', 0)    # *<

        self.AFE5801_REG_LVDS_MODE_NO_PATTERN            = int('0x0000', 0)    # *< normal ADC data output
        self.AFE5801_REG_LVDS_MODE_PATTERN_SYNC          = int('0x2000', 0)    # *< output pattern 111111000000
        self.AFE5801_REG_LVDS_MODE_PATTERN_DESKEW        = int('0x4000', 0)    # *< output pattern 010101010101
        self.AFE5801_REG_LVDS_MODE_PATTERN_CUSTOM        = int('0x6000', 0)    # *< output pattern <custom>
        self.AFE5801_REG_LVDS_MODE_PATTERN_ALL1          = int('0x8000', 0)    # *< output pattern 111111111111
        self.AFE5801_REG_LVDS_MODE_PATTERN_TOGGLE        = int('0xA000', 0)    # *< output pattern 111111111111 000000000000 ...
        self.AFE5801_REG_LVDS_MODE_PATTERN_ALL0          = int('0xC000', 0)    # *< output pattern 000000000000
        self.AFE5801_REG_LVDS_MODE_PATTERN_RAMP          = int('0xE000', 0)    # *< output increasing values
        self.AFE5801_REG_LVDS_AVERAGING_ENABLE           = int('0x0800', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS1                  = int('0x0008', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS2                  = int('0x0010', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS3                  = int('0x0020', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS4                  = int('0x0040', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS5                  = int('0x0080', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS6                  = int('0x0100', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS7                  = int('0x0200', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS8                  = int('0x0400', 0)    #

        self.AFE5801_REG_DATA_SERIAL_RATE12              = int('0x0000', 0)    #
        self.AFE5801_REG_DATA_SERIAL_RATE10              = int('0x2000', 0)    #
        self.AFE5801_REG_DATA_SERIAL_RATE16              = int('0x4000', 0)    #
        self.AFE5801_REG_DATA_SERIAL_RATE14              = int('0x6000', 0)    #
        self.AFE5801_REG_DATA_DIGITAL_GAIN_ENABLE        = int('0x1000', 0)    #
        self.AFE5801_REG_DATA_OFFSET_SUB_ENABLE          = int('0x0100', 0)    #

        self.AFE5801_REG_DFS_OFFSET_BINARY               = int('0x0008', 0)    # *< use if data format should be offset binary instead of default two's complement

        self.AFE5801_REG_FILTER_VCA_LOW_NOISE            = int('0x0400', 0)    # *< enable low noise, increases power consumtion!
        self.AFE5801_REG_FILTER_SELF_TEST_DISABLE        = int('0x0000', 0)    # *< disable self-test
        self.AFE5801_REG_FILTER_SELF_TEST_ENABLE100      = int('0x0100', 0)    # *< Self test with 100mV dc
        self.AFE5801_REG_FILTER_SELF_TEST_ENABLE150      = int('0x0180', 0)    # *< Self test with 150mV dc
        self.AFE5801_REG_FILTER_BW_14MHZ                 = int('0x0000', 0)    # *< 14MHz Filter bandwidth
        self.AFE5801_REG_FILTER_BW_10MHZ                 = int('0x0004', 0)    # *< 10MHz Filter bandwidth
        self.AFE5801_REG_FILTER_BW_7_5MHZ                = int('0x0008', 0)    # *< 7.5MHz Filter bandwidth
        self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING     = int('0x0002', 0)    # *< if used, internal DC coupled, else (=default) AC coupled

        self.AFE5801_REG_FILTER_BW_MASK                  = int('0x000C', 0)    # *< Bandwidth mask for filter

        self.AFE5801_REG_OFFSET_GAIN_GAIN_BITSHIFT       = 11
        self.AFE5801_REG_OFFSET_GAIN_OFFSET_BITSHIFT     = 2

        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB          = (0*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_2DB          = (1*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_4DB          = (2*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_6DB          = (3*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_8DB          = (4*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_0DB          = (5*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_2DB          = (6*2**11)

        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_4DB          = (7*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_6DB          = (8*2**11)

        self.AFE5801_REG_OFFSET_GAIN_GAIN_6_0DB          = (31*2**11)

        self.AFE5801_REG_HIGHPASS_ENABLE                 = int('0x0001', 0)    # *< enable digital highpass filter
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K2         = int('0x0002', 0)    # *< K=2, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K3         = int('0x0004', 0)    # *< K=3, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K4         = int('0x0006', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K5         = int('0x0008', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K6         = int('0x000A', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K7         = int('0x000C', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K8         = int('0x000E', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K9         = int('0x0010', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K10        = int('0x0012', 0)    # *< K=10, Datasheet-formula on page 22

        self.AFE5801_REG_CLAMP_DISABLE                   = int('0x4000', 0)

        self.AFE5801_REG_TGC_STARTGAIN_INTERP            = int('0x0080', 0)    # *< set the TGC step size, datasheet, rev.D, p.28
        self.AFE5801_REG_TGC_GAIN_MODE_SOFT_SYNC         = int('0x0020', 0)    # *< TGC running periodically without pulse on SYNC pin
        self.AFE5801_REG_TGC_GAIN_MODE_UNIFORM_GAIN      = int('0x0010', 0)    # *< Use a uniform gain, datasheet, rev.D, p.28
        self.AFE5801_REG_TGC_GAIN_MODE_STATIC_PGA        = int('0x0008', 0)    # *< Use static gain, only "fine" and "coarse" are active
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_0       = int('0x0000', 0)    # *< Fine gain, "+0dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_1       = int('0x0001', 0)    # *< Fine gain, "+0.125dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_2       = int('0x0002', 0)    # *< Fine gain, "+0.250dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_3       = int('0x0003', 0)    # *< Fine gain, "+0.375dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_4       = int('0x0004', 0)    # *< Fine gain, "+0.500dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_5       = int('0x0005', 0)    # *< Fine gain, "+0.625dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_6       = int('0x0006', 0)    # *< Fine gain, "+0.750dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_7       = int('0x0007', 0)    # *< Fine gain, "+0.875dB"

        self.AFE5801_REG_TGC_GAIN_M5DB                   = int('0x0000', 0)    # *< Coarse gain, "-5dB"
        self.AFE5801_REG_TGC_GAIN_M4DB                   = int('0x0001', 0)    # *< Coarse gain, "-4dB"
        self.AFE5801_REG_TGC_GAIN_M3DB                   = int('0x0002', 0)    # *< Coarse gain, "-3dB"
        self.AFE5801_REG_TGC_GAIN_M2DB                   = int('0x0003', 0)    # *< Coarse gain, "-2dB"
        self.AFE5801_REG_TGC_GAIN_M1DB                   = int('0x0004', 0)    # *< Coarse gain, "-1dB"
        self.AFE5801_REG_TGC_GAIN_00DB                   = int('0x0005', 0)    # *< Coarse gain, "0dB"
        self.AFE5801_REG_TGC_GAIN_05DB                   = int('0x000A', 0)    # *< Coarse gain, "5dB"
        self.AFE5801_REG_TGC_GAIN_10DB                   = int('0x000F', 0)    # *< Coarse gain, "10dB"
        self.AFE5801_REG_TGC_GAIN_15DB                   = int('0x0014', 0)    # *< Coarse gain, "15dB"
        self.AFE5801_REG_TGC_GAIN_20DB                   = int('0x0019', 0)    # *< Coarse gain, "20dB"
        self.AFE5801_REG_TGC_GAIN_25DB                   = int('0x001E', 0)    # *< Coarse gain, "25dB"
        self.AFE5801_REG_TGC_GAIN_31DB                   = int('0x0024', 0)    # *< Coarse gain, "31dB"
        #--------------------------------------------------------------------------
        # HMC703: Modes of operation
        #--------------------------------------------------------------------------
        self.cHMC703_REG_SDCFG_MOD_FRAC         =       0
        self.cHMC703_REG_SDCFG_MOD_INT          =       1
        self.cHMC703_REG_SDCFG_MOD_RAMP_1WAY    =       5
        self.cHMC703_REG_SDCFG_MOD_RAMP_2WAY    =       6
        self.cHMC703_REG_SDCFG_MOD_RAMP_CONT    =       7
        #--------------------------------------------------------------------------
        # Cic1 MMP: Control Register Bit Masks
        #--------------------------------------------------------------------------
        self.cCIC1_REG_CONTROL_EN               =   int("0x1", 0)       # Enable MMP*/
        self.cCIC1_REG_CONTROL_RST              =   int("0x2", 0)       # Reset the filter chain*/
        self.cCIC1_REG_CONTROL_RSTCNTRSOP       =   int("0x100", 0)     # Reset sample cntr on SoP signal*/
        self.cCIC1_REG_CONTROL_RSTFILTSOP       =   int("0x200", 0)     # Reset filter on SoP signal*/
        self.cCIC1_REG_CONTROL_RSTCNTREOP       =   int("0x400", 0)     # Reset sample cntr on EoP signal*/
        self.cCIC1_REG_CONTROL_RSTFILTEOP       =   int("0x800", 0)     # Reset filter on EoP signal*/
        self.cCIC1_REG_CONTROL_BYPASS           =   int("0x10000", 0)   # Bypass Cic filter */
        #--------------------------------------------------------------------------
        # FrmCtrl MMP: Channel Control Register Bit Masks
        #--------------------------------------------------------------------------
        self.cFRMCTRL_REG_CH0CTRL_TESTENA       =   int("0x1", 0)       # test mode enable*/
        self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA    =   int("0x2", 0)       # frame counter enable*/
        self.cFRMCTRL_REG_CH0CTRL_PADENA        =   int("0x4", 0)       # frame padding enable*/
        self.cFRMCTRL_REG_CH0CTRL_WAITENA       =   int("0x8", 0)       # wait enable*/
        self.cFRMCTRL_REG_CH0CTRL_GLOBWAITENA   =   int("0x10", 0)      # global wait signal enable (or of all channels)*/
        self.cFRMCTRL_REG_CH0CTRL_SOFTID        =   int("0x100", 0)     # frame identification number set by software


        #--------------------------------------------------------------------------
        # SeqTrig MMP: Configure Timing of Sequence Trigger
        #--------------------------------------------------------------------------
        self.cSEQTRIG_REG_CTRL_RST                      =   int("0x1", 0)       # Rst MMP*/
        self.cSEQTRIG_REG_CTRL_CH0ENA                   =   int("0x10", 0)      # Enable output channel 0*/
        self.cSEQTRIG_REG_CTRL_CH1ENA                   =   int("0x20", 0)      # Enable output channel 1*/
        self.cSEQTRIG_REG_CTRL_CH2ENA                   =   int("0x40", 0)      # Enable output channel 2*/
        self.cSEQTRIG_REG_CTRL_CH3ENA                   =   int("0x80", 0)      # Enable output channel 3*/
        self.cSEQTRIG_REG_CTRL_IRQ0ENA                  =   int("0x100", 0)     # Enable channel 0 sys irq*/
        self.cSEQTRIG_REG_CTRL_IRQ1ENA                  =   int("0x200", 0)     # Enable channel 1 sys irq*/
        self.cSEQTRIG_REG_CTRL_IRQ2ENA                  =   int("0x400", 0)     # Enable channel 2 sys irq*/
        self.cSEQTRIG_REG_CTRL_IRQ3ENA                  =   int("0x800", 0)     # Enable channel 3 sys irq*/
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_ENA              =   int("0x1", 0)           # sequence counter enable */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN            =   int("0x2", 0)           # sequence counter syncn enable */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE           =   int("0x4", 0)           # sequence counter external trigger for sequence update */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_ADRSELENA        =   int("0x8", 0)           # sequence address select enable */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_ADRSELMOD        =   int("0x10", 0)          # address update mode (1 single shoot, 0 continuous) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_RELOAD           =   int("0x20", 0)          # Reload counter */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA   =   int("0x100", 0)         # interval enable for chn0 (Chn0Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA   =   int("0x200", 0)         # interval enable for chn0 (Chn1Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN2_INTVALENA   =   int("0x400", 0)         # interval enable for chn0 (Chn2Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA   =   int("0x800", 0)         # interval enable for chn0 (Chn3Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CHNTIM_ENA                =   int("0x80000000", 0)    # channel tim enable */
        #--------------------------------------------------------------------------
        # FUsb Configuration
        #--------------------------------------------------------------------------
        self.Rad_FUsbCfg_Mask                      =   int(1)
        self.Rad_FUsbCfg_ChnEna                    =   int(255)
        self.Rad_FUsbCfg_DatSiz                    =   int(1024)
        self.Rad_FUsbCfg_WaitLev                   =   int(1024 + 512)
        #--------------------------------------------------------------------------
        # Constants for power supply
        #--------------------------------------------------------------------------
        self.cPWR_RF_P1_ON          =   1
        self.cPWR_RF_P2_ON          =   2
        self.cPWR_RF_P3_ON          =   4
        self.cPWR_RF_ON             =   7
        self.cPWR_RF_OFF            =   0

        self.cRAD_ROLE_MS = 0
        self.cRAD_ROLE_MSCLKOUT = 1
        self.cRAD_ROLE_SL = 128
        self.RAD_ROLE_SLCLKIN = 129
