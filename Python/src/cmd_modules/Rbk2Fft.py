# Radarbook2.py -- Radarbook2 class
#
# Copyright (C) 2017-04 Inras GmbH Haderer Andreas
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE file for details.

# Version 1.0.0

# Version 1.1.0
#   Get Fft Data from RadServe

# Version 1.2.0
#   Get LoMax Data from RadServe

# Version 1.3.0
#   Fix LoMax with Video
#   Change LoMax Buffer Size for VidRate

# Version 1.4.0
# Add Parameters to File in case of LoMax is stored
# Add Scaling for LoMax calculation

import  sys
import  os
import  socket
import  struct
from    numpy import *
import  time
import  src.cmd_modules.Connection as Connection
import  src.cmd_modules.SeqTrig as SeqTrig

class Rbk2Fft(Connection.Connection):
    """ Radarbook2 class object:
        (c) Haderer Andreas Inras GmbH

        Communication and access of Radarbook2 device with TCPIP Connection
    """
    def __init__(self, stConType='PNet', *args):
        super(Rbk2Fft, self).__init__('Radarbook2', stConType, *args)

        self.stVers = "1.4.0"
        self.Sts = "Not connected"                 # Status of object
        #self.stIpAdr = stIpAdr                        # Ip Address
        #self.CfgPort = 8001
        #self.DatPort = 6000
        self.HUid = 0                               # Hardware UID of radarbook
        self.SUid = 0                               # Software UID of radarbook FPGA framework
        self.SVers = 0

        # define Brd constants
        self.DefBrdConst()

        #self.cDebugInf = 0
        self.stName = "Rbk2Fft"
        #self.hCon = -1
        #self.hConDat = -1

        #------------------------------------------------------------------
        # Configure Sampling
        #------------------------------------------------------------------
        self.Rad_N = 1248
        self.Rad_NMult = 1
        self.Rad_Rst = 1

        self.Rad_SampCfg_Nr = 1248-1
        self.Rad_SampCfg_Rst = 1

        self.Rad_MaskChn = 2**8-1
        self.Rad_MaskChnEna = 2**8-1
        self.Rad_MaskChnRst = 0

        self.Rad_Afe5801_Mask = 3
        self.Rad_Afe5801_RegConfig = 0
        self.Rad_Afe5801_RegPower = 0
        self.Rad_Afe5801_RegLvds = int('0x0', 0)       # AFE5801_REG_LVDS_MODE_NO_PATTERN
        self.Rad_Afe5801_RegData = int('0x0', 0)       # AFE5801_REG_DATA_SERIAL_RATE12
        self.Rad_Afe5801_RegDfs = 0
        self.Rad_Afe5801_RegLvdsPattern = 8
        self.Rad_Afe5801_RegFilter = int('0x08', 0)      # AFE5801_REG_FILTER_BW_7_5MHZ
        self.Rad_Afe5801_RegOffsetGainCh1 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh2 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh3 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh4 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegHighpass14 = 0
        self.Rad_Afe5801_RegOffsetGainCh5 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh6 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh7 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegOffsetGainCh8 = (0*2**11)           # AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB
        self.Rad_Afe5801_RegHighpass58 = 0
        self.Rad_Afe5801_RegClamp = 0
        self.Rad_Afe5801_RegTgcStartIndex = 1
        self.Rad_Afe5801_RegTgcStopIndex = 1
        self.Rad_Afe5801_RegTgcStartGain = int('0x19', 0)     # AFE5801_REG_TGC_GAIN_20DB
        self.Rad_Afe5801_RegTgcHoldGainTime = 0
        self.Rad_Afe5801_RegTgcGainMode = int('0x08', 0)     # AFE5801_REG_TGC_GAIN_MODE_STATIC_PGA
        self.Rad_Afe5801_RegTgcGainCoarse = int('0x05', 0)     # AFE5801_REG_TGC_GAIN_10DB
        self.Rad_Afe5801_RegTgcUniformSlope = 0


        self.Rad_ArmMemCfg_Mask = 3
        self.Rad_ArmMemCfg_Ctrl = 0
        self.Rad_ArmMemCfg_Chn = 2**8-1
        self.Rad_ArmMemCfg_FrmSiz = 1024
        self.Rad_ArmMemCfg_WaitLev = 32768 - 4096
        self.Rad_ArmMemCfg_FifoSiz = 32768

        self.Rad_Role = 0
        self.Rad_fAdc = 20e6

        self.Rad_Mode = 'Raw'
        self.Rad_NrChn = 8
        self.Rad_NrDatChn = 2
        self.Rad_NrProcChn = 2              # processing channels
        self.Rad_EnaFrmCntr = 1
        self.Rad_NrBeams = 1
        self.Rad_FrmSiz = 1


        self.Rad_NrVirtChn = 8

        self.Rad_RangeWin_Type = 1  # Win Type: 0 = Custom; 1 = Boxcar; 2 = Hanning; 3 = Hamming
        self.Rad_RangeWin_Ctrl = 0 * 2**22 + 0 * 2**20 + 0 * 2**18 + 0 * 2**16 + 1 # Chn0 (2^16): 0 = In*Win; 1 = In; 2 = Win; 3 = zeros
        self.Rad_RangeWin_Coef = zeros((2048,1))
        self.Rad_RangeWin_Gain = 1

        self.Rad_RangeFftSiz = 1024
        self.Rad_RangeFftIdxMin = 0
        self.Rad_RangeFftIdxMax = 512 - 1
        self.Rad_RangeFftShift = 1

        self.Rad_VelWin_Type = 2    # Win Type: 0 = Custom; 1 = Boxcar; 2 = Hanning; 3 = Hamming
        self.Rad_VelWin_Ctrl = 0 * 2**22 + 0 * 2**20 + 0 * 2**18 + 0*2**16 + 1         # Chn0 (2^16): 0 = In*Win; 1 = In; 2 = Win; 3 = zeros
        self.Rad_VelWin_Gain = 2
        self.Rad_VelFftSiz = 256
        self.Rad_VelFftShift = 2

        self.Rad_DetMaxNrDet = 32
        self.Rad_DetStreamModulo = 20
        self.Rad_DetVelIdxMin0 = 0
        self.Rad_DetVelIdxMax0 = 512
        self.Rad_DetThres = 2.4
        self.Rad_DetInGain = 2

        self.Rad_DetThresCalcLimMin = 20
        self.Rad_DetThresCalcLimMax = 512-20

        self.Rad_BeamCoeff = []

        self.WinType_Custom = 0
        self.WinType_Boxcar = 1
        self.WinType_Hanning = 2
        self.WinType_Hamming = 3
        self.WinType_Gauss0_5 = 4
        self.WinType_Gauss0_4 = 5
        #------------------------------------------------------------------
        # Configure Cic filter
        #------------------------------------------------------------------
        self.Rad_CicCfg_FiltSel = 2**8-1                                          # Select Cic filters
        self.Rad_CicCfg_CombDel = 0                                               # Comb delay
        self.Rad_CicCfg_OutSel = 0                                               # Output select
        self.Rad_CicCfg_SampPhs = 0                                               # Sampling phase
        self.Rad_CicCfg_SampRed = 1                                               # Sampling rate reduction
        self.Rad_CicCfg_RegCtrl = int('0x800',0) + int('0x400',0) + int('0x1',0)  # Cic filter control register


        #------------------------------------------------------------------
        # Configure Frame Control MMP
        #------------------------------------------------------------------
        self.Rad_FrmCtrlCfg_ChnSel = 2**8 - 1                                            # Select Channels to be configured
        self.Rad_FrmCtrlCfg_Rst = 1                                                   # Reset MMP
        self.Rad_FrmCtrlCfg_RegChnCtrl = int('0x2',0) + int('0x8',0) + int('0x10',0)         # Channel control register
        self.Rad_FrmCtrlCfg_FrmCntrStrt = 1
        self.Rad_FrmCtrlCfg_FrmCntrInc = 1
        self.Rad_FrmCtrlCfg_RegCtrl = int('0x100',0)


        self.Rad_DmaMult = 1

        self.SeqCfg = { "Mask" : 1}

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

        self.Rad_StreamSiz = 1
        self.Rad_LoMaxRecCfg = 1
        self.Rad_NrRangeBins = 1
        self.Rad_NrFrms = 8
        self.Rad_2DFft = 1
        self.ScaRangeFft = 1
        self.ScaVelFft = 1
        self.ScaVelFftMag = 1

        self.Rad_NTx = 1258 # Number of samples for the data transmission
        self.Rad_DatSiz = 4

        self.FuSca = 0.25/2**12

        self.Rad_DmaSel = 0

        fSeqTrig = 20e6
        objSeqTrig = SeqTrig.SeqTrig(fSeqTrig)
        self.SeqCfg = objSeqTrig.ContUnifM1(1e-3)

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
        else:
            print('Update of SoC Firmware with webinterface required')
            print('FFT framework not installed on Soc')
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
                self.cDebugInf = varargin[0]
        elif stVal == 'Name':
            if len(varargin) > 0:
                if isinstance(varargin[0], str):
                    self.Name = varargin[0]
                self.ConSetFileParam(stVal, varargin[0], 'STRING');
        elif stVal == 'N':
            if len(varargin) > 0:
                Val = varargin[0]
                Val = floor(Val)
                Val = ceil(Val/8)*8
                self.Rad_N = Val
                self.ConSetFileParam('N', Val, 'INT');
        elif stVal == 'Samples':
            if len(varargin) > 0:
                Val = varargin[0]
                Val = floor(Val)
                Val = ceil(Val/8)*8
                self.Rad_N = Val
                self.ConSetFileParam('N', Val, 'INT');
        elif stVal == 'NTx':
            if len(varargin) > 0:
                Val = varargin[0]
                Val = floor(Val)
                Val = ceil(Val/2)*2
                self.Rad_NTx = Val
                self.ConSetFileParam('Rad_NTx', Val, 'INT');
        elif stVal  == 'CicDi':
            self.Rad_CicCfg_SampRed = 1
            self.Rad_CicCfg_RegCtrl = self.cCIC1_REG_CONTROL_BYPASS + self.cCIC1_REG_CONTROL_EN
        elif stVal  == 'CicEna':
            self.Rad_CicCfg_SampRed = 1
            self.Rad_CicCfg_RegCtrl = self.cCIC1_REG_CONTROL_EN + self.cCIC1_REG_CONTROL_RSTCNTREOP  + self.cCIC1_REG_CONTROL_RSTFILTEOP
        elif stVal == 'CicStages':
            if len(varargin) > 0:
                Val = floor(varargin[0] - 1)
                if Val > 3:
                    Val = 3
                if Val < 0:
                   Val = 0
                self.Rad_CicCfg_OutSel = Val
        elif stVal ==  'CicDelay':
            if len(varargin) > 0:
                Val = varargin[0]
                Val = floor(log2(Val)-1)
                if Val > 3:
                    Val = 3
                if Val < 0:
                    Val = 0
                self.Rad_CicCfg_CombDel = Val
        elif stVal ==  'CicR':
            if len(varargin) > 0:
                Val = floor(varargin[0])
                if Val > 2**10:
                    Val = 2**10
                if Val < 1:
                    Val = 1
                self.Rad_CicCfg_SampRed = Val
        elif stVal ==  'DmaMult':
            if len(varargin) > 0:
                Val = floor(varargin[0])
                if Val > 2048:
                    Val = 2048
                if Val < 1:
                    Val = 1
                self.Rad_DmaMult = Val
                self.ConSetFileParam('DmaMult', Val, 'INT')
        elif stVal ==  'NrChn':
            if len(varargin) > 0:
                NrChn = floor(varargin[0])
                if NrChn < 1:
                    NrChn = 1
                if NrChn > 16:
                    NrChn = 16
                print('Set NrChn to: ', NrChn)
                if NrChn >= 8:
                    self.GpsFrmCtrl = 2 + 4
                elif NrChn >= 4:
                    self.GpsFrmCtrl = 2
                else:
                    self.GpsFrmCtrl = 0

                Mask12 = 2**12 - 1
                Mask = 2**NrChn - 1
                self.Rad_CicCfg_FiltSel = Mask
                self.Rad_NrChn = NrChn
                self.Rad_NrVirtChn = NrChn
                self.Rad_DetStreamModulo = 4 + NrChn

                self.Computation.SetNrChn(NrChn);
                self.ConSetFileParam('NrChn', NrChn, 'INT')
        elif stVal == 'NrDatChn':
            if len(varargin) > 0:
                NrDatChn = floor(varargin[0])
                if NrDatChn < 1:
                    NrDatChn = 1
                elif NrDatChn > 2:
                    NrDatChn = 2
                print('Set NrDatChn to: ', NrDatChn)
                if NrDatChn >= 4:
                    self.GpsFrmCtrl = 2
                else:
                    self.GpsFrmCtrl = 0
                Mask = 2**NrDatChn - 1
                self.Rad_ArmMemCfg_Chn = Mask
                self.Rad_FrmCtrlCfg_ChnSel = Mask
                self.Rad_NrDatChn = NrDatChn
                self.ConSetFileParam('NrDatChn', NrDatChn, 'INT')
        elif stVal == 'NrProcChn':
            if len(varargin) > 0:
                NrProcChn = floor(varargin[0])
                if NrProcChn < 1:
                    NrProcChn = 1
                if NrProcChn > 2:
                    NrProcChn = 2
                print('Set NrProcChn to: ', NrProcChn)
                self.Rad_NrProcChn = NrProcChn;
                self.ConSetFileParam('NrProcChn', NrProcChn, 'INT')
        elif stVal == 'EnaFrmCntr':
            self.Rad_EnaFrmCntr = 1
            self.ConSetFileParam('Rad_EnaFrmCntr', 1, 'INT')
        elif stVal == 'DiFrmCntr':
            self.Rad_EnaFrmCntr = 0
            self.ConSetFileParam('Rad_EnaFrmCntr', 0, 'INT')
        elif stVal == 'FrmCntrStrt':
            if len(varargin) > 0:
                self.Rad_FrmCtrlCfg_FrmCntrStrt = floor(varargin[0]);
                self.ConSetFileParam('Rad_FrmCtrlCfg_FrmCntrStrt', floor(varargin[0]), 'INT')
        elif stVal == 'Range_WinType':
            if len(varargin) > 0:
                stVal = varargin[0];
                if isinstance(stVal, str):
                    if stVal == 'Custom':
                        self.Rad_RangeWin_Type = self.WinType_Custom
                    elif stVal == 'Boxcar':
                        self.Rad_RangeWin_Type = self.WinType_Boxcar
                    elif stVal == 'Hanning':
                        self.Rad_RangeWin_Type = self.WinType_Hanning
                    elif stVal == 'Hamming':
                        self.Rad_RangeWin_Type = self.WinType_Hamming
                    elif stVal == 'Gauss0_5':
                        self.Rad_RangeWin_Type = self.WinType_Gauss0_5
                    elif stVal == 'Gauss0_4':
                        self.Rad_RangeWin_Type = self.WinType_Gauss0_4
                    self.ConSetFileParam('Rad_RangeWin_Type', self.Rad_RangeWin_Type, 'INT')
                else:
                    self.Rad_RangeWin_Type = uint8(varargin[0])
                    self.ConSetFileParam('Rad_RangeWin_Type', self.Rad_RangeWin_Type, 'INT')
        elif stVal == 'Range_Win':
            if len(varargin) > 0:
                Vals = varargin[0]
                self.Rad_RangeWin_Coef = Vals[:]
        elif stVal == 'Range_WinGain':
            if len(varargin) > 0:
                Val = varargin[0]
                self.Rad_RangeWin_Gain = Val
                self.ConSetFileParam('Rad_RangeWin_Gain', self.Rad_RangeWin_Gain, 'DOUBLE');
        elif stVal == 'Range_EnaOutputWin':
            self.Rad_RangeWin_Ctrl = 0 * 2**22 + 2 * 2**20 + 0 * 2**18 + 2 * 2**16 + 1
            self.ConSetFileParam('Rad_RangeWin_Ctrl', self.Rad_RangeWin_Ctrl, 'INT')
        elif stVal == 'Range_DiOutputWin':
            self.Rad_RangeWin_Ctrl = 0 * 2**22 + 0 * 2**20 + 0 * 2**18 + 0 * 2**16 + 1
            self.ConSetFileParam('Rad_RangeWin_Ctrl', self.Rad_RangeWin_Ctrl, 'INT')
        elif stVal == 'Range_FftSiz':
            if len(varargin) > 0:
                Val = varargin[0];
                Val = 2**(ceil(log2(Val)))
                if Val > 2048:
                    Val = 2048
                elif Val < 32:
                    Val = 32
                self.Rad_RangeFftSiz = Val
                self.ConSetFileParam('Range_NFFT', uint32(Val), 'INT')
                self.ConSetFileParam('Rad_RangeFftSiz', uint32(Val), 'INT')
        elif stVal == 'Range_FftIdxMin':
            if len(varargin) > 0:
                Val = varargin[0]
                self.Rad_RangeFftIdxMin = Val
                self.ConSetFileParam('Range_IdxMin', uint32(Val), 'INT')
                self.ConSetFileParam('Rad_RangeFftIdxMin', uint32(Val), 'INT')
        elif stVal == 'Range_FftIdxMax':
            if len(varargin) > 0:
                Val = varargin[0]
                self.Rad_RangeFftIdxMax = Val
                self.ConSetFileParam('Range_IdxMax', uint32(Val), 'INT')
                self.ConSetFileParam('Rad_RangeFftIdxMax', uint32(Val), 'INT')
        elif stVal == 'Range_FftSym':
            if len(varargin) > 0:
                Val = varargin[0];
                if Val > 1:
                    Val = 1
                elif Val < 0:
                    Val = 0
                self.Rad_RangeFftShift = Val
                self.ConSetFileParam('Rad_RangeFftShift', uint32(Val), 'INT')
        elif stVal == 'Vel_EnaOutputWin':
            self.Rad_VelWin_Ctrl = 3 * 2**22 + 3 * 2**20 + 3 * 2**18 + 2 * 2**16 + 1
            self.ConSetFileParam('Rad_VelWin_Ctrl', uint32(self.Rad_VelWin_Ctrl), 'INT')
        elif stVal == 'Vel_DiOutputWin':
            self.Rad_VelWin_Ctrl = 0 * 2**22 + 0 * 2**20 + 0 * 2**18 + 0 * 2**16 + 1
            self.ConSetFileParam('Rad_VelWin_Ctrl', uint32(self.Rad_VelWin_Ctrl), 'INT')
        elif stVal == 'Vel_WinType':
            if len(varargin) > 0:
                stVal = varargin[0]
                if isinstance(stVal, str):
                    if stVal == 'Custom':
                        self.Rad_VelWin_Type = self.WinType_Custom
                    if stVal == 'Boxcar':
                        self.Rad_VelWin_Type = self.WinType_Boxcar
                    if stVal == 'Hanning':
                        self.Rad_VelWin_Type = self.WinType_Hanning
                    if stVal == 'Hamming':
                        self.Rad_VelWin_Type = self.WinType_Hamming
                    if stVal == 'Gauss0_5':
                        self.Rad_VelWin_Type = self.WinType_Gauss0_5
                    if stVal == 'Gauss0_4':
                        self.Rad_VelWin_Type = self.WinType_Gauss0_4
                    self.ConSetFileParam('Rad_VelWin_Type', uint32(self.Rad_VelWin_Type), 'INT')
                else:
                    self.Rad_VelWin_Type = uint8(varargin[0])
                    self.ConSetFileParam('Rad_VelWin_Type', uint32(self.Rad_VelWin_Type), 'INT')

        elif stVal == 'Vel_WinGain':
            if len(varargin) > 2:
                Val = varargin[0]
                self.Rad_VelWin_Gain = Val
                self.ConSetFileParam('Rad_VelWin_Gain', Val, 'DOUBLE')

        elif stVal == 'Vel_FftSiz':
            if len(varargin) > 0:
                Val = varargin[0]
                Val = 2**(ceil(log2(Val)))
                if Val > 512:
                    Val = 512
                if Val < 32:
                    Val = 32
                self.Rad_VelFftSiz = Val
                self.ConSetFileParam('Rad_VelFftSiz', uint32(Val), 'INT')

        elif stVal == 'Vel_FftSym':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 1:
                    Val = 1
                if Val < 0:
                    Val = 0
                self.Rad_VelFftShift = Val
                self.ConSetFileParam('Rad_VelFftShift', uint32(Val), 'INT')
        elif stVal == 'RD_NrChirps':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 512:
                    Val = 512
                    print('RD_NrChips limited: NrChirps = 512')
                if Val < 1:
                    Val = 1
                    print('RD_NrChips limited: NrChirps = 1')
                self.Rad_NrFrms = Val
                self.Rad_DmaMult = Val
                self.ConSetFileParam('RD_NrChirps', uint32(Val), 'INT')

        elif stVal == 'Det_VelIdxMin0':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 512:
                    Val = 512
                if Val < 0:
                    Val = 0
                self.Rad_DetVelIdxMin0 = Val
                self.ConSetFileParam('Det_VelIdxMin0', uint32(Val), 'INT')
        elif stVal == 'Det_VelIdxMax0':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 512:
                    Val = 512
                if Val < 0:
                    Val = 0
                self.Rad_DetVelIdxMax0 = Val
                self.ConSetFileParam('Rad_DetVelIdxMax0', uint32(Val), 'INT')
        elif stVal == 'Det_MaxNrDet':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 512:
                    Val = 512
                if Val < 1:
                    Val = 1
                self.Rad_DetMaxNrDet = Val
                self.ConSetFileParam('Rad_DetMaxNrDet', uint32(Val), 'INT')
        elif stVal == 'Det_Thres':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 255:
                    Val = 255
                if Val < 1:
                    Val = 1
                self.Rad_DetThres = Val
                self.ConSetFileParam('Rad_DetThres', Val, 'DOUBLE')
        elif stVal == 'Det_InGain':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val > 4:
                    Val = 4
                if Val < 0.5:
                    Val = 0.5
                self.Rad_DetInGain = Val
                self.ConSetFileParam('Det_InGain', Val, 'DOUBLE')
        elif stVal == 'Det_ThresCalc_LimMin':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val < 0:
                    Val = 0
                self.Rad_DetThresCalcLimMin = Val
                self.ConSetFileParam('Rad_DetThresCalcLimMin', uint32(Val), 'INT')
        elif stVal == 'Det_ThresCalc_LimMax':
            if len(varargin) > 0:
                Val = varargin[0]
                if Val < 0:
                    Val = 0
                self.Rad_DetThresCalcLimMax = Val
                self.ConSetFileParam('Rad_DetThresCalcLimMax', uint32(Val), 'INT')
        elif stVal == 'Mode':
            if len(varargin) > 0:
                stVal = varargin[0];
                if isinstance(stVal, str):
                    self.Rad_Mode = stVal
                    self.ConSetFileParam('Rad_Mode', stVal, 'STRING')
        elif stVal == 'NrBeams':
            if len(varargin) > 0:
                self.Rad_NrBeams = floor(varargin[0]);
                if self.Rad_NrBeams < 1:
                    self.Rad_NrBeams = 1;
                elif self.Rad_NrBeams > 16:
                    self.Rad_NrBeams = 16
                self.ConSetFileParam('Rad_NrBeams', uint32(self.Rad_NrBeams), 'INT')
        elif stVal == 'BeamCoeff':
            if len(varargin) > 0:
                self.Rad_BeamCoeff = varargin[0];
        elif stVal == 'AfeFilt':
            if len(varargin) > 0:
                stFilt = varargin[0]
                if isinstance(stFilt, str):
                    if stFilt == '14MHz':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_BW_14MHZ
                        self.Rad_Afe5801_RegFilter = Val
                    if stFilt == '10MHz':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_BW_10MHZ
                        self.Rad_Afe5801_RegFilter = Val
                    if stFilt == '7.5MHz':
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_BW_7_5MHZ
                        self.Rad_Afe5801_RegFilter = Val
                else:
                    if stFilt > 12e6:
                        Val = self.Rad_Afe5801_RegFilter
                        Cmp = ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                        Val = Val & Cmp
                        Val = Val + self.AFE5801_REG_FILTER_BW_14MHZ
                        self.Rad_Afe5801_RegFilter = Val
                    else:
                        if stFilt > 8.75e6:
                            Val = self.Rad_Afe5801_RegFilter
                            Cmp = ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                            Val = Val & Cmp
                            Val = Val + self.AFE5801_REG_FILTER_BW_10MHZ
                            self.Rad_Afe5801_RegFilter = Val
                        else:
                            Val = self.Rad_Afe5801_RegFilter
                            Cmp = ~uint32(self.AFE5801_REG_FILTER_BW_MASK)
                            Val = Val & Cmp
                            Val = Val + self.AFE5801_REG_FILTER_BW_7_5MHZ
                            self.Rad_Afe5801_RegFilter = Val
            else:
                print('RadarLog Err: Set(AfeFilt) bandwidth not specified!')
        elif stVal  == 'AfeGaindB':
            if len(varargin) > 0:
                Val = varargin[0]
                self.SetAfe5801GainCoarse(Val);
            else:
                print('RadarLog Err: Set(AfeGaindB) gain not specified!')
        elif stVal == 'AfeHighPass':
            if len(varargin) > 0:
                Val = varargin[0]
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


        elif stVal == 'GpsRstFifo':

            if len(varargin) > 0:
                Val = floor(varargin[0])
                if Val > 0:
                    self.GpsRstFifoOnTimStamp = 8
                else:
                    self.GpsRstFifoOnTimStamp = 0

        elif stVal == 'FrmSiz':
            if len(varargin) > 0:
                self.Rad_FrmSiz = floor(varargin[0])
        else:
            print('Parameter not known: ', stVal)


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
                Ret = self.cDebugInf
            elif stVal == 'Name':
                Ret = self.Name
            elif stVal == 'N':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'INT');
                    self.Rad_N = Ret;
                else:
                    Ret = int(self.Rad_N)
            elif stVal == 'Samples':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'INT');
                    self.Rad_N = Ret;
                else:
                    Ret = self.Rad_N
            elif stVal == 'NrDat':
                # calculate number of samples to
                Samp = self.Rad_N;
                if (int(self.Rad_FrmCtrlCfg_RegChnCtrl) & int(self.cFRMCTRL_REG_CH0CTRL_PADENA)) == int(self.cFRMCTRL_REG_CH0CTRL_PADENA):
                    Samp = Samp - floor(self.Rad_FrmCtrlCfg_RegChnCtrl/ 2**16) - 1
                Ret = int(Samp)
            elif stVal == 'CicStages':
                Ret = 1
            elif stVal == 'CicDelay':
                Ret = 2**(self.Rad_CicCfg_CombDel + 1)
            elif stVal == 'CicR':
                Ret = self.Rad_CicCfg_SampRed
            elif stVal == 'fAdc':
                Ret = self.Rad_fAdc
            elif stVal == 'fs':
                Ret = self.Rad_fAdc/self.Rad_CicCfg_SampRed
            elif stVal == 'NrChn':
                if self.cType == 'RadServe' and self.cReplay:
                    Ret = self.ConGetFileParam(stVal, 'INT');
                    self.Computation.SetNrChn(Ret);
                    self.Rad_NrChn = Ret;
                else:
                    Ret = int(self.Rad_NrChn)
            elif stVal == 'NrProcChn':
                Ret = self.Rad_NrProcChn
            elif stVal == 'NrDatChn':
                Ret = self.Rad_NrDatChn
            elif stVal == 'AfeGaindB':
                Ret = self.GaindB;
            elif stVal == 'AdcGaindB':
                Ret = self.GaindB;
            elif stVal == 'FuSca':
                Ret = self.FuSca
            elif stVal == 'AfeIntDcCoupling':
                Val = uint32(self.Rad_Afe5801_RegFilter)
                Cmp = uint32(self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING)
                Val = Val & Cmp
                if Val > 0:
                    Ret = 1
                else:
                    Ret = 0
            elif stVal == 'Range_FftSiz':
                Ret = self.Rad_RangeFftSiz
            elif stVal == 'Range_FftIdxMin':
                Ret = self.Rad_RangeFftIdxMin
            elif stVal == 'Range_FftIdxMax':
                Ret = self.Rad_RangeFftIdxMax
            elif stVal == 'Vel_FftSiz':
                Ret = self.Rad_VelFftSiz
            elif stVal == 'Det_MaxNrDet':
                Ret = self.Rad_DetMaxNrDet
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
            elif stVal == 'AfeIntDcCoupling':
                Val = uint32(self.Rad_Afe5801_RegFilter)
                Cmp = uint32(self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING)
                Val = Val & Cmp
                if Val > 0:
                    Ret = 1
                else:
                    Ret = 0

            else:
                print('Parameter not known: ', stVal)
                Ret = -1

        return Ret



    def Setfs (self):
        R = self.Rad_CicCfg_SampRed;
        self.Computation.SetParam('fs', self.Rad_fAdc/R);

    def SetFuSca (self):
        self.Computation.SetParam('FuSca', self.FuSca);

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
        Cmp1 = int(self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA)

        Samp = self.Rad_N

        if (int(self.Rad_FrmCtrlCfg_RegChnCtrl) & self.cFRMCTRL_REG_CH0CTRL_PADENA) == self.cFRMCTRL_REG_CH0CTRL_PADENA:
            print("Pad")
            Samp = Samp - floor(self.Rad_FrmCtrlCfg_RegChnCtrl/2**16) - 1

        dSampCfg = {
                                    "Nr"        :   Samp,
                                    "Rst"       :   1
                                }

        self.Fpga_SetSamp(dSampCfg)

        #--------------------------------------------------------------
        # Configure FArmMem Channels
        #--------------------------------------------------------------
        dArmMemCfg = {
                                    "Chn"            :   self.Rad_ArmMemCfg_Chn,                # Channel select Mask
                                    "Ctrl"           :   self.Rad_ArmMemCfg_Ctrl,               # Channel control register
                                    "FrmSiz"         :   self.Rad_NTx*self.Rad_NMult*self.Rad_DatSiz,
                                    "WaitLev"        :   self.Rad_ArmMemCfg_FifoSiz - self.Rad_NTx*self.Rad_NMult*self.Rad_DatSiz    # Frame wait level
                                }
        #--------------------------------------------------------------
        # Configure Frame Control
        #--------------------------------------------------------------
        dFrmCtrlCfg = {
                                    "ChnSel"              :   self.Rad_FrmCtrlCfg_ChnSel,                     # Select Channels to be configured
                                    "Rst"                 :   self.Rad_FrmCtrlCfg_Rst,                        # Reset MMP
                                    "RegChnCtrl"          :   self.Rad_FrmCtrlCfg_RegChnCtrl                  # Channel control register
                                }



        #----------------------------------------------------------
        # Start Sampling: Reset framecounter, and start sequence
        #----------------------------------------------------------
        self.Fpga_SetFrmCtrl(dFrmCtrlCfg)

        if self.Rad_2DFft > 0:
            #self.Fpga_CfgSigProc(ArmMemCfg);
            self.Fpga_CfgCpyDat(dArmMemCfg);

        else:
            self.Fpga_CfgArmMem(dArmMemCfg);


    def ChkFftCfg(self, dCfg):

        if self.Rad_Mode == 'Raw':

            if dCfg['N'] > 2048:
                print('Raw Mode: N > 2048 -> N set to 2048!')
                dCfg['N'] = 2048

            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    NrDatChn = 1
                else:
                    NrDatChn = 2
            else:
                NrDatChn = 1
            TimShift = NrChn/NrDatChn*dCfg['N']*1/100e6

            if TimShift > dCfg['Tp']:
                print('Raw Mode: Processing timing violated! Reduce number of channels or number of samples or increase chirp repetition interval')
                sys.exit()

        if self.Rad_Mode == 'FftCplx':

            if dCfg['N'] > 2048:
                print('FftCplx Mode: N > 2048 -> N set to 2048!')
                dCfg['N'] = 2048

            if dCfg['N'] > self.Rad_RangeFftSiz:
                print('FftCplx Mode: FftSiz < number of samples')
                sys.exit()

            if self.Rad_RangeFftIdxMin < 0:
                self.Rad_RangeFftIdxMin = 0
            if self.Rad_RangeFftIdxMax > self.Rad_RangeFftSiz - 1:
                self.Rad_RangeFftIdxMax = self.Rad_RangeFftSiz - 1

            NrVal = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
            if NrVal % 2 > 0:
                self.Rad_RangeFftIdxMax = self.Rad_RangeFftIdxMax - 1

            if self.Rad_RangeFftIdxMax < self.Rad_RangeFftIdxMin:
                print('FftCplx Mode: FftIdxMax < FftIdxMin' )
                sys.exit()

            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    NrDatChn = 1
                else:
                    NrDatChn = 2
            else:
                NrDatChn = 1

            TimShift = NrChn/NrDatChn*self.Rad_RangeFftSiz*1/100e6 + 1e-6
            print('FFT processing time: ', (TimShift/1e-6),' us')

            if TimShift > dCfg['Tp']:
                print('FFT Processing time: ', (TimShift/1e-6),' us')
                print('Min Tp: ', (dCfg['Tp']/1e-6),' us')
                print('FftCplx Mode: Processing timing violated! Reduce number of channels or FFT size or increase chirp repetition interval')
                sys.exit()

        if self.Rad_Mode == 'LoMax':

            if self.Rad_LoMaxRecCfg > 1:
                print('Magnitude RD map is output: If data rate is too high, RD map can be shifted (no flow control)')

            if dCfg['N'] > 2048:
                print('FftCplx Mode: N > 2048 -> N set to 2048!')
                dCfg['N'] = 2048

            if dCfg['N'] > self.Rad_RangeFftSiz:
                print('FftCplx Mode: FftSiz < number of samples')
                sys.exit()

            if self.Rad_RangeFftIdxMin < 0:
                self.Rad_RangeFftIdxMin = 0

            if self.Rad_RangeFftIdxMax > self.Rad_RangeFftSiz - 1:
                self.Rad_RangeFftIdxMax = self.Rad_RangeFftSiz - 1

            NrVal = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
            if NrVal % 2 > 0:
                self.Rad_RangeFftIdxMax = self.Rad_RangeFftIdxMax - 1

            if self.Rad_RangeFftIdxMax < self.Rad_RangeFftIdxMin:
                print('LoMax Mode: FftIdxMax < FftIdxMin' )
                sys.exit()

            if self.Rad_DmaMult != self.Rad_NrFrms:
                print('LoMax Mode: DmaMult is different from number or chirps')
                sys.exit()

            if self.Rad_NrFrms > self.Rad_VelFftSiz:
                print('LoMax Mode: Number of chirps greater than velocity FFT size')
                sys.exit()


            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    NrDatChn = 1
                else:
                    NrDatChn = 2
            else:
                NrDatChn = 1

            TimShift = NrChn/NrDatChn*self.Rad_RangeFftSiz*1/100e6 + 1e-6

            print('Range FFT processing time: ', (TimShift/1e-6),' us')

            if TimShift > dCfg['Tp']:
                print('FFT Processing time: ', (TimShift/1e-6),' us')
                print('Min Tp: ', (dCfg[Tp]/1e-6),' us')

                print('FftCplx Mode: Processing timing violated! Reduce number of channels or FFT size or increase chirp repetition interval')
                sys.exit()

            NrRangeBins = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1

            print('Estimated Doppler FFT processing time: ', (NrChn*NrRangeBins*self.Rad_VelFftSiz*10e-9/1e-3),' ms')


    def BrdCfgFft(self):

        N = self.Get('N');
        if self.Rad_Mode == 'Raw':
            self.Rad_2DFft = 0
            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    # for even number of channels only one data
                    # channel can be used
                    self.Set('NrDatChn', 1)
                    self.Set('NrProcChn', 1)
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn
                else:
                    self.Set('NrDatChn', 2)
                    self.Set('NrProcChn', 2)
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn/2
            else:
                self.Set('NrDatChn', 1)
                self.Set('NrProcChn', 1)
                self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn
            self.Rad_NTx = N
            self.Rad_DatSiz = 1
            self.Rad_FrmCtrlCfg_RegCtrl = self.cFRMCTRL_REG_CTRL_SWAPBYTES

            if self.Rad_RangeWin_Type == 0:
                self.Fpga_WinRe4Sc_ClrCoeff(1)
                self.Fpga_WinRe4Sc_SetCoeff(1, self.Rad_RangeWin_Coef)

            self.Fpga_MultSumRe16Sc_SetCplxUnitGain(1, 0, 1, 0, 1)

            # function Ret = Fpga_WinRe4Sc_Cfg(obj, Mask, Ctrl, WinSel, WinSiz, PadSiz, FftShift)
            # Mask 1 = range window
            self.Fpga_WinRe4Sc_Cfg(1,self.Rad_RangeWin_Ctrl, self.Rad_RangeWin_Type, N, N, 0, 1.0)
            # function Ret = Fpga_StRe16ToRe16Sc_CpyAllChn(obj, Mask, NrOut, FftShift)

            self.Fpga_StRe16ToRe16Sc_CpyAllChn(1, N, 0)
            self.Fpga_DatMux_SetMux(1, 0, 1)

            self.ConSet('InputType', self.Rad_Mode, self.Rad_NTx)

        elif self.Rad_Mode == 'RawCplx':
            # Channel 0 is real and channel 1 is imag part
            self.Rad_2DFft = 0
            print('RawCplx')

            NrChn = self.Get('NrChn')

            Dim = self.Rad_BeamCoeff.shape()
            Ny = Dim(0)
            Ny = Dim(1)

            if (Ny == NrChn) and (Nx == self.Rad_NrBeams):
                print('Configure Beam Coefficents')
                self.Fpga_MultSumRe16Sc_ClrCoeff(1)
                self.Fpga_MultSumRe16Sc_SetCoeff(1, 0, self.Rad_BeamCoeff)
                self.Fpga_MultSumRe16Sc_LoadCoeff(1)

            self.Rad_NTx = N
            self.Rad_DatSiz = 1
            self.Rad_FrmCtrlCfg_FrmCntrInc = self.Rad_NrBeams
            self.Rad_NrVirtChn = 2*self.Rad_NrBeams
            self.Rad_FrmCtrlCfg_RegCtrl = self.cFRMCTRL_REG_CTRL_SWAPBYTES

            self.Fpga_WinRe4Sc_Cfg(1,self.Rad_RangeWin_Ctrl, self.Rad_RangeWin_Type, N, N, 0, 1.0)
            self.Fpga_StRe16ToRe16Sc_CpyAllChnPar(1, N, 0)

            self.ConSet('InputType', self.Rad_Mode, self.Rad_NTx)

        elif self.Rad_Mode == 'FftCplx':
            self.Rad_2DFft = 0
            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    # for even number of channels only one data
                    # channel can be used
                    self.Set('NrDatChn', 1)
                    self.Set('NrProcChn', 1)
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn
                else:
                    self.Set('NrDatChn', 2)
                    self.Set('NrProcChn', 2)
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn/2
            else:
                self.Set('NrDatChn', 1)
                self.Set('NrProcChn', 1)
                self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn
            self.Rad_NTx = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
            self.Rad_DatSiz = 4
            self.Rad_FrmCtrlCfg_RegCtrl = self.cFRMCTRL_REG_CTRL_SWAPBYTES + self.cFRMCTRL_REG_CTRL_SELFFT

            self.Fpga_FftCtrl_Cfg(3, 0, self.Rad_RangeFftSiz, self.Rad_RangeFftIdxMin, self.Rad_RangeFftIdxMax)

            if self.Rad_RangeWin_Type == 0:
                self.Fpga_WinRe4Sc_ClrCoeff(1)
                self.Fpga_WinRe4Sc_SetCoeff(1, self.Rad_RangeWin_Coef)

            self.Fpga_MultSumRe16Sc_SetUnitGain(1, 0, 1)

            # function Ret = Fpga_WinRe4Sc_Cfg(obj, Mask, Ctrl, WinSel, WinSiz, PadSiz, FftShift)
            # Mask 1 = range window
            self.Fpga_WinRe4Sc_Cfg(1,self.Rad_RangeWin_Ctrl, self.Rad_RangeWin_Type, N, self.Rad_RangeFftSiz, self.Rad_RangeFftShift, self.Rad_RangeWin_Gain)
            # function Ret = Fpga_StRe16ToRe16Sc_CpyAllChn(obj, Mask, NrOut, FftShift)

            self.Fpga_StRe16ToRe16Sc_CpyAllChn(1, self.Rad_RangeFftSiz, self.Rad_RangeFftShift)
            #self.Fpga_DatMux_SetMux(1, 2, 3)
            self.Fpga_DatMux_SetMux(1, 0, 1)

            N = int(self.Get('N'))

            if self.Rad_RangeWin_Type == self.WinType_Boxcar:
                ScaWin = N
            elif self.Rad_RangeWin_Type == self.WinType_Hanning:
                ScaWin = sum(self.hanning(N,1))
            elif self.Rad_RangeWin_Type == self.WinType_Hamming:
                ScaWin = sum(self.hamming(N,1))
            elif self.Rad_RangeWin_Type == self.WinType_Custom:
                ScaWin = sum(self.Rad_RangeWin_Coef)
            else:
                ScaWin = 1

            if self.Rad_RangeFftSiz > 128:
                ScaFft = 2*self.Rad_RangeFftSiz
            else:
                ScaFft = self.Rad_RangeFftSiz

            # 12 bit data + 4 bit cic filter + 8 bit before FFT
            self.ScaRangeFft = 1/16*1/256*1/ScaWin*ScaFft*1/self.Rad_RangeWin_Gain

            self.ConSet('InputType', self.Rad_Mode, self.Rad_NTx)
            self.ConSetFileParam('Rad_ScaRangeFft', self.ScaRangeFft, 'DOUBLE')

        elif self.Rad_Mode == 'FftCplxDeb':
            self.Rad_2DFft = 0
            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    # for even number of channels only one data
                    # channel can be used
                    self.Set('NrDatChn', 1)
                    self.Set('NrProcChn', 1)
                    self.Rad_NTx = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn
                else:
                    self.Set('NrDatChn', 1)
                    self.Set('NrProcChn', 2)
                    self.Rad_ArmMemCfg_Ctrl = 2**16
                    self.Rad_NTx = 2*(self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1)
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn/2
                    self.Rad_NrVirtChn = NrChn/2
            else:
                self.Set('NrDatChn', 1)
                self.Set('NrProcChn', 1)
                self.Rad_NTx = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
                self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn

            self.Rad_DatSiz = 4
            self.Rad_FrmCtrlCfg_RegCtrl = self.cFRMCTRL_REG_CTRL_SWAPBYTES + self.FRMCTRL_REG_CTRL_SELFFT + self.FRMCTRL_REG_CTRL_ENADEBMOD

            self.Fpga_FftCtrl_Cfg(3, 0, self.Rad_RangeFftSiz, self.Rad_RangeFftIdxMin, self.Rad_RangeFftIdxMax)

            if self.Rad_RangeWin_Type == 0:
                self.Fpga_WinRe4Sc_ClrCoeff(1)
                self.Fpga_WinRe4Sc_SetCoeff(1, self.Rad_RangeWin_Coef)

            self.Fpga_MultSumRe16Sc_SetUnitGain(1, 0, 1)

            # function Ret = Fpga_WinRe4Sc_Cfg(obj, Mask, Ctrl, WinSel, WinSiz, PadSiz, FftShift)
            # Mask 1 = range window
            self.Fpga_WinRe4Sc_Cfg(1,self.Rad_RangeWin_Ctrl, self.Rad_RangeWin_Type, N, self.Rad_RangeFftSiz, self.Rad_RangeFftShift, self.Rad_RangeWin_Gain)
            # function Ret = Fpga_StRe16ToRe16Sc_CpyAllChn(obj, Mask, NrOut, FftShift)


            self.Fpga_FrmCtrl_FlowCtrl(1, self.Rad_FrmCtrlCfg_FrmCntrInc, 32, 16000)

            self.Fpga_StRe16ToRe16Sc_CpyAllChn(1, self.Rad_RangeFftSiz, self.Rad_RangeFftShift)
            #self.Fpga_DatMux_SetMux(1, 2, 3);
            self.Fpga_DatMux_SetMux(1, 0, 1)


            self.Rad_NrRangeBins = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1

            N = self.Get('N')

            if self.Rad_RangeWin_Type == self.WinType_Boxcar:
                ScaWin = N
            elif self.Rad_RangeWin_Type == self.WinType_Hanning:
                ScaWin = sum(self.hanning(N,1))
            elif self.Rad_RangeWin_Type == self.WinType_Hamming:
                ScaWin = sum(self.hamming(N,1))
            else:
                ScaWin = 1

            if self.Rad_RangeFftSiz > 128:
                ScaFft = 2*self.Rad_RangeFftSiz
            else:
                ScaFft = self.Rad_RangeFftSiz

            # 1/16 from CIC filter
            # 1/32 from 18 bit win to 24 bit
            self.ScaRangeFft = 1/16*1/64*1/ScaWin*ScaFft*1/self.Rad_RangeWin_Gain

            self.ConSet('InputType', self.Rad_Mode, self.Rad_NTx)

        elif self.Rad_Mode == 'LoMax':
            self.Rad_2DFft = 1
            NrChn = self.Get('NrChn')
            if NrChn > 1:
                if NrChn % 2 > 0:
                    # for even number of channels only one data
                    # channel can be used
                    self.Set('NrDatChn', 1)
                    self.Set('NrProcChn', 1)
                    self.Rad_NTx = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn
                else:
                    self.Set('NrDatChn', 1)
                    self.Set('NrProcChn', 2)
                    self.Rad_ArmMemCfg_Ctrl = 2**16;
                    self.Rad_NTx = 2*(self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1)
                    self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn/2
                    self.Rad_NrVirtChn = NrChn/2
            else:
                self.Set('NrDatChn', 1)
                self.Set('NrProcChn', 1)
                self.Rad_NTx = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1
                self.Rad_FrmCtrlCfg_FrmCntrInc = NrChn

            self.Rad_DatSiz = 4
            self.Rad_FrmCtrlCfg_RegCtrl = self.cFRMCTRL_REG_CTRL_SWAPBYTES + self.cFRMCTRL_REG_CTRL_SELFFT

            self.Fpga_FftCtrl_Cfg(3, 0, self.Rad_RangeFftSiz, self.Rad_RangeFftIdxMin, self.Rad_RangeFftIdxMax)

            if self.Rad_RangeWin_Type == 0:
                self.Fpga_WinRe4Sc_ClrCoeff(1)
                self.Fpga_WinRe4Sc_SetCoeff(1, self.Rad_RangeWin_Coef)

            self.Fpga_MultSumRe16Sc_SetUnitGain(1, 0, 1)

            # function Ret = Fpga_WinRe4Sc_Cfg(obj, Mask, Ctrl, WinSel, WinSiz, PadSiz, FftShift)
            # Mask 1 = range window
            self.Fpga_WinRe4Sc_Cfg(1,self.Rad_RangeWin_Ctrl, self.Rad_RangeWin_Type, N, self.Rad_RangeFftSiz, self.Rad_RangeFftShift, self.Rad_RangeWin_Gain)
            # function Ret = Fpga_StRe16ToRe16Sc_CpyAllChn(obj, Mask, NrOut, FftShift)

            self.Fpga_FrmCtrl_FlowCtrl(1, self.Rad_FrmCtrlCfg_FrmCntrInc, 32, 16000)

            # Configure Velecity processing (FFT2 and FFT3)
            self.Fpga_FftCtrl_Cfg(12, 0, self.Rad_VelFftSiz, 0, self.Rad_VelFftSiz-1)
            self.Fpga_WinRe4Sc_Cfg(2, self.Rad_VelWin_Ctrl, self.Rad_VelWin_Type, self.Rad_NrFrms, self.Rad_VelFftSiz, self.Rad_VelFftShift, self.Rad_VelWin_Gain)


            self.Rad_NrRangeBins = self.Rad_RangeFftIdxMax - self.Rad_RangeFftIdxMin + 1

            dLoMax16Cfg = dict()
            dLoMax16Cfg['Ctrl'] = 1 + 2**20
            dLoMax16Cfg['NrChn'] = NrChn/2                              # Stream on both channels
            dLoMax16Cfg['FrmSiz'] = self.Rad_VelFftSiz
            dLoMax16Cfg['NrFrms'] = self.Rad_NrRangeBins
            dLoMax16Cfg['FixedThres'] = 1000
            dLoMax16Cfg['Thres'] = self.Rad_DetThres
            dLoMax16Cfg['IniDelay'] = 20
            dLoMax16Cfg['InGain'] = 2
            dLoMax16Cfg['SumGain'] = (1-0.9)/(self.Rad_DetThresCalcLimMax - self.Rad_DetThresCalcLimMin)
            dLoMax16Cfg['MemGain'] = 0.9
            dLoMax16Cfg['MaxNrDet'] = self.Rad_DetMaxNrDet
            dLoMax16Cfg['StreamModulo'] = self.Rad_DetStreamModulo
            dLoMax16Cfg['StreamSiz'] = self.Rad_DetMaxNrDet
            dLoMax16Cfg['ColLim0Min'] = self.Rad_DetVelIdxMin0
            dLoMax16Cfg['ColLim0Max'] = self.Rad_DetVelIdxMax0
            dLoMax16Cfg['ColLim1Min'] = self.Rad_DetVelIdxMin0
            dLoMax16Cfg['ColLim1Max'] = self.Rad_DetVelIdxMax0
            dLoMax16Cfg['DetLimMin'] = self.Rad_DetThresCalcLimMin
            dLoMax16Cfg['DetLimMax'] = self.Rad_DetThresCalcLimMax

            self.Rad_StreamSiz = dLoMax16Cfg['StreamModulo']*dLoMax16Cfg['StreamSiz']

            self.Fpga_LoMax16_Cfg(1, dLoMax16Cfg)

            self.Fpga_StRe16ToRe16Sc_CpyAllChn(1, self.Rad_RangeFftSiz, self.Rad_RangeFftShift)
            #self.Fpga_DatMux_SetMux(1, 2, 3)
            self.Fpga_DatMux_SetMux(1, 0, 1)

            N = int(self.Get('N'))

            if self.Rad_RangeWin_Type == self.WinType_Boxcar:
                ScaRangeWin = N
            elif self.Rad_RangeWin_Type == self.WinType_Hanning:
                ScaRangeWin = sum(self.hanning(N,1))
            elif self.Rad_RangeWin_Type == self.WinType_Hamming:
                ScaRangeWin = sum(self.hamming(N,1))
            elif self.Rad_RangeWin_Type == self.WinType_Custom:
                ScaRangeWin = sum(self.Rad_RangeWin_Coef)
            else:
                ScaRangeWin = 1


            if self.Rad_VelWin_Type == self.WinType_Boxcar:
                print('Boxcar')
                ScaVelWin = self.Rad_NrFrms
            elif self.Rad_VelWin_Type == self.WinType_Hanning:
                print('Hanning')
                ScaVelWin = sum(self.hanning(self.Rad_NrFrms,1))
            elif self.Rad_VelWin_Type == self.WinType_Hamming:
                ScaVelWin = sum(self.hamming(self.Rad_NrFrms,1))
            else:
                ScaVelWin = 1

            if self.Rad_RangeFftSiz > 128:
                ScaRange = 2*self.Rad_RangeFftSiz
            else:
                ScaRange = self.Rad_RangeFftSiz

            ScaVel = 2*self.Rad_VelFftSiz
            ScaVelMag = 2*self.Rad_VelFftSiz/dLoMax16Cfg['InGain']/NrChn

            # 1/16 from CIC filter
            # 1/32 from 18 bit win to 24 bit
            self.ScaRangeFft = 1/16*1/256*1/ScaRangeWin*ScaRange*1/self.Rad_RangeWin_Gain
            self.ScaVelFft = 1/ScaVelWin*ScaVel*1/self.Rad_VelWin_Gain/256
            self.ScaVelFftMag = 1/ScaVelWin*ScaVelMag*1/self.Rad_VelWin_Gain

            self.ConSetFileParam('Rad_ScaRangeFft', self.ScaRangeFft, 'DOUBLE')
            self.ConSetFileParam('Rad_ScaVelFft', self.ScaVelFft, 'DOUBLE')
            self.ConSetFileParam('Rad_ScaVelFftMag', self.ScaVelFftMag, 'DOUBLE')

            self.ConSetFileParam('Rad_LoMax16Cfg_Thres', dLoMax16Cfg['Thres'], 'DOUBLE')
            self.ConSetFileParam('Rad_LoMax16Cfg_InGain', dLoMax16Cfg['InGain'], 'DOUBLE')

            self.ConSetFileParam('Rad_LoMax16Cfg_ColLim0Min', dLoMax16Cfg['ColLim0Min'], 'DOUBLE')
            self.ConSetFileParam('Rad_LoMax16Cfg_ColLim0Max', dLoMax16Cfg['ColLim0Max'], 'DOUBLE')
            self.ConSetFileParam('Rad_LoMax16Cfg_ColLim1Min', dLoMax16Cfg['ColLim1Min'], 'DOUBLE')
            self.ConSetFileParam('Rad_LoMax16Cfg_ColLim1Max', dLoMax16Cfg['ColLim1Max'], 'DOUBLE')
            self.ConSetFileParam('Rad_LoMax16Cfg_DetLimMin', dLoMax16Cfg['DetLimMin'], 'DOUBLE')
            self.ConSetFileParam('Rad_LoMax16Cfg_DetLimMax', dLoMax16Cfg['DetLimMax'], 'DOUBLE')

            self.ConSet('InputType', self.Rad_Mode, (self.Rad_NrChn * 2 + 8) * self.Rad_DetMaxNrDet, 1)

    def BrdDispProcTim(self):
        ProcTim = self.Fpga_GetProcTim(1)
        print('Processing Time: ', (ProcTim*10e-6), ' ms')
        return ProcTim*10e-9



    def Fpga_GetProcTim(self, Mask):
        FpgaCmd = zeros(2, dtype='uint32')
        Cod = int('0x9544', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def Fpga_LoMax16_Cfg(self, Mask, dCfg):
        FpgaCmd = zeros(21, dtype='uint32')
        Cod = int('0x9543', 0)

        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        FpgaCmd[2] = dCfg['Ctrl']
        FpgaCmd[3] = dCfg['NrChn']
        FpgaCmd[4] = dCfg['FrmSiz']
        FpgaCmd[5] = dCfg['NrFrms']
        FpgaCmd[6] = dCfg['FixedThres']
        FpgaCmd[7] = dCfg['Thres'] * 2**16
        FpgaCmd[8] = dCfg['IniDelay']
        FpgaCmd[9] = dCfg['InGain'] * 2**20
        FpgaCmd[10] = dCfg['SumGain'] * 2**20
        FpgaCmd[11] = dCfg['MemGain'] * 2**20
        FpgaCmd[12] = dCfg['MaxNrDet']
        FpgaCmd[13] = dCfg['StreamModulo']
        FpgaCmd[14] = dCfg['StreamSiz']
        FpgaCmd[15] = dCfg['ColLim0Min']
        FpgaCmd[16] = dCfg['ColLim0Max']
        FpgaCmd[17] = dCfg['ColLim1Min']
        FpgaCmd[18] = dCfg['ColLim1Max']
        FpgaCmd[19] = dCfg['DetLimMin']
        FpgaCmd[20] = dCfg['DetLimMax']

        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret


    # DOXYGEN ------------------------------------------------------
    #> @brief Configure Range/Velocity window (real window for four input channels)
    #>
    #>  Mask = 1 Range window MMP
    #>  Mask = 2 Velocity window MMP
    #>  @endcode
    def     Fpga_WinRe4Sc_Cfg(self, Mask, Ctrl, WinSel, WinSiz, PadSiz, FftShift, WinGain):
        FpgaCmd = zeros(8, dtype='uint32')
        Cod = int('0x9521', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        FpgaCmd[2] = Ctrl
        FpgaCmd[3] = WinSel
        FpgaCmd[4] = WinSiz
        FpgaCmd[5] = PadSiz
        FpgaCmd[6] = FftShift
        FpgaCmd[7] = WinGain* 2**16
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure Range window (real window for four input channels)
    #> Set window coefficients manually
    #>
    #>  @endcode
    def     Fpga_WinRe4Sc_SetCoeff(self, Mask, Coeff):
        FpgaCmd = zeros(7, dtype='uint32')
        Cod = int('0x9521', 0)
        #FpgaCmd[0] = Mask
        #FpgaCmd[1] = 1
        #FpgaCmd[2] = Ctrl
        #FpgaCmd[3] = WinSel
        #FpgaCmd[4] = WinSiz
        #FpgaCmd[5] = PadSiz
        #FpgaCmd[6] = FftShift
        #Ret = self.CmdSend(0, Cod, FpgaCmd)
        #Ret = self.CmdRecv()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure Range/Velocity window (real window for four input channels)
    #>
    #>  Mask = 1 Range window MMP
    #>  Mask = 2 Velocity window MMP
    #>  @endcode
    def     Fpga_WinRe4Sc_ClrCoeff(self, Mask):
        FpgaCmd = zeros(2, dtype='uint32')
        Cod = int('0x9521', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 3
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_StRe16ToRe16Sc_CpyAllChn(self, Mask, NrOut, FftShift):
        NrInChn = self.Get('NrChn')
        NrProcChn = self.Get('NrProcChn')
        NrDat = self.Get('NrDat')

        FpgaCmd = zeros(7, dtype='uint32')
        Cod = int('0x9520', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 3
        FpgaCmd[2] = NrDat
        FpgaCmd[3] = NrOut
        FpgaCmd[4] = NrInChn
        FpgaCmd[5] = NrProcChn
        FpgaCmd[6] = FftShift
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_StRe16ToRe16Sc_CpyAllChnPar(self, Mask, NrOut, FftShift):
        NrDat = self.Get('NrDat');

        FpgaCmd = zeros(6, dtype='uint32')
        Cod = int('0x9520', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 4
        FpgaCmd[2] = NrDat
        FpgaCmd[3] = NrOut
        FpgaCmd[4] = self.Rad_NrBeams
        FpgaCmd[5] = FftShift
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_MultSumRe16Sc_SetUnitGain(self, Mask, Chn0, Chn1):
        FpgaCmd = zeros(4, dtype='uint32')
        Cod = int('0x9527', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        FpgaCmd[2] = Chn0
        FpgaCmd[3] = Chn1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_MultSumRe16Sc_SetCplxUnitGain(self, Mask, Chn0Re, Chn0Im, Chn1Re, Chn1Im):
        FpgaCmd = zeros(6, dtype='uint32')
        Cod = int('0x9527', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 2
        FpgaCmd[2] = Chn0Re
        FpgaCmd[3] = Chn0Im
        FpgaCmd[4] = Chn1Re
        FpgaCmd[5] = Chn1Im
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_MultSumRe16Sc_ClrCoeff(self, Mask):
        FpgaCmd = zeros(2, dtype='uint32')
        Cod = int('0x9527', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 3
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_MultSumRe16Sc_LoadCoeff(self, Mask):
        FpgaCmd = zeros(2, dtype='uint32')
        Cod = int('0x9527', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 4
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_MultSumRe16Sc_SetCoeff(self, Mask, OutChn, Coeff):
        #FpgaCmd = zeros(2, dtype='uint32')
        #Cod = int('0x9527', 0)
        #FpgaCmd[0] = Mask
        #FpgaCmd[1] = 4
        #Ret = self.CmdSend(0, Cod, FpgaCmd)
        #Ret = self.CmdRecv()
        return Ret

    def Fpga_FrmCtrl_FlowCtrl(self, Mask, ChnCntr, Frms, Delay):
        FpgaCmd = zeros(5, dtype='uint32')
        Cod = int('0x9528', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        FpgaCmd[2] = ChnCntr
        FpgaCmd[3] = Frms
        FpgaCmd[4] = Delay

        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_FftCtrl_Cfg(self, Mask, Ctrl, Siz, StrtIdx, StopIdx):
        FpgaCmd = zeros(6, dtype='uint32')
        Cod = int('0x9522', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        FpgaCmd[2] = Ctrl
        FpgaCmd[3] = Siz
        FpgaCmd[4] = StrtIdx
        FpgaCmd[5] = StopIdx
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

    def     Fpga_DatMux_SetMux(self, Mask, Mux0, Mux1):
        FpgaCmd = zeros(4, dtype='uint32')
        Cod = int('0x9523', 0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 2
        FpgaCmd[2] = Mux0
        FpgaCmd[3] = Mux1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return Ret

#    def     Fpga_StRe16ToRe16Sc_Cfg(self, Mask, Ctrl, OutCtrl):
#        FpgaCmd = zeros(4, dtype='uint32')
#        Cod = int('0x9520', 0)
#        FpgaCmd[0] = Mask
#        FpgaCmd[1] = 1
#        FpgaCmd[2] = Ctrl
#        FpgaCmd[3] = OutCtrl
#        Ret = self.CmdSend(0, Cod, FpgaCmd)
#        Ret = self.CmdRecv()
#        return Ret
#
#    def     Fpga_StRe16ToRe16Sc_SetPhaseCtrl(self, Mask, Ctrl, OutCtrl):
#        FpgaCmd = zeros(4, dtype='uint32')
#        Cod = int('0x9520', 0)
#        FpgaCmd[0] = Mask
#        FpgaCmd[1] = 2
#        FpgaCmd[2] = Ctrl
#        FpgaCmd[3] = OutCtrl
#        Ret = self.CmdSend(0, Cod, FpgaCmd)
#        Ret = self.CmdRecv()
#        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Set Calibration Information
    #>
    #> This function writes the calibartion data (configuration and data) to the FPGA. The information is not stored in the EEPROM of the frontend.
    #>
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
    #>      CaldCfg[Mask] = 1;
    #>      CaldCfg['Len'] = 16;
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
        Ret = self.Fpga_GetCalData(dCfg)
        Ret = self.Cal2Num(Ret)
        dCfg["Type"] = 0
        if dCfg["Type"] == 0:
            CplxData = Ret[::2] + 1j*Ret[1::2]
        else:
            CplxData = -1

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
        Ret = self.Fpga_MimoSeqRst()
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief Set Role of Board (Master, Slave, )
    #>
    #> This function is used to reset the board. The reset function resets the timing unit SeqTrig and clears the entries in the sequencer.
    #>
    #> @note At the begin of a script the BrdRst function should be called in order to disable measurements.
    def     BrdSetRole(self, Role, AdcFreq):

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
                Sel = 0
        else:
            Sel = int(Role)

        if AdcFreq > 30e6 :
            AdcSel = 1;
            self.Rad_fAdc = 40e6
        else:
            AdcSel = 0
            self.Rad_fAdc = 20e6
        self.Rad_Role = Sel
        self.Fpga_SetRole(Sel, AdcSel);


    def BrdGetClkSts(self):
        return self.Fpga_GetBrdClkSts(0)

    def BrdDispClkSts(self):

        Ret = self.Fpga_GetBrdSts(0)
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
        dPwrCfg = {   "IntEna"        : 0,
                            "RfEna"         : self.cPWR_RF_ON
                        }
        Ret = self.Fpga_SetRfPwr(dPwrCfg)
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
        dPwrCfg = {   "IntEna"        : 0,
                            "RfEna"         : self.cPWR_RF_OFF
                        }
        Ret = self.Fpga_SetRfPwr(dPwrCfg)
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

        dFrmCtrlCfg = {
                                            "ChnSel"    :   0
                                        }
        dFrmCtrlCfg["ChnSel"] = self.Rad_FrmCtrlCfg_ChnSel                      # Select Channels to be configured
        dFrmCtrlCfg["Rst"] = self.Rad_FrmCtrlCfg_Rst                         # Reset MMP
        dFrmCtrlCfg["RegChnCtrl"] = self.Rad_FrmCtrlCfg_RegChnCtrl                  # Channel control register


        dCicCfg = {
                                            "FiltSel"    :   0
                                        }
        dCicCfg["FiltSel"] = self.Rad_CicCfg_FiltSel                         # Select Cic filters
        dCicCfg["CombDel"] = self.Rad_CicCfg_CombDel                         # Comb delay
        dCicCfg["OutSel"] = self.Rad_CicCfg_OutSel                          # Output select
        dCicCfg["SampPhs"] = self.Rad_CicCfg_SampPhs                         # Sampling phase
        dCicCfg["SampRed"] = self.Rad_CicCfg_SampRed                         # Sampling rate reduction
        dCicCfg["RegCtrl"] = self.Rad_CicCfg_RegCtrl                         # Cic filter control register

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
    #>      Temp1 (AD8283) = 62.1473°
    #>      Temp2 (Sup) = 56.4602°
    #>      VSup (Sup) = 16.4193V
    #>      ISup (Sup) = 0.74766A
    #>      -------------------------------------------------------------------
    #>   @endcode
    #>
    #>   @node The supply current is only monitored correct, if the RF supply is enabled.
    def BrdDispInf(self):
        Ret = self.Fpga_DispBrdInf()
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
    #> The BrdDispRfSts function prints the status of the board to the command window. If the ID of the frontend is knwon, the chip configuration is also printlayed.
    #> Most RF chips have an unique identification number. This number is printed in the command window.
    #>
    def BrdDispRfSts(self):
        Ret = self.Fpga_DispRfChipSts(1)
        return Ret

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
        Vers = self.Fpga_GetSwVers()
        dRet = {
                            "SwPatch"   :   -1,
                            "SwMin"     :   -1,
                            "SwMaj"     :   -1,
                            "SUid"      :   -1,
                            "HUid"      :   -1
                        }
        if len(Vers) > 2:
            dRet["SwPatch"] = Vers[0] % (2**8)
            TmpVal = floor(Vers[0]/(2**8))
            dRet["SwMin"] = TmpVal % (2**8)
            dRet["SwMaj"] = floor(TmpVal/(2**8))
            dRet["SUid"] = Vers[1]
            dRet["HUid"] = Vers[2]
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
        FpgaCmd = zeros(1, dtype='uint32')
        Cod = int('0x9013', 0)
        FpgaCmd[0] = 0
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
    #>      Type = 0
    #>      Date = 07-Jan-2016
    #>      Rev = 0.0.1
    #>      -------------------------------------------------------------------
    #>  @endcode
    #>
    def BrdDispCalInf(self, *varargin):
        if len(varargin) > 0:
            dCfg = varargin[0]
        else:
            dCfg = {
                                "Mask"    :   1
                            }
        print(' ')
        print('-------------------------------------------------------------------')
        print('Calibration Table Information')
        Val = self.Fpga_GetCalCfg(dCfg)
        if len(Val) > 2:
            print('Type = ',Val[0])
            print('Date = ',Val[1])
            print('Rev = ',self.RevStr(Val[2]))
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
            Data = zeros((int(self.Rad_N*NrPack*self.Rad_NMult), int(self.Rad_NrVirtChn)))
            for Idx in range(0,int(NrPack)):
                ArmData = self.Fpga_GetData()
                DataSiz = ArmData.shape
                if (DataSiz[0] == self.Rad_N*self.Rad_NMult) and (DataSiz[1] == self.Rad_NrVirtChn):
                    Data[int((Idx)*self.Rad_N*self.Rad_NMult):int((Idx+1)*self.Rad_N*self.Rad_NMult),:] = ArmData
            if self.Rad_EnaFrmCntr > 0:
                FrmCntr = Data[0::int(self.Rad_NTx*self.Rad_NMult),:];
                Data = Data / 16;
                Data[0::int(self.Rad_NTx*self.Rad_NMult),:] = FrmCntr;
            else:
                Data = Data / 16;
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
                UsbData = self.ConGetData(NrPack*self.Rad_NrChn*self.Rad_N*2)

                UsbData = double(UsbData);
                Data = zeros((int(self.Rad_N)*NrPack,int(self.Rad_NrChn)))
                for Idx in range(0, NrPack):
                    IdxStrt = int((Idx)*self.Rad_NrChn*self.Rad_N)
                    IdxStop = int((Idx+1)*self.Rad_NrChn*self.Rad_N)
                    Data1 = UsbData[IdxStrt:IdxStop]
                    Data1 = Data1.reshape((int(self.Rad_NrChn),int(self.Rad_N))).transpose()
                    Data1[1:,:] = Data1[1:,:]/16
                    Data[(Idx)*int(self.Rad_N):(Idx+1)*int(self.Rad_N),:] = Data1
                self.cDataIdx = self.cDataIdx + NrPack;
            elif not Extension:
                self.cDataIdx = self.cDataIdx + NrPack * self.cExtMult;

                return self.Computation.GetData(NrPack);
            else:
                Data = self.ConGetData(NrPack * self.cExtSize * 2);
                self.cDataIdx = self.cDataIdx + self.cExtMult * NrPack;

        return Data


    # function [Data Thres, LoMax] = BrdGetLoMaxDat(obj)

    #     Data = [];
    #     Thres = [];
    #     LoMax = [];
    #    if strcmp(self.cType, 'PNet')
    #         if self.Rad_LoMaxRecCfg == 1
    #             ArmData = double(self.Fpga_GetData32Len(2*self.Rad_StreamSiz));
    #             LoMax = ArmData;
    #         end
    #         if self.Rad_LoMaxRecCfg == 2
    #             ArmData = double(self.Fpga_GetData32Len(2*self.Rad_NrRangeBins*self.Rad_VelFftSiz));
    #             Data = ArmData(1:2:end);
    #             Thres = ArmData(2:2:end);
    #         end
    #         if self.Rad_LoMaxRecCfg == 3
    #             ArmData = double(self.Fpga_GetData32Len(2*self.Rad_NrRangeBins*self.Rad_VelFftSiz + 2*self.Rad_StreamSiz));
    #             Data = ArmData(1:2:2*self.Rad_NrRangeBins*self.Rad_VelFftSiz);
    #             Thres = ArmData(2:2:2*self.Rad_NrRangeBins*self.Rad_VelFftSiz);
    #             LoMax = ArmData(2*self.Rad_NrRangeBins*self.Rad_VelFftSiz+1:end);


    def BrdGetLoMax(self):

        dLoMax = dict()
        if self.cType == 'PNet':
            if self.Rad_LoMaxRecCfg == 1:
                ArmData = double(self.Fpga_GetData32Len(2*self.Rad_StreamSiz))
                Dat = ArmData
                #np.reshape(Data[:,0],(Range_N, int(dCfg['NLoop'])), order='F')
                Dat = reshape(Dat,(int(2*self.Rad_DetStreamModulo),int(self.Rad_DetMaxNrDet)), order='F')


                Mag = Dat[3,:]
                Idcs = argwhere(Mag > 1)

                Dat = Dat[:,0:len(Idcs)]

                dLoMax['FrmCntr'] = Dat[0,:]
                dLoMax['Mag'] = Dat[3,:]
                dLoMax['Thres'] = Dat[2,:]

                ColIdx = (Dat[1,:] % 2**16)
                RowIdx = floor(Dat[1,:]/2**16)
                dLoMax['VelIdx'] = array(ColIdx, dtype='int16')
                dLoMax['RangeIdx'] = array(RowIdx, dtype='int16')

                dLoMax['MagNeighbors'] = Dat[4:8,:]

                #print('Dat: ', Dat.shape)

                dLoMax['CplxAmp'] = Dat[8::2,:] + 1j*Dat[9::2,:]
        elif self.cType == 'RadServe':
            #----------------------------------------------------------
            # Open Data Port
            #----------------------------------------------------------
            if self.hConDat < 0:
                #if not Extension:
                self.GetDataPort()
                self.cDataIdx = 0;
                #else:
                #    self.GetExtensionPort(Extension)
                #    self.cDataIdx = 0;
                if self.cDataOpened < 0:
                    quit();

            Dat = double(self.ConGetData(self.Rad_StreamSiz * 2 * 4, 'int32'));

            #np.reshape(Data[:,0],(Range_N, int(dCfg['NLoop'])), order='F')
            Dat = reshape(Dat,(int(2*self.Rad_DetStreamModulo),int(self.Rad_DetMaxNrDet)), order='F')

            Mag = Dat[3,:]
            Idcs = argwhere(Mag > 1)

            Dat = Dat[:,0:len(Idcs)]

            dLoMax['FrmCntr'] = Dat[0,:]
            dLoMax['Mag'] = Dat[3,:]
            dLoMax['Thres'] = Dat[2,:]

            ColIdx = (Dat[1,:] % 2**16)
            RowIdx = floor(Dat[1,:]/2**16)
            dLoMax['VelIdx'] = array(ColIdx, dtype='int16')
            dLoMax['RangeIdx'] = array(RowIdx, dtype='int16')

            dLoMax['MagNeighbors'] = Dat[4:8,:]

            dLoMax['CplxAmp'] = Dat[8::2,:] + 1j*Dat[9::2,:]

            self.cDataIdx = self.cDataIdx + 1

        return dLoMax

    def BrdGetFftData(self,  NrPack=1):
        if self.cType == 'PNet':
            Data = zeros((int(self.Rad_NTx*NrPack*self.Rad_NMult), int(self.Rad_NrVirtChn)), dtype='complex')
            for Idx in range(0,int(NrPack)):
                ArmData = self.Fpga_GetData32()
                DataSiz = ArmData.shape
                if (DataSiz[0] == self.Rad_NTx*self.Rad_NMult*self.Rad_DatSiz/2) and (DataSiz[1] == self.Rad_NrVirtChn):
                    Data[int((Idx)*self.Rad_NTx*self.Rad_NMult):int((Idx+1)*self.Rad_NTx*self.Rad_NMult),:] = ArmData[0::2, :] + 1j*ArmData[1::2, :];

            Aux = (real(Data) % 256) + 2**8*(imag(Data) % 256)
            Data = (Data / 256)

            return (Data, Aux)

        elif self.cType == 'RadServe':
            #----------------------------------------------------------
            # Open Data Port
            #----------------------------------------------------------
            if self.hConDat < 0:
                #if not Extension:
                self.GetDataPort()
                self.cDataIdx = 0;
                #else:
                #    self.GetExtensionPort(Extension)
                #    self.cDataIdx = 0;
                if self.cDataOpened < 0:
                    quit();

            Data = zeros((int(self.Rad_NTx*NrPack*self.Rad_NMult), int(self.Rad_NrVirtChn)), dtype='complex')
            for Idx in range(0,int(NrPack)):
                Dat = self.ConGetData(self.Rad_DatSiz * 2 * self.Rad_NTx * self.Rad_NrVirtChn, 'int32');
                ArmData = reshape(Dat, (int(self.Rad_NrVirtChn), int(self.Rad_NTx * self.Rad_DatSiz / 2))).transpose();
                DataSiz = ArmData.shape

                if (DataSiz[0] == self.Rad_NTx*self.Rad_NMult*self.Rad_DatSiz/2) and (DataSiz[1] == self.Rad_NrVirtChn):
                    Data[int((Idx)*self.Rad_NTx*self.Rad_NMult):int((Idx+1)*self.Rad_NTx*self.Rad_NMult),:] = ArmData[0::2, :] + 1j*ArmData[1::2, :];

            Aux = (real(Data) % 256) + 2**8*(imag(Data) % 256)
            Data = (Data / 256)

            self.cDataIdx = self.cDataIdx + NrPack

            return (Data, Aux)

        # elif self.cType == 'RadServe':
        #     #inParser = inputParser;
        #     #addRequired(inParser,'NrPack',@(x) isnumeric(x) && isscalar(x) && (x > 0));
        #     #addParameter(inParser,'Extension', -1, @(x) isnumeric(x) && isscalar(x) && (x >= -1));
        #     #parse(inParser, NrPack, varargin{:});

        #     #----------------------------------------------------------
        #     # Open Data Port
        #     #----------------------------------------------------------
        #     if self.hConDat < 0
        #         % GetDataPort(obj, Num, DataType)
        #         self.ConSetTimeout(self.Usb_Timeout);
        #         %if (inParser.Results.Extension <= 0)
        #             self.GetDataPort();
        #         %else
        #         %    self.GetExtensionPort(inParser.Results.Extension);
        #         %end
        #         self.cDataIdx = 0;
        #     end

        #     Data = zeros(self.Rad_NTx*NrPack*self.Rad_NMult, self.Rad_NrVirtChn);

        #         for Idx = 1:NrPack
        #             ArmData = zeros(self.Rad_NTx, self.Rad_NrVirtChn);

        #             Dat = self.ConGetData(self.Rad_DatSiz.*2.*self.Rad_NTx*self.Rad_NrVirtChn); % data len in bytes
        #             Dat = typecast(Dat, 'int32'); % cast to int32

        #             ArmData = reshape(double(Dat(:)), self.Rad_NTx*2, self.Rad_NrVirtChn);

        #             [Ny Nx] = size(ArmData);

        #             if (Ny == self.Rad_NTx*self.Rad_NMult.*self.Rad_DatSiz./2) && (Nx == self.Rad_NrVirtChn)
        #                 Data((Idx-1)*self.Rad_NTx*self.Rad_NMult+1:Idx*self.Rad_NTx*self.Rad_NMult,:) = ArmData(1:2:end,:) + i.*ArmData(2:2:end,:);
        #             end
        #         end

        #         Aux = mod(real(Data),256) + 2^8.*mod(imag(Data),256);
        #         Data = floor(Data/256);
        #     end

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
        Cod = int('0x9510',0)
        FpgaCmd = zeros(2, dtype='uint32')
        FpgaCmd[0] = 1
        FpgaCmd[1] = Sel
        self.CmdSend(0,Cod,FpgaCmd)
        return self.CmdRecv()

    def     SetUBloxMsgCtrl(self, Reg):
        Cod = int('0x9510',0)
        FpgaCmd = zeros(3, dtype='uint32')
        FpgaCmd[0] = 1
        FpgaCmd[1] = 5
        FpgaCmd[2] = Reg
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
                print('Clear')

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
            dCfg = varargin[0]
            if 'Mask' in dCfg:
                dCfg['Mask'] = self.Rad_Afe5801_Mask
            else:
                self.Rad_Afe5801_Mask = dCfg['Rad_Afe5801_Mask']
            if 'RegConfig' in dCfg:
                dCfg['RegConfig'] = self.Rad_Afe5801_RegConfig
            else:
                self.Rad_Afe5801_RegConfig = dCfg['Rad_Afe5801_RegConfig']
            if 'RegPower' in dCfg:
                dCfg['RegPower'] = self.Rad_Afe5801_RegPower
            else:
                self.Rad_Afe5801_RegPower = dCfg['Rad_Afe5801_RegPower']
            if 'RegLvds' in dCfg:
                dCfg['RegLvds'] = self.Rad_Afe5801_RegLvds
            else:
                self.Rad_Afe5801_RegLvds = dCfg['Rad_Afe5801_RegLvds']
            if 'RegData' in dCfg:
                dCfg['RegData'] = self.Rad_Afe5801_RegData
            else:
                self.Rad_Afe5801_RegData = dCfg['Rad_Afe5801_RegData']
            if 'RegDfs' in dCfg:
                dCfg['RegDfs'] = self.Rad_Afe5801_RegDfs
            else:
                self.Rad_Afe5801_RegDfs = dCfg['Rad_Afe5801_RegDfs']
            if 'RegLvdsPattern' in dCfg:
                dCfg['RegLvdsPattern'] = self.Rad_Afe5801_RegLvdsPattern
            else:
                self.Rad_Afe5801_RegLvdsPattern = dCfg['Rad_Afe5801_RegLvdsPattern']
            if 'RegFilter' in dCfg:
                dCfg['RegFilter'] = self.Rad_Afe5801_RegFilter
            else:
                self.Rad_Afe5801_RegFilter = dCfg['Rad_Afe5801_RegFilter']
            if 'RegOffsetGainCh1' in dCfg:
                dCfg['RegOffsetGainCh1'] = self.Rad_Afe5801_RegOffsetGainCh1
            else:
                self.Rad_Afe5801_RegOffsetGainCh1 = dCfg['Rad_Afe5801_RegOffsetGainCh1']
            if 'RegOffsetGainCh2' in dCfg:
                dCfg['RegOffsetGainCh2'] = self.Rad_Afe5801_RegOffsetGainCh2
            else:
                self.Rad_Afe5801_RegOffsetGainCh2 = dCfg['Rad_Afe5801_RegOffsetGainCh2']
            if 'RegOffsetGainCh3' in dCfg:
                dCfg['RegOffsetGainCh3'] = self.Rad_Afe5801_RegOffsetGainCh3
            else:
                self.Rad_Afe5801_RegOffsetGainCh3 = dCfg['Rad_Afe5801_RegOffsetGainCh3']
            if 'RegOffsetGainCh4' in dCfg:
                dCfg['RegOffsetGainCh4'] = self.Rad_Afe5801_RegOffsetGainCh4
            else:
                self.Rad_Afe5801_RegOffsetGainCh4 = dCfg['Rad_Afe5801_RegOffsetGainCh4']
            if 'RegHighpass14' in dCfg:
                dCfg['RegHighpass14'] = self.Rad_Afe5801_RegHighpass14
            else:
                self.Rad_Afe5801_RegHighpass14 = dCfg['Rad_Afe5801_RegHighpass14']
            if 'RegOffsetGainCh5' in dCfg:
                dCfg['RegOffsetGainCh5'] = self.Rad_Afe5801_RegOffsetGainCh5
            else:
                self.Rad_Afe5801_RegOffsetGainCh5 = dCfg['Rad_Afe5801_RegOffsetGainCh5']
            if 'RegOffsetGainCh6' in dCfg:
                dCfg['RegOffsetGainCh6'] = self.Rad_Afe5801_RegOffsetGainCh6
            else:
                self.Rad_Afe5801_RegOffsetGainCh6 = dCfg['Rad_Afe5801_RegOffsetGainCh6']
            if 'RegOffsetGainCh7' in dCfg:
                dCfg['RegOffsetGainCh7'] = self.Rad_Afe5801_RegOffsetGainCh7
            else:
                self.Rad_Afe5801_RegOffsetGainCh7 = dCfg['Rad_Afe5801_RegOffsetGainCh7']
            if 'RegOffsetGainCh8' in dCfg:
                dCfg['RegOffsetGainCh8'] = self.Rad_Afe5801_RegOffsetGainCh8
            else:
                self.Rad_Afe5801_RegOffsetGainCh8 = dCfg['Rad_Afe5801_RegOffsetGainCh8']
            if 'RegHighpass58' in dCfg:
                dCfg['RegHighpass58'] = self.Rad_Afe5801_RegHighpass58
            else:
                self.Rad_Afe5801_RegHighpass58 = dCfg['Rad_Afe5801_RegHighpass58']
            if 'RegClamp' in dCfg:
                dCfg['RegClamp'] = self.Rad_Afe5801_RegClamp
            else:
                self.Rad_Afe5801_RegClamp = dCfg['Rad_Afe5801_RegClamp']
            if 'RegTgcStartIndex' in dCfg:
                dCfg['RegTgcStartIndex'] = self.Rad_Afe5801_RegTgcStartIndex
            else:
                self.Rad_Afe5801_RegTgcStartIndex = dCfg['Rad_Afe5801_RegTgcStartIndex']
            if 'RegTgcStopIndex' in dCfg:
                dCfg['RegTgcStopIndex'] = self.Rad_Afe5801_RegTgcStopIndex
            else:
                self.Rad_Afe5801_RegTgcStopIndex = dCfg['Rad_Afe5801_RegTgcStopIndex']
            if 'RegTgcStartGain' in dCfg:
                dCfg['RegTgcStartGain'] = self.Rad_Afe5801_RegTgcStartGain
            else:
                self.Rad_Afe5801_RegTgcStartGain = dCfg['Rad_Afe5801_RegTgcStartGain']
            if 'RegTgcHoldGainTime' in dCfg:
                dCfg['RegTgcHoldGainTime'] = self.Rad_Afe5801_RegTgcHoldGainTime
            else:
                self.Rad_Afe5801_RegTgcHoldGainTime = dCfg['Rad_Afe5801_RegTgcHoldGainTime']
            if 'RegTgcGainMode' in dCfg:
                dCfg['RegTgcGainMode'] = self.Rad_Afe5801_RegTgcGainMode
            else:
                self.Rad_Afe5801_RegTgcGainMode = dCfg['Rad_Afe5801_RegTgcGainMode']
            if 'RegTgcGainCoarse' in dCfg:
                dCfg['RegTgcGainCoarse'] = self.Rad_Afe5801_RegTgcGainCoarse
            else:
                self.Rad_Afe5801_RegTgcGainCoarse = dCfg['Rad_Afe5801_RegTgcGainCoarse']
            if 'RegTgcUniformSlope' in dCfg:
                dCfg['RegTgcUniformSlope'] = self.Rad_Afe5801_RegTgcUniformSlope
            else:
                self.Rad_Afe5801_RegTgcUniformSlope = dCfg['Rad_Afe5801_RegTgcUniformSlope']
        else:
            dCfg = dict()
            dCfg['Mask'] = self.Rad_Afe5801_Mask
            dCfg['RegConfig'] = self.Rad_Afe5801_RegConfig
            dCfg['RegPower'] = self.Rad_Afe5801_RegPower
            dCfg['RegLvds'] = self.Rad_Afe5801_RegLvds
            dCfg['RegData'] = self.Rad_Afe5801_RegData
            dCfg['RegDfs'] = self.Rad_Afe5801_RegDfs
            dCfg['RegLvdsPattern'] = self.Rad_Afe5801_RegLvdsPattern
            dCfg['RegFilter'] = self.Rad_Afe5801_RegFilter
            dCfg['RegOffsetGainCh1'] = self.Rad_Afe5801_RegOffsetGainCh1
            dCfg['RegOffsetGainCh2'] = self.Rad_Afe5801_RegOffsetGainCh2
            dCfg['RegOffsetGainCh3'] = self.Rad_Afe5801_RegOffsetGainCh3
            dCfg['RegOffsetGainCh4'] = self.Rad_Afe5801_RegOffsetGainCh4
            dCfg['RegHighpass14'] = self.Rad_Afe5801_RegHighpass14
            dCfg['RegOffsetGainCh5'] = self.Rad_Afe5801_RegOffsetGainCh5
            dCfg['RegOffsetGainCh6'] = self.Rad_Afe5801_RegOffsetGainCh6
            dCfg['RegOffsetGainCh7'] = self.Rad_Afe5801_RegOffsetGainCh7
            dCfg['RegOffsetGainCh8'] = self.Rad_Afe5801_RegOffsetGainCh8
            dCfg['RegHighpass58'] = self.Rad_Afe5801_RegHighpass58
            dCfg['RegClamp'] = self.Rad_Afe5801_RegClamp
            dCfg['RegTgcStartIndex'] = self.Rad_Afe5801_RegTgcStartIndex
            dCfg['RegTgcStopIndex'] = self.Rad_Afe5801_RegTgcStopIndex
            dCfg['RegTgcStartGain'] = self.Rad_Afe5801_RegTgcStartGain
            dCfg['RegTgcHoldGainTime'] = self.Rad_Afe5801_RegTgcHoldGainTime
            dCfg['RegTgcGainMode'] = self.Rad_Afe5801_RegTgcGainMode
            dCfg['RegTgcGainCoarse'] = self.Rad_Afe5801_RegTgcGainCoarse
            dCfg['RegTgcUniformSlope'] = self.Rad_Afe5801_RegTgcUniformSlope

        Cod = int('0x910D',0)
        FpgaCmd = zeros(26, dtype='uint32')
        FpgaCmd[0] = dCfg['Mask']
        FpgaCmd[1] = dCfg['RegConfig']
        FpgaCmd[2] = dCfg['RegPower']
        FpgaCmd[3] = dCfg['RegLvds']
        FpgaCmd[4] = dCfg['RegData']
        FpgaCmd[5] = dCfg['RegDfs']
        FpgaCmd[6] = dCfg['RegLvdsPattern']
        FpgaCmd[7] = dCfg['RegFilter']
        FpgaCmd[8] = dCfg['RegOffsetGainCh1']
        FpgaCmd[9] = dCfg['RegOffsetGainCh2']
        FpgaCmd[10] = dCfg['RegOffsetGainCh3']
        FpgaCmd[11] = dCfg['RegOffsetGainCh4']
        FpgaCmd[12] = dCfg['RegHighpass14']
        FpgaCmd[13] = dCfg['RegOffsetGainCh5']
        FpgaCmd[14] = dCfg['RegOffsetGainCh6']
        FpgaCmd[15] = dCfg['RegOffsetGainCh7']
        FpgaCmd[16] = dCfg['RegOffsetGainCh8']
        FpgaCmd[17] = dCfg['RegHighpass58']
        FpgaCmd[18] = dCfg['RegClamp']
        FpgaCmd[19] = dCfg['RegTgcStartIndex']
        FpgaCmd[20] = dCfg['RegTgcStopIndex']
        FpgaCmd[21] = dCfg['RegTgcStartGain']
        FpgaCmd[22] = dCfg['RegTgcHoldGainTime']
        FpgaCmd[23] = dCfg['RegTgcGainMode']
        FpgaCmd[24] = dCfg['RegTgcGainCoarse']
        FpgaCmd[25] = dCfg['RegTgcUniformSlope']


        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
            dFrmCtrlCfg = varargin[0]
            if not ('ChnSel' in dFrmCtrlCfg):
                dFrmCtrlCfg["ChnSel"] = self.Rad_FrmCtrlCfg_ChnSel
            else:
                self.Rad_FrmCtrlCfg_ChnSel = dFrmCtrlCfg["ChnSel"]
            if not ('Rst' in dFrmCtrlCfg):
                dFrmCtrlCfg["Rst"] = self.Rad_FrmCtrlCfg_Rst
            else:
                self.Rad_FrmCtrlCfg_Rst = dFrmCtrlCfg["Rst"]
            if not('RegChnCtrl' in dFrmCtrlCfg):
                dFrmCtrlCfg["RegChnCtrl"] = self.Rad_FrmCtrlCfg_RegChnCtrl
            else:
                self.Rad_FrmCtrlCfg_RegChnCtrl = dFrmCtrlCfg["RegChnCtrl"]
            if not('FrmCntrStrt' in dFrmCtrlCfg):
                dFrmCtrlCfg["FrmCntrStrt"] = self.Rad_FrmCtrlCfg_FrmCntrStrt
            else:
                self.Rad_FrmCtrlCfg_FrmCntrStrt = dFrmCtrlCfg["FrmCntrStrt"]
            if not('FrmCntrInc' in dFrmCtrlCfg):
                dFrmCtrlCfg["FrmCntrInc"] = self.Rad_FrmCtrlCfg_FrmCntrInc
            else:
                self.Rad_FrmCtrlCfg_FrmCntrInc = dFrmCtrlCfg["FrmCntrInc"]
            if not('RegCtrl' in dFrmCtrlCfg):
                dFrmCtrlCfg["RegCtrl"] = self.Rad_FrmCtrlCfg_RegCtrl
            else:
                self.Rad_FrmCtrlCfg_RegCtrl = dFrmCtrlCfg["RegCtrl"]
        else:
            dFrmCtrlCfg = {
                                            ["ChnSel"]      :   self.Rad_FrmCtrlCfg_ChnSel,
                                            ["Rst"]         :   self.Rad_FrmCtrlCfg_Rst,
                                            ["RegChnCtrl"]  :   self.Rad_FrmCtrlCfg_RegChnCtrl,
                                            ["FrmCntrStrt"] :   self.Rad_FrmCtrlCfg_FrmCntrStrt,
                                            ["FrmCntrInc"]  :   self.Rad_FrmCtrlCfg_FrmCntrInc,
                                            ["RegCtrl"]     :   self.Rad_FrmCtrlCfg_RegCtrl
                                        }

        Cod = int('0x9012',0);
        FpgaCmd = zeros(6, dtype='uint32')
        FpgaCmd[0] = dFrmCtrlCfg["ChnSel"]
        FpgaCmd[1] = dFrmCtrlCfg["Rst"]
        FpgaCmd[2] = dFrmCtrlCfg["RegChnCtrl"]
        FpgaCmd[3] = dFrmCtrlCfg["FrmCntrStrt"]
        FpgaCmd[4] = dFrmCtrlCfg["FrmCntrInc"]
        FpgaCmd[5] = dFrmCtrlCfg["RegCtrl"]
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetFrmCtrl Cfg:", FpgaCmd)
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
            dCicCfg = varargin[0]
            if not ('FiltSel' in dCicCfg):
                dCicCfg["FiltSel"] = self.Rad_CicCfg_FiltSel
            else:
                self.Rad_CicCfg_FiltSel = dCicCfg["FiltSel"]
            if not ('CombDel' in dCicCfg):
                dCicCfg["CombDel"] = self.Rad_CicCfg_CombDel
            else:
                self.Rad_CicCfg_CombDel = dCicCfg["CombDel"]
            if not ('OutSel' in dCicCfg):
                dCicCfg["OutSel"] = self.Rad_CicCfg_OutSel
            else:
                self.Rad_CicCfg_OutSel = dCicCfg["OutSel"]
            if not ('SampPhs' in dCicCfg):
                dCicCfg["SampPhs"] = self.Rad_CicCfg_SampPhs
            else:
                self.Rad_CicCfg_SampPhs = dCicCfg["SampPhs"]
            if not ('SampRed' in dCicCfg):
                dCicCfg["SampRed"] = self.Rad_CicCfg_SampRed
            else:
                self.Rad_CicCfg_SampRed = dCicCfg["SampRed"]
            if not ('RegCtrl' in dCicCfg):
                dCicCfg["RegCtrl"] = self.Rad_CicCfg_RegCtrl
            else:
                self.Rad_CicCfg_RegCtrl = dCicCfg["RegCtrl"]
        else:
            dCicCfg = {
                                        "FiltSel"   :   self.Rad_CicCfg_FiltSel,
                                        "CombDel"   :   self.Rad_CicCfg_CombDel,
                                        "OutSel"    :   self.Rad_CicCfg_OutSel,
                                        "SampPhs"   :   self.Rad_CicCfg_SampPhs,
                                        "SampRed"   :   self.Rad_CicCfg_SampRed,
                                        "RegCtrl"   :   self.Rad_CicCfg_RegCtrl
                                    }
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetCic Cfg:", dCicCfg)
        Cod = int('0x9101',0)
        FpgaCmd = zeros(6, dtype='uint32')
        FpgaCmd[0] = dCicCfg["FiltSel"]
        FpgaCmd[1] = dCicCfg["CombDel"]
        FpgaCmd[2] = dCicCfg["OutSel"]
        FpgaCmd[3] = dCicCfg["SampPhs"]
        FpgaCmd[4] = dCicCfg["SampRed"]
        FpgaCmd[5] = dCicCfg["RegCtrl"]
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
            dSampCfg = varargin[0]
            if not ('Nr' in dSampCfg):
                dSampCfg["Nr"] = self.Rad_N
                self.Rad_SampCfg_Nr = self.Rad_N
            if not ('Rst' in dSampCfg):
                dSampCfg["Rst"] = self.Rad_Rst
                self.Rad_SampCfg_Rst = self.Rad_Rst
        else:
            dSampCfg = {
                                    "Nr"        :   self.Rad_SampCfg_Nr,
                                    "Rst"       :   self.Rad_SampCfg_Rst
                                }

        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetSamp Cfg", dSampCfg)

        Cod = int('0x9102',0)
        FpgaCmd = zeros(2,dtype='uint32')
        FpgaCmd[0] = dSampCfg["Nr"]
        FpgaCmd[1] = dSampCfg["Rst"]
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x9213',0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(1,dtype='uint32')
        Cod = int('0x9212',0)
        FpgaCmd[0] = 1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x9212',0)
        FpgaCmd[0] = 2
        FpgaCmd[1] = NrEntries
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x9212',0)
        FpgaCmd[0] = 0
        FpgaCmd[1] = Mod
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
            Data = Data[0:28]

        FpgaCmd = zeros(int(2 + len(Data)),dtype='uint32')
        Cod = int('0x9212',0)
        FpgaCmd[0] = 4
        FpgaCmd[1] = SeqNr
        FpgaCmd[2:] = Data
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
            Data = Data[0:28]
        FpgaCmd = zeros(int(3 + len(Data)),dtype='uint32')
        Cod = int('0x9212',0)
        FpgaCmd[0] = 3
        FpgaCmd[1] = Chn
        FpgaCmd[2] = SeqNr
        FpgaCmd[3:] = Data
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
    #>      Temp1 (AD8283) = 62.1473°
    #>      Temp2 (Sup) = 56.4602°
    #>      VSup (Sup) = 16.4193V
    #>      ISup (Sup) = 0.74766A
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
        Val = self.Fpga_GetBrdInf()
        Ret = -1
        if len(Val) > 1:
            uidHigh = '%08X' % (Val[1]);
            uidLow = '%08X' % (Val[0]);
            print('UID = ', uidHigh, uidLow, 'h');
            if len(Val) > 2:
                Temp = Val[2] - 273;
                print('Temp = ', Temp, 'deg');
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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x9015',0)
        if "IntEna" in dCfg:
            FpgaCmd[0] = dCfg["IntEna"]
        else:
            FpgaCmd[0] = 0
        if "RfEna" in dCfg:
            FpgaCmd[1] = dCfg["RfEna"]
        else:
            FpgaCmd[1] = 0
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetRfPwr Ret:", Ret)
        return Ret

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Disp status of RF frontend
    #>
    #> All RF frontends are equipped with a EEPROM device, to store the board information and the calibration data
    #> The BrdDispRfSts function prints the status of the board to the command window. If the ID of the frontend is knwon, the chip configuration is also printlayed.
    #> Most RF chips have an unique identification number. This number is printed in the command window.
    #>
    #>   After calling BrdDispRfSts the configuration is plotted in the Matlab command window.
    #>   In case of the MIMO77 frontend the status of the chips is printlayed too.
    #>   @code
    #>      -------------------------------------------------------------------
    #>      RF Board Configuration
    #>      MIMO-77-TX4RX8
    #>      RF UID = 64
    #>      RF RevNr = 1.1.0
    #>      RF SerialNr = 64000E
    #>      RF Date = 07-Jan-2016
    #>      RF Startup = 7
    #>      --------------------
    #>      RCC = 40091739537F
    #>      DPA1 = 1067295654D8
    #>      DPA2 = 1067284754D8
    #>      DPA3 = 1067253254D8
    #>      MRX1 = 1000000200
    #>      MRX2 = 1000000200
    #>      -------------------------------------------------------------------
    #>   @endcode
    #>   @note To printlay the ID of the RF chips the RF supply must be enabled (BrdPwrEna())
    def     Fpga_DispRfChipSts(self, Mask):
        print(' ')
        print('-------------------------------------------------------------------')
        print('RF Board Configuration')
        Val = array([self.Fpga_GetRfChipSts(Mask)])


        if len(Val) > 2:
                print('BrdInf');
                print('RF UID = ', Val[0])
                print('RF RevNr = ', self.RevStr(Val[1]))
                print('RF SerialNr = ', Val[2])
                print('RF Date = ', Val[3])
                print('RF Startup = ', Val[4])
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
    #>   @note To printlay the ID of the RF chips the RF supply must be enabled (BrdPwrEna())
    def     Fpga_SeqTrigCfgSeq(self, Mask, SeqNr, dSeq):
        FpgaCmd = zeros(19,dtype='uint32')
        Cod = int('0x9213',0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 2
        FpgaCmd[2] = SeqNr
        FpgaCmd[3] = dSeq["CntrCtrl"]
        FpgaCmd[4] = dSeq["CntrPerd"]
        FpgaCmd[5] = dSeq["NextAdr"]
        FpgaCmd[6] = dSeq["SeqId"]
        FpgaCmd[7] = dSeq["Chn0Tim0"]
        FpgaCmd[8] = dSeq["Chn0Tim1"]
        FpgaCmd[9] = dSeq["Chn0Cfg"]
        FpgaCmd[10] = dSeq["Chn1Tim0"]
        FpgaCmd[11] = dSeq["Chn1Tim1"]
        FpgaCmd[12] = dSeq["Chn1Cfg"]
        FpgaCmd[13] = dSeq["Chn2Tim0"]
        FpgaCmd[14] = dSeq["Chn2Tim1"]
        FpgaCmd[15] = dSeq["Chn2Cfg"]
        FpgaCmd[16] = dSeq["Chn3Tim0"]
        FpgaCmd[17] = dSeq["Chn3Tim1"]
        FpgaCmd[18] = dSeq["Chn3Cfg"]
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigCfgSeq: ", dSeq)
        Ret = self.CmdSend(0, Cod, FpgaCmd);
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(1,dtype='uint32')
        Cod = int('0x9013',0)
        FpgaCmd[0] = 0
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(4,dtype='uint32')
        Cod = int('0x9213',0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 3
        FpgaCmd[2] = RegCtrl
        FpgaCmd[3] = RegChnEna
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SeqTrigLoad Cfg:", FpgaCmd)

        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()

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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x9213',0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 4
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        Result = self.Fpga_GetSwVers()
        Result = ((self.SVers % 256),)
        Dummy = self.SVers // 256
        Result      +=  ((Dummy//256),)
        Dummy = Dummy // 256
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
        Vers = self.Fpga_GetSwVers()
        if len(Vers) > 2:
            Tmp = Vers[0]
            SwPatch = Tmp % 2**8
            Tmp = floor(Tmp/2**8)
            SwMin = Tmp % 2**8
            SwMaj = floor(Tmp/2**8)
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
        FpgaCmd = zeros(1,dtype='uint32')
        Cod = int('0x900E',0)
        FpgaCmd[0] = 0
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return  Ret


    def Fpga_CfgCpyDat(self,*varargin):
        if len(varargin) > 0:
            dArmMemCfg = varargin[0]
            if not ('Mask' in dArmMemCfg):
                dArmMemCfg['Mask'] = self.Rad_ArmMemCfg_Mask;
            else:
                self.Rad_ArmMemCfg_Mask = dArmMemCfg['Mask']
            if not ('Chn' in dArmMemCfg):
                dArmMemCfg['Chn'] = self.Rad_ArmMemCfg_Chn
            else:
                self.Rad_ArmMemCfg_Chn = dArmMemCfg['Chn']
            if not ('Ctrl' in dArmMemCfg):
                dArmMemCfg['Ctrl'] = self.Rad_ArmMemCfg_Ctrl
            else:
                self.Rad_ArmMemCfg_Ctrl = dArmMemCfg['Ctrl']

            if not ('FrmSiz' in dArmMemCfg):
                dArmMemCfg['FrmSiz'] = self.Rad_ArmMemCfg_FrmSiz
            else:
                self.Rad_ArmMemCfg_FrmSiz = dArmMemCfg['FrmSiz']
            if not ('WaitLev' in dArmMemCfg):
                dArmMemCfg['WaitLev'] = self.Rad_ArmMemCfg_WaitLev
            else:
                self.Rad_ArmMemCfg_WaitLev = dArmMemCfg['WaitLev']
        else:
            dArmMemCfg['Mask'] = self.Rad_ArmMemCfg_Mask
            dArmMemCfg['Ctrl'] = self.Rad_ArmMemCfg_Ctrl
            dArmMemCfg['Chn'] = self.Rad_ArmMemCfg_Chn
            dArmMemCfg['FrmSiz'] = self.Rad_ArmMemCfg_FrmSiz
            dArmMemCfg['WaitLev'] = self.Rad_ArmMemCfg_WaitLev

        NrChn = self.Get('NrChn');
        Cod = int('0x9400', 0)
        FpgaCmd = zeros(12, dtype='uint32')
        #self.DatPort = 6000 + floor(1000.*rand(1));
        FpgaCmd[0] = dArmMemCfg['Mask']
        FpgaCmd[1] = 2
        FpgaCmd[2] = dArmMemCfg['Ctrl']
        FpgaCmd[3] = dArmMemCfg['Chn']
        FpgaCmd[4] = self.Rad_NrRangeBins
        FpgaCmd[5] = self.Rad_NrFrms
        FpgaCmd[6] = NrChn
        FpgaCmd[7] = self.Rad_VelFftSiz
        FpgaCmd[8] = self.Rad_VelFftShift
        FpgaCmd[9] = self.Rad_StreamSiz
        FpgaCmd[10] = self.Rad_LoMaxRecCfg
        FpgaCmd[11] = self.cRbkDat
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return  Ret

    def     Fpga_CfgArmMem(self,*varargin):
        if len(varargin) > 0:
            dArmMemCfg = varargin[0]
            if not ('Mask' in dArmMemCfg):
                dArmMemCfg['Mask'] = self.Rad_ArmMemCfg_Mask
            else:
                self.Rad_ArmMemCfg_Mask = dArmMemCfg['Mask']
            if not ('Chn' in dArmMemCfg):
                dArmMemCfg['Chn'] = self.Rad_ArmMemCfg_Chn
            else:
                self.Rad_ArmMemCfg_Chn = dArmMemCfg['Chn']
            if not ('Ctrl' in dArmMemCfg):
                dArmMemCfg['Ctrl'] = self.Rad_ArmMemCfg_Ctrl
            else:
                self.Rad_ArmMemCfg_Ctrl = dArmMemCfg['Ctrl']

            if not ('FrmSiz' in dArmMemCfg):
                dArmMemCfg['FrmSiz'] = self.Rad_ArmMemCfg_FrmSiz
            else:
                self.Rad_ArmMemCfg_FrmSiz = dArmMemCfg['FrmSiz']

            if not ('WaitLev' in dArmMemCfg):
                dArmMemCfg['WaitLev'] = self.Rad_ArmMemCfg_WaitLev
            else:
                self.Rad_ArmMemCfg_WaitLev = dArmMemCfg['WaitLev']
        else:
            dArmMemCfg = {
                    'Mask'          :   self.Rad_ArmMemCfg_Mask,
                    'Ctrl'          :   self.Rad_ArmMemCfg_Ctrl,
                    'Chn'           :   self.Rad_ArmMemCfg_Chn,
                    'FrmSiz'        :   self.Rad_ArmMemCfg_FrmSiz,
                    'WaitLev'       :   self.Rad_ArmMemCfg_WaitLev
                    }

        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_CfgArmMem:", dArmMemCfg)

        Cod = int('0x9400', 0)
        FpgaCmd = zeros(9, dtype='uint32')
        FpgaCmd[0] = dArmMemCfg['Mask']
        FpgaCmd[1] = 1
        FpgaCmd[2] = dArmMemCfg['Ctrl']
        FpgaCmd[3] = dArmMemCfg['Chn']
        FpgaCmd[4] = dArmMemCfg['FrmSiz']
        FpgaCmd[5] = dArmMemCfg['WaitLev']
        FpgaCmd[6] = self.cRbkDat
        FpgaCmd[7] = self.Rad_DmaMult
        FpgaCmd[8] = self.Rad_DmaSel

        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        return  Ret

    def Fpga_GetData(self):
        #   @function       CmdRecv
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Receive response from baseboard
        TxData = 0
        if self.hConDat > -1:
            TxData = 1
        else:
            self.OpenTcpIpDatCom(self.cRbkIp, int(self.cRbkDat))
            TxData = 1

        Ret = zeros((int(self.Rad_N), int(self.Rad_NrVirtChn)), dtype='int16')

        if TxData > 0:



            try:
                RxBytes = self.cRadDatSocket.recv(int(2*self.Rad_N*self.Rad_NrVirtChn))
            except Exception:
                print("Error in Arm_ReadFrm recv: Exception raised")
                return Ret

            while len(RxBytes) < (2*self.Rad_N*self.Rad_NrChn):
                LenData = len(RxBytes)
                RxBytes = RxBytes + self.cRadDatSocket.recv(int(2*self.Rad_N*self.Rad_NrChn - LenData))

            Res = fromstring(RxBytes, dtype = 'int16')
            Ret = reshape(Res, (int(self.Rad_NrChn), int(self.Rad_N)))


        return Ret.transpose()

    def Fpga_GetData32Len(self, Len):
        #   @function       CmdRecv
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Receive response from baseboard
        TxData = 0
        if self.hConDat > -1:
            TxData = 1
        else:
            self.OpenTcpIpDatCom(self.cRbkIp, int(self.cRbkDat))
            TxData = 1

        if TxData > 0:

            try:
                RxBytes = self.cRadDatSocket.recv(int(4*Len))
            except Exception:
                print("Error in Arm_ReadFrm recv: Exception raised")
                return Ret

            while len(RxBytes) < (4*Len):
                LenData = len(RxBytes)
                RxBytes = RxBytes + self.cRadDatSocket.recv(int(4*Len - LenData))

            Res = fromstring(RxBytes, dtype = 'int32')

        return Res


    def Fpga_GetData32(self):
        #   @function       CmdRecv
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Receive response from baseboard
        TxData = 0
        if self.hConDat > -1:
            TxData = 1
        else:
            self.OpenTcpIpDatCom(self.cRbkIp, int(self.cRbkDat))
            TxData = 1

        if TxData > 0:



            try:
                RxBytes = self.cRadDatSocket.recv(int(2*self.Rad_DatSiz*self.Rad_NTx*self.Rad_NrVirtChn))
            except Exception:
                print("Error in Arm_ReadFrm recv: Exception raised")
                return Ret

            while len(RxBytes) < (2*self.Rad_DatSiz*self.Rad_NTx*self.Rad_NrVirtChn):
                LenData = len(RxBytes)
                RxBytes = RxBytes + self.cRadDatSocket.recv(int(2*self.Rad_DatSiz*self.Rad_NTx*self.Rad_NrVirtChn - LenData))

            Res = fromstring(RxBytes, dtype = 'int32')
            Ret = reshape(Res, (int(self.Rad_NrVirtChn), int(self.Rad_NTx*self.Rad_DatSiz/2)))


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
        FpgaCmd = zeros(5, dtype='uint32')
        Cod = int('0x901F',0)
        if not ('Type' in dCfg):
            dCfg["Type"] = 0

        FpgaCmd[0] = dCfg["Mask"]
        FpgaCmd[1] = 2
        FpgaCmd[2] = dCfg["Type"]
        FpgaCmd[3] = dCfg["RevNr"]
        FpgaCmd[4] = dCfg["DateNr"]
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x901F',0)
        FpgaCmd[0] = dCfg["Mask"]
        FpgaCmd[1] = 4
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        N = dCfg["Len"]
        StrtIdx = 1
        Data = zeros(int(N), dtype='int32')
        while(StrtIdx < N):
            if (StrtIdx + 24) < N:
                Len = 24
            else:
                Len = N - StrtIdx + 1
            FpgaCmd = zeros(4, dtype='uint32')
            Cod = int('0x901F',0)
            FpgaCmd[0] = dCfg["Mask"]
            FpgaCmd[1] = 5
            FpgaCmd[2] = StrtIdx
            FpgaCmd[3] = Len
            Ret = self.CmdSend(0, Cod, FpgaCmd)
            Ret = self.CmdRecv()
            Data[int(StrtIdx-1):int(StrtIdx+Len-1)] = Ret
            StrtIdx = StrtIdx + Len
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
        Data = dCfg["Data"]

        N = len(Data)
        StrtIdx = 1
        while(StrtIdx < N):
            Cod = int('0x901F',0)
            if (StrtIdx + 24) < N:
                TmpData = Data[StrtIdx-1:StrtIdx + 24 - 1]
                Len = 24
            else:
                TmpData = Data[StrtIdx-1:]
                Len = N - StrtIdx + 1

            FpgaCmd = zeros(int(Len + 4), dtype='uint32')
            FpgaCmd[0] = dCfg["Mask"]
            FpgaCmd[1] = 3
            FpgaCmd[2] = StrtIdx
            FpgaCmd[3] = Len
            FpgaCmd[4:] = TmpData
            if (int(self.cDebugInf) & 8) > 0:
                print("Fpga_SetCalData Cfg:", Fpga_SetCalData)

            Ret = self.CmdSend(0, Cod, FpgaCmd)
            Ret = self.CmdRecv()
            if (int(self.cDebugInf) & 8) > 0:
                print("Fpga_SetCalData Ret:", Ret)
            StrtIdx = StrtIdx + Len



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
        N = dCfg['Len']
        StrtIdx = 1
        Data = []
        while(StrtIdx < N):
            if (StrtIdx + 24) < N:
                Len = 24
            else:
                Len = N - StrtIdx + 1

            FpgaCmd = zeros(int(5), dtype='uint32')
            Cod = int('0x901F',0)
            FpgaCmd[0] = dCfg["Mask"]
            FpgaCmd[1] = 15
            FpgaCmd[2] = dCfg["Table"]
            FpgaCmd[3] = StrtIdx
            FpgaCmd[4] = Len
            Ret = self.CmdSend(0, Cod, FpgaCmd);
            Ret = self.CmdRecv()
            StrtIdx = StrtIdx + Len
            if Ret[0]:
                Data = append(asarray(Data), asarray(Ret[1]))
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
        N = len(dCfg['Data'])
        StrtIdx = 0
        while(StrtIdx < N):
            Cod = int('0x901F',0)
            if (StrtIdx + 24) < N:
                Data = dCfg['Data'][StrtIdx:StrtIdx + 24]
                Len = 24
            else:
                Data = dCfg['Data'][StrtIdx:]
                Len = N - StrtIdx + 1

            FpgaCmd = zeros(int(5 + len(Data)), dtype='uint32')
            FpgaCmd[0] = dCfg["Mask"]
            FpgaCmd[1] = 13
            FpgaCmd[2] = dCfg["Table"]
            FpgaCmd[3] = StrtIdx + 1
            FpgaCmd[4] = Len
            FpgaCmd[5:] = asarray(Data)
            Ret = self.CmdSend(0,Cod,FpgaCmd)
            Ret = self.CmdRecv()
            StrtIdx = StrtIdx + Len

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
        Ret = []
        if dInf['Type'] == 1:
            self.Fpga_SetTableInf(dCfg, dInf)
            dCfg['Data'] = asarray([dTab1['VcoCTune'], dTab1['VcoCoreBiasing'], dTab1['CTuneTx'], dTab1['CTuneRx']])
            Ret = self.Fpga_SetTableData(dCfg)
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
        dTable = dict()
        self.Fpga_RdTable(dCfg)
        dTable['dInf'] = self.Fpga_GetTableInf(dCfg)
        if dTable['dInf']['Type'] == 1:
            dCfg['Len'] = dTable['dInf']['Entrs']
            Data = self.Fpga_GetTableData(dCfg)
            if len(Data) == 4:
                dTable['dTab1'] = dict()
                dTable['dTab1']['VcoCTune'] = Data[0]
                dTable['dTab1']['VcoCoreBiasing'] = Data[1]
                dTable['dTab1']['CTuneTx'] = Data[2]
                dTable['dTab1']['CTuneRx'] = Data[3]
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
        Cod = int('0x901F',0)
        dInf = dict()
        if not ('Table' in dCfg):
            dCfg['Table'] = 0
        FpgaCmd = zeros(3,dtype='uint32')
        FpgaCmd[0] = dCfg['Mask']
        FpgaCmd[1] = 14
        FpgaCmd[2] = dCfg['Table']
        Ret = self.CmdSend(0,Cod,FpgaCmd)
        Ret = self.CmdRecv()
        if Ret[0]:
            Ret = Ret[1]
            dInf['Entrs'] = Ret[0]
            dInf['Chn'] = Ret[1]
            dInf['Type'] = Ret[2]
            dInf['Date'] = Ret[3]
            dInf['RevNr'] = Ret[4]
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
        Cod = int('0x901F', 0)
        if not ('Table' in dCfg):
            dCfg['Table'] = 0
        FpgaCmd = zeros(8,dtype='uint32')
        FpgaCmd[0] = dCfg['Mask']
        FpgaCmd[1] = 12
        FpgaCmd[2] = dCfg['Table']
        FpgaCmd[3] = dInf['Entrs']
        FpgaCmd[4] = dInf['Chn']
        FpgaCmd[5] = dInf['Type']
        FpgaCmd[6] = dInf['DateNr']
        FpgaCmd[7] = dInf['RevNr']

        Ret = self.CmdSend(0,Cod,FpgaCmd)
        Ret = self.CmdRecv()
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
        Cod = inf('0x901F',0)
        if not ('Table' in dCfg):
            dCfg['Table'] = 0
        FpgaCmd = zeros(3,dtype='uint32')
        FpgaCmd[0] = dCfg['Mask']
        FpgaCmd[1] = 10
        FpgaCmd[2] = dCfg['Table']
        Ret = self.CmdSend(0,Cod,FpgaCmd)
        Ret = self.CmdRecv()
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
        Cod = int('0x901F',0)
        if not ('Table' in dCfg):
            dCfg['Table'] = 0
        FpgaCmd = zeros(3,dtype='uint32')
        FpgaCmd[0] = dCfg['Mask']
        FpgaCmd[1] = 11
        FpgaCmd[2] = dCfg['Table']
        Ret = self.CmdSend(0,Cod,FpgaCmd)
        Ret = self.CmdRecv()
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
        Cod = int('0x901F',0)
        if not ('Table' in dCfg):
            dCfg['Table'] = 0
        FpgaCmd = zeros(3,dtype='uint32')
        FpgaCmd[0] = dCfg['Mask']
        FpgaCmd[1] = 9
        FpgaCmd[2] = dCfg['Table']
        Ret = self.CmdSend(0,Cod,FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(4,dtype='uint32')
        Cod = int('0x901F',0)
        if not ('Table' in dCfg):
            dCfg["Table"] = 0

        FpgaCmd[0] = dCfg["Mask"]
        FpgaCmd[1] = 1
        FpgaCmd[2] = dCfg["Type"]
        FpgaCmd[3] = dCfg["Table"]
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        FpgaCmd = zeros(2,dtype='uint32')
        Cod = int('0x901A',0)
        FpgaCmd[0] = Mask
        FpgaCmd[1] = 1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_GetRfChipSts Ret:", Ret)
        return Ret


    def     Fpga_MimoRstSequencer(self):
        FpgaCmd = zeros(1,dtype='uint32')
        Cod = int('0x9212',0)
        FpgaCmd[0] = 1
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        # Regs = Regs.flatten()
        if (len(Regs) > 28):
            Regs = Regs[0:28];

        FpgaCmd = zeros(int(3 + len(Regs)), dtype='uint32')
        Cod = int('0x9017',0)
        FpgaCmd[0] = dUSpiCfg["Mask"]
        FpgaCmd[1] = 1
        FpgaCmd[2] = dUSpiCfg["Chn"]
        FpgaCmd[3:] = Regs
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
        Regs = array(Regs)
        # Regs = Regs.flatten()
        if (len(Regs) > 28):
            Regs = Regs[0:28]

        FpgaCmd = zeros(int(3 + len(Regs)), dtype='uint32')
        Cod = int('0x9018',0)
        FpgaCmd[0] = dUSpiCfg["Mask"]
        FpgaCmd[1] = 1
        FpgaCmd[2] = dUSpiCfg["Chn"]
        FpgaCmd[3:] = Regs
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
        if (int(self.cDebugInf) & 8) > 0:
            print("Fpga_SetUSpi8Data Ret:", Ret)
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
        Cod = int('0x9004', 0)
        FpgaCmd = zeros(3, dtype='uint32')
        FpgaCmd[0] = 1
        FpgaCmd[1] = Role
        FpgaCmd[2] = AdcSel
        Ret = self.CmdSend(0, Cod, FpgaCmd)
        Ret = self.CmdRecv()
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
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K10 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K10 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 9:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K9 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K9 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 8:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K8 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K8 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 7:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K7 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K7 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 6:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K6 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K6 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >=5:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K5 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K5 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 4:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K4 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K4 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 3:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K3 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K3 + self.AFE5801_REG_HIGHPASS_ENABLE
        elif Val >= 2:
            self.Rad_Afe5801_RegHighpass14 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K2 + self.AFE5801_REG_HIGHPASS_ENABLE
            self.Rad_Afe5801_RegHighpass58 = self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K2 + self.AFE5801_REG_HIGHPASS_ENABLE
        else:
            self.Rad_Afe5801_RegHighpass14 = 0
            self.Rad_Afe5801_RegHighpass58 = 0

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
        RevMaj = int(floor(Val/2**16))
        RevMin = int(floor(Val/2**8) % 2**8)
        RevPat = int(Val % 2**8)
        stRev = str(RevMaj) + '.' + str(RevMin) + '.' + str(RevPat)
        return stRev

    def Fpga_GetSwCfgString(self):
        Result = self.Fpga_GetSwCfg()
        return      "SwVers: " + str(Result[2]) + "-" + str(Result[1]) + "-" + str(Result[0])

    def GetAdcChn(self, *varargin):
        #   @function    GetAdcChn
        #   @author      Haderer Andreas (HaAn)
        #   @date        2013-12-01
        #   @brief       Return nr of configured channels for a single AD8283
        #   @param[in]   AdcCfg (optional): If no parameter is provided, then the field of the obj is analyzed
        #                Required field:
        #                   ChCtrl          Channel control register of ADC
        Nr = 0
        if len(varargin) > 0:
            dAdcCfg = varargin[0]
            if 'ChCtrl' in dAdcCfg:
                if AdcdCfg['ChCtrl'] == self.cAD8283_REG_MUXCNTRL_AB:
                    Nr = 2
                elif dAdcCfg["ChCtrl"] == self.cAD8283_REG_MUXCNTRL_ABC:
                    Nr = 3
                elif dAdcCfg["ChCtrl"] ==  self.cAD8283_REG_MUXCNTRL_ABCD:
                        Nr = 4;
                elif dAdcCfg["ChCtrl"] ==  self.cAD8283_REG_MUXCNTRL_ABCDE:
                        Nr = 5;
                elif dAdcCfg["ChCtrl"] ==  self.cAD8283_REG_MUXCNTRL_ABCDEF:
                        Nr = 6;
                else:
                    print('ADC Cfg: Error channel setting!')
                    Nr = -1
            else:
                print('ADC Cfg: ChCtrl field missing!')
        else:
            if self.Rad_AdcCfg_ChCtrl == self.cAD8283_REG_MUXCNTRL_AB:
                Nr = 2
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABC:
                Nr = 3
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABCD:
                Nr = 4
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABCDE:
                Nr = 5
            elif self.Rad_AdcCfg_ChCtrl ==  self.cAD8283_REG_MUXCNTRL_ABCDEF:
                Nr = 6;
            else:
                print('ADC Cfg: Error channel setting!')
                Nr = -1;
        return Nr


    def RcsCornerCube(self, fc, aCorner):
        #   @file        RefCornerCube.m
        #   @author      Andreas Haderer
        #   @date        2014-05-05
        #   @brief       Calcuate RCS of corner cube, aCorner: length of connecting edge
        c0 = 3e8
        Lambdac = c0/fc
        Rcs = 4*pi*aCorner**4/(3*Lambdac**2)
        return      Rcs

    def GetAdcImp(self, *varargin):
        #   @function    GetAdcImp
        #   @author      Haderer Andreas (HaAn)
        #   @date        2013-12-01
        #   @brief       Return AD8283 Input Impedance
        #   @param[in]   AdcCfg (optional): If no parameter is provided, then the field of the obj is analyzed
        #                Required field:
        #                   ImpCtrl          Channel impedance control register of ADC

        Imp = 0
        if len(varargin) > 0:
            dAdcCfg = varargin[0]
            if 'ImpCtrl' in dAdcCfg:
                if dAdcCfg["ChCtrl"] == self.cAD8283_REG_CHIMP_200E:
                    Imp = 200
                elif dAdcCfg["ChCtrl"] == self.cAD8283_REG_CHIMP_200K:
                    Imp = 200e3
                else:
                    print('ADC Cfg: Error impedance setting!')
                    Imp = -1
            else:
                print('ADC Cfg: ImpCtrl field missing!')
        else:
            if self.Rad_AdcCfg_ImpCtrl == self.cAD8283_REG_CHIMP_200E:
                Imp = 200
            elif self.Rad_AdcCfg_ImpCtrl == self.cAD8283_REG_CHIMP_200K:
                Imp = 200e3
            else:
                print('ADC Cfg: Error impedance setting!')
                Imp = -1
        return Imp

    def Num2Cal(self, Val):
        #   @function       Num2Cal
        #   @author         Haderer Andreas (HaAn)
        #   @date           2013-12-01
        #   @brief          Convert calibration number to s8.24 number format
        #   @param[in]      Double array with cal
        #   @return         S8.24 values for eeprom
        # if isreal(Val)
        #     Val = round(Val*2**24);
        #     Idcs = find(Val < 0);
        #     Val(Idcs) = Val(Idcs) + 2**32;
        #     Cal = Val;
        # else
        #     Val1 = round(real(Val)*2**24);
        #     Idcs = find(Val1 < 0);
        #     Val1(Idcs) = Val1(Idcs) + 2**32;
        #     CalReal = Val1;
        #     Val1 = round(imag(Val)*2**24);
        #     Idcs = find(Val1 < 0);
        #     Val1(Idcs) = Val1(Idcs) + 2**32;
        #     CalImag = Val1;
        #     Cal = CalReal + i.*CalImag;
        if isreal(Val).all:
            Cal = array(Val*2**24, dtype=int32)
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
        Ret = Cal/2**24

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
        #m = [-(M-1)/2: (M-1)/2].';
        m = linspace(-(M-1)/2,(M-1)/2,M)
        Win = 0.5 + 0.5*cos(2*pi*m/M)
        if len(varargin) > 0:
            N = varargin[0]
            Win = broadcast_to(Win,(N,M)).T
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
        self.AFE5801_REG_CONFIG_TGC_WREN = int('0x0004', 0)    # *< Use to enable access to TGC registers
        self.AFE5801_REG_CONFIG_READOUT_EN = int('0x0002', 0)    # *< Use to enable register readout
        self.AFE5801_REG_CONFIG_SOFT_RESET = int('0x0001', 0)    # *< Use to reset the AFE5801

        self.AFE5801_REG_POWER_OUTRATE2X = int('0x4000', 0)    # *<
        self.AFE5801_REG_POWER_EXT_REF = int('0x2000', 0)    # *<
        self.AFE5801_REG_POWER_LOW_FREQ_NOISE_SUPPRESS = int('0x0800', 0)    # *<
        self.AFE5801_REG_POWER_STANDBY = int('0x0400', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH1 = int('0x0004', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH2 = int('0x0008', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH3 = int('0x0010', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH4 = int('0x0020', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH5 = int('0x0040', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH6 = int('0x0080', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH7 = int('0x0100', 0)    # *<
        self.AFE5801_REG_POWER_PDN_CH8 = int('0x0200', 0)    # *<
        self.AFE5801_REG_POWER_OUTPUT_DISABLE = int('0x0002', 0)    # *<
        self.AFE5801_REG_POWER_GLOBAL_PDN = int('0x0001', 0)    # *<

        self.AFE5801_REG_LVDS_MODE_NO_PATTERN = int('0x0000', 0)    # *< normal ADC data output
        self.AFE5801_REG_LVDS_MODE_PATTERN_SYNC = int('0x2000', 0)    # *< output pattern 111111000000
        self.AFE5801_REG_LVDS_MODE_PATTERN_DESKEW = int('0x4000', 0)    # *< output pattern 010101010101
        self.AFE5801_REG_LVDS_MODE_PATTERN_CUSTOM = int('0x6000', 0)    # *< output pattern <custom>
        self.AFE5801_REG_LVDS_MODE_PATTERN_ALL1 = int('0x8000', 0)    # *< output pattern 111111111111
        self.AFE5801_REG_LVDS_MODE_PATTERN_TOGGLE = int('0xA000', 0)    # *< output pattern 111111111111 000000000000 ...
        self.AFE5801_REG_LVDS_MODE_PATTERN_ALL0 = int('0xC000', 0)    # *< output pattern 000000000000
        self.AFE5801_REG_LVDS_MODE_PATTERN_RAMP = int('0xE000', 0)    # *< output increasing values
        self.AFE5801_REG_LVDS_AVERAGING_ENABLE = int('0x0800', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS1 = int('0x0008', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS2 = int('0x0010', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS3 = int('0x0020', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS4 = int('0x0040', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS5 = int('0x0080', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS6 = int('0x0100', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS7 = int('0x0200', 0)    #
        self.AFE5801_REG_LVDS_PDN_LVDS8 = int('0x0400', 0)    #

        self.AFE5801_REG_DATA_SERIAL_RATE12 = int('0x0000', 0)    #
        self.AFE5801_REG_DATA_SERIAL_RATE10 = int('0x2000', 0)    #
        self.AFE5801_REG_DATA_SERIAL_RATE16 = int('0x4000', 0)    #
        self.AFE5801_REG_DATA_SERIAL_RATE14 = int('0x6000', 0)    #
        self.AFE5801_REG_DATA_DIGITAL_GAIN_ENABLE = int('0x1000', 0)    #
        self.AFE5801_REG_DATA_OFFSET_SUB_ENABLE = int('0x0100', 0)    #

        self.AFE5801_REG_DFS_OFFSET_BINARY = int('0x0008', 0)    # *< use if data format should be offset binary instead of default two's complement

        self.AFE5801_REG_FILTER_VCA_LOW_NOISE = int('0x0400', 0)    # *< enable low noise, increases power consumtion!
        self.AFE5801_REG_FILTER_SELF_TEST_DISABLE = int('0x0000', 0)    # *< disable self-test
        self.AFE5801_REG_FILTER_SELF_TEST_ENABLE100 = int('0x0100', 0)    # *< Self test with 100mV dc
        self.AFE5801_REG_FILTER_SELF_TEST_ENABLE150 = int('0x0180', 0)    # *< Self test with 150mV dc
        self.AFE5801_REG_FILTER_BW_14MHZ = int('0x0000', 0)    # *< 14MHz Filter bandwidth
        self.AFE5801_REG_FILTER_BW_10MHZ = int('0x0004', 0)    # *< 10MHz Filter bandwidth
        self.AFE5801_REG_FILTER_BW_7_5MHZ = int('0x0008', 0)    # *< 7.5MHz Filter bandwidth
        self.AFE5801_REG_FILTER_INTERNAL_DC_COUPLING = int('0x0002', 0)    # *< if used, internal DC coupled, else (=default) AC coupled

        self.AFE5801_REG_FILTER_BW_MASK = int('0x000C', 0)    # *< Bandwidth mask for filter

        self.AFE5801_REG_OFFSET_GAIN_GAIN_BITSHIFT = 11
        self.AFE5801_REG_OFFSET_GAIN_OFFSET_BITSHIFT = 2

        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_0DB = (0*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_2DB = (1*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_4DB = (2*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_6DB = (3*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_0_8DB = (4*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_0DB = (5*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_2DB = (6*2**11)

        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_4DB = (7*2**11)
        self.AFE5801_REG_OFFSET_GAIN_GAIN_1_6DB = (8*2**11)

        self.AFE5801_REG_OFFSET_GAIN_GAIN_6_0DB = (31*2**11)

        self.AFE5801_REG_HIGHPASS_ENABLE = int('0x0001', 0)    # *< enable digital highpass filter
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K2 = int('0x0002', 0)    # *< K=2, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K3 = int('0x0004', 0)    # *< K=3, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K4 = int('0x0006', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K5 = int('0x0008', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K6 = int('0x000A', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K7 = int('0x000C', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K8 = int('0x000E', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K9 = int('0x0010', 0)    # *< K=4, Datasheet-formula on page 22
        self.AFE5801_REG_HIGHPASS_CORNER_FREQ_K10 = int('0x0012', 0)    # *< K=10, Datasheet-formula on page 22

        self.AFE5801_REG_CLAMP_DISABLE = int('0x4000', 0)

        self.AFE5801_REG_TGC_STARTGAIN_INTERP = int('0x0080', 0)    # *< set the TGC step size, datasheet, rev.D, p.28
        self.AFE5801_REG_TGC_GAIN_MODE_SOFT_SYNC = int('0x0020', 0)    # *< TGC running periodically without pulse on SYNC pin
        self.AFE5801_REG_TGC_GAIN_MODE_UNIFORM_GAIN = int('0x0010', 0)    # *< Use a uniform gain, datasheet, rev.D, p.28
        self.AFE5801_REG_TGC_GAIN_MODE_STATIC_PGA = int('0x0008', 0)    # *< Use static gain, only "fine" and "coarse" are active
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_0 = int('0x0000', 0)    # *< Fine gain, "+0dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_1 = int('0x0001', 0)    # *< Fine gain, "+0.125dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_2 = int('0x0002', 0)    # *< Fine gain, "+0.250dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_3 = int('0x0003', 0)    # *< Fine gain, "+0.375dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_4 = int('0x0004', 0)    # *< Fine gain, "+0.500dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_5 = int('0x0005', 0)    # *< Fine gain, "+0.625dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_6 = int('0x0006', 0)    # *< Fine gain, "+0.750dB"
        self.AFE5801_REG_TGC_GAIN_MODE_FINE_GAIN_7 = int('0x0007', 0)    # *< Fine gain, "+0.875dB"

        self.AFE5801_REG_TGC_GAIN_M5DB = int('0x0000', 0)    # *< Coarse gain, "-5dB"
        self.AFE5801_REG_TGC_GAIN_M4DB = int('0x0001', 0)    # *< Coarse gain, "-4dB"
        self.AFE5801_REG_TGC_GAIN_M3DB = int('0x0002', 0)    # *< Coarse gain, "-3dB"
        self.AFE5801_REG_TGC_GAIN_M2DB = int('0x0003', 0)    # *< Coarse gain, "-2dB"
        self.AFE5801_REG_TGC_GAIN_M1DB = int('0x0004', 0)    # *< Coarse gain, "-1dB"
        self.AFE5801_REG_TGC_GAIN_00DB = int('0x0005', 0)    # *< Coarse gain, "0dB"
        self.AFE5801_REG_TGC_GAIN_05DB = int('0x000A', 0)    # *< Coarse gain, "5dB"
        self.AFE5801_REG_TGC_GAIN_10DB = int('0x000F', 0)    # *< Coarse gain, "10dB"
        self.AFE5801_REG_TGC_GAIN_15DB = int('0x0014', 0)    # *< Coarse gain, "15dB"
        self.AFE5801_REG_TGC_GAIN_20DB = int('0x0019', 0)    # *< Coarse gain, "20dB"
        self.AFE5801_REG_TGC_GAIN_25DB = int('0x001E', 0)    # *< Coarse gain, "25dB"
        self.AFE5801_REG_TGC_GAIN_31DB = int('0x0024', 0)    # *< Coarse gain, "31dB"
        #--------------------------------------------------------------------------
        # HMC703: Modes of operation
        #--------------------------------------------------------------------------
        self.cHMC703_REG_SDCFG_MOD_FRAC = 0
        self.cHMC703_REG_SDCFG_MOD_INT = 1
        self.cHMC703_REG_SDCFG_MOD_RAMP_1WAY = 5
        self.cHMC703_REG_SDCFG_MOD_RAMP_2WAY = 6
        self.cHMC703_REG_SDCFG_MOD_RAMP_CONT = 7
        #--------------------------------------------------------------------------
        # Cic1 MMP: Control Register Bit Masks
        #--------------------------------------------------------------------------
        self.cCIC1_REG_CONTROL_EN = int("0x1", 0)       # Enable MMP*/
        self.cCIC1_REG_CONTROL_RST = int("0x2", 0)       # Reset the filter chain*/
        self.cCIC1_REG_CONTROL_RSTCNTRSOP = int("0x100", 0)     # Reset sample cntr on SoP signal*/
        self.cCIC1_REG_CONTROL_RSTFILTSOP = int("0x200", 0)     # Reset filter on SoP signal*/
        self.cCIC1_REG_CONTROL_RSTCNTREOP = int("0x400", 0)     # Reset sample cntr on EoP signal*/
        self.cCIC1_REG_CONTROL_RSTFILTEOP = int("0x800", 0)     # Reset filter on EoP signal*/
        self.cCIC1_REG_CONTROL_BYPASS = int("0x10000", 0)   # Bypass Cic filter */
        #--------------------------------------------------------------------------
        # FrmCtrl MMP: Channel Control Register Bit Masks
        #--------------------------------------------------------------------------
        self.cFRMCTRL_REG_CH0CTRL_TESTENA = int("0x1", 0)       # test mode enable*/
        self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA = int("0x2", 0)       # frame counter enable*/
        self.cFRMCTRL_REG_CH0CTRL_PADENA = int("0x4", 0)       # frame padding enable*/
        self.cFRMCTRL_REG_CH0CTRL_WAITENA = int("0x8", 0)       # wait enable*/
        self.cFRMCTRL_REG_CH0CTRL_GLOBWAITENA = int("0x10", 0)      # global wait signal enable (or of all channels)*/
        self.cFRMCTRL_REG_CH0CTRL_SOFTID = int("0x100", 0)     # frame identification number set by software

        self.cFRMCTRL_REG_CTRL_ENAFIFORST = int("0x8", 0)
        self.cFRMCTRL_REG_CTRL_SWAPBYTES = int("0x100", 0)
        self.cFRMCTRL_REG_CTRL_SELFFT = int("0x10000", 0)
        self.cFRMCTRL_REG_CTRL_ENAFFTAUX = int("0x20000", 0)
        self.cFRMCTRL_REG_CTRL_ENADEBMOD = int("0x40000", 0)


        #--------------------------------------------------------------------------
        # SeqTrig MMP: Configure Timing of Sequence Trigger
        #--------------------------------------------------------------------------
        self.cSEQTRIG_REG_CTRL_RST = int("0x1", 0)       # Rst MMP*/
        self.cSEQTRIG_REG_CTRL_CH0ENA = int("0x10", 0)      # Enable output channel 0*/
        self.cSEQTRIG_REG_CTRL_CH1ENA = int("0x20", 0)      # Enable output channel 1*/
        self.cSEQTRIG_REG_CTRL_CH2ENA = int("0x40", 0)      # Enable output channel 2*/
        self.cSEQTRIG_REG_CTRL_CH3ENA = int("0x80", 0)      # Enable output channel 3*/
        self.cSEQTRIG_REG_CTRL_IRQ0ENA = int("0x100", 0)     # Enable channel 0 sys irq*/
        self.cSEQTRIG_REG_CTRL_IRQ1ENA = int("0x200", 0)     # Enable channel 1 sys irq*/
        self.cSEQTRIG_REG_CTRL_IRQ2ENA = int("0x400", 0)     # Enable channel 2 sys irq*/
        self.cSEQTRIG_REG_CTRL_IRQ3ENA = int("0x800", 0)     # Enable channel 3 sys irq*/
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_ENA = int("0x1", 0)           # sequence counter enable */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN = int("0x2", 0)           # sequence counter syncn enable */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE = int("0x4", 0)           # sequence counter external trigger for sequence update */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_ADRSELENA = int("0x8", 0)           # sequence address select enable */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_ADRSELMOD = int("0x10", 0)          # address update mode (1 single shoot, 0 continuous) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_RELOAD = int("0x20", 0)          # Reload counter */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA = int("0x100", 0)         # interval enable for chn0 (Chn0Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA = int("0x200", 0)         # interval enable for chn0 (Chn1Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN2_INTVALENA = int("0x400", 0)         # interval enable for chn0 (Chn2Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA = int("0x800", 0)         # interval enable for chn0 (Chn3Trig = 1 for Tim0 < t <= Tim1) */
        self.cSEQTRIG_MEM_SEQ_CHNTIM_ENA = int("0x80000000", 0)    # channel tim enable */
        #--------------------------------------------------------------------------
        # FUsb Configuration
        #--------------------------------------------------------------------------
        self.Rad_FUsbCfg_Mask = int(1)
        self.Rad_FUsbCfg_ChnEna = int(255)
        self.Rad_FUsbCfg_DatSiz = int(1024)
        self.Rad_FUsbCfg_WaitLev = int(1024 + 512)
        #--------------------------------------------------------------------------
        # Constants for power supply
        #--------------------------------------------------------------------------
        self.cPWR_RF_P1_ON = 1
        self.cPWR_RF_P2_ON = 2
        self.cPWR_RF_P3_ON = 4
        self.cPWR_RF_ON = 7
        self.cPWR_RF_OFF = 0

        self.cRAD_ROLE_MS = 0
        self.cRAD_ROLE_MSCLKOUT = 1
        self.cRAD_ROLE_SL = 128
        self.RAD_ROLE_SLCLKIN = 129
