# SeqTrig.py -- SeqTrig class  
#
# Copyright (C) 2015-11 Inras GmbH Haderer Andreas
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE file for details.

from numpy import *

class SeqTrig():
    
    def __init__(self, freq):

        # Define Constants
        self.DefineConst()


        self.SeqTrigCfg_Mask            =   1
        self.SeqTrigCfg_Ctrl            =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        self.SeqTrigCfg_ChnEna          =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        self.SeqTrigCfg_Seq             =   list()
        self.fSeqTrig                   =   freq
        self.Mask                       =   1       
    
        
    def ContUnifM1(self, Tp):
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]              =   self.Mask
        SeqTrigCfg["Ctrl"]              =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]            =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                             = list()         
        # Phase0: Init Sequence 
        # Trigger dummy frame on ADC
        caSeq   =   self.IniSeq('IniExt', 1e-3, 'IniExt')
        # Phase1 Meas: 
        # Trigger Adc and External PLL
        # Meas: Tp, NxtAdr, Id, Name; Tri
        caSeq   =   self.AddSeq(caSeq, 'RccMeas', Tp, 1, 0, 'Meas')

        SeqTrigCfg["Seq"] = caSeq

        return SeqTrigCfg

    def ContUnifM1PWPT(self, Tp, ExtEve, Trig):
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]              =   self.Mask
        SeqTrigCfg["Ctrl"]              =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]            =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                             =   list()         
        # Phase0: Init Sequence 
        # Trigger dummy frame on ADC
        caSeq   =   self.IniSeq('IniExt', 10e-3, 'IniExt')
        # Phase1 Meas: 
        # Trigger Adc and External PLL
        # Meas: Tp, NxtAdr, Id, Name; Tri
        if Trig > 0:
            if ExtEve > 0:
                caSeq   =   self.AddSeq(caSeq, 'RccMeasWait', Tp, 1, 0, 'MeasWait')
            else:
                caSeq   =   self.AddSeq(caSeq, 'RccMeas', Tp, 1, 0, 'Meas')
        else:
            if ExtEve > 0:
                caSeq   =   self.AddSeq(caSeq, 'MeasWait', Tp, 1, 0, 'MeasWait')
            else:
                caSeq   =   self.AddSeq(caSeq, 'Meas', Tp, 1, 0, 'Meas')
        SeqTrigCfg["Seq"] = caSeq

        return SeqTrigCfg
         
    def RccSeqContUnifM1(self, Tp, TCfg):
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]      =   self.Mask
        SeqTrigCfg["Ctrl"]      =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]    =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                     =   list()

        TMeas = Tp - TCfg
        

        # Phase0: Init Sequence 
        caSeq               =   self.IniSeq('IniExt', 1e-3, 'IniExt')
        # RCC configuration phase: Trigger int
        caSeq               =   self.AddSeq(caSeq, 'RccCfg', TCfg, 2, 0, "Cfg")
        # RCC measurement phase: jump back to configuration
        caSeq               =   self.AddSeq(caSeq, 'RccMeas', TMeas, 1, 0, "Meas")

        SeqTrigCfg["Seq"]   = caSeq
       
        return SeqTrigCfg

    def RccSeqContUnifM1PW(self, Tp, TCfg, ExtEve):
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]      =   self.Mask
        SeqTrigCfg["Ctrl"]      =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]    =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                     =   list()

        TMeas = Tp - TCfg
        
        # Phase0: Init Sequence 
        caSeq               =   self.IniSeq('IniExt', 1e-3, 'IniExt')
        # RCC configuration phase: Trigger int
        caSeq               =   self.AddSeq(caSeq, 'RccCfg', TCfg, 2, 0, "Cfg")
        if ExtEve > 0:
            # RCC measurement phase: jump back to configuration
            caSeq               =   self.AddSeq(caSeq, 'RccMeasWait', TMeas, 1, 0, "Meas")        
        else:
            # RCC measurement phase: jump back to configuration
            caSeq               =   self.AddSeq(caSeq, 'RccMeas', TMeas, 1, 0, "Meas")

        SeqTrigCfg["Seq"]   = caSeq
       
        return SeqTrigCfg

    def RccSeqContUnifMx(self, Tp, TCfg, TWait, Np):
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]      =   self.Mask
        SeqTrigCfg["Ctrl"]      =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]    =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                     =   list()

        TMeas = Tp - TCfg
        
        # Phase0: Init Sequence 
        caSeq               =   self.IniSeq('IniExt', 1e-3, 'IniExt')
        # RCC configuration phase: Trigger int
        caSeq               =   self.AddSeq(caSeq, 'RccCfg', TCfg, 2, 0, "Cfg")
        # RCC measurement phase: 
        # Arguments: Tp, LoopAdr, ExitAdr, NLoop, Id
        caSeq               =   self.AddSeq(caSeq, 'RccMeasN', TMeas, 1, 3, Np, 0, "MeasLoop")
        # RCC measurement phase: jump back to configuration
        # Arguments: TWait, NxtAdr, Id
        print("TWait", TWait)
        caSeq               =   self.AddSeq(caSeq, 'Wait', TWait, 1,  0, "Wait")

        SeqTrigCfg["Seq"]   = caSeq
       
        return SeqTrigCfg

    def RccSeqContUnifMxPW(self, Tp, TCfg, TWait, Np, ExtEve):
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]      =   self.Mask
        SeqTrigCfg["Ctrl"]      =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]    =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                     =   list()

        TMeas = Tp - TCfg
        
        # Phase0: Init Sequence 
        caSeq               =   self.IniSeq('IniExt', 1e-3, 'IniExt')
        # RCC configuration phase: Trigger int
        caSeq               =   self.AddSeq(caSeq, 'RccCfg', TCfg, 2, 0, "Cfg")
        # RCC measurement phase: 
        # Arguments: Tp, LoopAdr, ExitAdr, NLoop, Id
        caSeq               =   self.AddSeq(caSeq, 'RccMeasN', TMeas, 1, 3, Np, 0, "MeasLoop")
        # RCC measurement phase: jump back to configuration
        # Arguments: TWait, NxtAdr, Id
        if ExtEve > 0:
            caSeq               =   self.AddSeq(caSeq, 'WaitEve', TWait, 1,  0, "Wait")
        else:
            caSeq               =   self.AddSeq(caSeq, 'Wait', TWait, 1,  0, "Wait")

        SeqTrigCfg["Seq"]   = caSeq
       
        return SeqTrigCfg

    def     RccSeqContUnifM1_RccMs(self, Tp, TCfg):

        print('RccSeqContUnifM1_RccMs')
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = {}
        SeqTrigCfg["Mask"]      =   self.Mask
        SeqTrigCfg["Ctrl"]      =   self.SEQTRIG_REG_CTRL_IRQ2ENA
        SeqTrigCfg["ChnEna"]    =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        Seq                     =   list()

        TMeas                   =   Tp - TCfg

        # Chn0 is connected to Croc
        # Chn1 is connected to AD8283
        # Chn2 is used as internal interrupt event to signal sequencer
        # Chn3 is unused
        # Adr Sel input is connected to ground: can not be used            

        ########### Phase0: Init Sequence 
        #   Description: Synchronize SeqTrigger to ADC sync signal and switch to sequence 1
        caSeq               =   self.IniSeq('IniExt', 1e-3, 'IniExt')

        ########### Phase0: Cfg Phase -> Execute processing call
        #   Description: Synchronize SeqTrigger to ADC sync signal and switch to sequence 1
        caSeq               =   self.AddSeq(caSeq, 'RccCfg', TCfg, 2, 0, "Cfg")
        dSeq                =   caSeq[1]
        dSeq["Chn3Cfg"]     =   2**31

        ########### Phase1 Meas: Measurement sequence with configuration for RCC
        #   Description: Trigger RCC and ADC for sampling
        #       Enable intervall function for Chn0 and Chn1 -> ensure timing constraints
        #       Output 0 on FPIO[3..1]  
        caSeq               =   self.AddSeq(caSeq, 'RccMeas', TMeas, 1, 0, 'Meas')
        dSeq                =   caSeq[2]
        dSeq["Chn3Cfg"]     =   2**31


        SeqTrigCfg["Seq"]   =   caSeq;

        return SeqTrigCfg

    def RccSeqContUnifMTxPaCon(self, Tp, TUp, TCfg, TxChn):

        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg                  =   {}
        SeqTrigCfg["Mask"]          =   self.Mask 
        SeqTrigCfg["Ctrl"]          =   self.SEQTRIG_REG_CTRL_IRQ2ENA                  # Enable interrupt event on channel 2 
        SeqTrigCfg["ChnEna"]        =   self.SEQTRIG_REG_CTRL_CH0ENA + self.SEQTRIG_REG_CTRL_CH1ENA
        
        PaConTxOff                  =   0
        PaConTx1On                  =   1
        PaConTx2On                  =   2
        PaConTx3On                  =   4
        PaConTx4On                  =   8
        
        TxChn                       =   TxChn.flatten()
        NTx                         =   len(TxChn)
        
        TWait                       =   Tp - (TUp + TCfg)*NTx
        if TWait < 10e-6:
            TWait   =   10e-6
            print("RccSeqContUnifMTxPaCon: TWait changed to ", TWait)

        caSeq                       =   self.IniSeq('IniExt', 1e-3, "Ini")

        Adr                         =   2
        for Idx in range(0, NTx):
            if TxChn[Idx] == 0:
                PaCon   =   PaConTxOff 
            elif TxChn[Idx] == 1:
                PaCon   =   PaConTx1On 
            elif TxChn[Idx] == 2:
                PaCon   =   PaConTx2On  
            elif TxChn[Idx] == 3:
                PaCon   =   PaConTx3On  
            elif TxChn[Idx] == 4:
                PaCon   =   PaConTx4On  
            else:
                PaCon   =   PaConTxOff


            caSeq           =   self.AddSeq(caSeq, 'RccCfgPaCon', TCfg, Adr, TxChn[Idx], PaCon, ("Cfg" + str(int(Idx))))
            Adr             =   Adr + 1
            caSeq           =   self.AddSeq(caSeq, 'RccMeasPaCon', TUp, Adr, TxChn[Idx], PaCon, ("Meas" + str(int(Idx))))
            Adr             =   Adr + 1

        caSeq               =   self.AddSeq(caSeq, 'Wait', TWait, 1, 0, "Wait");

        SeqTrigCfg["Seq"]   =   caSeq;

        return SeqTrigCfg
    
    def IniSeq(self, stSeq, Tp, *varargin):
        Seq     =   list()
        dSeq    =   {}
        Add     =   0

        if stSeq == 'IniExt':
            if (len(varargin)) > 0:
                dSeq["Name"]     =   varargin[0]
            dSeq["CntrCtrl"]    =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE
            dSeq["CntrPerd"]    =   round(self.fSeqTrig*Tp)                     # dummy delay
            dSeq["NextAdr"]     =   1                                           # Switch to sequence 1 Adr start with zero
            dSeq["SeqId"]       =   0                                           # Sequence ID: not used during init                               
            dSeq["Chn0Tim0"]    =   0                                           # disable event
            dSeq["Chn0Tim1"]    =   0                                           # disable event
            dSeq["Chn0Cfg"]     =   0                                           # disable event
            dSeq["Chn1Tim0"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0         # generate dummy frame -> used to release fifo from reset
            dSeq["Chn1Tim1"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3         # disable event
            dSeq["Chn1Cfg"]     =   0                                           # disable event
            dSeq["Chn2Tim0"]    =   0                                           # disable event
            dSeq["Chn2Tim1"]    =   0                                           # disable event
            dSeq["Chn2Cfg"]     =   0                                           # disable event
            dSeq["Chn3Tim0"]    =   0                                           # disable event
            dSeq["Chn3Tim1"]    =   0                                           # disable event
            dSeq["Chn3Cfg"]     =   0                                           # disable event
            Add                 =   1
        elif stSeq == 'Ini':
            if (len(varargin)) > 0:
                dSeq["Name"]     =   varargin[0]            
            dSeq["CntrCtrl"]    =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA
            dSeq["CntrPerd"]    =   round(self.fSeqTrig*Tp)                   # dummy delay
            dSeq["NextAdr"]     =   1                                         # Switch to sequence 1 Adr start with zero
            dSeq["SeqId"]       =   0                                         # Sequence ID: not used during init                               
            dSeq["Chn0Tim0"]    =   0                                         # disable event
            dSeq["Chn0Tim1"]    =   0                                         # disable event
            dSeq["Chn0Cfg"]     =   0                                         # disable event
            dSeq["Chn1Tim0"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0       # generate dummy frame -> used to release fifo from reset
            dSeq["Chn1Tim1"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3       # disable event
            dSeq["Chn1Cfg"]     =   0                                         # disable event
            dSeq["Chn2Tim0"]    =   0                                         # disable event
            dSeq["Chn2Tim1"]    =   0                                         # disable event
            dSeq["Chn2Cfg"]     =   0                                         # disable event
            dSeq["Chn3Tim0"]    =   0                                         # disable event
            dSeq["Chn3Tim1"]    =   0                                         # disable event
            dSeq["Chn3Cfg"]     =   0                                         # disable event
            Add                 =   1
        elif stSeq == 'IniNoFrm':
            if (len(varargin)) > 0:
                dSeq["Name"]     =   varargin[0]            
            dSeq["CntrCtrl"]    =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN
            dSeq["CntrPerd"]    =   round(self.fSeqTrig*Tp)                   # dummy delay
            dSeq["NextAdr"]     =   1                                         # Switch to sequence 1 Adr start with zero
            dSeq["SeqId"]       =   0                                         # Sequence ID: not used during init                               
            dSeq["Chn0Tim0"]    =   0                                         # disable event
            dSeq["Chn0Tim1"]    =   0                                         # disable event
            dSeq["Chn0Cfg"]     =   0                                         # disable event
            dSeq["Chn1Tim0"]    =   0                                         # generate dummy frame -> used to release fifo from reset
            dSeq["Chn1Tim1"]    =   0                                         # disable event
            dSeq["Chn1Cfg"]     =   0                                         # disable event
            dSeq["Chn2Tim0"]    =   0                                         # disable event
            dSeq["Chn2Tim1"]    =   0                                         # disable event
            dSeq["Chn2Cfg"]     =   0                                         # disable event
            dSeq["Chn3Tim0"]    =   0                                         # disable event
            dSeq["Chn3Tim1"]    =   0                                         # disable event
            dSeq["Chn3Cfg"]     =   0                                         # disable event
            Add                 =   1
        elif stSeq == 'IniExtTrig':
            if (len(varargin)) > 0:
                dSeq["Name"]     =   varargin[0]
            dSeq["CntrCtrl"]    =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE
            dSeq["CntrPerd"]    =   round(self.fSeqTrig*Tp)                     # dummy delay
            dSeq["NextAdr"]     =   1                                           # Switch to sequence 1 Adr start with zero
            dSeq["SeqId"]       =   0                                           # Sequence ID: not used during init                               
            dSeq["Chn0Tim0"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0         # disable event
            dSeq["Chn0Tim1"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 10        # disable event
            dSeq["Chn0Cfg"]     =   0                                           # disable event
            dSeq["Chn1Tim0"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0         # generate dummy frame -> used to release fifo from reset
            dSeq["Chn1Tim1"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3         # disable event
            dSeq["Chn1Cfg"]     =   0                                           # disable event
            dSeq["Chn2Tim0"]    =   0                                           # disable event
            dSeq["Chn2Tim1"]    =   0                                           # disable event
            dSeq["Chn2Cfg"]     =   0                                           # disable event
            dSeq["Chn3Tim0"]    =   0                                           # disable event
            dSeq["Chn3Tim1"]    =   0                                           # disable event
            dSeq["Chn3Cfg"]     =   0                                           # disable event
            Add                 =   1
        elif stSeq == 'IniTrig':
            if (len(varargin)) > 0:
                dSeq["Name"]     =   varargin[0]            
            dSeq["CntrCtrl"]    =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA
            dSeq["CntrPerd"]    =   round(self.fSeqTrig*Tp)                   # dummy delay
            dSeq["NextAdr"]     =   1                                         # Switch to sequence 1 Adr start with zero
            dSeq["SeqId"]       =   0                                         # Sequence ID: not used during init                               
            dSeq["Chn0Tim0"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0       # disable event
            dSeq["Chn0Tim1"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 10      # disable event
            dSeq["Chn0Cfg"]     =   0                                         # disable event
            dSeq["Chn1Tim0"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0       # generate dummy frame -> used to release fifo from reset
            dSeq["Chn1Tim1"]    =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3       # disable event
            dSeq["Chn1Cfg"]     =   0                                         # disable event
            dSeq["Chn2Tim0"]    =   0                                         # disable event
            dSeq["Chn2Tim1"]    =   0                                         # disable event
            dSeq["Chn2Cfg"]     =   0                                         # disable event
            dSeq["Chn3Tim0"]    =   0                                         # disable event
            dSeq["Chn3Tim1"]    =   0                                         # disable event
            dSeq["Chn3Cfg"]     =   0                                         # disable event
            Add                 =   1
        else:
            raise NotImplementedError("Sorry, this is not defined!!!")
       
        if Add > 0:
            Seq.append(dSeq)

        return Seq

    def AddSeq(self, caTrig, stSeq, *varargin):
        N       =   len(caTrig)
        Add     =   0
        Seq     =   {}
        if stSeq == 'SigCfg':
            if len(varargin) > 3:
                dCfg                =   varargin[0]
                Tp                  =   varargin[1]
                NxtAdr              =   varargin[2]
                Id                  =   varargin[3]
                

                SeqCntr["Ctrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                       
                Seq["SeqId"]        =   Id;                                              

                if 'Chn0Int' in dCfg:
                    Seq["CntrCtrl"]     =   Seq["CntrCtrl"]  + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA 
                    Seq["Chn0Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn0Int"][0]
                    Seq["Chn0Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn0Int"][1]
                else: 
                    Seq["Chn0Tim0"]     =   0
                    Seq["Chn0Tim1"]     =   0                                  
                    
                if 'Chn1Int' in dCfg:
                    Seq["CntrCtrl"]     =   Seq["CntrCtrl"]  + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA 
                    Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn1Int"][0]
                    Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn1Int"][1]  
                else:
                    Seq["Chn1Tim0"]     =   0
                    Seq["Chn1Tim1"]     =   0                                                        

                if 'Chn2Int' in dCfg:
                    Seq["CntrCtrl"]     =   Seq["CntrCtrl"]  + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN2_INTVALENA
                    Seq["Chn2Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn2Int"][0]
                    Seq["Chn2Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn2Int"][1]   
                else: 
                    Seq["Chn2Tim0"]     =   0
                    Seq["Chn2Tim1"]     =   0                                                  
                if 'Chn3Int' in dCfg:
                    Seq["CntrCtrl"]     =   Seq["CntrCtrl"]  + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA
                    Seq["Chn3Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn3Int"][0]
                    Seq["Chn3Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA  + dCfg["Chn3Int"][1] 
                else: 
                    Seq["Chn3Tim0"]     =   0
                    Seq["Chn3Tim1"]     =   0                                                                                      
                if 'N' in dCfg:
                    Seq["CntrCtrl"]     =   Seq["CntrCtrl"]     + self.SEQTRIG_MEM_SEQ_CNTRCTRL_RELOAD 
                    if 'NAdr' in dCfg:
                        Seq["NextAdr"]  =   NxtAdr  + 2**8*NAdr                                
                    else:
                        Seq["NextAdr"]  =   NxtAdr + 2**8*N                 # Switch to sequence 1 Adr start with zero                        
                else:
                    Seq["NextAdr"]      =   NxtAdr                          # Switch to sequence 1 Adr start with zero
                        
                if 'ExtEve' in dCfg:
                    Seq.CntrCtrl        =   Seq.CntrCtrl  + self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE; 
                if 'Chn0Cfg' in dCfg:
                    Seq["Chn0Cfg"]      =   dCfg["Chn0Cfg"]
                else:
                    Seq["Chn0Cfg"]      =   0
                if 'Chn1Cfg' in dCfg:
                    Seq["Chn1Cfg"]      =   dCfg["Chn1Cfg"]
                else:
                    Seq["Chn1Cfg"]      =   0

                if 'Chn2Cfg' in dCfg:
                    Seq["Chn2Cfg"]      =   dCfg["Chn2Cfg"]
                else:
                    Seq["Chn2Cfg"]      =   0

                if 'Chn3Cfg' in dCfg:
                    Seq["Chn3Cfg"]      =   dCfg["Chn3Cfg"]
                else:
                    Seq["Chn3Cfg"]      =   0
                Add     =   1;

        elif stSeq == 'RccCfgPaCon':
            if len(varargin) > 3:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]
                PaCtrl              =   varargin[3]
                if (len(varargin)) > 4:
                    Seq["Name"]     =   varargin[4]

                Seq["CntrCtrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   PaCtrl                                          # disable event
                Seq["Chn1Tim0"]     =   0                                               # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   0                                               # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn3Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 100           # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1                   
        elif stSeq == 'RccCfg':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]

                Seq["CntrCtrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   0                                               # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   0                                               # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn3Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 100          # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1
        elif stSeq == 'SyncnChn3':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]

                Seq["CntrCtrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   0                                               # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   0                                               # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn3Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 100          # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1                
        elif stSeq == 'RccMeas':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]

                Seq["CntrCtrl"]     =   (   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_HOLDPNT + self.SEQTRIG_MEM_SEQ_CNTRCTRL_INCCNTR)      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn0Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 10            # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1
        elif stSeq == 'RccMeasWait':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]

                Seq["CntrCtrl"]     =   (   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE + self.SEQTRIG_MEM_SEQ_CNTRCTRL_HOLDPNT +
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_INCCNTR)      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn0Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 10            # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1                
        elif stSeq == 'RccMeasN':
            # Arguments: Tp, LoopAdr, ExitAdr, NLoop, Id
            if len(varargin) > 4:
                Tp                  =   varargin[0]
                LoopAdr             =   varargin[1]
                ExitAdr             =   varargin[2]
                NLoop               =   varargin[3]   
                Id                  =   varargin[4]

                Seq["CntrCtrl"]     =   ((NLoop-1)*2**16 + (NLoop-1)*2**24 + 
                                        self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + 
                                        self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + 
                                        self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + 
                                        self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA +
                                        self.SEQTRIG_MEM_SEQ_CNTRCTRL_RELOAD
                                        )      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   LoopAdr*2**8 + ExitAdr                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn0Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 11            # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1

        elif stSeq == 'Meas':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]

                Seq["CntrCtrl"]     =   (   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA  + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_HOLDPNT + self.SEQTRIG_MEM_SEQ_CNTRCTRL_INCCNTR
                                        )   
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1
        
        elif stSeq == 'MeasWait':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]

                Seq["CntrCtrl"]     =   (   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE + self.SEQTRIG_MEM_SEQ_CNTRCTRL_HOLDPNT + 
                                            self.SEQTRIG_MEM_SEQ_CNTRCTRL_INCCNTR
                                        )    
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1
                             
        elif stSeq == 'RccMeasPaCon':
            if len(varargin) > 3:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]
                PaCtrl              =   varargin[3]
                if (len(varargin)) > 4:
                    Seq["Name"]     =   varargin[4]

                Seq["CntrCtrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA      
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn0Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 11            # disable event
                Seq["Chn0Cfg"]      =   PaCtrl                                          # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1    
        elif stSeq == 'RccMeasPaConN':
            if len(varargin) > 3:
                Tp                  =   varargin[0]
                LoopAdr             =   varargin[1]
                ExitAdr             =   varargin[2]
                NLoop               =   varargin[3]
                Id                  =   varargin[4]
                PaCtrl              =   varargin[5]


                Seq["CntrCtrl"]     =   (NLoop-1)*2**16 + (NLoop-1)*2**24 + self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA  + self.SEQTRIG_MEM_SEQ_CNTRCTRL_RELOAD     
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   LoopAdr*2**8 + ExitAdr                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # disable event
                Seq["Chn0Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 11            # disable event
                Seq["Chn0Cfg"]      =   PaCtrl                                          # disable event
                Seq["Chn1Tim0"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 0             # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   self.SEQTRIG_MEM_SEQ_CHNTIM_ENA + 3             # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1                                 
        elif stSeq == 'Wait':
            if len(varargin) > 2:
                # Arguments: TWait, NxtAdr, Id
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]
                if (len(varargin)) > 3:
                    Seq["Name"]     =   varargin[3]
                Seq["CntrCtrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   0                                               # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   0                                               # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1    
        elif stSeq == 'WaitEve':
            if len(varargin) > 2:
                Tp                  =   varargin[0]
                NxtAdr              =   varargin[1]
                Id                  =   varargin[2]
                if (len(varargin)) > 3:
                    Seq["Name"]     =   varargin[3]
                Seq["CntrCtrl"]     =   self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA + self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN + self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE + self.SEQTRIG_MEM_SEQ_CNTRCTRL_INCCNTR     
                Seq["CntrPerd"]     =   round(self.fSeqTrig*Tp)                         # dummy delay   
                Seq["NextAdr"]      =   NxtAdr                                          # Switch to sequence 1 Adr start with zero
                Seq["SeqId"]        =   Id                                              # Sequence ID: not used during init                               
                Seq["Chn0Tim0"]     =   0                                               # disable event
                Seq["Chn0Tim1"]     =   0                                               # disable event
                Seq["Chn0Cfg"]      =   0                                               # disable event
                Seq["Chn1Tim0"]     =   0                                               # generate dummy frame -> used to release fifo from reset
                Seq["Chn1Tim1"]     =   0                                               # disable event
                Seq["Chn1Cfg"]      =   0                                               # disable event
                Seq["Chn2Tim0"]     =   0                                               # generate interrupt
                Seq["Chn2Tim1"]     =   0                                               # disable event
                Seq["Chn2Cfg"]      =   0                                               # disable event
                Seq["Chn3Tim0"]     =   0                                               # disable event
                Seq["Chn3Tim1"]     =   0                                               # disable event
                Seq["Chn3Cfg"]      =   0                                               # disable event
                Add                 =   1    
              
        else:
            pass

        if Add > 0:
            caTrig.append(Seq)     

        return caTrig

    def Prnt(self, dSeq):
        if "Seq" in dSeq:
            lCfg        =       dSeq["Seq"]
            print("Number of Elements", len(lCfg))

            for Idx in range(0, len(lCfg)):
                dCfg    =   lCfg[Idx]

                print(Idx)
                print(" ")
                if "Name" in dCfg:
                    print("Name:      ", dCfg["Name"])
                print("CntrCtrl:  ", dCfg["CntrCtrl"])
                print("CntrPerd:  ", dCfg["CntrPerd"])
                print("NextAdr:   ", dCfg["NextAdr"])
                print("Chn0Tim0:  ", dCfg["Chn0Tim0"], " Tim: ", (dCfg["Chn0Tim0"] % 2**31))
                print("Chn0Tim1:  ", dCfg["Chn0Tim1"], " Tim: ", (dCfg["Chn0Tim1"] % 2**31))
                print("Chn0Cfg:   ", dCfg["Chn0Cfg"])
                print("Chn1Tim0:  ", dCfg["Chn1Tim0"], " Tim: ", (dCfg["Chn1Tim0"] % 2**31))
                print("Chn1Tim1:  ", dCfg["Chn1Tim1"], " Tim: ", (dCfg["Chn1Tim1"] % 2**31))
                print("Chn1Cfg:   ", dCfg["Chn1Cfg"])    
                print("Chn2Tim0:  ", dCfg["Chn2Tim0"], " Tim: ", (dCfg["Chn2Tim0"] % 2**31))
                print("Chn2Tim1:  ", dCfg["Chn2Tim1"], " Tim: ", (dCfg["Chn2Tim1"] % 2**31))
                print("Chn2Cfg:   ", dCfg["Chn2Cfg"])        
                print(" ")
                print(" ")  
        else:
            print("No Seq entry found")      

    def DefineConst(self):

        #--------------------------------------------------------------------------
        # SeqTrig MMP: Configure Timing of Sequence Trigger
        #--------------------------------------------------------------------------
        self.SEQTRIG_REG_CTRL_RST                       =   int('0x1', 0)         #   /**< @brief Rst MMP*/
        self.SEQTRIG_REG_CTRL_CH0ENA                    =   int('0x10', 0)        #   /**< @brief Enable output channel 0*/
        self.SEQTRIG_REG_CTRL_CH1ENA                    =   int('0x20', 0)        #   /**< @brief Enable output channel 1*/
        self.SEQTRIG_REG_CTRL_CH2ENA                    =   int('0x40', 0)        #   /**< @brief Enable output channel 2*/
        self.SEQTRIG_REG_CTRL_CH3ENA                    =   int('0x80', 0)        #   /**< @brief Enable output channel 3*/
        self.SEQTRIG_REG_CTRL_IRQ0ENA                   =   int('0x100', 0)       #   /**< @brief Enable channel 0 sys irq*/
        self.SEQTRIG_REG_CTRL_IRQ1ENA                   =   int('0x200', 0)       #   /**< @brief Enable channel 1 sys irq*/
        self.SEQTRIG_REG_CTRL_IRQ2ENA                   =   int('0x400', 0)       #   /**< @brief Enable channel 2 sys irq*/
        self.SEQTRIG_REG_CTRL_IRQ3ENA                   =   int('0x800', 0)       #   /**< @brief Enable channel 3 sys irq*/

        self.SEQTRIG_MEM_SEQ_CNTRCTRL_ENA               =   int('0x1', 0)         #   /**< @brief sequence counter enable */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_SYNCN             =   int('0x2', 0)         #   /**< @brief sequence counter syncn enable */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_EXTEVE            =   int('0x4', 0)         #   /**< @brief sequence counter external trigger for sequence update */

        self.SEQTRIG_MEM_SEQ_CNTRCTRL_ADRSELENA         =   int('0x8', 0)         #  /**< @brief sequence address select enable */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_ADRSELMOD         =   int('0x10', 0)        #  /**< @brief address update mode (1 single shoot, 0 continuous) */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_RELOAD            =   int('0x20', 0)        #  /**< @brief Reload counter */

        self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN0_INTVALENA    =   int('0x100', 0)       #  /**< @brief interval enable for chn0 (Chn0Trig = 1 for Tim0 < t <= Tim1) */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN1_INTVALENA    =   int('0x200', 0)       #  /**< @brief interval enable for chn0 (Chn1Trig = 1 for Tim0 < t <= Tim1) */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN2_INTVALENA    =   int('0x400', 0)       #  /**< @brief interval enable for chn0 (Chn2Trig = 1 for Tim0 < t <= Tim1) */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_CHN3_INTVALENA    =   int('0x800', 0)       #  /**< @brief interval enable for chn0 (Chn3Trig = 1 for Tim0 < t <= Tim1) */

        self.SEQTRIG_MEM_SEQ_CNTRCTRL_HOLDPNT           =   int('0x1000', 0);        #   /**< @brief Wait flag */
        self.SEQTRIG_MEM_SEQ_CNTRCTRL_INCCNTR           =   int('0x2000', 0);        #   /**< @brief increment counter */
        

        self.SEQTRIG_MEM_SEQ_CHNTIM_ENA                 =   int('0x80000000', 0)  #  /**< @brief channel tim enable */
