# ADF24Tx2RX8 -- Class for 24-GHz Radar 
#
# Copyright (C) 2015-11 Inras GmbH Haderer Andreas
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE file for details.
#import  src.cmd_modules.RadarLog   as RadarLog

import  src.cmd_modules.Rbk2 as Rbk2 
import  src.cmd_modules.SeqTrig     as SeqTrig 
import  src.cmd_modules.DevAdf5904  as DevAdf5904
import  src.cmd_modules.DevAdf5901  as DevAdf5901
import  src.cmd_modules.DevAdf4159  as DevAdf4159
import time
from    numpy import *
import weakref
import traceback


# Version 1.0.0
#   Set number of frames correctly
#   Correct print in RfGetVers

# Version 1.0.1
#   Constructor with Port-Number

# Version 1.1.0
#   - Add Master Slave Support

class Rbk2Adf24Tx2Rx8(Rbk2.Rbk2):

    def __init__(self, stConType, *args):
        super(Rbk2Adf24Tx2Rx8, self).__init__(stConType, *args)
    
        #>  Object of first receiver (DevAdf5904 class self.ct)
        self.Adf_Rx1 = [];
        #>  Object of second receiver (DevAdf5904 clas self.ct)
        self.Adf_Rx2 = [];
        #>  Object of transmitter (DevAdf5901 class self.ct)
        self.Adf_Tx = [];
        #>  Object of transmitter Pll (DevAdf4159 class self.ct)
        self.Adf_Pll = [];
        
        self.Rf_USpiCfg_Mask = 1;
        self.Rf_USpiCfg_Pll_Chn = 1;
        self.Rf_USpiCfg_Tx_Chn = 0;
        self.Rf_USpiCfg_Rx1_Chn = 2;
        self.Rf_USpiCfg_Rx2_Chn = 3;

        self.Rf_Adf5904_FreqStrt = 24.125e9;
        self.Rf_Adf5904_RefDiv = 1;
        self.Rf_Adf5904_SysClk = 80e6;

        self.Rf_fStrt = 24e9;
        self.Rf_fStop = 24.2e9;
        self.Rf_TRampUp = 256e-6;
        self.Rf_TRampDo = 256e-6;
        
        self.Rf_VcoDiv = 2;

        self.stRfVers = '1.1.0';

        self.eqTrig = SeqTrig.SeqTrig(80e6);
        self.SeqCfg = self.eqTrig.ContUnifM1(1e-3);

        # Initialize Receiver
        dUSpiCfg = dict()
        dUSpiCfg = {   "Mask"          : self.Rf_USpiCfg_Mask,
                                                "Chn"           : self.Rf_USpiCfg_Rx1_Chn
                                            }
        self.Adf_Rx1 = DevAdf5904.DevAdf5904(weakref.ref(self), dUSpiCfg)
        
        dUSpiCfg = dict()
        dUSpiCfg = {   "Mask"          : self.Rf_USpiCfg_Mask,
                                                "Chn"           : self.Rf_USpiCfg_Rx2_Chn
                                            }
        self.Adf_Rx2 = DevAdf5904.DevAdf5904(weakref.ref(self), dUSpiCfg)
        dUSpiCfg = dict()
        dUSpiCfg = {   "Mask"          : self.Rf_USpiCfg_Mask,
                                                "Chn"           : self.Rf_USpiCfg_Tx_Chn
                                            }
        
        self.Adf_Tx = DevAdf5901.DevAdf5901(weakref.ref(self), dUSpiCfg);
        
        dUSpiCfg = dict()
        dUSpiCfg = {   "Mask"          : self.Rf_USpiCfg_Mask,
                                                "Chn"           : self.Rf_USpiCfg_Pll_Chn
                                            }
        self.Adf_Pll = DevAdf4159.DevAdf4159(weakref.ref(self), dUSpiCfg) # add weakref of self, to allow garbage collecting
        
        self.Rad_FUsbCfg_WaitLev = 2**16 - 4096;

        self.Computation.Enable();
        self.Set('NrChn', 8)

    # DOXYGEN ------------------------------------------------------
    #> @brief Get Version information of Adf24Tx2Rx8 class
    #>
    #> Get version of class 
    #>      - Version string is returned as string
    #>
    #> @return  Returns the version string of the class (e.g. 0.5.0)           
    def     RfGetVers(self):            
        return self.stRfVers

    def     RfGetChipSts(self):
        lChip = list()
        Val = self.Fpga_GetRfChipSts(1)
        print(Val)
        if Val[0] == True:
            Val = Val[1]
            if len(Val) > 2:
                if Val[0] == 202:
                    lChip.append(('Adf4159 PLL', 'No chip information available'))
                    lChip.append(('Adf5901 TX', 'No chip information available'))
                    lChip.append(('Adf5904 RX1', 'No chip information available'))
                    lChip.append(('Adf5904 RX2', 'No chip information available'))
        return lChip

    # DOXYGEN ------------------------------------------------------
    #> @brief Set attribute of class self.ct
    #>
    #> Sets different attributes of the class self.ct
    #>
    #> @param[in]     stSel: String to select attribute
    #> 
    #> @param[in]     Val: value of the attribute (optional); can be a string or a number  
    #>  
    #> Supported parameters  
    #>              - Currently no set parameter supported  
    def RfSet(self, *varargin):
        if len(varargin) > 0:
            stVal = varargin[0]

    # DOXYGEN ------------------------------------------------------
    #> @brief Get attribute of class self.ct
    #>
    #> Reads back different attributs of the self.ct
    #>
    #> @param[in]   stSel: String to select attribute
    #> 
    #> @return      Val: value of the attribute (optional); can be a string or a number  
    #>  
    #> Supported parameters  
    #>      -   <span style = "color: #ff9900;"> 'TxPosn': </span> Array containing positions of Tx antennas 
    #>      -   <span style = "color: #ff9900;"> 'RxPosn': </span> Array containing positions of Rx antennas
    #>      -   <span style = "color: #ff9900;"> 'ChnDelay': </span> Channel delay of receive channels
    def RfGet(self, *varargin):
        if len(varargin) > 0:
            stVal = varargin[0]
            if stVal == 'TxPosn':
                Ret = zeros(2)
                Ret[0]  =   -63.068e-3
                Ret[1]  =   -19.545e-3
                return Ret
            elif stVal == 'RxPosn':
                Ret = arange(8)
                Ret = Ret*6.2170e-3
                return Ret
            elif stVal == 'B':
                Ret = (self.Rf_fStop - self.Rf_fStrt)  
                return  Ret
            elif stVal == 'kf':
                Ret = (self.Rf_fStop - self.Rf_fStrt)/self.Rf_TRampUp
                return  Ret
            elif stVal == 'kfUp':
                Ret = (self.Rf_fStop - self.Rf_fStrt)/self.Rf_TRampUp
                return  Ret                
            elif stVal == 'kfDo':
                Ret = (self.Rf_fStop - self.Rf_fStrt)/self.Rf_TRampDo  
                return  Ret 
            elif stVal == 'fc':
                Ret = (self.Rf_fStop + self.Rf_fStrt)/2                          
                return  Ret
        return -1

    # DOXYGEN ------------------------------------------------------
    #> @brief Enable all receive channels
    #>
    #> Enables all receive channels of frontend
    #>
    #>  
    #> @note: This command calls the Adf4904 self.cts Adf_Rx1 and Adf_Rx2 without altering the class settings
    #>        In the default configuration all Rx channels are enabled. The configuration of the self.cts can be changed before 
    #>        calling the RxEna command.
    #>
    #> @code 
    #> CfgRx1.Rx1 = 0;
    #> Brd.Adf_Rx1.SetCfg(CfgRx1);
    #> CfgRx2.All = 0;
    #> Brd.Adf_Rx2.SetCfg(CfgRx2);    
    #> @endcode
    #>  In the above example Chn1 of receiver 1 is disabled and all channels of receiver Rx2 are disabled
    def     RfRxEna(self):     
        self.RfAdf5904Ini(1);
        self.RfAdf5904Ini(2);   

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure receivers 
    #>
    #> Configures selected receivers
    #>
    #> @param[in]   Mask: select receiver: 1 receiver 1; 2 receiver 2
    #>  
    def     RfAdf5904Ini(self, Mask):
        if Mask == 1:
            self.Adf_Rx1.Ini();
            if self.cDebugInf > 10:
                print('Rf Initialize Rx1 (ADF5904)')
        elif Mask == 2:
            self.Adf_Rx2.Ini();
            if self.cDebugInf > 10:                                  
                print('Rf Initialize Rx2 (ADF5904)')  
                                
        else:
            pass

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure registers of receiver
    #>
    #> Configures registers of receivers
    #>
    #> @param[in]   Mask: select receiver: 1 receiver 1; 2 receiver 2
    #>  
    #> @param[in]   Data: register values
    #>
    #> @note Function is obsolete in class version >= 1.0.0: use function Adf_Rx1.SetRegs() and Adf_Rx2.SetRegs() to configure receiver 
    def     RfAdf5904SetRegs(self, Mask, Data):
        if Mask == 1:
            self.Adf_Rx1.SetRegs(Data)
            if self.cDebugInf > 10:
                print('Rf Initialize Rx1 (ADF5904)')
        elif Mask == 2:
            Adf_Rx2.SetRegs(Data);
            if self.cDebugInf > 10:                                  
                print('Rf Initialize Rx2 (ADF5904)')  
        else:
            pass     

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure registers of transmitter
    #>
    #> Configures registers of transmitter
    #>
    #> @param[in]   Mask: select receiver: 1 transmitter 1
    #>  
    #> @param[in]   Data: register values
    #>
    #> @note Function is obsolete in class version >= 1.0.0: use function Adf_Tx.SetRegs() to configure transmitter 
    def     RfAdf5901SetRegs(self, Mask, Data):
        if Mask == 1:
            self.Adf_Tx.SetRegs(Data);


    # DOXYGEN ------------------------------------------------------
    #> @brief Configure registers of PLL
    #>
    #> Configures registers of PLL
    #>
    #> @param[in]   Mask: select receiver: 1 transmitter 1
    #>  
    #> @param[in]   Data: register values
    #>
    #> @note Function is obsolete in class version >= 1.0.0: use function Adf_Pll.SetRegs() to configure PLL 
    def     RfAdf4159SetRegs(self, Mask, Data):
        Data = Data.flatten()

        dUSpiCfg["Mask"]    =   self.Rf_USpiCfg_Mask
        dUSpiCfg["Chn"]     =   self.Rf_USpiCfg_Pll_Chn
        Ret = self.Fpga_SetUSpiData(dUSpiCfg, Data);
        return Ret

    # DOXYGEN -------------------------------------------------
    #> @brief Displays status of frontend in Matlab command window  
    #>
    #> Display status of frontend in Matlab command window         
    def     BrdDispSts(self):
        self.BrdDispInf()
        self.Fpga_DispRfChipSts(1)

    # DOXYGEN ------------------------------------------------------
    #> @brief Enable transmitter
    #>
    #> Configures TX device
    #>
    #> @param[in]   TxChn
    #>                  - 0: off
    #>                  - 1: Transmitter 1 on
    #>                  - 2: Transmitter 2 on
    #> @param[in]   TxPwr: Power register setting 0 - 256; only 0 to 100 has effect
    #>
    #>  
    def     RfTxEna(self, TxChn, TxPwr):
        TxChn = (TxChn % 3)
        TxPwr = (TxPwr % 2**8)
        if self.cDebugInf > 10:
            stOut = "Rf Initialize Tx (ADF5901): Chn: " + str(TxChn) + " | Pwr: " + str(TxPwr)                                     
            print(stOut)
        dCfg = dict()
        dCfg["TxChn"]   =   TxChn
        dCfg["TxPwr"]   =   TxPwr
        self.Adf_Tx.SetCfg(dCfg)
        self.Adf_Tx.Ini()

    def     Fpga_DispBrdInf(self):
        print(' ')
        print('-------------------------------------------------------------------')
        print('Radarbook2: Board Information')
        Val = self.Fpga_GetBrdInf()
        Ret = -1
        if len(Val) > 1:
            uidHigh = '%08X' % (Val[1]);
            uidLow = '%08X' % (Val[0]);
            print('UID = ', uidHigh, uidLow, ' h');
            if len(Val) > 2:
                Temp = Val[2] - 273;
                print('Temp = ', Temp, ' deg');
                Temp = Val[3];
                print('RfTemp = ', Temp, ' deg');   
            if len(Val) > 5:
                print('U Rf =   ', Val[4]/1000, ' V');   
                print('I Rf =   ', Val[5], ' mA');   
        else:
            print('Board does not respond!')

        print('-------------------------------------------------------------------')
        return Val

    def     RfMeas(self, *varargin):   

        ErrCod = 0       
        if len(varargin) > 1:
            stMod = varargin[0]

            if stMod == 'ExtTrigUp':
                print('Measurement Mode: ExtTrigUp')
                dCfg = varargin[1]

                if not ('fStrt' in dCfg):
                    print('RfMeas: fStrt not specified!')
                    traceback.print_exc()
                if not ('fStop' in dCfg):
                    print('RfMeas: fStop not specified!')
                    traceback.print_exc()
                if not ('TRampUp' in dCfg):
                    print('RfMeas: TRampUp not specified!')
                    traceback.print_exc()            
                if not ('N' in dCfg):
                    print('RfMeas: N not specified!')
                    traceback.print_exc()
                if not ('TInt' in dCfg):
                    print('RfMeas: TInt not specified!')
                    traceback.print_exc()                  
                if not ('Np' in dCfg):
                    Np = 1
                    Tp = dCfg['TRampUp'] 
                else:
                    if not ('Tp' in dCfg):
                        print('RfMeas: Tp not sepecified!')
                        traceback.print_exc()

                    Np = dCfg['Np']
                    Tp = dCfg['Tp']
              
                dCfg = self.ChkMeasCfg(dCfg);
                
                self.Rf_fStrt = dCfg["fStrt"]
                self.Rf_fStop = dCfg["fStop"]
                self.Rf_TRampUp = dCfg["TRampUp"]

                if 'SynthesizerCfg' in dCfg:
                    if dCfg["SynthesizerCfg"] > 0:
                        # Configure ADF4159 PLL
                        dCfgAdf4159 = dict()
                        dCfgAdf4159["fStrt"] = dCfg["fStrt"]
                        dCfgAdf4159["fStop"] = dCfg["fStop"]
                        dCfgAdf4159["TRampUp"] = dCfg["TRampUp"]
                        self.RfAdf4159Ini(dCfgAdf4159)
                else:
                    # Configure ADF4159 PLL
                    dCfgAdf4159 = dict()
                    dCfgAdf4159["fStrt"] = dCfg["fStrt"]
                    dCfgAdf4159["fStop"] = dCfg["fStop"]
                    dCfgAdf4159["TRampUp"] = dCfg["TRampUp"]
                    self.RfAdf4159Ini(dCfgAdf4159)                        

                # Set Nr Frames
                self.cNumPackets = 0

                CicIni = True
                if CicIni in dCfg:
                    if CicIni > 0:    
                        CicIni = True
                    else:
                        CicIni = False

                # Calculate Sampling Rate
                N = dCfg["N"];
                Ts = dCfg["TRampUp"]/N
                if CicIni:
                    fs = 1/Ts
                    fAdc = self.Get('fAdc') 
                    Div = floor(fAdc/fs)
                    print('-------------------------------------------------------------------')
                    print('  fAdc:  ', (fAdc/1e6), ' MHz')
                    print('  CicR:  ', (Div), ' ')
                    print('  TSamp: ', (N/(fAdc/Div)/1e-6), ' us')                      
                    print('-------------------------------------------------------------------');
                    if Div < 1:
                        Div = 1
                    self.CfgCicFilt(Div)  

                fs = self.Get('fs')
                self.Set('N', N)
                
                if floor(dCfg['TRampUp']*fs) != N:
                    print(' ')
                    print('Sampled duration is smaller than chirp duration');
                    print('Number of samples: ', N)
                    print('Sampled duration: ', (1/fs*N/1e-6),' | Chirp duration: ', (dCfg['TRampUp']/1e-6))
                    print(' ')


                CfgTim = 10e-6
                if 'CfgTim' in dCfg:
                    CfgTim = dCfg['CfgTim']


                TWait = dCfg['TInt'] - (Tp*Np) - CfgTim
                if TWait < 10e-6:
                    TWait = 10e-6

                self.Rad_FrmCtrlCfg_RegChnCtrl = self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA + self.cFRMCTRL_REG_CH0CTRL_WAITENA + self.cFRMCTRL_REG_CH0CTRL_GLOBWAITENA    
                self.BrdSampIni()

                # Use External events to synchronize boards to
                # master timing
                if (self.Rad_Role >= self.cRAD_ROLE_SL):
                    dCfg['IniEve'] = 1
                    dCfg['ExtEve'] = 1
                

                IniEve = 1
                if 'IniEve' in dCfg:
                    IniEve = dCfg['IniEve']
                
                IniTim = 0.2e-3
                if 'IniTim' in dCfg:
                    IniTim = dCfg['IniTim']
                
                WaitEve = 0
                if 'ExtEve' in dCfg:
                    WaitEve = dCfg['ExtEve']
                    if dCfg['ExtEve'] > 0:
                        TWait = 0.1e-3
                
                dTimCfg = dict()                     
                dTimCfg['IniEve'] = IniEve                             #  Use ExtEve after Ini phase
                dTimCfg['IniTim'] = IniTim                             #  Duration of Ini phase in us 
                dTimCfg['MeasTim'] = Tp
                dTimCfg['MeasNp'] = Np
                dTimCfg['WaitEve'] = WaitEve
                dTimCfg['WaitTim'] = TWait
                dTimCfg['CfgTim'] = CfgTim
                
                self.Setfs();
                self.SetFuSca();
                self.Computation.SetParam('Np', Np);
                self.Computation.SetParam('Tp', Tp);
                                    
                self.BrdSetTim_MxPW(dTimCfg)
                    
                if 'Strt' in dCfg:
                    if dCfg["Strt"] > 0:
                        self.BrdAccessData()
                        self.BrdSampStrt()
                else:
                    self.BrdAccessData()
                    self.BrdSampStrt()



            elif stMod == 'Cw':
                print(' ')
                print('Measurement Mode Cw')
                dCfg = varargin[1]
                
                if not ('fCenter' in dCfg):
                    print('RfMeas: fCenter not specified!');
                    traceback.print_exc()                    
                if not ('N' in dCfg):
                    print('RfMeas: N not specified!');
                    traceback.print_exc()
                if not ('TInt' in dCfg):
                    print('RfMeas: TInt not specified!');
                    traceback.print_exc()
                if not ('TMeas' in dCfg):
                    print('RfMeas: TMeas not specified!');
                    traceback.print_exc()                        
                
                dCfg = self.ChkMeasCfg(dCfg)
                
                self.Rf_fStrt = dCfg['fCenter']
                self.Rf_fStop = dCfg['fCenter']
                self.Rf_TRampUp = 0
                
                if ('SynthesizerCfg' in dCfg):
                    if dCfg['SynthesizerCfg'] > 0:
                        # Configure ADF4159 PLL
                        # Configure dummy ramp. In CW no trigger is
                        # generated and board is at fStart
                        dCfgAdf4159 = dict()
                        dCfgAdf4159['fStrt'] = dCfg['fCenter']
                        dCfgAdf4159['fStop'] = dCfg['fCenter'] + 10e6
                        dCfgAdf4159['TRampUp'] = 10e-6
                        self.RfAdf4159Ini(dCfgAdf4159)
                else:
                    # Configure ADF4159 PLL
                    # Configure dummy ramp. In CW no trigger is
                    # generated and board is at fStart
                    dCfgAdf4159 = dict()
                    dCfgAdf4159['fStrt'] = dCfg['fCenter']
                    dCfgAdf4159['fStop'] = dCfg['fCenter'] + 10e6
                    dCfgAdf4159['TRampUp'] = 10e-6
                    self.RfAdf4159Ini(dCfgAdf4159)                        
            

                # Set Nr Frames
                self.cNumPackets = 0                  

                # Calculate Sampling Rate
                N = dCfg['N']

                IniCic  =   1
                if ('IniCic' in dCfg):
                    if dCfg['IniCic'] == 0:
                        IniCic = 0

                if IniCic > 0:
                    Ts = dCfg['TMeas']/N
                    fs = 1/Ts
                    fAdc = self.Get('fAdc') 
                    Div = floor(fAdc/fs)
                    if Div < 1:
                        Div = 1
                    
                    print('-------------------------------------------------------------------')
                    print('  fAdc:  ', (fAdc/1e6), ' MHz')
                    print('  CicR:  ', (Div), ' ')
                    print('  TSamp: ', (N/(fAdc/Div)/1e-6), ' us')                      
                    print('-------------------------------------------------------------------')
                    # Configure CIC Filter 
                    self.CfgCicFilt(Div)
            
                self.Set('N', N)
                fs = self.Get('fs')

                
                self.Rad_FrmCtrlCfg_RegChnCtrl = self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA + self.cFRMCTRL_REG_CH0CTRL_WAITENA + self.cFRMCTRL_REG_CH0CTRL_GLOBWAITENA
                self.BrdSampIni()
                
                # TODO Set Timing 
                self.BrdSetTimCont('ContUnifM1', dCfg['TInt'])
                time.sleep(1e-3)
                
                if ('Strt' in dCfg):
                    if dCfg['Strt'] > 0:
                        self.BrdAccessData()
                        self.BrdSampStrt()
                    
                else:
                    self.BrdAccessData();
                    self.BrdSampStrt();
 

            elif stMod == 'ExtTrigUp_TxSeq':
                
                print(' ')
                print('Measurement Mode ExtTrigUp_TxSeq')                 
            
                dCfg = varargin[1]

                if not ('fStrt' in dCfg):
                    print('RfMeas: fStrt not specified!')
                    traceback.print_exc()
                
                if not ('fStop' in dCfg):
                    print('RfMeas: fStop not specified!')
                    traceback.print_exc()
                
                if not ('TRampUp' in dCfg):
                    print('RfMeas: TRampUp not specified!')
                    traceback.print_exc()
            
                if not ('N' in dCfg):
                    print('RfMeas: N not specified!')
                    traceback.print_exc()

                if not ('TInt' in dCfg):
                    print('RfMeas: TInt not specified!')
                    traceback.print_exc()

                if not ('Tp' in dCfg):
                    print('RfMeas: Tp not specified!')
                    traceback.print_exc()

                if not ('Np' in dCfg):
                    Np = 1
                else:
                    Np = dCfg['Np']
                 
                
                dCfg = self.ChkMeasCfg(dCfg)
                
                self.Rf_fStrt = dCfg['fStrt']
                self.Rf_fStop = dCfg['fStop']
                self.Rf_TRampUp = dCfg['TRampUp']
                
                if ('SynthesizerCfg' in dCfg):
                    if dCfg['SynthesizerCfg'] > 0:
                        # Configure ADF4159 PLL
                        dCfgAdf4159 = dict()
                        dCfgAdf4159['fStrt'] = dCfg['fStrt']
                        dCfgAdf4159['fStop'] = dCfg['fStop']
                        dCfgAdf4159['TRampUp']= dCfg['TRampUp']
                        self.RfAdf4159Ini(dCfgAdf4159)
                else:
                    # Configure ADF4159 PLL
                    dCfgAdf4159 = dict()
                    dCfgAdf4159['fStrt'] = dCfg['fStrt']
                    dCfgAdf4159['fStop'] = dCfg['fStop']
                    dCfgAdf4159['TRampUp'] = dCfg['TRampUp']
                    self.RfAdf4159Ini(dCfgAdf4159)             
            

                # Set Nr Frames is only needed for Radserve
                self.cNumPackets = 0                  
                                    
                TxSeq = dCfg['TxSeq']
                NEntr = len(TxSeq)

                # Calculate Sampling Rate
                N = dCfg['N']
                IniCic  =   1
                if ('IniCic' in dCfg):
                    if dCfg['IniCic'] == 0:
                        IniCic  =   0

                if IniCic > 0:
                    Ts = dCfg['TRampUp']/N
                    fs = 1/Ts
                    fAdc = self.Get('fAdc')
                    Div = floor(fAdc/fs)
                    if Div < 1:
                        Div = 1

                    print('-------------------------------------------------------------------')
                    print('  fAdc:  ', (fAdc/1e6), ' MHz')
                    print('  CicR:  ', (Div), ' ')
                    print('  TSamp: ', (N/(fAdc/Div)/1e-6), ' us')                      
                    print('-------------------------------------------------------------------')
                    # Configure CIC Filter 
                    self.CfgCicFilt(Div)

                self.Set('N', N)
        
                fs = self.Get('fs')
                if floor(dCfg['TRampUp']*fs) != N:
                    print('Sampled duration is smaller than chirp duration')
                    print('Number of samples: ', (N))
                    print('Sampled duration: ', (1/fs*N/1e-6),' | Chirp duration: ', (dCfg['TRampUp']/1e-6))
                
                                    
                self.Fpga_MimoSeqRst()
                self.Fpga_MimoSeqNrEntries(3)
                
                RegTxOff = self.Adf_Tx.GenRegFlag('R0', 0, 'AuxBufGain', 4, 'AuxDiv', 1, 'PupNCntr', 1, 'PupRCntr', 1, 'PupVco', 1, 'PupLo', 1, 'PupAdc', 1)
                RegTx1 = self.Adf_Tx.GenRegFlag('R0', 0, 'AuxBufGain', 4, 'AuxDiv', 1, 'PupNCntr', 1, 'PupRCntr', 1, 'PupVco', 1, 'PupTx1', 1, 'PupLo', 1, 'PupAdc', 1)
                RegTx2 = self.Adf_Tx.GenRegFlag('R0', 0, 'AuxBufGain', 4, 'AuxDiv', 1, 'PupNCntr', 1, 'PupRCntr', 1, 'PupVco', 1, 'PupTx2', 1, 'PupLo', 1, 'PupAdc', 1)
                
                # Configure Sequencer: to switch between antennas
                # ADF5901 is connected to channel 0 of sequencer
                self.Fpga_MimoSeqSetChnRegs(0, 0, array([RegTxOff]))
                self.Fpga_MimoSeqSetChnRegs(0, 1, array([RegTx1]))
                self.Fpga_MimoSeqSetChnRegs(0, 2, array([RegTx2]))
                
                
                self.Rad_FrmCtrlCfg_RegChnCtrl = self.cFRMCTRL_REG_CH0CTRL_WAITENA + self.cFRMCTRL_REG_CH0CTRL_GLOBWAITENA + self.cFRMCTRL_REG_CH0CTRL_FRMCNTRENA    
                self.BrdSampIni()

                # Use External events to synchronize boards to
                # master timing
                if (self.Rad_Role >= self.cRAD_ROLE_SL):
                    dCfg['IniEve'] = 1
                    dCfg['ExtEve'] = 1
                    print('Set Role Slave')             

                ExtEve  =   0;
                if ('ExtEve' in dCfg):
                    if dCfg['ExtEve'] > 0:
                        ExtEve = 1
                    
                IniEve = 1
                if ('IniEve' in dCfg):
                    IniEve  =   dCfg['IniEve']
    
                IniTim = 2e-3
                if ('IniTim' in dCfg):
                    IniTim = dCfg['IniTim']

                if 'CfgTim' in dCfg:
                    CfgTim = dCfg['CfgTim']
                    if CfgTim + dCfg['TRampUp'] > dCfg['Tp']:
                        dCfg['Tp'] = (dCfg['TRampUp'] + CfgTim)
                        print('AdfTx2Rx8: Configuration time to short -> Set Tp to : ', dCfg['Tp'])                        
                else:                             
                    # Check the timing in this mode
                    CfgTim = dCfg['Tp'] - (dCfg['TRampUp'])
                    if (CfgTim < 10e-6):
                        CfgTim = 10e-6
                        dCfg['Tp'] = (dCfg['TRampUp'] + CfgTim)
                        print('AdfTx2Rx8: Configuration time to short -> Set Tp to : ', (dCfg['Tp']))
                
                WaitTim = dCfg['TInt'] - dCfg['Tp']*NEntr*Np;
                if WaitTim < 1e-3:
                    WaitTim = 1e-3
                    print('AdfTx2Rx8: Wait time to short -> Set WaitTim to : ', (WaitTim))
                
                if ('ExtEve' in dCfg):
                    WaitEve = dCfg['ExtEve']
                    if dCfg['ExtEve'] > 0:
                        WaitTim = 0.1e-3
                
                dTimCfg = dict()
                dTimCfg['IniEve'] = IniEve                     #   Use ExtEve after Ini phase
                dTimCfg['IniTim'] = IniTim                     #   Duration of Ini phase in us 
                dTimCfg['CfgTim'] = CfgTim                     #   Configuration: Configure RCC for ExtTrig 
                dTimCfg['MeasTim'] = dCfg['Tp'] - dTimCfg['CfgTim']
                dTimCfg['MeasNp'] = Np
                dTimCfg['WaitEve'] = ExtEve
                dTimCfg['WaitTim'] = WaitTim
                dTimCfg['NTx'] = NEntr
                dTimCfg['TxSeq'] = TxSeq
                 
                self.BrdSetTim_MxPaconPW(dTimCfg)
                
                if ('Strt' in dCfg):
                    if dCfg['Strt'] > 0:
                        self.BrdAccessData()
                        self.BrdSampStrt()
                else:
                    self.BrdAccessData()
                    self.BrdSampStrt()
                

         
        return ErrCod


    # DOXYGEN ------------------------------------------------------
    #> @brief Reset frontend
    #>
    #> Reset frontend; Disables and enabled supply to reset frontend
    #> 
    def     RfRst(self):
        self.BrdPwrDi()
        self.BrdPwrEna()

    # DOXYGEN ------------------------------------------------------
    #> @brief Initialize PLL with selected configuration
    #>
    #> Configures PLL
    #>
    #> @param[in]   Cfg: structure with PLL configuration
    #>      -   <span style = "color: #ff9900;"> 'fStrt': </span> Start frequency in Hz
    #>      -   <span style = "color: #ff9900;"> 'fStrt': </span> Stop frequency in Hz
    #>      -   <span style = "color: #ff9900;"> 'TRampUp': </span> Upchirp duration in s
    #>
    #> %> @note Function is obsolete in class version >= 1.0.0: use function Adf_Pll.SetCfg() and Adf_Pll.Ini()          
    def     RfAdf4159Ini(self, Cfg):
        self.Computation.SetParam('fStrt',   Cfg["fStrt"]);
        self.Computation.SetParam('fStop',   Cfg["fStop"]);
        self.Computation.SetParam('TRampUp', Cfg["TRampUp"]);
    
        self.Adf_Pll.SetCfg(Cfg);
        self.Adf_Pll.Ini();


    def     BrdSetTim_MxPW(self, dCfg):

        fSeqTrig = self.Rad_fAdc
        Seq = SeqTrig.SeqTrig(fSeqTrig)

        print('BrdSetTim_MxPW')

        SeqTrigCfg = dict()
        SeqTrigCfg["Mask"] = 1
        SeqTrigCfg["Ctrl"] = Seq.SEQTRIG_REG_CTRL_IRQ2ENA                   # Enable interrupt event on channel 2 
        SeqTrigCfg["ChnEna"] = Seq.SEQTRIG_REG_CTRL_CH0ENA + Seq.SEQTRIG_REG_CTRL_CH1ENA + Seq.SEQTRIG_REG_CTRL_CH3ENA
        
        if dCfg['MeasNp'] <= 256:
            # Phase 0: Ini with dummy frame: (caSeq, 'RccCfg', TCfg, Adr, TxChn(Idx)-1);
            if dCfg["IniEve"] > 0:
                lSeq = Seq.IniSeq('IniExt', dCfg["IniTim"])  
            else:
                lSeq = Seq.IniSeq('Ini', dCfg["IniTim"]) 
            
            # Phase 2: Synchronisation
            lSeq = Seq.AddSeq(lSeq,'SyncnChn3', dCfg["CfgTim"], 2, 0)
            # Phase 3: Meas 
            lSeq = Seq.AddSeq(lSeq,'RccMeasN', dCfg["MeasTim"], 2, 3, dCfg["MeasNp"], 0)
            
            if dCfg["WaitEve"] > 0:
                lSeq = Seq.AddSeq(lSeq,'WaitEve', dCfg["WaitTim"], 1, 0)
            else:           
                lSeq = Seq.AddSeq(lSeq,'Wait', dCfg["WaitTim"], 1, 0)

            if self.Rad_Role >= self.cRAD_ROLE_SL:
                for Elem in lSeq:
                    Elem['Chn3Cfg'] = (2**30)

            SeqTrigCfg["Seq"] = lSeq;
            self.SeqCfg = SeqTrigCfg;

        else:

            NpLoop = floor(dCfg['MeasNp']/256)
            NpResidual = floor(dCfg['MeasNp'] - NpLoop*256)
            
            # Phase 0: Ini with dummy frame: (caSeq, 'RccCfg', TCfg, Adr, TxChn(Idx)-1);
            if dCfg['IniEve'] > 0:
                lSeq = Seq.IniSeq('IniExt', dCfg['IniTim'])    
            else:
                lSeq = Seq.IniSeq('Ini', dCfg['IniTim'])     

            # Phase 2: Synchronisation
            lSeq = Seq.AddSeq(lSeq,'SyncnChn3', dCfg["CfgTim"], 2, 0)            
            # Phase 3: Meas NpLoop * 256 and then exit to residual
            # measurements
            lSeq = Seq.AddSeq(lSeq,'RccMeasN', dCfg['MeasTim'], 2, 3, 255, 0)
            lSeq = Seq.AddSeq(lSeq,'RccMeasN', dCfg['MeasTim'], 2, 4, NpLoop, 0)
            if NpResidual > 0:
                lSeq = Seq.AddSeq(lSeq,'RccMeasN', dCfg['MeasTim'], 4, 5, NpResidual, 0)
            if dCfg['WaitEve'] > 0:
                lSeq = Seq.AddSeq(lSeq,'WaitEve', dCfg['WaitTim'], 1, 0)
            else:           
                lSeq = Seq.AddSeq(lSeq,'Wait', dCfg['WaitTim'], 1, 0)

            if self.Rad_Role >= self.cRAD_ROLE_SL:
                for Elem in lSeq:
                    Elem['Chn3Cfg'] = (2**30)

            SeqTrigCfg["Seq"]               =   lSeq;
            self.SeqCfg = SeqTrigCfg;

        self.Fpga_SeqTrigRst(self.SeqCfg["Mask"]);
        NSeq = len(self.SeqCfg["Seq"]);
        if self.cDebugInf > 0:
            print('Run SeqTrig: ', NSeq, ' Entries')
        #Tmp = self.SeqCfg["Seq"]
        for Idx in range (0, NSeq):
            self.Fpga_SeqTrigCfgSeq(self.SeqCfg["Mask"], Idx, self.SeqCfg["Seq"][Idx])


    def     BrdSetTim_MxPaconPW(self, dCfg):
        fSeqTrig = self.Rad_fAdc
        Seq = SeqTrig.SeqTrig(fSeqTrig)
        if self.cDebugInf > 0:
            print('BrdSetTim_MxPaconPW')
    
        #--------------------------------------------------------------------------
        # Configure Sequence Trigger Unit
        #--------------------------------------------------------------------------
        SeqTrigCfg = dict()
        SeqTrigCfg['Mask']          =   1
        SeqTrigCfg['Ctrl']          =   Seq.SEQTRIG_REG_CTRL_IRQ2ENA                        # Enable interrupt event on channel 2 
        SeqTrigCfg['ChnEna']        =   Seq.SEQTRIG_REG_CTRL_CH0ENA + Seq.SEQTRIG_REG_CTRL_CH1ENA + Seq.SEQTRIG_REG_CTRL_CH3ENA
        
        # PaCon have no effect with ADF frontend: no digital signals
        # for TX switching available
        # SPI Transfer is used to set the antenna of ADF5901
        PaConTxOff = 0
        PaConTx1On = 1
        PaConTx2On = 2
        
        TxChn = dCfg['TxSeq']
        NTx = len(TxChn)
        
        if dCfg['MeasNp'] <= 256:
            
            if dCfg['IniEve'] > 0:
                lSeq = Seq.IniSeq('IniExt', dCfg['IniTim'])
            else:
                lSeq = Seq.IniSeq('Ini', dCfg['IniTim'])
            Adr = 2
            for Idx in range(0,NTx):
                if TxChn[Idx] == 0:
                    PaCon = PaConTxOff 
                elif TxChn[Idx] == 1:
                    PaCon = PaConTx1On 
                elif TxChn[Idx] == 2:
                    PaCon = PaConTx2On  
                else: 
                    PaCon = PaConTxOff  
            
                lSeq = Seq.AddSeq(lSeq, 'RccCfgPaCon', dCfg['CfgTim'], Adr, TxChn[Idx], PaCon)
                Adr = Adr + 1
                if Idx == NTx-1:
                    #Tp, LoopAdr, ExitAdr, NLoop, Id, PaCtrl  
                    lSeq = Seq.AddSeq(lSeq, 'RccMeasPaConN', dCfg['MeasTim'], 1, Adr, dCfg['MeasNp'], TxChn[Idx], PaCon)   
                else:
                    lSeq = Seq.AddSeq(lSeq, 'RccMeasPaCon', dCfg['MeasTim'], Adr, TxChn[Idx], PaCon)
                Adr = Adr + 1
            
            if dCfg['WaitEve'] > 0:
                lSeq = Seq.AddSeq(lSeq, 'WaitEve', dCfg['WaitTim'], 1, 0)
            else:
                lSeq = Seq.AddSeq(lSeq, 'Wait', dCfg['WaitTim'], 1, 0)
        
            if self.Rad_Role >= self.cRAD_ROLE_SL:
                for Elem in lSeq:
                    Elem['Chn3Cfg'] = (2**30)

            SeqTrigCfg["Seq"]               =   lSeq
            self.SeqCfg = SeqTrigCfg
        else:
            
            NpLoop = floor(dCfg['MeasNp']/256)
            NpResidual = floor(dCfg['MeasNp'] - NpLoop*256)            
            if dCfg['IniEve'] > 0:
                lSeq = Seq.IniSeq('IniExt', dCfg['IniTim'])
            else:
                lSeq = Seq.IniSeq('Ini', dCfg['IniTim'])

            Adr = 2
        
            for Idx in range(0,NTx):
                if TxChn[Idx] == 0:
                    PaCon = PaConTxOff 
                elif TxChn[Idx] == 1:
                    PaCon = PaConTx1On 
                elif TxChn[Idx] == 2:
                    PaCon = PaConTx2On  
                else: 
                    PaCon = PaConTxOff  

                lSeq = Seq.AddSeq(lSeq, 'RccCfgPaCon', dCfg['CfgTim'], Adr, TxChn[Idx], PaCon)
                Adr = Adr + 1
                if Idx == NTx - 1:
                    #Tp, LoopAdr, ExitAdr, NLoop, Id, PaCtrl  
                    lSeq = Seq.AddSeq(lSeq, 'RccMeasPaConN', dCfg['MeasTim'], 1, Adr, 255, TxChn[Idx], PaCon)   
                else:
                    caSeq = Seq.AddSeq(lSeq, 'RccMeasPaCon', dCfg['MeasTim'], Adr, TxChn[Idx], PaCon)
                Adr = Adr + 1
              
            for Idx in range(0,NTx):
                if TxChn[Idx] == 0:
                    PaCon = PaConTxOff 
                elif TxChn[Idx] == 1:
                    PaCon = PaConTx1On 
                elif TxChn[Idx] == 2:
                    PaCon = PaConTx2On  
                else: 
                    PaCon = PaConTxOff  

                lSeq = Seq.AddSeq(lSeq, 'RccCfgPaCon', dCfg['CfgTim'], Adr, TxChn[Idx], PaCon)
                Adr = Adr + 1
                if Idx == NTx - 1:
                    #Tp, LoopAdr, ExitAdr, NLoop, Id, PaCtrl  
                    lSeq = Seq.AddSeq(lSeq, 'RccMeasPaConN', dCfg['MeasTim'], 1, Adr, NpLoop, TxChn[Idx], PaCon)   
                else:
                    caSeq = Seq.AddSeq(lSeq, 'RccMeasPaCon', dCfg['MeasTim'], Adr, TxChn[Idx], PaCon)
                AdrRes = Adr  
                Adr = Adr + 1
                
            if NpResidual > 0:
                for Idx in range(0,NTx):
                    if TxChn[Idx] == 0:
                        PaCon = PaConTxOff 
                    elif TxChn[Idx] == 1:
                        PaCon = PaConTx1On 
                    elif TxChn[Idx] == 2:
                        PaCon = PaConTx2On  
                    else: 
                        PaCon = PaConTxOff  
                    
                    lSeq = Seq.AddSeq(lSeq, 'RccCfgPaCon', dCfg['CfgTim'], Adr, TxChn[Idx], PaCon)
                    Adr = Adr + 1
                    if Idx == NTx - 1:
                        #Tp, LoopAdr, ExitAdr, NLoop, Id, PaCtrl  
                        lSeq = Seq.AddSeq(lSeq, 'RccMeasPaConN', dCfg['MeasTim'], AdrRes, Adr, NpResidual, TxChn[Idx], PaCon)
                    else:
                        caSeq = Seq.AddSeq(lSeq, 'RccMeasPaCon', dCfg['MeasTim'], Adr, TxChn[Idx], PaCon)
                    Adr = Adr + 1            

            if dCfg['WaitEve'] > 0:
                lSeq = Seq.AddSeq(lSeq, 'WaitEve', dCfg['WaitTim'], 1, 0)
            else:
                lSeq = Seq.AddSeq(lSeq, 'Wait', dCfg['WaitTim'], 1, 0)

            if self.Rad_Role >= self.cRAD_ROLE_SL:
                for Elem in lSeq:
                    Elem['Chn3Cfg'] = (2**30)
                    
            SeqTrigCfg["Seq"]               =   lSeq
            self.SeqCfg = SeqTrigCfg               
            
        
        self.Fpga_SeqTrigRst(self.SeqCfg["Mask"]);
        NSeq = len(self.SeqCfg["Seq"]);
        if self.cDebugInf > 0:
            print('Run SeqTrig: ', NSeq, ' Entries')
        Tmp = self.SeqCfg["Seq"]
        for Idx in range (0, NSeq):
            self.Fpga_SeqTrigCfgSeq(self.SeqCfg["Mask"], Idx, self.SeqCfg["Seq"][Idx])

   

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure CIC filter for clock divider
    #>
    #> Configure CIC filter: Filter transfer function is adjusted to sampling rate divider
    #> 
    #> @param[in] Div: 
    def     CfgCicFilt(self, Div):             
        if Div >= 16:
            self.Set('CicEna')                         
            self.Set('CicR',Div)
            self.Set('CicDelay',16)
            self.Set('CicStages',4)
        elif Div >= 8:
            self.Set('CicEna')                         
            self.Set('CicR',Div)
            self.Set('CicDelay',8)
            self.Set('CicStages',4)
        elif Div >= 4:
            self.Set('CicEna')                         
            self.Set('CicR',Div)
            self.Set('CicDelay',4)
            self.Set('CicStages',4)                        
        elif Div >= 2:    
            self.Set('CicEna')                           
            self.Set('CicR',Div)
            self.Set('CicDelay',2)
            self.Set('CicStages',4)
        else:
            self.Set('CicDi')       


    # DOXYGEN ------------------------------------------------------
    #> @brief Check measurement configuration 
    #> 
    #> Check measurement configuration structure
    #> 
    #> @param[in] Cfg: 
    def     ChkMeasCfg(self, dCfg):
        
        if 'IniEve' in dCfg:
            IniEve = floor(dCfg["IniEve"])
            IniEve = (IniEve % 2)
            if IniEve != dCfg["IniEve"]:
                print("Rbk2Adf24Tx2RX8: IniEve set to ", IniEve)
            dCfg["IniEve"]  =   IniEve;
        
        if 'ExtEve' in dCfg:
            ExtEve = floor(dCfg["ExtEve"])
            ExtEve = (ExtEve % 2)
            if ExtEve != dCfg["ExtEve"]:
                print("Rbk2Adf24Tx2RX8: ExtEve set to ", ExtEve)
            dCfg["ExtEve"]      =   ExtEve
                      
        if 'N' in dCfg:
            N = ceil(dCfg["N"]/8)*8
            if (N != dCfg["N"]):
                print("Rbk2Adf24Tx2RX8: N must be a mulitple of 8 -> Set N to ", N)
            if N > 4096:
                N = 4096
                print("Rbk2Adf24Tx2RX8: N to large -> Set N to ", N)
            dCfg["N"]   =   N
            if  N < 8:
                N = 8
                print("Rbk2Adf24Tx2RX8: N to small -> Set N to ", N)

        # Check number of repetitions: standard timing modes can only can be used with 65536 repttions
        # Np must be greater or equal to 1 and less or equal than 65536
        if 'Np' in dCfg:
            Np = ceil(dCfg["Np"])
            if Np < 1: 
                Np = 1
            if Np > 65536:
                Np = 65536

            if Np != dCfg["Np"]:
                print("Rbk2Adf24Tx2RX8: Np -> Set Np to ", Np)
            dCfg["Np"]   =   Np

        # Check number of frames: at least one frame must be measured
        if 'NrFrms' in dCfg:
            dCfg["NrFrms"]  =   ceil(dCfg["NrFrms"])
            if dCfg["NrFrms"] < 1:
                dCfg["NrFrms"]  =   1
                print("Rbk2Adf24Tx2RX8: NrFrms < 1 -> Set to 1")
            
        
        # Initialization time (used to generate dummy frame and reset signal processing chain and data interface)
        if 'IniTim' in dCfg:
            IniEve = 1
            if 'IniEve' in dCfg:
                IniEve = dCfg["IniEve"]
            # If external event is programmed after init, a too long ini time can result that the event is missed
            if (IniEve > 0) and (dCfg["IniTim"] > 5e-3):
                print("Rbk2Adf24Tx2RX8: Trigger Event could be missed (Meas does not start)")

        # Tx Field: SeqTrig MMP < 2.0.0 support 16 entries in the sequence table:
        if 'TxSeq' in dCfg:
            TxSeq = array(dCfg["TxSeq"])
            TxSeq = TxSeq.flatten()
            NSeq = len(TxSeq)
            if NSeq > 16:
                print("Rbk2Adf24Tx2RX8: TxSeq to long -> limit to 16")    
                TxSeq = TxSeq[0:16]
            if NSeq < 1:
                TxSeq = 0
                print("Rbk2Adf24Tx2RX8: TxSeq empty -> Set to 0")  
            dCfg["TxSeq"]   =   TxSeq   
        return dCfg

    def RefCornerCube(self, fc, aCorner, R, *varargin):        
        
        c0 = 3e8
        #--------------------------------------------------------------------------
        # Transmit Power 
        #--------------------------------------------------------------------------
        PtdBm = 8
        GtdB = 15
        GrdB = 15
        GcdB = 22  
                   
        if len(varargin) > 0:
            dCfg = varargin[0]
            if 'PtdBm' in dCfg:
                PtdBm = dCfg['PtdBm']
        
        #--------------------------------------------------------------------------
        # Calculate Values
        #--------------------------------------------------------------------------
        Pt = 10**(PtdBm/10)*1e-3
        Gt = 10**(GtdB/10)
        Gr = 10**(GrdB/10)
        Lambdac = c0/fc
        
        RCS = 4*pi*aCorner**4/(3*Lambdac**2)
        
        Pr = Pt/(4*pi*R**4)*Gt/(4*pi)*Gr/(4*pi)*RCS*Lambdac**2
        PrdBm = 10*log10(Pr/1e-3)
        PIfdBm = PrdBm + GcdB
        UIf = sqrt(50*10**(PIfdBm/10)*1e-3)*sqrt(2)
        UIfdB = 20*log10(UIf)        
