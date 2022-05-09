# FMCW Basics with two systems synchronized;
# Master triggers a single chirp and the synchronisation impulse is
# forwarded to the slave system that conducts a synchronized measurement


# (1) Connect to Radarbook2 Master and Slave
# (2) Set Role
# (3) Configure RX
# (4) Configure TX
# (5) Start Synchronized Measurements

#--------------------------------------------------------------------------
# Include all necessary directories
#--------------------------------------------------------------------------
import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy as np
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

Disp_FrmNr = 1
Disp_TimSig = 0
Disp_RP = 1

c0 = 3e8

if Disp_TimSig > 0 or Disp_RP > 0:
    App = QtGui.QApplication([])

if Disp_TimSig > 0:
    

    WinTim = pg.GraphicsWindow(title="Time signals")
    WinTim.setBackground((255, 255, 255))
    WinTim.resize(1000,600)

    PltTim = WinTim.addPlot(title="TimSig", col=0, row=0)
    PltTim.showGrid(x=True, y=True)

if Disp_RP > 0:

    WinRP = pg.GraphicsWindow(title="Range Profile")
    WinRP.setBackground((255, 255, 255))
    WinRP.resize(1000,600)

    PltRP = WinRP.addPlot(title="Range", col=0, row=0)
    PltRP.showGrid(x=True, y=True)
#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
BrdMs = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')
BrdSl = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.2')

BrdMs.BrdRst()
BrdMs.BrdPwrEna()

# Roles for Master: 
# (1) Ms: Board acts as master: no signals are output on USB syncn
# connector
# (2) MsClkOut: Clock and RampSyncn signal are output on USB syncn
# connector
BrdMs.BrdSetRole('MsClkOut', 40e6)

# Roles for Slave: 
# (1) Sl: Board acts as slave: measurement waits for trigger signal on
# external ramp syncn signal
# (2) SlClkIn: Board acts as slave: measurements are from external trigger
# signal and clock is derived from external connector (fully synchronized)

BrdSl.BrdRst()
BrdSl.BrdPwrEna()
BrdSl.BrdSetRole('SlClkIn', 40e6)

#--------------------------------------------------------------------------
# Configure Receiver and Transmitters (static setup with single Tx turned on)
#--------------------------------------------------------------------------
BrdMs.RfRxEna()
BrdMs.RfTxEna(1, 60)

BrdSl.RfRxEna()
BrdSl.RfTxEna(1, 60)

#--------------------------------------------------------------------------
# Configure AFE5801
#--------------------------------------------------------------------------
BrdMs.Set('AfeLowNoise',0)
BrdMs.Set('AfeIntDcCoupling',0)
BrdMs.Set('AfeGaindB',20)

BrdSl.Set('AfeLowNoise',0)
BrdSl.Set('AfeIntDcCoupling',0)
BrdSl.Set('AfeGaindB',20)

#--------------------------------------------------------------------------
# Configure Up-Chirp and timing
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.0e9         
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 2*51.2e-6
dCfg['TInt'] = 500e-3               # duration between two measurements
dCfg['N'] = 4096
dCfg['IniTim'] = 10e-3                  
dCfg['IniEve'] = 0                  # Start automatically after IniTim
dCfg['CfgTim'] = 14e-6              # use configuration time to shift the start of the chrips
dCfg['Tp'] = 400e-6
dCfg['Np'] = 2

BrdSl.Set('DmaMult', dCfg['Np'])
BrdMs.Set('DmaMult', dCfg['Np'])

BrdSl.ConSet('Mult', dCfg['Np'])
BrdMs.ConSet('Mult', dCfg['Np'])
#--------------------------------------------------------------------------
# Use FPGA triggered measurement mode:
# FPGA generates timing and triggers ADF4159
# The timing is generated in multiples of ADC clock cycles. The default
# clock is set to 20 MHz.
#--------------------------------------------------------------------------
# Required to generate a copy of the dCfg dict
BrdSl.RfMeas('ExtTrigUp',dCfg.copy())

dCfg['CfgTim'] = 10e-6
BrdMs.RfMeas('ExtTrigUp',dCfg.copy())

DataRate = 8*16*dCfg['N']/dCfg['TInt']

N = int(BrdMs.Get('N'))
NrChn = int(BrdMs.Get('NrChn'))
fs = BrdMs.Get('fs')

#--------------------------------------------------------------------------
# Configure Signal processing
#--------------------------------------------------------------------------
NFFT = 2**14

Win = BrdMs.hanning(N-1,NrChn)
ScaWin = np.sum(Win[:,0])

kf = (dCfg['fStop'] - dCfg['fStrt'])/dCfg['TRampUp']
vRange = np.arange(NFFT//2)/NFFT*fs*c0/(2*kf)
vFreq = np.arange(NFFT//2)/NFFT*fs

print('DataRate: ', (DataRate/1e6), 'MBit/s')

#--------------------------------------------------------------------------
# Generate Curves for plots
#--------------------------------------------------------------------------
if Disp_TimSig:
    n = np.arange(int(N))
    CurveTim = []
    for IdxChn in np.arange(int(2*NrChn)):
        CurveTim.append(PltTim.plot(pen=pg.intColor(IdxChn, hues=2*NrChn)))

if Disp_RP:
    CurveRP = []
    for IdxChn in np.arange(2*NrChn):
        CurveRP.append(PltRP.plot(pen=pg.intColor(IdxChn, hues=2*NrChn)))    

BrdMs.GetDataPort()
BrdSl.GetDataPort()



for Cycles in range(0,200000):
    
    DataMs      =   BrdMs.BrdGetData(1) 
    DataSl      =   BrdSl.BrdGetData(1)
    
    if Disp_FrmNr > 0:
        FrmCntrMs = DataMs[0,:]
        FrmCntrSl = DataSl[0,:]
        print("Ms: ", FrmCntrMs)
        print("Sl: ", FrmCntrSl)

    if Disp_TimSig > 0:
        # Display time domain signals
        for IdxChn in np.arange(NrChn):
            CurveTim[IdxChn].setData(n[1:],DataMs[1:,IdxChn])
        for IdxChn in np.arange(NrChn):
            CurveTim[IdxChn+NrChn].setData(n[1:],DataSl[1:,IdxChn])            

    if Disp_RP > 0:
        # Calculate range profiles and display them

        RPMs = 2*np.fft.fft(DataMs[1:,:]*Win, n=NFFT, axis=0)/ScaWin*BrdMs.FuSca
        RPMs = RPMs[:NFFT//2,:]
        RPSl = 2*np.fft.fft(DataSl[1:,:]*Win, n=NFFT, axis=0)/ScaWin*BrdSl.FuSca
        RPSl = RPSl[:NFFT//2,:]

        for IdxChn in np.arange(NrChn):
            CurveRP[IdxChn].setData(vFreq[0:],20*np.log10(np.abs(RPMs[0:,IdxChn])))
        for IdxChn in np.arange(NrChn):
            CurveRP[IdxChn+NrChn].setData(vFreq[0:],20*np.log10(np.abs(RPSl[0:,IdxChn])))

    if Disp_TimSig > 0 or Disp_RP > 0:
        # Generate Event to update plots
        pg.QtGui.QApplication.processEvents()


BrdMs.BrdDispInf()
BrdMs.BrdRst()
BrdMs.BrdPwrDi()


BrdMs.BrdDispInf()
BrdMs.BrdRst()
BrdMs.BrdPwrDi()


del BrdMs
del BrdSl