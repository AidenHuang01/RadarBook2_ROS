# FMCW Basics: basic mode of operation

# (1) Connect to Radarbook2
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Configure Range profile calculation and select complex range profile
# (6) Start Measurements
# (7) Display range profile or scale to RCS

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8Fft as Rbk2Adf24Tx2Rx8Fft
import  time as time
import  numpy as np
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

Disp_FrmNr  = 1
Disp_RP = 1

c0 = 299792458

if Disp_RP > 0:
    App = QtGui.QApplication([])

if Disp_RP > 0:

    WinRP = pg.GraphicsWindow(title="Range Profile")
    WinRP.setBackground((255, 255, 255))
    WinRP.resize(1000,600)

    PltRP = WinRP.addPlot(title="Range", col=0, row=0)
    PltRP.showGrid(x=True, y=True)

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
#Brd = Rbk2Adf24Tx2Rx8Fft.Rbk2Adf24Tx2Rx8Fft('RadServe', '192.168.135.70', 8000, '192.168.135.58')
Brd = Rbk2Adf24Tx2Rx8Fft.Rbk2Adf24Tx2Rx8Fft('PNet','192.168.1.1')

# Check if FFT framework is installed
Brd.BrdChkSocSysId()

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply
Brd.BrdPwrEna()

# Configure Board as Master and set ADC clock to 40 MHz
Brd.BrdSetRole('Ms', 40e6)

# Display status of board: Just an auxiliary information
Brd.BrdDispSts()

#--------------------------------------------------------------------------
# Configure Receiver and Transmitters (static setup with single Tx turned on)
#--------------------------------------------------------------------------
Brd.RfRxEna()
# Enable transmitter 1 and set power register to 80:
# Register for the output power is in the range 0-100
Brd.RfTxEna(1, 80)

#--------------------------------------------------------------------------
# Configure AFE5801
#--------------------------------------------------------------------------
Brd.Set('AfeLowNoise',0)
# Enable/Disable internal DC coupling
Brd.Set('AfeIntDcCoupling',1);
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off');
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 15);

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.00e9
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 102.4e-6
dCfg['TRampDo'] = 64e-6
dCfg['TInt'] =   100e-3
dCfg['N'] = 512
dCfg['Np']  = 1
dCfg['Tp']  = 200e-6
dCfg['IniTim'] = 10e-3
dCfg['IniEve'] = 0

Brd.Set('DmaMult', dCfg['Np'])              

#--------------------------------------------------------------------------
# Configure Range profile calculation
#--------------------------------------------------------------------------
Brd.Set('NrChn', 8)
Brd.Set('EnaFrmCntr')
Brd.Set('Range_FftSiz', 1024)       # FFTSiz: 512, 1024, 2048 supported
Brd.Set('Range_FftIdxMin', 16)       # Min; max FFT indices; Min <= k <= Max
Brd.Set('Range_FftIdxMax', 255)
Brd.Set('Range_FftSym', 1)          # select symmetrical FFT implementation
Brd.Set('Range_WinType','Hanning')  # select range window: Boxcar, Hamming, Hanning, Custom
Brd.Set('Range_WinGain', 1)         # select gain of window

Brd.Set('Mode','FftCplx')           # switch to complex fft output

Brd.RfMeas('ExtTrigUp', dCfg)

#-------------------------------------------------------------------------

#--------------------------------------------------------------------------
# Read configured values; 
# In the FPGA the CIC sampling rate reduction filter is configured
# automatically; only an integer R reduction value can be configured;
# Readback the actual sampling frequency
#--------------------------------------------------------------------------
N = int(Brd.Get('N'))
NrChn = int(Brd.Get('NrChn'))
fs = Brd.Get('fs')
fc = Brd.RfGet('fc')


Brd.BrdDispInf()
#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
# Processing of range profile
FftSiz = Brd.Get('Range_FftSiz')
FftMinIdx = Brd.Get('Range_FftIdxMin')
FftMaxIdx = Brd.Get('Range_FftIdxMax')

kf = Brd.RfGet('kf')
vFreq = np.arange(FftMinIdx,FftMaxIdx+1.)/FftSiz*fs
vRange = np.arange(FftMinIdx,FftMaxIdx+1.)/FftSiz*fs*c0/(2*kf)

# display range interval: FftMinIdx and FftMaxIdx can be used to alter
# interval
print('Minimum range: ', vRange[0], ' m')
print('Maximum range: ', vRange[-1], ' m')

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
RawDataRate = 16*NrChn*len(vRange)*dCfg['Np']/dCfg['TInt']
print('RawDataRate: ', (RawDataRate/1e6), 'MBit/s')

# TCPIP data rate
DataRate = 64*NrChn*len(vRange)*dCfg['Np']/dCfg['TInt']
print('TCpDataRate: ', (DataRate/1e6), 'MBit/s')

# Data rate required to store chirps in DDR3 memory
DataRate = 64*NrChn*len(vRange)/dCfg['Tp']
print('InstDataRate: ', (DataRate/1e6), 'MBit/s')

if Disp_RP:
    CurveRP = []
    for IdxChn in np.arange(NrChn):
        CurveRP.append(PltRP.plot(pen=pg.intColor(IdxChn, hues=NrChn)))    

for Cycles in range(0, 1000):
    
    Ret = Brd.BrdGetFftData(1)     
    
    Data = Ret[0]
    Aux = Ret[1]

    Data = 2*Data*Brd.ScaRangeFft*Brd.FuSca

    if Disp_FrmNr > 0:
        print(Aux[0,:])

    if Disp_RP > 0:
        # Calculate range profiles and display them        

        for IdxChn in np.arange(NrChn):
            CurveRP[IdxChn].setData(vRange,20*np.log10(np.abs(Data[:,IdxChn] + 1e-6)))

        
    if Disp_RP > 0:
        # Generate Event to update plots
        pg.QtGui.QApplication.processEvents()   


Brd.BrdDispInf()
    
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd
