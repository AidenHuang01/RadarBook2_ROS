# FMCW Basics: Range-Doppler processing.
# Configure multiple chirp and read back range profiles from RBK2
# FFT framework is used to calculate the complex valued range profiles
# The scripts shows how to configure the windowed FFT calculation
# Velocity FFT is calculated with Matlab

# (1) Connect to Radarbook2
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Configure Range profile calculation and select complex range profile
# (6) Start Measurements
# (7) Calculate Velocity FFT for a single channel and display with Matlab

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8Fft as Rbk2Adf24Tx2Rx8Fft
import  time as time
import  numpy as np
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

Disp_FrmNr = 1
Disp_RP = 0
Disp_RD = 1    


c0 = 299792458


if Disp_RP > 0 or Disp_RD > 0:
    App = QtGui.QApplication([])

if Disp_RP > 0:

    WinRP = pg.GraphicsWindow(title="Range Profile")
    WinRP.setBackground((255, 255, 255))
    WinRP.resize(1000,600)

    PltRP = WinRP.addPlot(title="Range", col=0, row=0)
    PltRP.showGrid(x=True, y=True)

if Disp_RD:
    View = pg.PlotItem(title='Cross Range Plot')
    View.setLabel('left', 'R', units='m')
    View.setLabel('bottom', 'v', units='m/s')

    Img = pg.ImageView(view=View)
    Img.show()
    Img.ui.roiBtn.hide()
    Img.ui.menuBtn.hide()
    #Img.ui.histogram.hide()
    Img.getHistogramWidget().gradient.loadPreset('flame')

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
Brd.Set('AfeIntDcCoupling', 0);
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off');
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20);

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.00e9
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 102.4e-6
dCfg['TRampDo'] = 64e-6
dCfg['TInt'] =   250e-3
dCfg['N'] = 256
dCfg['Np'] = 32
dCfg['Tp'] = 200e-6
dCfg['IniTim'] = 200e-3
dCfg['IniEve'] = 0

# Record NLoop channels with one DMA transfer/ required for high data
# throuput
Brd.Set('DmaMult',dCfg['Np'])

#--------------------------------------------------------------------------
# Configure Range profile calculation
#--------------------------------------------------------------------------
Brd.Set('NrChn', 8)
Brd.Set('EnaFrmCntr')
Brd.Set('Range_FftSiz', 1024)       # FFTSiz: 512, 1024, 2048 supported
Brd.Set('Range_FftIdxMin', 0)       # Min; max FFT indices; Min <= k <= Max
Brd.Set('Range_FftIdxMax', 255)
Brd.Set('Range_FftSym', 1)          # select symmetrical FFT implementation
Brd.Set('Range_WinType','Hanning')  # select range window: Boxcar, Hamming, Hanning, Custom
Brd.Set('Range_WinGain', 1)         # select gain of window
Brd.Set('Mode','FftCplx');          # switch to complex fft output

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

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
DataRate = 64*NrChn*dCfg['N']/dCfg['TInt']
print('DataRate: ', (DataRate/1e6), ' MBit/s')

Brd.BrdDispInf();

Range_FftSiz = int(Brd.Get('Range_FftSiz'))
FftMinIdx = int(Brd.Get('Range_FftIdxMin'))
FftMaxIdx = int(Brd.Get('Range_FftIdxMax'))

kf = Brd.RfGet('kf');
vFreq = np.arange(FftMinIdx,FftMaxIdx + 1)/Range_FftSiz*fs
vRange = np.arange(FftMinIdx,FftMaxIdx + 1)/Range_FftSiz*fs*c0/(2*kf)

Range_N = int(FftMaxIdx - FftMinIdx + 1)
# display range interval: FftMinIdx and FftMaxIdx can be used to alter
# interval
print('Minimum range: ', vRange[0], ' m')
print('Maximum range: ', vRange[-1], ' m')

Vel_FftSiz = 256
Vel_Win2D = Brd.hanning(int(dCfg['Np']), Range_N)
Vel_ScaWin = sum(Vel_Win2D[:,0])
Vel_Win2D = Vel_Win2D.transpose()

vFreqVel = np.arange(-Vel_FftSiz//2,Vel_FftSiz//2)/Vel_FftSiz*(1/dCfg['Tp'])
vVel = vFreqVel*c0/(2*fc)

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
# TCPIP data rate
DataRate = 64*NrChn*len(vRange)*dCfg['Np']/dCfg['TInt']
print('TcpDataRate: ', (DataRate/1e6), ' MBit/s')

# Data rate required to store chirps in DDR3 memory
DataRate = 64*NrChn*len(vRange)/dCfg['Tp']
print('InstDataRate: ', DataRate/1e6, ' MBit/s')

PadIdxStrt = int((Vel_FftSiz - dCfg['Np'])/2)
PadIdxStop = int(PadIdxStrt + dCfg['Np'])

if Disp_RP:
    CurveRP = []
    for IdxChn in np.arange(NrChn):
        CurveRP.append(PltRP.plot(pen=pg.intColor(IdxChn, hues=NrChn)))    

for Cycles in range(0, 1000000):
    
    Ret = Brd.BrdGetFftData(int(dCfg['Np']))     
    
    Data = Ret[0]
    Aux = Ret[1]

    print(Aux[0,:])

    if Disp_RP > 0:
        for IdxChn in np.arange(NrChn):
            CurveRP[IdxChn].setData(20*np.log10(1+np.abs(Data[0:Range_N,IdxChn])))

    Chn1 = np.reshape(Data[:,0],(Range_N, int(dCfg['Np'])), order='F')
    Chn1Pad = np.zeros((Range_N, Vel_FftSiz), dtype='complex')
    Chn1Pad[:,PadIdxStrt:PadIdxStop] = Chn1*Vel_Win2D
    RD1 = np.fft.fftshift(np.fft.fft(np.fft.fftshift(Chn1Pad,axes=1), Vel_FftSiz, axis=1)/Vel_ScaWin, axes=1)
    RDMap1 = 20*np.log10(1+np.abs(RD1))    
        
    if Disp_RP > 0:
        # Generate Event to update plots
        pg.QtGui.QApplication.processEvents()   

    if Disp_RD > 0:

        Img.setImage(RDMap1.transpose(), pos=[vVel[0],vRange[0]], scale=[(vVel[-1] - vVel[0])/Vel_FftSiz,(vRange[-1] - vRange[0])/len(vRange)]) 
        
        View.setAspectLocked(False)    

    pg.QtGui.QApplication.processEvents()

Brd.BrdDispInf()
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd
