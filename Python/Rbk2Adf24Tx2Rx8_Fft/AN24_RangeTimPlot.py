#FMCW Basics: basic mode of operation configure up chirp and return the
#range profile; Plot the magnitude rane profile over time

#(1) Connect to Radarbook2
#(2) Enable Supply
#(3) Configure RX
#(4) Configure TX
#(5) Configure Range profile calculation and select complex range profile
#(6) Start Measurements
#(7) Display range profile over time

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8Fft as Rbk2Adf24Tx2Rx8Fft
import  time as time
import  numpy as np
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

#Configure script
Disp_FrmNr = 1   


c0 = 2.99792458e8

App = QtGui.QApplication([])

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
#Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8Fft.Rbk2Adf24Tx2Rx8Fft('PNet','192.168.1.1')

#Check if FFT framework is installed
Brd.BrdChkSocSysId()

#Reset board: resets timing unit in case a measurement has been configured
#previously
Brd.BrdRst()
#Enable RF power supply and wait for supply 
Brd.BrdPwrEna()

#Use 40 MHz clock for the ADC (AFE5801)
Brd.BrdSetRole('Ms', 40e6)

Brd.BrdDispSts()

#--------------------------------------------------------------------------
#Configure Receiver and Transmitters (static setup with single Tx turned on)
#--------------------------------------------------------------------------
Brd.RfRxEna()
#Enable transmitter 1 and set power register to 80:
#Register for the output power is in the range 0-100
Brd.RfTxEna(1, 80)

#--------------------------------------------------------------------------
#Configure AFE5801
#--------------------------------------------------------------------------
Brd.Set('AfeLowNoise',0)
#Enable/Disable internal DC coupling
Brd.Set('AfeIntDcCoupling',  0)
#Ramp pattern can be used to test communication: sample data is replaced
#by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
#Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20)

#--------------------------------------------------------------------------
#Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()

dCfg['fStrt'] = 24.00e9
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 64e-6
dCfg['TRampDo'] = 20e-6
dCfg['TInt'] = 20e-3
dCfg['N'] = 1024
dCfg['Np'] = 1
dCfg['Tp'] = 200e-6
dCfg['IniTim'] = 200e-3
dCfg['IniEve'] = 0

Brd.Set('DmaMult', dCfg['Np'])            

#--------------------------------------------------------------------------
#Configure Range profile calculation
#--------------------------------------------------------------------------
Brd.Set('NrChn', 8)
Brd.Set('EnaFrmCntr')
Brd.Set('Range_FftSiz', 1024)       #FFTSiz: 512, 1024, 2048 supported
Brd.Set('Range_FftIdxMin', 0)       #Min; max FFT indices; Min <= k <= Max
Brd.Set('Range_FftIdxMax', 511)
Brd.Set('Range_FftSym', 1)          #select symmetrical FFT implementation
Brd.Set('Range_WinType','Hanning')  #select range window: Boxcar, Hamming, Hanning, Custom
Brd.Set('Range_WinGain', 1)         #select gain of window

Brd.Set('Mode','FftCplx')           #switch to complex fft output

Brd.RfMeas('ExtTrigUp', dCfg)

#--------------------------------------------------------------------------
#Read configured values; 
#In the FPGA the CIC sampling rate reduction filter is configured
#automatically; only an integer R reduction value can be configured;
#Readback the actual sampling frequency
#--------------------------------------------------------------------------
N = Brd.Get('N')
NrChn = Brd.Get('NrChn')
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

NHist = 100
nHist = np.arange(1,NHist+1.)
DataHist = np.zeros((len(vRange), NHist))

for Cycles in range(0, 1000):
    
    #Read data for one chirp and all enabled channels
    #Data is a N x NrChn 2D array containing the raw data in multiples of
    #the ADC LSB.
    #The values in the FPGA are in a 12.4 integer number format and
    #converted to a double in the Matlab class
    
    #Multiple Frames can be read with setting the argument > 1

    Ret = Brd.BrdGetFftData(1)     
    
    Data = Ret[0]
    Aux = Ret[1]
    

    if Disp_FrmNr > 0:
        print(Aux[0,:])
    
    Data = Data*Brd.ScaRangeFft*Brd.FuSca
    Data = np.mean(np.abs(Data), axis=1)
    
    DataHist[:,0:-1] = DataHist[:,1:]
    DataHist[:,-1] = 20*np.log10(abs(Data));
    
    Img.setImage(DataHist.transpose(), pos=[nHist[0],vRange[0]], scale=[(nHist[-1] - nHist[0])/NHist,(vRange[-1] - vRange[0])/len(vRange)])  
    View.setAspectLocked(False)    

    pg.QtGui.QApplication.processEvents()


Brd.BrdDispInf()
Brd.BrdRst()
Brd.BrdPwrDi()

