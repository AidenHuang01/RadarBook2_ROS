# Configure range-Doppler processing and return point cloud
# Calculate range-Doppler maps for all channels and extract point cloud


# (1) Connect to Radarbook2
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8Fft as Rbk2Adf24Tx2Rx8Fft
import  time as time
import  numpy as np
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg


App = QtGui.QApplication([])

ViewRD = pg.PlotWidget()
ViewRD.resize(800, 600)
ViewRD.setLabel('left', 'R', units='m')
ViewRD.setLabel('bottom', 'v', units='m/s')
ViewRD.show()

ScatterRD = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='o', size=1)
ViewRD.addItem(ScatterRD)

ViewPosn = pg.PlotWidget()
ViewPosn.resize(800, 600)
ViewPosn.setLabel('left', 'y', units='m')
ViewPosn.setLabel('bottom', 'x', units='m')
ViewPosn.show()

ScatterPosn = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='o', size=1)
ViewPosn.addItem(ScatterPosn)

UseCal = 1

c0 = 2.99792458e8

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8Fft.Rbk2Adf24Tx2Rx8Fft('PNet','192.168.1.1')

# Check if FFT framework is installed
Brd.BrdChkSocSysId()

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply 
Brd.BrdPwrEna()

# Use 40 MHz clock for the ADC (AFE5801)
# Brd.BrdSetRole('Ms', 20e6); to configure 20 MHz clock
Brd.BrdSetRole('Ms', 40e6)

Brd.BrdDispSts()

#--------------------------------------------------------------------------
# Load Calibration Data
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 32
CalData = Brd.BrdGetCalData(dCalCfg)

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
Brd.Set('AfeIntDcCoupling',  0)
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 31)

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.00e9
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 102.4e-6
dCfg['TInt'] = 75e-3
dCfg['N'] = 1024
dCfg['Np'] = 256
dCfg['Tp'] = 120e-6
dCfg['IniTim'] = 100e-3
dCfg['IniEve'] = 0

#--------------------------------------------------------------------------
# Configure Range profile calculation
#--------------------------------------------------------------------------
Brd.Set('NrChn', 8)
Brd.Set('EnaFrmCntr')
Brd.Set('Range_FftSiz', 1024)               # FFT size, 64, 128, 256, 512, 1024, 2048 supported
Brd.Set('Range_FftIdxMin', 32)              # Min; max FFT indices; Min <= k <= Max
Brd.Set('Range_FftIdxMax', 511)
Brd.Set('Range_FftSym', 1)                  # select symmetrical FFT implementation; FFTshift = 1
Brd.Set('Range_WinType','Hanning')          # select range window: Boxcar, Hamming, Hanning 
Brd.Set('Range_WinGain', 2)                 # select range window: Boxcar, Hamming, Hanning 

#--------------------------------------------------------------------------
# Configure Velocity calculation for Range-Doppler map
#--------------------------------------------------------------------------
Brd.Set('RD_NrChirps', dCfg['Np'])
Brd.Set('Vel_FftSiz', 512)
Brd.Set('Vel_FftSym', 1);                   # select symmetrical FFT implementation; FFTshift = 1
Brd.Set('Vel_WinType','Hanning')
Brd.Set('Vel_WinGain', 2)

Vel_FftSiz = Brd.Get('Vel_FftSiz')
#--------------------------------------------------------------------------
# Configure Local maximum detection
#--------------------------------------------------------------------------
Brd.Set('Det_VelIdxMin0', 8)                    # select indices for detection 0 to Vel_FftSiz - 1 (all velocity bins)
Brd.Set('Det_VelIdxMax0', Vel_FftSiz-8)
Brd.Set('Det_MaxNrDet', 200)                    # Maximum number of detections (256)
Brd.Set('Det_Thres', 2.4)                       # select threshold
Brd.Set('Det_InGain', 2)                        # input gain for magnitude calculation
Brd.Set('Det_ThresCalc_LimMin', 8)
Brd.Set('Det_ThresCalc_LimMax', Vel_FftSiz - 8)

Brd.Set('Mode','LoMax')                         # switch to complex fft output

Brd.RfMeas('ExtTrigUp', dCfg)

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

Det_MaxNrDet = Brd.Get('Det_MaxNrDet')
Vel_FftSiz = Brd.Get('Vel_FftSiz')
Brd.BrdDispInf()

#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
# Processing of range profile
FftSiz = Brd.Get('Range_FftSiz')
FftMinIdx = Brd.Get('Range_FftIdxMin')
FftMaxIdx = Brd.Get('Range_FftIdxMax')

kf = Brd.RfGet('kf')
fc = Brd.RfGet('fc')

vFreq = np.arange(FftMinIdx,FftMaxIdx+1)/FftSiz*fs
vRange = np.arange(FftMinIdx,FftMaxIdx+1)/FftSiz*fs*c0/(2*kf)
NrRangeBins = len(vRange)

vFreqVel = np.arange(-Vel_FftSiz/2,Vel_FftSiz/2)/Vel_FftSiz*1/dCfg['Tp']
vVel = vFreqVel*c0/(2*fc) 
vVel = np.fft.fftshift(vVel)

Ang_FftSiz = 256
WinAng2D = Brd.hanning(NrChn, Det_MaxNrDet)

if UseCal > 0:
    mCalDat = np.tile(CalData,(Det_MaxNrDet,1))
    mCalDat = mCalDat.transpose()
else:
    mCalDat = 1


vAng = np.arcsin(2*np.arange(-Ang_FftSiz//2,Ang_FftSiz//2)/Ang_FftSiz)

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
RawDataRate = 16*NrChn*N*dCfg['Np']/dCfg['TInt']
print('RawDataRate:   ', (RawDataRate/1e6), ' MBit/s')
DataRate = 64*NrChn*len(vRange)*dCfg['Np']/dCfg['TInt']
print('RpDataRate:    ', (DataRate/1e6), ' MBit/s')
InstDataRate = 64*NrChn*len(vRange)/dCfg['Tp']
print('InstDataRate:  ', (InstDataRate/1e6), ' MBit/s')
TcpDataRate = 64*Det_MaxNrDet*40/dCfg['TInt']
print('TcpDataRate:   ', (TcpDataRate/1e6), ' MBit/s')

TMeas = dCfg['Tp']*dCfg['Np']
print('TMeas:         ', (TMeas/1e-3), ' ms')

FrmNrOld = 1

for Idx in range(2000000):
    
    # LoMax data fields
    # FrmCntr: [NrDet x 1 double]       
    # Mag: [NrDet x 1 double]               # magnitude sum of all channels
    # Thres: [NrDet x 1 double]             # threshold for detection
    # VelIdx: [NrDet x 1 double]            # ColIdx: position in RDMap velocity
    # RangeIdx: [NrDet x 1 double]          # RangeIdx: position in RDMap range 
    # MagNeighbors: [NrDet x 4 double]      # Neighbors in Mag [RangeIdx + 1, RangeIdx - 1, VelIdx + 1, VelIdx - 1]
    # CplxAmp: [NrDet x NrChn double]       # Complex amplitudes
    
    dLoMax = Brd.BrdGetLoMax()
    NrDet = len(dLoMax['FrmCntr'])
    
    if NrDet > 0:
        print(dLoMax['FrmCntr'][0])
        Range = vRange[dLoMax['RangeIdx']]
        Vel = vVel[dLoMax['VelIdx']]
        
        
        MagAng = np.abs(np.fft.fftshift(np.fft.fft(dLoMax['CplxAmp']*WinAng2D[:,0:NrDet]*mCalDat[0:NrChn,0:NrDet],Ang_FftSiz,axis=0),0))
        MaxIdcs = np.argmax(MagAng,axis=0)
        

        dLoMax['Ang'] = vAng[MaxIdcs];
        VelPos = np.argwhere(Vel > 0.5)
        VelNeg = np.argwhere(Vel < -0.5)
        VelStat = np.argwhere(abs(Vel) < 0.2)
        
  
        ScatterRD.setData(Vel, Range)
        ScatterPosn.setData(Range*np.sin(dLoMax['Ang']), Range*np.cos(dLoMax['Ang']))


    if (Idx % 100) == 0:
        Dur = Brd.BrdDispProcTim();

    pg.QtGui.QApplication.processEvents()
    


