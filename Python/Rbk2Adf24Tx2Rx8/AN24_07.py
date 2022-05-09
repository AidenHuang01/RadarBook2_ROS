# Description:
# Configure FMCW Mode with sequential activation of Tx antennas and measure upchirp IF signal
#
# (1) Connect to Radarbook2 with ADF24 Frontend
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements with TX sequence
# (6) Configure signal processing
# (7) Calculate DBF algorithm

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy as np
import  numpy.matlib
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

Disp_FrmNr  =   1
Disp_TimSig = 0         # display time signals
Disp_RP = 0             # display range profile
Disp_JOpt = 1           # display cost function for DBF

c0 = 299792458


if Disp_TimSig > 0 or Disp_RP > 0 or Disp_JOpt > 0:
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

if Disp_JOpt:
    View = pg.PlotItem(title='Cross Range Plot')
    View.setLabel('left', 'R', units='m')
    View.setLabel('bottom', 'u')

    Img = pg.ImageView(view=View)
    Img.show()
    Img.ui.roiBtn.hide()
    Img.ui.menuBtn.hide()
    #Img.ui.histogram.hide()
    Img.getHistogramWidget().gradient.loadPreset('flame')

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
#Brd     =   Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')

Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Verify if sampling framework is installed
Brd.BrdChkSocSysId()

#--------------------------------------------------------------------------
# Reset Board and Enable Power Supply
#--------------------------------------------------------------------------
Brd.BrdRst()
Brd.BrdPwrEna()

#--------------------------------------------------------------------------
# Software Version
#--------------------------------------------------------------------------
Brd.BrdDispSwVers()

#--------------------------------------------------------------------------
# Status Information
#--------------------------------------------------------------------------
Brd.BrdDispInf()


# Configure Board as Master and set ADC clock to 40 MHz
Brd.BrdSetRole('Ms', 40e6)

#--------------------------------------------------------------------------
# Load Calibration Data for the entire array
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 32
CalData = Brd.BrdGetCalData(dCalCfg)

#--------------------------------------------------------------------------
# Enable Receive Chips
#--------------------------------------------------------------------------
Brd.RfRxEna()
Brd.RfTxEna(1, 80)

#--------------------------------------------------------------------------
# Configure AFE5801
#--------------------------------------------------------------------------
Brd.Set('AfeLowNoise',0)
# Enable/Disable internal DC coupling
Brd.Set('AfeIntDcCoupling',0)
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20)

#--------------------------------------------------------------------------
# Configure Up-Chirp with a sequence
# SPI Command is required to configure TX to different antenna the spi
# command takes < 10 us;
# 
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.0e9
dCfg['fStop'] = 24.2e9
dCfg['TRampUp'] = 100e-6
dCfg['TInt'] = 200e-3
dCfg['Tp'] = 120e-6
dCfg['N'] = 1000
dCfg['IniTim'] = 100e-3                  
dCfg['IniEve'] = 0                      # Start automatically after IniTim
dCfg['TxSeq'] = [1, 2]                  # Activate 1 and then antenna2
dCfg['Np'] = 1                          # repeat sequence Np times

#--------------------------------------------------------------------------
# Configure DMA Transfer to copy numel(dCfg['TxSeq'])*Np frames simultaneously.
# Required to achiev maximal data transfer between FPGA and Soc 
Brd.Set('DmaMult', len(dCfg['TxSeq'])*dCfg['Np'])

Brd.RfMeas('ExtTrigUp_TxSeq',dCfg)

#--------------------------------------------------------------------------
# Read Settings for N and fs
#--------------------------------------------------------------------------
N = int(Brd.Get('N'))
fs = Brd.Get('fs')
NrChn = int(Brd.Get('NrChn'))
#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
# Processing of range profile
NFFT = 2**12

Win2D = Brd.hanning(N-1,2*NrChn-1)
ScaWin = np.sum(Win2D[:,0])
kf = Brd.RfGet('kf')
vRange = np.arange(NFFT)/NFFT*fs*c0/(2*kf)

# Configure range interval to be displayed
RMin = 1
RMax = 10

RMinIdx = np.argmin(np.abs(vRange - RMin))
RMaxIdx = np.argmin(np.abs(vRange - RMax))
vRangeExt = vRange[RMinIdx:RMaxIdx]

# Window function for receive channels
NFFTAnt = 256
WinAnt2D = Brd.hanning(2*NrChn-1, len(vRangeExt))
ScaWinAnt = np.sum(WinAnt2D[:,0])
WinAnt2D = WinAnt2D.transpose()
vAngDeg  = np.arcsin(2*np.arange(-NFFTAnt//2, NFFTAnt//2)/NFFTAnt)/np.pi*180


CalData = np.concatenate((CalData[0:NrChn], CalData[NrChn+1:]))
mCalData = np.matlib.repmat(CalData, N-1,1)

#--------------------------------------------------------------------------
# Generate Curves for plots
#--------------------------------------------------------------------------
if Disp_TimSig:
    n = np.arange(int(N))
    CurveTim = []
    for IdxChn in np.arange(2*NrChn-1):
        CurveTim.append(PltTim.plot(pen=pg.intColor(IdxChn, hues=2*NrChn-1)))

if Disp_RP:
    CurveRP = []
    for IdxChn in np.arange(2*NrChn-1):
        CurveRP.append(PltRP.plot(pen=pg.intColor(IdxChn, hues=2*NrChn-1))) 


#--------------------------------------------------------------------------
# Measure and calculate DBF
#--------------------------------------------------------------------------
for Cycles in range(0, 1000):
    
    # Record data for Tx1 and Tx2
    Data = Brd.BrdGetData(2); 

    if Disp_FrmNr > 0:
        # Framenumber is used to check measurement sequence.
        # Odd Framenumbers are for TX1 and even frame numbers for TX2
        # If a frame is missing: DBF processing will fail!!
        print(Data[0,:])

    # Format data for virtual array and remove overlapping element
    DataV = np.concatenate((Data[1:N,:], Data[N+1:,1:]), axis=1)
    #DataV = DataA(2:end,AntIdx);
    
    # Calculate range profile including calibration
    RP = 2*np.fft.fft(DataV[:,:]*Win2D*mCalData, n=NFFT, axis=0)/ScaWin*Brd.FuSca
    RP = RP[RMinIdx:RMaxIdx,:]
    
    if Disp_TimSig > 0:      
        # Display time domain signals
        for IdxChn in np.arange(2*NrChn-1):
            CurveTim[IdxChn].setData(n[1:],DataV[:,IdxChn])

    if Disp_RP> 0:
        for IdxChn in np.arange(2*NrChn-1):
            CurveRP[IdxChn].setData(vRangeExt,20*np.log10(np.abs(RP[:,IdxChn])))

    if Disp_JOpt > 0:
        JOpt = np.fft.fftshift(np.fft.fft(RP*WinAnt2D, NFFTAnt, axis=1)/ScaWinAnt, axes=1)
        
        JdB = 20*np.log10(np.abs(JOpt))
        JMax = np.max(JdB)
        JNorm = JdB - JMax
        JNorm[JNorm < -25] = -25
    
        Img.setImage(JNorm.T, pos=[-1,RMin], scale=[2.0/NFFTAnt,(RMax - RMin)/vRangeExt.shape[0]]) 
        View.setAspectLocked(False)   


    pg.QtGui.QApplication.processEvents()
    

Brd.BrdRst()
Brd.BrdPwrDi()

del Brd



