# Calculate digital beamforming with one TX antenna

# (1) Connect to Radarbook2 with ADF24 Frontend
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements
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

# Configure script
Disp_FrmNr = 1
Disp_TimSig = 1         # display time signals
Disp_RP = 1             # display range profile
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
#Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Verify if sampling framework is installed
Brd.BrdChkSocSysId();

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply 
Brd.BrdPwrEna()

# Use 40 MHz clock for the ADC (AFE5801)
Brd.BrdSetRole('Ms', 40e6)

#--------------------------------------------------------------------------
# Load Calibration Data
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 16
CalData = Brd.BrdGetCalData(dCalCfg)

#--------------------------------------------------------------------------
# Configure RF Transceivers
#--------------------------------------------------------------------------
Brd.RfRxEna()
Brd.RfTxEna(1, 80)

#--------------------------------------------------------------------------
# Configure AFE5801
#--------------------------------------------------------------------------
# Enable/Disable internal DC coupling
Brd.Set('AfeIntDcCoupling',0)
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20)

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.00e9        
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 256e-6
dCfg['TRampDo'] = 64e-6
dCfg['TInt'] = 200e-3
dCfg['N'] = 1024
dCfg['IniTim'] = 100e-3                  
dCfg['IniEve'] = 0                      # Start automatically after IniTim

Brd.RfMeas('ExtTrigUp',dCfg);
#--------------------------------------------------------------------------
# Read actual configuration
#--------------------------------------------------------------------------
NrChn = int(Brd.Get('NrChn'))
N = int(Brd.Get('N'))
fs = Brd.Get('fs')

#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
# Processing of range profile
Win2D = Brd.hanning(N-1,NrChn)
ScaWin = sum(Win2D[:,0])
NFFT = 2**12
kf = (dCfg['fStop'] - dCfg['fStrt'])/dCfg['TRampUp']
vRange = np.arange(NFFT//2)/NFFT*fs*c0/(2*kf)

RMin = 1
RMax = 10
RMinIdx = np.argmin(np.abs(vRange - RMin))
RMaxIdx = np.argmin(np.abs(vRange - RMax))
vRangeExt = vRange[RMinIdx:RMaxIdx]

# Window function for receive channels
NFFTAnt = 256
WinAnt2D = Brd.hanning(NrChn, len(vRangeExt))
ScaWinAnt = np.sum(WinAnt2D[:,0])
WinAnt2D = WinAnt2D.transpose()
vAngDeg  = np.arcsin(2*np.arange(-NFFTAnt//2, NFFTAnt//2)/NFFTAnt)/np.pi*180

mCalData = np.matlib.repmat(CalData, N-1,1)

#--------------------------------------------------------------------------
# Generate Curves for plots
#--------------------------------------------------------------------------
if Disp_TimSig:
    n = np.arange(int(N))
    CurveTim = []
    for IdxChn in np.arange(NrChn):
        CurveTim.append(PltTim.plot(pen=pg.intColor(IdxChn, hues=NrChn)))

if Disp_RP:
    CurveRP = []
    for IdxChn in np.arange(NrChn):
        CurveRP.append(PltRP.plot(pen=pg.intColor(IdxChn, hues=NrChn)))   



#--------------------------------------------------------------------------
# Measure and calculate DBF
#--------------------------------------------------------------------------
for Cycles in range(0, 100):

    Data = Brd.BrdGetData(1);

    if Disp_FrmNr > 0:
        # Show Frame number:
        # The board can buffer multiple frames but if the communication
        # speed is too low, or the time between calls of BrdGetData is
        # higher than TInt, then frames will be lost if the FIFOs in the
        # RBK2 overflow.
        # Use the framecounter to check if no data is missing
        print(Data[0,:])
   
    # Remove Framenumber from processing
    Data = Data[1:,:]
    
    if Disp_TimSig > 0:      
        # Display time domain signals
        for IdxChn in np.arange(NrChn):
            CurveTim[IdxChn].setData(n[1:],Data[:,IdxChn])

    # Calculate range profiles and display them
    RP = 2*np.fft.fft(Data[:,:]*Win2D*mCalData, n=NFFT, axis=0)/ScaWin*Brd.FuSca
    RP = RP[RMinIdx:RMaxIdx,:]
    
    if Disp_RP> 0:
        for IdxChn in np.arange(NrChn):
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



Brd.BrdDispInf()
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd