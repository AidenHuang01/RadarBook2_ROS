# This application note shows how to extract detections from the
# measurements
# This appnote only works with one TX antenna enabled.
# First the range-Doppler maps are calculated for all the enabled channels
# and in the next step a detection process is carried out and then the
# angles for the detections are calculated and returned as list of
# detections to python.

# (1) Connect to RadarLog
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements

#--------------------------------------------------------------------------
# Include all necessary directories
#--------------------------------------------------------------------------
import sys, os
sys.path.append("../")

import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy
from    numpy import *

from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

#--------------------------------------------------------------------------
# Configure Script
#--------------------------------------------------------------------------
Disp_TT = 1
# Display Video (only possible if camera is enabled in RadServe)
Disp_Video = 0

if Disp_TT > 0:
    App = QtGui.QApplication([])
    
    viewWin = QtGui.QMainWindow()
    view = pg.GraphicsLayoutWidget()
    viewWin.setCentralWidget(view)
    viewWin.show()
    viewWin.setWindowTitle('Target Tracker')
    viewPlot = view.addPlot()
    viewPlot.setXRange(-150, 150, padding=0)
    viewPlot.setYRange(0, 200, padding=0)
    viewDet = pg.ScatterPlotItem(size=10, pen=None, pxMode=True)
    viewPlot.addItem(viewDet)

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply
Brd.BrdPwrEna()

#--------------------------------------------------------------------------
# Load Calibration Data
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 16
CalData = Brd.BrdGetCalData(dCalCfg)

# Configure Board as Master and set ADC clock to 40 MHz
# Brd.BrdSetRole('Ms', 20e6); Configure to 20 MHz sampling clock
# Brd.BrdSetRole('Ms', 40e6);
Brd.BrdSetRole('Ms', 40e6)

# Display status of board: Just an auxiliary information
Brd.BrdDispSts()
Brd.BrdDispInf()

#--------------------------------------------------------------------------
# Configure Receiver and Transmitters (static setup with single Tx turned on)
#--------------------------------------------------------------------------
Brd.RfRxEna()
Brd.RfTxEna(1, 80)

#--------------------------------------------------------------------------
# Configure AFE5801
#--------------------------------------------------------------------------
Brd.Set('AfeLowNoise',0)
# Enable/Disable internal DC coupling
Brd.Set('AfeIntDcCoupling',1)
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20)


if Disp_Video > 0:
    # Configure RadServe to output video on TCP/IP connection
    Brd.CfgRadServe('AddVideoToDataPort',1)

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24e9
dCfg['fStop'] = 24.2e9
dCfg['TRampUp'] = 128e-6
dCfg['TInt'] = 80e-3
dCfg['N'] = 1024
dCfg['IniEve'] = 0
dCfg['IniTim'] = 100e-3
dCfg['Np'] = 128
dCfg['Tp'] = 300e-6


Brd.Set('DmaMult', dCfg['Np'])
Brd.ConSet('Mult', dCfg['Np'])

#--------------------------------------------------------------------------
# Use FPGA triggered measurement mode:
# FPGA generates timing and triggers RCC1010
# The timing is generated in multiples of ADC clock cycles. The default
# clock is set to 20 MHz.
#--------------------------------------------------------------------------
Brd.RfMeas('ExtTrigUp', dCfg)

#--------------------------------------------------------------------------
# Read actual configuration
#--------------------------------------------------------------------------
NrChn           =   Brd.Get('NrChn')
N               =   Brd.Get('N')
fs              =   Brd.Get('fs')

#--------------------------------------------------------------------------
# Configure Range Profile
#--------------------------------------------------------------------------
# Set the fft size for the range profiles
Brd.Computation.SetParam('Range_NFFT', 2048)
# Set the minimum and maximum range for returning the range profiles
Brd.Computation.SetParam('Range_RMin', 20)
Brd.Computation.SetParam('Range_RMax', 200)
# subtract the mean value of all range profiles that were computed at once (= filter non-moving detections)
Brd.Computation.SetParam('Range_SubtractMean', 1)

#--------------------------------------------------------------------------
# Configure Velocity FFT
#--------------------------------------------------------------------------
# Configure Doppler FFT
Brd.Computation.SetParam('Vel_NFFT', pow(2, ceil(log2(dCfg['Np']))))

#--------------------------------------------------------------------------
# Configure Detector (Local Maxima in Range-Doppler map)
#--------------------------------------------------------------------------
Brd.Computation.SetParam('Thres_Old', 0.95)
Brd.Computation.SetParam('Thres_Mult', 2)
#Brd.Computation.SetParam('Thres_Range1', 10)
#Brd.Computation.SetParam('Thres_Mult2', 1.5)
#Brd.Computation.SetParam('Thres_Range2', 22)

#--------------------------------------------------------------------------
# Configure Angular FFT
#--------------------------------------------------------------------------
# Angular FFT
Brd.Computation.SetParam('Ang_NFFT', 128)
Brd.Computation.SetParam('Ang_Interpolate', 1)

Brd.Computation.SetParam('Track_SigmaX', 0.2)
Brd.Computation.SetParam('Track_SigmaY', 0.2)

Brd.Computation.SetParam('Track_VarX', 0.025)
Brd.Computation.SetParam('Track_VarY', 0.025)
Brd.Computation.SetParam('Track_VarVel', 0.05)

Brd.Computation.SetParam('Track_dT', dCfg['TInt'])


Brd.Computation.SetParam('TT_NumDetections', 200)

# changing the remCnt influences how long a track may not receive an update without it being deleted
Brd.Computation.SetParam('TT_RemCnt', 20);

NrTracks = 10
Brd.Computation.SetParam('TT_NumTracks', NrTracks)

if Disp_TT > 0:
    Pen = [];
    Pen.append(pg.mkPen(color=(  0,  95, 162), width=1))
    Pen.append(pg.mkPen(color=(253, 188,   0), width=1))
    Pen.append(pg.mkPen(color=(253,  68,   0), width=1))
    Pen.append(pg.mkPen(color=(  0,  59, 100), width=1))
    Pen.append(pg.mkPen(color=(156, 116,   0), width=1))
    Pen.append(pg.mkPen(color=(156,  42,   0), width=1))
    Pen.append(pg.mkPen(color=( 11, 152, 251), width=1))
    Pen.append(pg.mkPen(color=(  0,   5, 176), width=1))
    
    Pen.append(pg.mkPen(color=(  0,  36,  60), width=1))
    Pen.append(pg.mkPen(color=(151, 112,   0), width=1))
    Pen.append(pg.mkPen(color=(151,  41,   0), width=1))
    Pen.append(pg.mkPen(color=(  0,   0,   0), width=1))
    Pen.append(pg.mkPen(color=(140, 108,  16), width=1))
    Pen.append(pg.mkPen(color=(140,  49,  16), width=1))
    Pen.append(pg.mkPen(color=(  3,  94, 157), width=1))
    Pen.append(pg.mkPen(color=( 18,  22, 158), width=1))
    
    Brush = [];
    Brush.append(pg.mkBrush(  0,  95, 162))
    Brush.append(pg.mkBrush(253, 188,   0))
    Brush.append(pg.mkBrush(253,  68,   0))
    Brush.append(pg.mkBrush(  0,  59, 100))
    Brush.append(pg.mkBrush(156, 116,   0))
    Brush.append(pg.mkBrush(156,  42,   0))
    Brush.append(pg.mkBrush( 11, 152, 251))
    Brush.append(pg.mkBrush(  0,   5, 176))
    
    Brush.append(pg.mkBrush(  0,  36,  60))
    Brush.append(pg.mkBrush(151, 112,   0))
    Brush.append(pg.mkBrush(151,  41,   0))
    Brush.append(pg.mkBrush(  0,   0,   0))
    Brush.append(pg.mkBrush(140, 108,  16))
    Brush.append(pg.mkBrush(140,  49,  16))
    Brush.append(pg.mkBrush(  3,  94, 157))
    Brush.append(pg.mkBrush( 18,  22, 158))
    
    viewTrack = []
    viewHist = []
    for idxTrack in range(0, int(NrTracks)):
        viewHist.append(viewPlot.plot(pen=Pen[idxTrack]))
        view = pg.ScatterPlotItem(size=10, pen=Pen[idxTrack], pxMode=True)
        viewPlot.addItem(view)
        viewTrack.append(view)

Brd.Computation.SetType('TargetTracker')

#--------------------------------------------------------------------------
# Measure and calculate DBF
#--------------------------------------------------------------------------
for MeasIdx in range(0, 10000):

    Data        =   Brd.BrdGetData()
        
   
#        Data of Tracker contains struct array with the following entries
#         Id
#         X
#         Y
#         Vel
#         VelX
#         VelY
#         Mag
#         VarX
#         VarY
#         HistX
#         HistY

    # also plot untracked detections
    viewDet.clear()
    det = []
    if len(Data['Detections']) > 0:
        for Idx in range(0, len(Data['Detections'])):
            det.append({'pos': (Data['Detections'][Idx]['X'], Data['Detections'][Idx]['Y']), 'data': 1, 'pen': pg.mkPen(255,255,255), 'brush': pg.mkBrush(None), 'size': 7});
    
    viewDet.addPoints(det);
    
    for Idx in range(0, len(viewTrack)):
        viewTrack[Idx].clear()
        viewHist[Idx].clear()
        viewTrack[Idx].addPoints([])
        
    for Idx in range(0, len(Data['Tracks'])):
        viewTrack[Idx].addPoints([{'pos': (Data['Tracks'][Idx]['X'], Data['Tracks'][Idx]['Y']), 'data': 1, 'pen':Pen[Idx], 'brush': Brush[Idx], 'size': 10}]);
        viewHist[Idx].setData(Data['Tracks'][Idx]['HistX'], Data['Tracks'][Idx]['HistY'])
        
    pg.QtGui.QApplication.processEvents()


Brd.BrdRst()
Brd.BrdPwrDi()

del Brd

App.quit()
