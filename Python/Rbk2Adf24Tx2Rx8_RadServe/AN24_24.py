# This application note shows how to extract detections from the
# measurements
# This appnote only works with one TX antenna enabled.
# First the range-Doppler maps are calculated for all the enabled channels
# and in the next step a detection process is carried out and then the
# angles for the detections are calculated and returned as list of
# detections to matlab.

# (1) Connect to RadarLog
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements


#--------------------------------------------------------------------------
# Include all necessary packages
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
Disp_DT = 1
# Display Video (only possible if camera is enabled in RadServe)
Disp_Video = 0  

if Disp_DT > 0:
    App = QtGui.QApplication([])
    
    viewWin = QtGui.QMainWindow()
    view = pg.GraphicsLayoutWidget()
    viewWin.setCentralWidget(view)
    viewWin.show()
    viewWin.setWindowTitle('Detections')
    viewPlot = view.addPlot()
    viewPlot.setXRange(-150, 150, padding=0)
    viewPlot.setYRange(0, 200, padding=0)
    viewImg = pg.ScatterPlotItem(size=10, pen=None, pxMode=True)
    viewPlot.addItem(viewImg)

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
    Brd.CfgRadServe('AddVideoToDataPort',1);

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24e9
dCfg['fStop'] = 24.2e9
dCfg['TRampUp'] = 256e-6
dCfg['TInt'] = 80e-3
dCfg['N'] = 1024
dCfg['IniEve'] = 0
dCfg['IniTim'] = 100e-3
dCfg['Np'] = 64
dCfg['Tp'] = 300e-6


Brd.Set('DmaMult', dCfg['Np'])
Brd.ConSet('Mult', dCfg['Np'])

#--------------------------------------------------------------------------
# Use FPGA triggered measurement mode:
# FPGA generates timing and triggers RCC1010
# The timing is generated in multiples of ADC clock cycles. The default
# clock is set to 20 MHz.
#--------------------------------------------------------------------------
Brd.RfMeas('ExtTrigUp',dCfg)

#--------------------------------------------------------------------------
# Read actual configuration
#--------------------------------------------------------------------------
NrChn = Brd.Get('NrChn')
N = Brd.Get('N')
fs = Brd.Get('fs')

#--------------------------------------------------------------------------
# Configure Range Profile
#--------------------------------------------------------------------------
# Set the fft size for the range profiles
Brd.Computation.SetParam('Range_NFFT', dCfg['N'] * 2)
# Set the minimum and maximum range for returning the range profiles
Brd.Computation.SetParam('Range_RMin', 20)
Brd.Computation.SetParam('Range_RMax', 200)

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
# Brd.Computation.SetParam('Thres_Range1', 70)
# Brd.Computation.SetParam('Thres_Mult2', 2)
# Brd.Computation.SetParam('Thres_Range2', 80)

#--------------------------------------------------------------------------
# Configure Angular FFT
#--------------------------------------------------------------------------
# Angular FFT
Brd.Computation.SetParam('Ang_NFFT', 256)
Brd.Computation.SetParam('Ang_Interpolate', 1)


Brd.Computation.SetType('DetectionList')

for Cycles in range(0, 10000):

    Detections        =   Brd.BrdGetData(1)

    viewImg.clear()
    tar = []
    for Idx in range(size(Detections)):
        
        x = Detections[Idx]['Range'] * math.sin(Detections[Idx]['Ang'])
        y = Detections[Idx]['Range'] * math.cos(Detections[Idx]['Ang'])
        if (Detections[Idx]['Vel'] < 0):
            if (Detections[Idx]['Vel'] < -0.5):
                tar.append({'pos': (x,y), 'data': 1, 'brush': pg.mkBrush(255,0,0,255), 'size': 7})
            else:
                tar.append({'pos': (x,y), 'data': 1, 'brush': pg.mkBrush(255,0,0,255), 'size': 7})
        else:
            if (Detections[Idx]['Vel'] > 0.5):
                tar.append({'pos': (x,y), 'data': 1, 'brush': pg.mkBrush(0,0,255,255), 'size': 7})
            else:
                tar.append({'pos': (x,y), 'data': 1, 'brush': pg.mkBrush(0,0,255,255), 'size': 7})
    
    viewImg.addPoints(tar)
    pg.QtGui.QApplication.processEvents()
    
        
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd

App.quit()
