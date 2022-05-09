# This application note shows how to calculate the range FFT with the
# calculation software in the RadServe 
# The licensed version of the RadServe is required 

# (1) Connect to RadarLog
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements
# (6) Configure signal processing with radServe: Extracted range profile is
# returned instead of time domain data: Range Profile is calculated in C++
#

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
Disp_RP = 1;
# Display Video (only possible if camera is enabled in RadServe)
Disp_Video = 0;  

if Disp_RP > 0:
    App = QtGui.QApplication([])
    viewWin = pg.GraphicsWindow("Plot");          
    viewWin.setBackground((255, 255, 255))
    viewWin.resize(1000,600)
    viewPlot = viewWin.addPlot(title="Range Profile", col=0, row=0)
    viewPlot.showGrid(x=True, y=True)

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

Brd.ConSet('Mult', 2)

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.00e9
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 256e-6
dCfg['TInt'] = 25e-3
dCfg['N'] = 1024
dCfg['IniEve'] = 0
dCfg['IniTim'] = 100e-3

#--------------------------------------------------------------------------
# Use FPGA triggered measurement mode:
# FPGA generates timing and triggers RCC1010
# The timing is generated in multiples of ADC clock cycles. The default
# clock is set to 20 MHz.
#--------------------------------------------------------------------------
Brd.RfMeas('ExtTrigUp',dCfg)

#--------------------------------------------------------------------------
# Read configured values; 
# In the FPGA the CIC sampling rate reduction filter is configured
# automatically; only an integer R reduction value can be configured;
# Readback the actual sampling frequency
#--------------------------------------------------------------------------
N = int(Brd.Get('N'))
NrChn = int(Brd.Get('NrChn'))
fs = Brd.Get('fs')

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
DataRate = 16*NrChn*N/dCfg['TInt']
print('DataRate: ', (DataRate/1e6), ' MBit/s')

#--------------------------------------------------------------------------
# Configure Range Profile
#--------------------------------------------------------------------------
# Set the fft size for the range profiles
Brd.Computation.SetParam('Range_NFFT', 2048)
# Set the minimum and maximum range for returning the range profiles
Brd.Computation.SetParam('Range_RMin', 1)
Brd.Computation.SetParam('Range_RMax', 20)

Brd.Computation.SetType('RangeProfile');

if Disp_RP > 0:
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
    
    viewCurves = []
    for idxChn in range(0, int(NrChn)):
        viewCurves.append(viewPlot.plot(pen=Pen[idxChn]))

# Read back range bins for the calculated range profile
vRange = Brd.Computation.GetRangeBins()

for Cycles in range(0,100):
    
    # Returns a 3D array [Range, Chn, Arg1]
    # if arg 1 is set greater 1 then multiple measurements are returned
    # with one call to the function
    RP = Brd.BrdGetData(1)
        
    if Disp_RP > 0:
        for idxChn in range(0, int(NrChn)):
            viewCurves[idxChn].setData(vRange[0:], 20*log10(abs(RP[:,idxChn])));
        pg.QtGui.QApplication.processEvents()
    
Brd.BrdRst()
Brd.BrdPwrDi()
    
del Brd

App.quit()