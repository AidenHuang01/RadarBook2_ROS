# This application note shows how to calculate the range Doppler map with the
# calculation software in the RadServe 
# The licensed version of the RadServe is required 

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
Disp_RD = 1
# Display Video (only possible if camera is enabled in RadServe)
Disp_Video = 0

if Disp_RD > 0:

    App = QtGui.QApplication([]);
    viewPlot = pg.PlotItem();
    viewPlot.setLabel('left', 'R', units='m')
    viewPlot.setLabel('bottom', 'u')
    
    viewImg = pg.ImageView(view = viewPlot, name='Adf24Tx2Rx8_AN24_39')
    viewImg.show()
    viewImg.ui.roiBtn.hide()
    viewImg.ui.menuBtn.hide()
    viewImg.ui.histogram.hide()
    viewImg.getHistogramWidget().gradient.loadPreset('bipolar')

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
dCfg['Np'] = 64
dCfg['Tp'] = 200e-6


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
# Read actual configuration
#--------------------------------------------------------------------------
NrChn = Brd.Get('NrChn')
N = Brd.Get('N')
fs = Brd.Get('fs')

NFFTVel = 128

#--------------------------------------------------------------------------
# Configure Range Profile
#--------------------------------------------------------------------------
Brd.Computation.SetParam('Np', dCfg['Np'])
Brd.Computation.SetParam('Tp', dCfg['Tp'])
Brd.Computation.SetParam('TInt', dCfg['TInt'])

Brd.Computation.SetParam('Range_NFFT', pow(2, ceil(log2(dCfg['N'] * 2))));
Brd.Computation.SetParam('Range_RMin', 0)
Brd.Computation.SetParam('Range_RMax', 20)
Brd.Computation.SetParam('Range_SubtractMean', 1)

# set fft size for the velocity
Brd.Computation.SetParam('Vel_NFFT', NFFTVel)

# activate the range profile computation on RadServe and prepare the
# computation class for receiving range-Doppler maps
Brd.Computation.SetType('RangeDoppler')

Brd.CfgRadServe('AddVideoToDataPort',1)

# Read back range bins for the calculated range profile
vRange = Brd.Computation.GetRangeBins()
vVel = Brd.Computation.GetVelBins()

#--------------------------------------------------------------------------
# Measure and calculate DBF
#--------------------------------------------------------------------------
for MeasIdx in range(0,100):

    Data = Brd.BrdGetData(1)
    
    if Disp_RD > 0:
        RD = mean(abs(Data),2)

        viewImg.setImage(20*numpy.log10(numpy.abs(Data[:,:,0]))+0.5e-6, pos=[vVel[0], vRange[0]], scale=[(vVel[-1]-vVel[0])/len(vVel), (vRange[-1]-vRange[0])/len(vRange)]);
        viewPlot.setAspectLocked(False);
        
        pg.QtGui.QApplication.processEvents()
    

Brd.BrdRst()
Brd.BrdPwrDi()

del Brd
