# CW Basics: basic mode of operation of board in CW mode


# (1) Connect to Radarbook2 with ADF24 frontend
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy as np
from    pyqtgraph.Qt import QtGui, QtCore
import  pyqtgraph as pg

# Configure script
Disp_FrmNr = 0
Disp_TimSig = 0
Disp_Spec = 1


if Disp_TimSig > 0 or Disp_Spec > 0:
    App = QtGui.QApplication([])

if Disp_TimSig > 0:
    WinTim = pg.GraphicsWindow(title="Time signals")
    WinTim.setBackground((255, 255, 255))
    WinTim.resize(1000,600)

    PltTim = WinTim.addPlot(title="TimSig", col=0, row=0)
    PltTim.showGrid(x=True, y=True)

if Disp_Spec > 0:

    WinSpec = pg.GraphicsWindow(title="Range Profile")
    WinSpec.setBackground((255, 255, 255))
    WinSpec.resize(1000,600)

    PltSpec = WinSpec.addPlot(title="Range", col=0, row=0)
    PltSpec.showGrid(x=True, y=True)

c0 = 299792458
#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
#Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Verify if sampling framework is installed
Brd.BrdChkSocSysId()

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply 
Brd.BrdPwrEna()

# Configure Board as Master and set ADC clock to 20 MHz
# 40 and 20 Mhz are supported
Brd.BrdSetRole('Ms', 40e6)

Brd.BrdDispSts()

#--------------------------------------------------------------------------
# Configure Receiver and Transmitters (static setup with single Tx turned on)
#--------------------------------------------------------------------------
Brd.RfRxEna()
# Enable transmitter 1 and set power register to 60:
# Register for the output power is in the range 0-100
Brd.RfTxEna(1, 80)

#--------------------------------------------------------------------------
# Configure AFE5801
# Internal DC coupling is required in CW mode: otherwise Doppler signals
# will be attenuated by high-pass filter of AFE5801
#--------------------------------------------------------------------------
Brd.Set('AfeLowNoise',0)
# Enable/Disable internal DC coupling; 
Brd.Set('AfeIntDcCoupling',1)
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20)

#--------------------------------------------------------------------------
# Configure Constant frequency and measurement interval
# fCenter:  constant output frequency
# TMeas:    Duration of measurement (sampled interval)
# TInt:     Measurement repetition interval
# N:        Number of Samples taken during TMeas 
#           fs = fAdc/ceil(fAdc/(N/TMeas))
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fCenter'] = 24.125e9
dCfg['TMeas'] = 100e-3
dCfg['TInt'] = 101e-3        
dCfg['N'] = 2048
dCfg['NrFrms'] = 100
dCfg['IniTim'] = 100e-3
dCfg['IniEve'] = 0

#--------------------------------------------------------------------------
# Use FPGA triggered measurement mode:
# FPGA generates timing and triggers RCC1010
# The timing is generated in multiples of ADC clock cycles. The default
# clock is set to 20 MHz.
#--------------------------------------------------------------------------
Brd.RfMeas('Cw',dCfg)

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
DataRate = 16*NrChn*dCfg['N']/dCfg['TInt']
print('DataRate: ', (DataRate/1e6), ' MBit/s')

Brd.BrdDispInf()
#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
# Processing of range profile
NFFT = 2**12;
Win = Brd.hanning(N-1, NrChn);
ScaWin = np.sum(Win[:,0])
# Read back ramp slope: configured ramp slope can deviate from configured

vFreq = np.arange(NFFT//2)/NFFT*fs

#--------------------------------------------------------------------------
# Generate Curves for plots
#--------------------------------------------------------------------------
if Disp_TimSig:
    n = np.arange(int(N))
    CurveTim = []
    for IdxChn in np.arange(NrChn):
        CurveTim.append(PltTim.plot(pen=pg.intColor(IdxChn, hues=NrChn)))

if Disp_Spec:
    CurveSpec = []
    for IdxChn in np.arange(NrChn):
        CurveSpec.append(PltSpec.plot(pen=pg.intColor(IdxChn, hues=NrChn)))    

for Cycles in range(0, 1000):
    
    # Read data for one chirp and all enabled channels
    # Data is a N x NrChn 2D array containing the raw data in multiples of
    # the ADC LSB.
    # The values in the FPGA are in a 12.4 integer number format and
    # converted to a double in the Matlab class
    
    # Multiple Frames can be read with setting the argument > 1
    Data = Brd.BrdGetData(1) 
    
    if Disp_FrmNr > 0:
        # Show Frame number:
        # The board can buffer multiple frames but if the communication
        # speed is too low, or the time between calls of BrdGetData is
        # higher than TInt, then frames will be lost if the FIFOs in the
        # RBK2 overflow.
        # Use the framecounter to check if no data is missing
        print(Data[0,:])
 
    Data = Data[1:,:]       # - repmat(mean(Data(2:end,:),1),N-1,1);
    
    if Disp_TimSig > 0:
        # Display sampled signals for the collected chirp
        # Display time domain signals
        for IdxChn in np.arange(NrChn):
            CurveTim[IdxChn].setData(n[1:],Data[:,IdxChn])
    
    if Disp_Spec > 0:
        # Calculate range profile
        # Scale data to the input of the ADC in volt (Brd.FuSca)
        # ADC full scale constant; automatically calculated when ADC gain
        # is changed with Brd.Set('AfeGaindB')
        # Range profile is corrected with gain of window function and
        # additional factor two is used to scale the amplitude of the
        # signal as real input samples are processed       
        Spec = 2*np.fft.fft(Data[:,:]*Win, n=NFFT, axis=0)/ScaWin*Brd.FuSca
        Spec = Spec[:NFFT//2,:]
        for IdxChn in np.arange(NrChn):
            CurveSpec[IdxChn].setData(vFreq[0:],20*np.log10(np.abs(Spec[0:,IdxChn])))        


    if Disp_TimSig > 0 or Disp_Spec > 0:
        # Generate Event to update plots
        pg.QtGui.QApplication.processEvents()       

Brd.BrdDispInf()
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd