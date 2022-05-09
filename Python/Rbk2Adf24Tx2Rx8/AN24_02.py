# FMCW Basics: basic mode of operation

# (1) Connect to Radarbook2
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

Disp_FrmNr = 1
Disp_TimSig = 1
Disp_RP = 1

c0 = 299792458

if Disp_TimSig > 0 or Disp_RP > 0:
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

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
#Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')
Brd     =   Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Verify if sampling framework is installed
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
Brd.BrdDispInf()

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
Brd.Set('AfeIntDcCoupling',0)
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 20)

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24e9
dCfg['fStop'] = 24.2e9
dCfg['TRampUp'] = 256e-6
dCfg['TInt'] = 100e-3
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
# Configure Signal processing
#--------------------------------------------------------------------------
NFFT = 2**14

Win = Brd.hanning(N-1,NrChn)
ScaWin = np.sum(Win[:,0])

kf = (dCfg['fStop'] - dCfg['fStrt'])/dCfg['TRampUp']
vRange = np.arange(NFFT//2)/NFFT*fs*c0/(2*kf)

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
        FrmCntr = Data[0,:]
        print("FrmCntr:", FrmCntr)

    if Disp_TimSig > 0:
        # Display time domain signals
        for IdxChn in np.arange(NrChn):
            CurveTim[IdxChn].setData(n[1:],Data[1:,IdxChn])

    if Disp_RP > 0:
        # Calculate range profile
        # Scale data to the input of the ADC in volt (Brd.FuSca)
        # ADC full scale constant; automatically calculated when ADC gain
        # is changed with Brd.Set('AfeGaindB')
        # Range profile is corrected with gain of window function and
        # additional factor two is used to scale the amplitude of the
        # signal as real input samples are processed
        RP = 2*np.fft.fft(Data[1:,:]*Win, n=NFFT, axis=0)/ScaWin*Brd.FuSca
        RP = RP[:NFFT//2,:]
        for IdxChn in np.arange(NrChn):
            CurveRP[IdxChn].setData(vRange[0:],20*np.log10(np.abs(RP[0:,IdxChn])))

    if Disp_TimSig > 0 or Disp_RP > 0:
        # Generate Event to update plots
        pg.QtGui.QApplication.processEvents()


Brd.BrdRst()
Brd.BrdPwrDi()
Brd.BrdDispInf()

del Brd
