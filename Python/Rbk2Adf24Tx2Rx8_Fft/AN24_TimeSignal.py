# FMCW Basics: basic mode of operation

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

Disp_FrmNr = 1
Disp_TimSig = 1
Disp_RP = 1
ScaRcs = 1

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
#Brd = Rbk2Adf24Tx2Rx8Fft.Rbk2Adf24Tx2Rx8Fft('RadServe', '192.168.135.70', 8000, '192.168.135.58')
Brd = Rbk2Adf24Tx2Rx8Fft.Rbk2Adf24Tx2Rx8Fft('PNet','192.168.1.1')

# Check if FFT framework is installed
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
Brd.Set('AfeIntDcCoupling',1);
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off');
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 15);

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.00e9
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 102.4e-6
dCfg['TRampDo'] = 64e-6
dCfg['TInt'] =   80e-3
dCfg['N'] = 1024
dCfg['Np']  = 1
dCfg['Tp'] = 200e-6
dCfg['IniTim'] = 20e-3
dCfg['IniEve'] = 0

Brd.Set('DmaMult', dCfg['Np'])            

#--------------------------------------------------------------------------
# Configure Range profile calculation
#--------------------------------------------------------------------------
Brd.Set('NrChn', 8)
Brd.Set('EnaFrmCntr')
Brd.Set('Mode','Raw')

Brd.RfMeas('ExtTrigUp', dCfg)

#-------------------------------------------------------------------------

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

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
DataRate = 64*NrChn*dCfg['N']/dCfg['TInt']
print('DataRate: ', (DataRate/1e6), ' MBit/s')

Brd.BrdDispInf()
#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
NFFT = 2**12
Win = Brd.hanning(int(N-1),int(NrChn))

ScaWin = np.sum(Win[:,0])
# Read back ramp slope: configured ramp slope can deviate from configured
kf = Brd.RfGet('kf')

vFreq = np.arange(NFFT//2)/NFFT*fs
vRange = np.arange(NFFT//2)/NFFT*fs*c0/(2*kf)

RMin = 0.1
RMax = 100

RMinIdx = np.argmin(abs(vRange-RMin))
RMaxIdx = np.argmin(abs(vRange-RMax))
vRangeExt = vRange[RMinIdx:RMaxIdx]
vFreqExt = vFreq[RMinIdx:RMaxIdx]

#Brd.BrdDispClkSts();
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

FrmNrOld = 0

for Cycles in range(0,100000):
    
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
        FrmNr = np.mean(Data[0,:])
        print("Frm: ", Data[0,:])
        FrmNrOld = FrmNrOld + 1
        if FrmNrOld >= 2**15:
            FrmNrOld = FrmNrOld - 2**16
        
        if FrmNr != FrmNrOld:
            print('Data Frame is missing');

        FrmNrOld = FrmNr 
        

    if Disp_TimSig > 0:
        # Display time domain signals
        for IdxChn in np.arange(NrChn):
            CurveTim[IdxChn].setData(n[1:],Data[1:,IdxChn])

    
    if Disp_RP > 0:
        # Calculate range profiles and display them
        RP = 2*np.fft.fft(Data[1:,:]*Win, n=NFFT, axis=0)/ScaWin*Brd.FuSca
        RPExt = RP[RMinIdx:RMaxIdx,:]
        
        for IdxChn in np.arange(NrChn):
            CurveRP[IdxChn].setData(vRangeExt,20*np.log10(np.abs(RPExt[:,IdxChn])))
        
    if Disp_TimSig > 0 or Disp_RP > 0:
        # Generate Event to update plots
        pg.QtGui.QApplication.processEvents()       

Brd.BrdDispInf()
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd