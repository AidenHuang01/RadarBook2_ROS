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

Disp_FrmNr = 1
Disp_RP = 1

c0 = 299792458

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd     =   Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply
Brd.BrdPwrEna()

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
Brd.RfTxEna(1, 100)

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
DataRate        =   16*NrChn*N/dCfg['TInt']
print('DataRate: ', (DataRate/1e6), ' MBit/s')

#--------------------------------------------------------------------------
# Configure Signal processing
#--------------------------------------------------------------------------
NFFT = 2**14

Win = Brd.hanning(N-1,NrChn)
ScaWin = np.sum(Win[:,0])

kf = (dCfg['fStop'] - dCfg['fStrt'])/dCfg['TRampUp']
vRange = np.arange(NFFT//2)/NFFT*fs*c0/(2*kf)

for Cycles in range(0, 100):
    
    Data        =   Brd.BrdGetData(1)     
    
    if Disp_FrmNr > 0:
        FrmCntr     =   Data[0,:]
        print("FrmCntr:", FrmCntr)


    if Disp_RP > 0:
        # Calculate range profiles and display them
        RP = 2*np.fft.fft(Data[1:,:]*Win, n=NFFT, axis=0)/ScaWin*Brd.FuSca
        RP = RP[:NFFT//2,:]

Brd.BrdRst()
Brd.BrdPwrDi()
Brd.BrdDispInf()

del Brd
