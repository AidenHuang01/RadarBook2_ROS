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

# Configure script
Disp_FrmNr = 1
Disp_RP = 0      # display range profile
Disp_JOpt = 1      # display cost function for DBF

c0 = 299792458

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply 
Brd.BrdPwrEna()

# Use 40 MHz clock for the ADC (AFE5801)
# Brd.BrdSetRole('Ms', 20e6); to configure 20 MHz clock
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
dCfg['fStrt'] = 23.8e9        
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

#--------------------------------------------------------------------------
# Measure and calculate DBF
#--------------------------------------------------------------------------
for Cycles in range(0, 1000):

    Data = Brd.BrdGetData(1);

    if Disp_FrmNr > 0:
        print(Data[0,:])
   
    # Remove Framenumber from processing
    Data = Data[1:,:]
    
    # Calculate range profiles and display them
    RP = 2*np.fft.fft(Data[:,:]*Win2D, n=NFFT, axis=0)/ScaWin*Brd.FuSca
    RP = RP[RMinIdx:RMaxIdx,:]
    
    
    if Disp_JOpt > 0:
        JOpt = np.fft.fftshift(np.fft.fft(RP*WinAnt2D, NFFTAnt, axis=1)/ScaWinAnt, axes=1)
        
        JdB = 20*np.log10(np.abs(JOpt))
        JMax = np.max(JdB)
        JNorm = JdB - JMax
        JNorm[JNorm < -25] = -25
    




Brd.BrdDispInf()
Brd.BrdRst()
Brd.BrdPwrDi()

del Brd