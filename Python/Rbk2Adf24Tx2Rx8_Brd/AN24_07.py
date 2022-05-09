# Description:
# Configure FMCW Mode with sequential activation of Tx antennas and measure upchirp IF signal
#
# (1) Connect to Radarbook2 with ADF24 Frontend
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements with TX sequence
# (6) Configure signal processing
# (7) Calculate DBF algorithm

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy as np

# Configure script
Disp_FrmNr = 1
Disp_RP = 1      # display range profile
Disp_JOpt = 1      # display cost function for DBF

c0 = 299792458


#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

#--------------------------------------------------------------------------
# Reset Board and Enable Power Supply
#--------------------------------------------------------------------------
Brd.BrdRst()
Brd.BrdPwrEna()

#--------------------------------------------------------------------------
# Software Version
#--------------------------------------------------------------------------
Brd.BrdDispSwVers()

#--------------------------------------------------------------------------
# Status Information
#--------------------------------------------------------------------------
Brd.BrdDispInf()


# Configure Board as Master and set ADC clock to 40 MHz
Brd.BrdSetRole('Ms', 40e6)

#--------------------------------------------------------------------------
# Load Calibration Data for the entire array
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 32
CalData = Brd.BrdGetCalData(dCalCfg)

#--------------------------------------------------------------------------
# Enable Receive Chips
#--------------------------------------------------------------------------
Brd.RfRxEna()
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
# Configure Up-Chirp with a sequence
# SPI Command is required to configure TX to different antenna the spi
# command takes < 10 us;
# 
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.0e9
dCfg['fStop'] = 24.2e9
dCfg['TRampUp'] = 100e-6
dCfg['TInt'] = 200e-3
dCfg['Tp'] = 120e-6
dCfg['N'] = 1000
dCfg['IniTim'] = 100e-3                  
dCfg['IniEve'] = 0                      # Start automatically after IniTim
dCfg['TxSeq'] = [1, 2]                  # Activate 1 and then antenna2
dCfg['Np'] = 1                          # repeat sequence Np times

#--------------------------------------------------------------------------
# Configure DMA Transfer to copy numel(dCfg['TxSeq'])*Np frames simultaneously.
# Required to achiev maximal data transfer between FPGA and Soc 
Brd.Set('DmaMult', len(dCfg['TxSeq'])*dCfg['Np'])

Brd.RfMeas('ExtTrigUp_TxSeq',dCfg)

#--------------------------------------------------------------------------
# Read Settings for N and fs
#--------------------------------------------------------------------------
N = int(Brd.Get('N'))
fs = Brd.Get('fs')
NrChn = int(Brd.Get('NrChn'))
#--------------------------------------------------------------------------
# Configure Signal Processing
#--------------------------------------------------------------------------
# Processing of range profile
NFFT = 2**12

Win2D = Brd.hanning(N-1,2*NrChn-1)
ScaWin = np.sum(Win2D[:,0])
kf = Brd.RfGet('kf')
vRange = np.arange(NFFT)/NFFT*fs*c0/(2*kf)

# Configure range interval to be displayed
RMin = 1
RMax = 10

RMin = 1
RMax = 10
RMinIdx = np.argmin(np.abs(vRange - RMin))
RMaxIdx = np.argmin(np.abs(vRange - RMax))
vRangeExt = vRange[RMinIdx:RMaxIdx]

# Window function for receive channels
NFFTAnt = 256
WinAnt2D = Brd.hanning(2*NrChn-1, len(vRangeExt))
ScaWinAnt = np.sum(WinAnt2D[:,0])
WinAnt2D = WinAnt2D.transpose()
vAngDeg  = np.arcsin(2*np.arange(-NFFTAnt//2, NFFTAnt//2)/NFFTAnt)/np.pi*180


#--------------------------------------------------------------------------
# Measure and calculate DBF
#--------------------------------------------------------------------------
for Cycles in range(0, 1000):
    
    # Record data for Tx1 and Tx2
    Data = Brd.BrdGetData(2); 

    if Disp_FrmNr > 0:
        # Framenumber is used to check measurement sequence.
        # Odd Framenumbers are for TX1 and even frame numbers for TX2
        # If a frame is missing: DBF processing will fail!!
        print(Data[0,:])

    # Format data for virtual array and remove overlapping element
    DataV = np.concatenate((Data[1:N,:], Data[N+1:,1:]), axis=1)
    #DataV = DataA(2:end,AntIdx);
    
    # Calculate range profile including calibration
    RP = 2*np.fft.fft(DataV[:,:]*Win2D, n=NFFT, axis=0)/ScaWin*Brd.FuSca
    RP = RP[RMinIdx:RMaxIdx,:]
    
    if Disp_JOpt > 0:
        JOpt = np.fft.fftshift(np.fft.fft(RP*WinAnt2D, NFFTAnt, axis=1)/ScaWinAnt, axes=1)
        
        JdB = 20*np.log10(np.abs(JOpt))
        JMax = np.max(JdB)
        JNorm = JdB - JMax
        JNorm[JNorm < -25] = -25
    

    

Brd.BrdRst()
Brd.BrdPwrDi()

del Brd



