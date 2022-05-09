# AN_10 -- Create HDF File
#
# Copyright (C) 2015-11 Inras GmbH Haderer Andreas
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE file for details.
import sys, os
sys.path.append("../")

import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time

# (1) Connect to RadarLog
# (2) Enable Supply
# (3) Configure RX
# (4) Configure TX
# (5) Start Measurements with a delay of 500 ms
# (6) Configure write to file 

c0 = 299792458

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe','127.0.0.1', 8000, '192.168.1.1')


# Reset board: resets timing unit in case a measurement has been configured
# previously
Brd.BrdRst()
# Enable RF power supply and wait for supply
Brd.BrdPwrEna()

# Configure Board as Master and set ADC clock to 40 MHz
Brd.BrdSetRole('Ms', 40e6)

# Display status of board: Just an auxiliary information
Brd.BrdDispInf()


#--------------------------------------------------------------------------
# Load Calibration Data
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 16
CalData = Brd.BrdGetCalData(dCalCfg)

# Configure Board as Master and set ADC clock to 40 MHz
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
Brd.Set('AfeIntDcCoupling',0)
# Ramp pattern can be used to test communication: sample data is replaced
# by linarly increasing ramp
Brd.Set('AfePatRamp','Off')
# Set AfeGain in dB (-5 - 30 dB); The closest available value is configured
Brd.Set('AfeGaindB', 31)

#--------------------------------------------------------------------------
# Configure Up-Chirp
#--------------------------------------------------------------------------
dCfg = dict()
dCfg['fStrt'] = 24.0e9         
dCfg['fStop'] = 24.25e9
dCfg['TRampUp'] = 128e-6
dCfg['TInt'] = 100e-3
dCfg['N'] = 1024
dCfg['IniTim'] = 500e-3
dCfg['IniEve'] = 0
dCfg['Np'] = 128                    # number of chirps for range doppler
dCfg['Tp'] = 0.15e-3                # chirp repetition interval

#--------------------------------------------------------------------------
# Configure DMA Transfer to copy dCfg['NLoop'] frames simultaneously.
# Required to achiev maximal data transfer between FPGA and Soc 
Brd.Set('DmaMult', dCfg['Np'])
# set mult to 4, to reduce the size of collected packets at RadServe
Brd.ConSet('Mult', dCfg['Np'])

Brd.ConSet('BufSiz', 64)
# Copy only the data of a single channel. RX1. The data of the residual channels
# is sampled but not transfered

#--------------------------------------------------------------------------
# Use FPGA triggered measurement mode:
# FPGA generates timing and triggers RCC1010
# The timing is generated in multiples of ADC clock cycles. The default
# clock is set to 20 MHz.
#--------------------------------------------------------------------------
Brd.RfMeas('ExtTrigUp',dCfg)

for Elem in dCfg:
    if isinstance(dCfg[Elem],list):
        print(dCfg[Elem])
        Brd.SetFileParam('Cfg_' + str(Elem), np.array(dCfg[Elem], dtype='float64'), 'ARRAY64')
    else:
        Brd.SetFileParam('Cfg_' + str(Elem),dCfg[Elem], 'DOUBLE')


#---------------------------------------------------------------------------
# Stream Data to File
# (1) File Name HDF5 file:
#  - use RadServe HDF5->Settings menu to select default folder
# (2) Max number of packets stored: number of stored frames is Mult * specified value 
#--------------------------------------------------------------------------

print('Stream Measurement Data to File')
NrChn = Brd.Get('NrChn')
N = Brd.Get('N')

#--------------------------------------------------------------------------
# Check TCP/IP data rate:
# 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
# the interval TInt. If the data rate is too high, than frames can be losed
#--------------------------------------------------------------------------
DataRate = 16*NrChn*N*dCfg['Np']/dCfg['TInt']
print('DataRate: ', (DataRate/1e6), ' MBit/s')

# Add configured videos to the file
Brd.CfgRadServe('AddVideoToFile', 1)

Brd.CreateFile('File_01', 4000)

time.sleep(50)

Brd.CloseFile()
Brd.BrdPwrDi()

del Brd
