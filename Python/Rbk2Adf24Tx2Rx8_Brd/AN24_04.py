# Read-back the calibration data;
# If requird the calibartion data can be replaced with own call data:
# The ADF frontend has equal length traces and hence it is already
# calibrated without the need. Still the channel misalignment can be
# improved

# (1) Connect to Radarbook2 with ADF24
# (2) Read positions of antenna
# (3) Read calibration data


import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy as np

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

# Read position of antennas
TxPosn = Brd.RfGet('TxPosn')
RxPosn = Brd.RfGet('RxPosn')


#--------------------------------------------------------------------------
# Read Calibration Data
# dCalCfg['Len'] = selects number of calibration coefficients (real + cplx)
# To read 32 complex coefficients the Len field is set to 64 values
# CalData is an array with Len/2 complex entries
# (Tx1, RX1), (Tx1, Rx2), ... , (Tx1,Rx16), (Tx2, Rx1), ... (Tx2, Rx16)
#--------------------------------------------------------------------------
dCalCfg = dict()
dCalCfg['Mask'] = 1
dCalCfg['Len'] = 32
CalData = Brd.BrdGetCalData(dCalCfg)

print(CalData)

#--------------------------------------------------------------------------
# Write Calibration Data
# Generate new set of calibration coefficients
#--------------------------------------------------------------------------
Dat = 2*np.arange(32)

dCalCfg['Mask'] = 1
dCalCfg['RevNr'] = 2
dCalCfg['DateNr'] = 0
dCalCfg['Len'] = 32
dCalCfg['Data'] = Brd.Num2Cal(Dat)

# Calibration data is written to calibration table in the ARM processor;
# Data is lost after a reboot of the system
Brd.BrdSetCal(dCalCfg)

# Attention!!!!!
# This command stores the new calibration data to the EEPROM on the RF frontend
# Existing calibration data is overwritten
#Brd.Fpga_StoreCalTable(dCalCfg);

dCalCfg['Mask'] = 1
dCalCfg['Len'] = 32
CalData = Brd.BrdGetCalData(dCalCfg)

print(CalData)

Brd.BrdDispCalInf()


del Brd

