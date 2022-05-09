# Getting Started: Test the basic connection to the Radarbook 2

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2 as Rbk2
import  time as time


# (1) Connect to Radarbook2
# (2) Display FPGA software version 
# (3) Display Board Information (Temp)

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2.Rbk2('RadServe', '127.0.0.1', 8000, '192.168.1.1')

# Verify if sampling framework is installed
Brd.BrdChkSocSysId();

# Enable RF power supply: After calling this function the LEDS for the
# power supply are turned on
Brd.BrdPwrEna()

#--------------------------------------------------------------------------
# Software Version
#--------------------------------------------------------------------------
Brd.BrdDispSwVers()

#--------------------------------------------------------------------------
# Status Information
#--------------------------------------------------------------------------
Brd.BrdDispInf()

#--------------------------------------------------------------------------
# Disable Power Supply
#--------------------------------------------------------------------------
Brd.BrdPwrDi()

del Brd
