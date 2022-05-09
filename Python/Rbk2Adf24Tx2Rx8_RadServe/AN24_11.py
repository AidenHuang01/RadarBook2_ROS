# AN_11 Replay File

import sys, os
sys.path.append("../")
import  src.cmd_modules.Rbk2 as Rbk2
import  time as time
import cv2

#--------------------------------------------------------------------------
# Setup Connection
#--------------------------------------------------------------------------
Brd = Rbk2.Rbk2('RadServe', '127.0.0.1');

#% Start Replay Mode
# Brd.ReplayFile(<name>, <startIdx, first = 1>, [video - optional param, can be anything => if given video port will be tried])
Brd.ReplayFile('File_01', FrameIdx=1, WithVideo=1);

N = Brd.Get('N')
NrChn = Brd.Get('NrChn')
NumFrms = Brd.Get('FileSize')

cfgStrt = Brd.GetFileParam('fStrt', 'DOUBLE');
cfgStop = Brd.GetFileParam('fStop', 'DOUBLE');
cfgTRampUp = Brd.GetFileParam('TRampUp', 'DOUBLE');
cfgTRampDo = Brd.GetFileParam('TRampDo', 'DOUBLE');
intVal = Brd.GetFileParam('IntVal', 'INT');
str = Brd.GetFileParam('TestStr', 'STRING');

print('fStrt: ', cfgStrt);
print('fStop: ', cfgStop);
print('TRampUp: ', cfgTRampUp);
print('TRampDo: ', cfgTRampDo);
print('intVal: ', intVal);
print('TestStr: ', str);

print("NumFrms: ", NumFrms)

#fs = Brd.GetFileParam('fs', 'DOUBLE');

for Idx in range(0,int(NumFrms)):
    Data = Brd.BrdGetData(1);
    print('FrmCntr: ', Data[0,:])
    
    if (Brd.DispVideo(Idx)):
        cv2.waitKey(25);
        
Brd.StopReplayFile();

del Brd