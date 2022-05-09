import numpy as np
import h5py


class RadSig():
    c0 = 299_792_458
    Cfg = None
    FuSca = None
    FreqSamp = None
    kf = None
    NrChn = None
    NrVirtChn = None
    NrTxSeq = None
    DatIdx = 0
    CfgValid = 0
    NrSamp = None
    NrChirp = None
    TimChirp = None
    stFileH5 = None
    FileInf = None

    # Parameters for time signals
    Dat = None
    CorDat = None
    # Parameters for Range Profile
    Range = dict()
    RP = None
    RPExt = None
    RPPsd = None
    RPAvail = False

    # Parameters for Range Doppler
    Vel = dict()
    RD = None
    RDExt = None
    RDPsd = None
    RDAvail = 0

    # Parameters for angular estimation
    Ang = dict()
    RadCube = None
    RadCubeExt = None
    RadCubeAvail = 0

    # Parameters for detector in range-Doppler map
    DetRD = dict()
    DetRDThres = None

    # Parameters for Video
    lstVideo = list()

    Val = None

    def __init__(self):
        self.CfgValid = 0

    def Set(self, stParam, Val):
        if stParam == 'Range_FftSiz':
            self.Range['FftSiz'] = Val
        if stParam == 'Range_RMin':
            self.Range['RMin'] = Val
        if stParam == 'Range_RMax':
            self.Range['RMax'] = Val
        if stParam == 'Range_Psd':
            self.Range['Psd'] = True
            self.Range['PsdCntr'] = 0
        if stParam == 'Vel_FftSiz':
            self.Vel['FftSiz'] = Val
        if stParam == 'Vel_Psd':
            self.Vel['Psd'] = True
            self.Vel['PsdCntr'] = 0
        if stParam == 'Ang_FftSiz':
            self.Ang['FftSiz'] = Val
        if stParam == 'Ang_VirtIdcs':
            self.Ang['VirtIdcs'] = Val.flatten()
        if stParam == 'Ang_DistElemUla':
            self.Ang['DistElemUla'] = Val

    def IniFromFile(self, stFileH5, *varargin):
        self.stFileH5 = stFileH5

        with h5py.File(stFileH5, 'r') as fH5:
            self.FuSca = fH5.attrs['FuSca'][0]
            self.FreqSamp = fH5.attrs['fs'][0]
            self.NrChn = int(fH5.attrs['NrChn'][0])
            self.NrSamp = int(fH5.attrs['N'][0])

            # Parse Cfg structure and read back all variables
            self.Cfg = dict()
            for key in fH5.attrs:
                keySplit = key.split('_')
                if keySplit[0] == 'Cfg':
                    val = fH5.attrs[key]
                    if len(val) != 1:
                        self.Cfg[keySplit[1]] = np.ndarray(val.shape,
                                                           buffer=val.tobytes(),
                                                           dtype='float64')
                    else:
                        # heuristic to detect integers
                        if val[0] - int(val[0]) < 1e-16:
                            self.Cfg[keySplit[1]] = int(val[0])
                        else:
                            self.Cfg[keySplit[1]] = val[0]

            if len(varargin) > 0:
                print('Overwrite values with Cfg configuration structure')
                Cfg = varargin[0]
                if isinstance(Cfg, dict):
                    for key, value in Cfg.items():
                        self.Cfg[key] = value

            self.NrTxSeq = len(self.Cfg['TxSeq']) if 'TxSeq' in self.Cfg else 1
            self.NrVirtChn = self.NrChn*self.NrTxSeq

            if 'Tp' in self.Cfg:
                self.TimChirp = self.Cfg['Tp']*self.NrTxSeq
            else:
                self.TimChirp = self.Cfg['TRampUp'] + self.Cfg['TRampDo']

            self.CfgValid = self.ChkCfg()
            self.SetRangeCfg()
            self.SetVelCfg()
            self.SetAngCfg()
            self.SetCorDat()
            self.SetDetRDCfg()

            print('Number of Samples: %d' % self.NrSamp)
            print('Number of Chirps: %d' % self.NrChirp)

    def GetData(self):
        if self.CfgValid > 0:
            Dat = np.zeros((self.Cfg['N']*self.NrChirp*self.NrTxSeq, self.NrChn))
            self.Dat = np.zeros((self.Cfg['N'], self.NrChirp, self.NrChn*self.NrTxSeq))
            with h5py.File(self.stFileH5, 'r') as fH5:
                for ChnIdx in np.arange(self.NrChn):
                    strtIdx = self.DatIdx*self.NrChirp*self.NrTxSeq
                    stopIdx = (self.DatIdx+1)*self.NrChirp*self.NrTxSeq
                    Val = fH5['Chn%d' % (ChnIdx + 1)][strtIdx:stopIdx, 0:self.NrSamp]
                    Dat[:, ChnIdx] = Val.flatten()/16
                    for TxIdx in np.arange(self.NrTxSeq):
                        self.Dat[:, :, ChnIdx + TxIdx*self.NrChn] = Val[TxIdx::self.NrTxSeq, :].T
                self.Dat = self.Dat*self.CorDat/16
                self.DatIdx += 1
            return Dat
        else:
            print('@RadSig::GetData -> No valid configuration!')

    def GetVideo(self, stName):
        Img = None
        ImgIdx = None
        for Idx, Vid in enumerate(self.lstVideo):
            if Vid['stName'] == stName:
                ImgIdx = Idx
                break

        if ImgIdx is not None:
            Video = self.lstVideo[ImgIdx]
            StrtIdx = int(self.DatIdx*self.NrChirp*self.NrTxSeq
                          / (Video['RadServeMult']*Video['Rate']))
            if StrtIdx > Video['StrtIdx']:
                with h5py.File(self.stFileH5, 'r') as fH5:
                    ChungSize = Video['NrCol']*Video['NrChn']
                    Pict = fH5[stName][StrtIdx*ChungSize:(StrtIdx+1)*ChungSize, 0:Video['NrRow']]
                    Pict.shape = (Video['NrRow'], Video['NrCol'], Video['NrChn'])
                    Img = Pict[:, :, ::-1]
        return Img

    def EnaVideo(self, stName):
        with h5py.File(self.stFileH5, 'r') as fH5:
            RadServeMult = fH5.attrs['RadServe_Mult'][0]
            try:
                Video = dict()
                Video['stName'] = stName
                Video['NrRow'] = fH5.attrs[stName + '_Rows'][0]
                Video['NrCol'] = fH5.attrs[stName + '_Cols'][0]
                Video['NrChn'] = fH5.attrs[stName + '_Channels'][0]
                Video['Rate'] = fH5.attrs[stName + '_Rate'][0]
                Video['Type'] = fH5.attrs[stName + '_Type'][0]
                Video['RadServeMult'] = RadServeMult
                Video['StrtIdx'] = -1
                self.lstVideo.append(Video)
            except KeyError as er:
                print('@RadSig::EnaVideo ' + stName + '-> Required parameters not available!')
                print(er.args[0])

    def CalcRP(self):
        # Evaluate range fft; symmetrical implementation with fftshift
        DatPad = np.zeros((self.Range['FftSiz'], self.NrChirp, self.NrVirtChn), dtype=complex)
        StrtDatPadIdx = int((self.Range['FftSiz'] - len(self.Range['Win']))/2)
        StopDatPadIdx = StrtDatPadIdx + len(self.Range['Win'])
        StrtDatIdx = self.Range['NrSampRmStrt']
        StopDatIdx = StrtDatIdx + len(self.Range['Win'])

        DatPad[StrtDatPadIdx:StopDatPadIdx, :, :] = \
            self.Dat[StrtDatIdx:StopDatIdx, :, :] * self.Range['Win3D']

        self.RP = 2*np.fft.fft(np.fft.fftshift(DatPad, axes=0), axis=0) \
            / self.Range['WinSca'] * self.FuSca

        self.RPExt = self.RP[self.Range['RMinIdx']:self.Range['RMaxIdx'], :, :]
        self.RPAvail = True

        if 'Psd' in self.Range:
            if self.Range['Psd']:
                if self.Range['PsdCntr'] == 0:
                    self.RPPsd = np.abs(self.RPExt)**2
                else:
                    self.RPPsd = self.RPPsd + np.abs(self.RPExt)**2
                self.Range['PsdCntr'] = self.Range['PsdCntr'] + 1
        return self.RPExt

    def CalcRD(self):
        RD = self.CalcRP()

        # Evalaute velocity fft; symmetrical implementation with fftshift
        DatRPPad = np.zeros((len(self.Range['vRangeExt']), self.Vel['FftSiz'], self.NrVirtChn),
                            dtype='complex')
        StrtIdx = int((self.Vel['FftSiz'] - len(self.Vel['Win']))/2)
        StopIdx = StrtIdx + len(self.Vel['Win'])
        DatRPPad[:, StrtIdx:StopIdx, :] = RD*self.Vel['Win3D']
        self.RD = np.fft.fftshift(np.fft.fft(np.fft.fftshift(DatRPPad, axes=1), axis=1), axes=1) \
            / self.Vel['WinSca']
        RD = self.RD[:, self.Vel['VelMinIdx']:self.Vel['VelMaxIdx'], :]
        self.RDExt = RD
        self.RDAvail = 1

        if 'Psd' in self.Vel:
            if self.Vel['Psd']:
                if self.Vel['PsdCntr'] == 0:
                    self.RDPsd = np.abs(self.RD)**2
                else:
                    self.RDPsd = self.RDPsd + np.abs(self.RD)**2
                self.Vel['PsdCntr'] = self.Vel['PsdCntr'] + 1
        return RD

    def CalcRadCube(self):
        RD = self.CalcRD()

        # Evalaute velocity fft; symmetrical implementation with fftshift
        DatRCPad = np.zeros((len(self.Range['vRangeExt']), len(self.Vel['vVelExt']), self.Ang['FftSiz']),
                            dtype='complex')
        StrtIdx = int((self.Ang['FftSiz'] - len(self.Ang['Win']))/2)
        StopIdx = StrtIdx + len(self.Ang['Win'])
        DatRCPad[:, :, StrtIdx:StopIdx] = RD[:, :, self.Ang['VirtIdcs']]*self.Ang['Win3D']
        self.RadCube = np.fft.fftshift(np.fft.fft(np.fft.fftshift(DatRCPad, axes=2), axis=2), axes=2) \
            / self.Ang['WinSca']
        RadCube = self.RadCube[:, :, self.Ang['AngMinIdx']:self.Ang['AngMaxIdx']]
        self.RadCubeExt = RadCube
        self.RadCubeAvail = 1
        return RadCube

    def ChkCfg(self):
        if self.NrChn is None:
            if self.NrChn < 1:
                print('@RadSig::ChkCfg -> Number of channels (NrChn) not set!')
                return False

        if 'NLoop' in self.Cfg:
            print('@RadSig::ChkCfg -> Set number of chirps to %d' % self.Cfg['NLoop'])
            self.NrChirp = self.Cfg['NLoop']
        elif 'Np' in self.Cfg:
            print('@RadSig::ChkCfg -> Set number of chirps to %d' % self.Cfg['Np'])
            self.NrChirp = self.Cfg['Np']
        else:
            return False

        return True

    def SetRangeCfg(self):
        if 'FftSiz' in self.Range:
            if self.Range['FftSiz'] < self.NrSamp:
                self.Range['FftSiz'] = int(2**(np.ceil(np.log2(self.NrSamp))))
        else:
            self.Range['FftSiz'] = int(2**(np.ceil(np.log2(self.NrSamp) + 1)))

        if self.kf is None:
            self.kf = (self.Cfg['fStop'] - self.Cfg['fStrt']) / self.Cfg['TRampUp']

        self.Range['vFreqRange'] = np.arange(self.Range['FftSiz']/2) / \
            self.Range['FftSiz'] * self.FreqSamp
        self.Range['vRange'] = self.Range['vFreqRange'] * self.c0 / (2 * self.kf)

        if 'RMin' in self.Range:
            Idx = np.argmin(np.abs(self.Range['vRange'] - self.Range['RMin']))
            self.Range['RMinIdx'] = Idx
        else:
            self.Range['RMin'] = 0
            self.Range['RMinIdx'] = 1

        if 'RMax' in self.Range:
            Idx = np.argmin(np.abs(self.Range['vRange'] - self.Range['RMax']))
            self.Range['RMaxIdx'] = Idx
        else:
            self.Range['RMax'] = self.Range['vRange'][-1]
            self.Range['RMaxIdx'] = self.Range['FftSiz']//2

        self.Range['NrSampRmStrt'] = 1  # Remove at leaset the frame counter
        self.Range['NrSampRmStop'] = 0

        self.Range['vRangeExt'] = self.Range['vRange'][self.Range['RMinIdx']:self.Range['RMaxIdx']]
        NWin = self.NrSamp - self.Range['NrSampRmStrt'] - self.Range['NrSampRmStop']
        self.Range['Win'] = self.hanning(NWin)
        self.Range['WinSca'] = sum(self.Range['Win'])
        self.Range['Win3D'] = np.broadcast_to(self.Range['Win'][:, np.newaxis, np.newaxis],
                                              (NWin, self.NrChirp, self.NrVirtChn))
        self.RPAvail = False

    def GetRangeBins(self):
        vRangeExt = self.Range['vRangeExt']
        return vRangeExt

    def SetVelCfg(self):
        if 'FftSiz' in self.Vel:
            if self.Vel['FftSiz'] < self.NrChirp:
                self.Vel['FftSiz'] = int(2**(np.ceil(np.log2(self.NrChirp))))
        else:
            self.Vel['FftSiz'] = int(2**(np.ceil(np.log2(self.NrChirp))+1))

        fc = (self.Cfg['fStop'] + self.Cfg['fStrt'])/2
        self.Vel['vFreqVel'] = np.arange(-self.Vel['FftSiz']//2, self.Vel['FftSiz']//2) \
            / self.Vel['FftSiz'] / self.TimChirp

        self.Vel['vVel'] = self.Vel['vFreqVel']*self.c0/(2*fc)
        if 'VelMin' in self.Vel:
            Idx = np.argmin(np.abs(self.Vel['vVel'] - self.Vel['VelMin']))
            self.Vel['VelMinIdx'] = Idx
        else:
            self.Vel['VelMin'] = self.Vel['vVel'][0]
            self.Vel['VelMinIdx'] = 0

        if 'VelMax' in self.Vel:
            Idx = np.argmin(np.abs(self.Vel['vVel'] - self.Vel['VelMax']))
            self.Vel['VelMaxIdx'] = Idx
        else:
            self.Vel['VelMax'] = self.Vel['vVel'][-1]
            self.Vel['VelMaxIdx'] = self.Vel['FftSiz']

        self.Vel['vVelExt'] = self.Vel['vVel'][self.Vel['VelMinIdx']:self.Vel['VelMaxIdx']]
        self.Vel['Win'] = self.hanning(self.NrChirp)
        self.Vel['WinSca'] = sum(self.Vel['Win'])
        self.Vel['Win3D'] = np.broadcast_to(self.Vel['Win'][np.newaxis, :, np.newaxis],
                                            (len(self.Range['vRangeExt']),
                                             len(self.Vel['Win']),
                                             self.NrVirtChn))
        self.RDAvail = 0

    def GetVelBins(self):
        vVelExt = self.Vel['vVelExt']
        return vVelExt

    def SetAngCfg(self):
        if 'VirtIdcs' not in self.Ang:
            self.Ang['VirtIdcs'] = np.arange(self.NrVirtChn)

        if 'FftSiz' in self.Ang:
            if self.Ang['FftSiz'] < self.NrChirp:
                self.Ang['FftSiz'] = int(2**(np.ceil(np.log2(len(self.Ang['VirtIdcs']))) + 2))
        else:
            self.Ang['FftSiz'] = int(2**(np.ceil(np.log2(len(self.Ang['VirtIdcs']))) + 2))

        FreqCentr = (self.Cfg['fStop'] + self.Cfg['fStrt'])/2
        LambdaCentr = self.c0/FreqCentr

        if 'DistElemUla' not in self.Ang:
            self.Ang['DistElemUla'] = LambdaCentr/2

        self.Ang['vU'] = np.arange(-self.Ang['FftSiz']//2, self.Ang['FftSiz']//2)/self.Ang['FftSiz']
        self.Ang['vAng'] = np.arcsin(self.Ang['vU'])
        self.Ang['vAngDeg'] = self.Ang['vAng']/np.pi*180
        if 'AngMinDeg' in self.Ang:
            Idx = np.argmin(np.abs(self.Ang['vAngDeg'] - self.Ang['AngMinDeg']))
            self.Ang['AngMinIdx'] = Idx
        else:
            self.Ang['AngMinDeg'] = self.Ang['vAngDeg'][0]
            self.Ang['AngMinIdx'] = 0

        if 'AngMaxDeg' in self.Vel:
            Idx = np.argmin(np.abs(self.Ang['vAngDeg'] - self.Ang['AngMaxDeg']))
            self.Ang['AngMaxIdx'] = Idx
        else:
            self.Ang['AngMaxDeg'] = self.Ang['vAngDeg'][-1]
            self.Ang['AngMaxIdx'] = self.Ang['FftSiz']

        self.Ang['vUExt'] = self.Ang['vU'][self.Ang['AngMinIdx']:self.Ang['AngMaxIdx']]
        self.Ang['vAngExt'] = self.Ang['vAng'][self.Ang['AngMinIdx']:self.Ang['AngMaxIdx']]
        self.Ang['vAngDegExt'] = self.Ang['vAngDeg'][self.Ang['AngMinIdx']:self.Ang['AngMaxIdx']]
        self.Ang['Win'] = self.hanning(len(self.Ang['VirtIdcs']))
        self.Ang['WinSca'] = np.sum(self.Ang['Win'])
        self.Ang['Win3D'] = np.broadcast_to(self.Ang['Win'][np.newaxis, np.newaxis, :],
                                            (len(self.Range['vRangeExt']),
                                             len(self.Vel['vVelExt']),
                                             len(self.Ang['Win'])))
        # repmat(self.Ang['Win'], 1, numel(self.Range['vRangeExt']), numel(self.Vel['vVelExt']))
        # self.Ang['Win']3D = permute(self.Ang['Win']3D,[2,3,1])
        self.RadCubeAvail = 0

    def GetUBins(self):
        vUExt = self.Ang['vUExt']
        return vUExt

    def GetWgtCor(self, Cal, TxWgtLen, RxWgtLen, Wgta, RCor):
        NRx = len(RxWgtLen)
        NTx = len(TxWgtLen)

        vCal = Cal.flatten()

        mRxWgtLen = np.broadcast_to(RxWgtLen[np.newaxis, np.newaxis, :],
                                    (self.NrSamp, NTx, NRx))
        mRxWgtLen = np.reshape(mRxWgtLen, (self.NrSamp, NRx*NTx))

        mTxWgtLen = np.broadcast_to(TxWgtLen[np.newaxis, :, np.newaxis],
                                    (self.NrSamp, NTx, NRx))
        mTxWgtLen = np.reshape(mTxWgtLen, (self.NrSamp, NRx*NTx))

        WgtFreq = np.linspace(self.Cfg['fStrt'], self.Cfg['fStop'], self.NrSamp)
        WgtBeta = np.sqrt((2*np.pi*WgtFreq/self.c0)**2 - (np.pi/Wgta)**2)  # see Pozar p. 117 ff.

        mWgtBeta = np.broadcast_to(WgtBeta[:, np.newaxis], (self.NrSamp, NRx*NTx))
        mWgtFreq = np.broadcast_to(WgtFreq[:, np.newaxis], (self.NrSamp, NRx*NTx))
        mCal = np.broadcast_to(vCal[np.newaxis, :], (self.NrSamp, NRx*NTx))

        Ret = mCal*np.exp(-1j*mWgtBeta*mRxWgtLen) \
            * np.exp(-1j*mWgtBeta*mTxWgtLen) \
            * np.exp(-1j*RCor*2*np.pi*mWgtFreq/self.c0)
        return Ret

    def SetCorDat(self, Dat=None):
        if Dat is not None:
            N1, N2 = Dat.shape

            if N1 == self.NrSamp and N2 == self.NrVirtChn:
                self.CorDat = np.broadcast_to(Dat[:, np.newaxis, :], (N1, self.NrChirp, N2))
            else:
                self.CorDat = np.ones((self.NrSamp, self.NrChirp, self.NrVirtChn))
                print('@RadSig::SetCorDat -> CorDat dimension missmatch')
        else:
            self.CorDat = np.ones((self.NrSamp, self.NrChirp, self.NrVirtChn))

    def SetDetRDCfg(self, *varargin):
        Cfg = dict()
        if len(varargin) > 0:
            if isinstance(varargin[0], dict):
                Cfg = varargin[0]

        if 'Thres' in Cfg:
            self.DetRD['Thres'] = Cfg['Thres']
        else:
            self.DetRD['Thres'] = 2.5

        if 'UpdConst' in Cfg:
            if Cfg['UpdConst'] < 1 and Cfg['UpdConst'] > 0:
                self.DetRD['UpdConst'] = Cfg['UpdConst']
            else:
                self.DetRD['UpdConst'] = 0.9
        else:
            self.DetRD['UpdConst'] = 0.9

        self.DetRD['UpdCntr'] = 0

    # DOXYGEN ------------------------------------------------------
    # @brief <span style="color: #ff0000;"> LowLevel: </span> Generate hanning window
    #
    # This function returns a hanning window.
    #
    #  @param[in]   M: window size
    #
    #  @param[In]   N (optional): Number of copies in second dimension
    def hanning(self, M, N=1):
        m = np.arange(-(M-1)//2, (M-1)//2 + 1)
        Win = 0.5 + 0.5*np.cos(2*np.pi*m/M)
        if N > 1:
            Win = np.broadcast_to(Win[:, np.newaxis], (M, N))
        return Win

    # DOXYGEN ------------------------------------------------------
    # @brief <span style="color: #ff0000;"> LowLevel: </span> Generate hanning window
    #
    # This function returns a hamming window.
    #
    #  @param[in]   M: window size
    #
    #  @param[In]   N (optional): Number of copies in second dimension
    def hamming(self, M, N=1):
        m = np.arange(-(M-1)//2, (M-1)//2 + 1)
        Win = 0.54 + 0.46*np.cos(2*np.pi*m/M)
        if N > 1:
            Win = np.broadcast_to(Win[:, np.newaxis], (M, N))
        return Win

    # DOXYGEN ------------------------------------------------------
    # @brief <span style="color: #ff0000;"> LowLevel: </span> Generate boxcar window
    #
    # This function returns a boxcar window.
    #
    #  @param[in]   M: window size
    #
    #  @param[In]   N (optional): Number of copies in second dimensionc
    def boxcar(self, M, N=1):
        Win = np.ones((M,))
        if N > 1:
            Win = np.broadcast_to(Win[:, np.newaxis], (M, N))
        return Win
