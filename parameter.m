%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%	2.Matthew G. Anderson, http://www.personal.psu.edu/mga5036/blogs/ee_497a_network_mimo/project-documents.html
%	3.Yong Soo Cho, Jaekwon Kim, Won Young Yang, Chung G. Kang, MIMO-OFDM Wireless Communications with MATLAB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters from CoMP_sim
global Num_AP Num_User;
% Debug parameters
global event_Debug detail_Debug power_Debug MIMO_Debug queue_Debug powercontrol_Debug control_frame_Debug;
% some distances for making decision
global all_STA_distance;
global def_CoMP_L cover_range;
% PHY parameters
global Num_Tx Num_Rx; 
global MCSAlgo per_order MCS MCS_ctrl; 
global BW GI R_data SymbolTime;
global default_power;
global Gt Gr freq L ht hr pathLossExp std_db d0;
global rmodel white_noise_variance cs_threshold;
% MAC parameters 
global Mode_AMPDU_AMSDU;
global size_MAC_body size_MAC_header size_RTS size_CTS size_ACK size_BA T_VHTPHY T_VHT_LTF size_CVBFReport size_NDP_Ann size_ReportPoll ; 
global slotTime CSITime SIFS DIFS aPHY_RX_START_Delay cca_time NDPTime CompressedBeamformingTime NDPAnnTime ReportPollTime NDP NDP_response;
global basic_rate ACK_tx_time CTS_tx_time RTS_tx_time CW_min CW_max;
global backoff_counter backoff_attempt nav packet_id pending_id;
% Other parameters
global traffic_queue mac_status queue_size soundingperiod;
% Statics parameters
global statics_rv_bits;
global DataSentSucceessed DataSent DataSentFailNotRx DataSentFailNoACK; 
global Interferencepkt InterferenceDatapkt InterferenceCtrlpkt; 
global CoMPdone CoMPfail CoMPfailbutsave Soundingdone;
global DebugDataTime DebugBOTime soundingIndex;
global Pr_edge;  %added by jing-wen
global ClusterSize; %add by jing-wen
global Max_Report_P; %add by jing-wen
global skip_cal_MCS_sp_Debug %add by jing-wen
global PHY_CH_module;
global num_msdu;
global spatial_stream;
% ========== New for CoMP parameter ==========
Max_Report_P = 3; %added by jing-wen
ClusterSize = 3; %added by jing-wen (not yet)
Soundingdone = 0; %added by jing-wen (not yet)
% ============================================
soundingIndex = 0;
numSTAs = Num_AP + Num_User;
% ========== Our 802.11ac channel model ========== 
PHY_CH_module = 'new'; %old:shuyu; new:new
select_algo = 1; %0:做MU-SISO盡量服務到最多user 1:做MU-MIMO:2-2,4-2,4-4,6-2
Npkt=30;
Npkt_length=1;% parameter use in fc_beamtracking.m
AI=[1 -1 2 3 4 5 -2 -3 -4 -5 0];
communication_type=1;
%1:802.11ac:MCS mode=0~9
%2:802.11n:MCS mode=0~7(1 spatial tream)
if communication_type==1
    MCS_in_beamtracking=5;% use in fc_beamtracking.m
else
    MCS_in_beamtracking=4;% use in fc_beamtracking.m
end
% # of collision wall times
reflection_times=2; % max times of signal reflections from the wall
channel_model=6; %% 'F' 
Bandwidth=40*10^6;           % Bandwidth in Hz
Temperature=298;          % Temperature in k (273+25)  
Thermal_noise = 1*1.38*10^(-23)*Temperature*Bandwidth; 
tx_ant_gain = ones(Num_Tx,361);
mpdu_size=11454*8; %MPDU size
FCS_size=4*8;
num_msdu = floor((mpdu_size-FCS_size-size_MAC_header)/size_MAC_body);
%Symbol time: 256/80 MHz + 800 ns = 4 ?s
% Nsubcarrier=512; % number of subcarriers  (should follow Bandwidth)
% ========== Debug parameters ==========
event_Debug = 0; 
detail_Debug = 1;
power_Debug = 0;
MIMO_Debug = 0;
queue_Debug = 0;
powercontrol_Debug = 0; % added by jing-wen
control_frame_Debug = 0; %added by jing-wen, Make control frame packet error rate = 0;
skip_cal_MCS_sp_Debug = 1;    %added by jing-wen, STA return CBF when it has new enough CSI, do not cal MCS for save time;

% ==== AP+User to AP+User distances ====
all_STA_distance = zeros(numSTAs, numSTAs); % Distances of (APs+Users) by (APs+Users), unit meter 
def_CoMP_L = 75; % The maximum distance for a pair of APs doing CoMP transmission, unit meter
%def_CoMP_L = 150; % The maximum distance for a pair of APs doing CoMP transmission, unit meter
% cover_range is an important parameter that related to PHY parameters, how much do I choose it? 
% db(log_normal_shadowing(0.2, 1, 1, 3e8/5e9, 1, 2.5, 0.1, 1, 100)/white_noise_variance, 'power') = 10.0044 dB
% white_noise_variance = 4.6511e-012 and log_normal_shadowing(0.2, 1, 1, 3e8/5e9, 1, 2.5, 0.1, 1, 100) = 4.6558e-011
cover_range = 50; % unit meter
%cover_range = 100; % unit meter

% =========== PHY parameters ===========
% Antenna size
Num_Tx = 6;
Num_Rx = 2;

if select_algo == 1
    spatial_stream = Num_Rx;
elseif select_algo == 0
    spatial_stream = 1;
end

% Dynmic MCS, please see estimate_SNR.m, PER_approximation.m and Init_CoMPpkt.m for details
% which involves A-MSDU or A-MPDU (global variable: Mode_AMPDU_AMSDU)
MCS_ctrl = 1; % MCS for contorl frames
MCSAlgo = 1; % Dynamic MCS = 1, Static MCS = 0
per_order = 1e-2; % input argument for PER_approximation.m
MCS = 3; % Static MCS: 64-QAM 5/6

% 802.11ac P.297 PHYdata for VHT MCSs
% R_data(MCS0-9, BandWidth 20/40/80/160 MHz, Guard Interval, Number of spatial stream);
% 802.11ac MCS0-9: 
% [BPSK 1/2, QPSK 1/2, QPSK 3/4, 16-QAM 1/2, 16-QAM 3/4, 64-QAM 2/3, 64-QAM 3/4, 64-QAM 5/6, 256-QAM 3/4, 256-QAM 5/6]
% BandWidth:
% [20 MHz, 40 MHz, 80 MHz, 160 MHz]
switch Bandwidth
    case 20000000
        BW = 1; %20 MHz
        Nsubcarrier = 64;
    case 40000000
        BW = 2; %40 MHz
        Nsubcarrier = 128;
    case 80000000
        BW = 3; %80 MHz
        Nsubcarrier = 256;
    case 160000000
        BW = 4; %160 MHz
        Nsubcarrier = 512;
    otherwise
        error 'Wrong Bandwidth in parameter!';
end
% Guard Interval:
% [800 ns, 400 ns]
GI = 1; % 800 ns
% Number of spatial stream:
% [1, 2], N_ss at most 8, but I only consider 2 spatial streams, i.e, Num_Tx = 2*Num_Rx
R_data = zeros(10,4,2,2);
% GI = 800 ns and N_ss = 1
R_data(:,:,1,1) = [		6.5,     13.5,     29.3,     58.5; ...
						 13,       27,     58.5,      117; ...
					   19.5,     40.5,     87.8,    175.5; ...
						 26,      54,       117,      234; ...
						 39,      81,     175.5,      351; ...
						 52,     108,       234,      468; ...
					   58.5,   121.5,     263.3,    526.5; ...
						 65,     135,     292.5,      585; ...
						 78,     162,       351,      702; ...
						  0,     180,       390,      780] * 1e6;
% GI = 400 ns and N_ss = 1
R_data(:,:,2,1) = [     7.2,      15,      32.5,       65; ...
                       14.4,      30,        65,      130; ...
                       21.7,      45,      97.5,      195; ...
                       28.9,      60,       130,      260; ...
                       43.3,      90,       195,      390; ...
                       57.8,     120,       260,      520; ...
                         65,     135,     292.5,      585; ...
                       72.2,     150,       325,      650; ...
                       86.7,     180,       390,      780; ...
                          0,     200,     433.3,    866.7] * 1e6;
% GI = 800 ns and N_ss = 2
R_data(:,:,1,2) = [      13,      27,      58.5,      117; ...
                         26,      54,       117,      234; ...
                         39,      81,     175.5,      351; ...
                         52,     108,       234,      468; ...
                         78,     162,       351,      702; ...
                        104,     216,       468,      936; ...
                        117,     243,     526.5,     1053; ...
                        130,     270,       585,     1170; ...
                        156,     324,       702,     1404; ...
                          0,     360,       780,     1560] * 1e6;
% GI = 400 ns and N_ss = 2
R_data(:,:,2,2) = [    14.4,      30,        65,      130; ...
                       28.9,      60,       130,      260; ...
                       43.3,      90,       195,      390; ...
                       57.8,     120,       260,      520; ...
                       86.7,     180,       390,      780; ...
                      115.6,     240,       520,     1040; ...
                        130,     270,       585,     1170; ...
                      144.4,     300,       650,     1300; ...
                      173.3,     360,       780,     1560; ...
                          0,     400,     866.7,   1733.3] * 1e6;
% GI = 800 ns and N_ss = 3
R_data(:,:,1,3) = [    19.5,     40.5,       87.8,     175.5; ...
                       39.0,     81.0,      175.5,     351.0; ...
                       58.5,    121.5,      263.3,     526.5; ...
                       78.0,    162.0,      351.0,     702.0; ...
                      117.0,    243.0,      526.5,    1053.0; ...
                      156.0,    324.0,      702.0,    1404.0; ...
                      175.5,    364.5,          0,    1579.5; ...
                      195.0,    405.0,      877.5,    1755.0; ...
                      234.0,    486.0,     1053.0,    2106.0; ...
                      260.0,    540.0,     1170.0,         0] * 1e6;
% GI = 400 ns and N_ss = 3
R_data(:,:,2,3) = [    21.7,     45.0,       97.5,     195.0; ...
                       43.3,     90.0,      195.0,     390.0; ...
                       65.0,    135.0,      292.5,     585.0; ...
                       86.7,    180.0,      390.0,     780.0; ...
                      130.0,    270.0,      585.0,    1170.0; ...
                      173.3,    360.0,      780.0,    1560.0; ...
                      195.0,    405.0,          0,    1755.0; ...
                      216.7,    450.0,      975.0,    1950.0; ...
                      260.0,    540.0,     1170.0,    2340.0; ...
                      288.9,    600.0,     1300.0,         0] * 1e6;
SymbolTime = [4, 3.6] * 1e-6; % T_SYML = 4 us(GI = 800 ns), T_SYMS = 3.6 us(GI = 400 ns)
% Radio propagation parameters
rmodel = 'shadowing';
% The transmission power of most APs ranges from 1 mW up to 100 mW, 20 dBm = 0.1 Watts
default_power = 0.316; % The unit is Watts, 10*log10(200 mw/1mw) is about 25 dBm
Gt = 1;
Gr = 1;
freq = 5e9; % IEEE 802.11ac
L = 1;
ht = 1; % Unit is meter
hr = 1; % Unit is meter
%pathLossExp = 2;
pathLossExp = 2.5;
std_db = 0.1; % variance used in shadowing (approximately: 10^(std_db/10) = 2%)
d0 = 1; % reference distance used in shadowing
Pr_edge = 4.655839604247911e-11; % for power control added by jing-wen
% Find N0
% Note: for all three radio propagation models, they are almost the same
% when d is not too large or the log-normal fading is not large.
% when Gt=Gr=L=ht=hr=1 and freq=5 GHz, Pr=Pt*(lambda/4/pi/d)^2
% so when d=d0=1, Pr=Pt*1e-6/d^2
% so we choose background noise N0=Pt*1e10 in order to achieve SNR=40 dB
% so we should choose rv_threshold be somewhere below 40 dB
lambda = 3e8 / freq;
d = d0;
switch rmodel
    case 'friis'
        Pr = friis(default_power, Gt, Gr, lambda, L, d);
    case 'tworay'
        [Pr, crossover_dist] = tworay(default_power, Gt, Gr, lambda, L, ht, hr, d);
    case 'shadowing'
        Pr = log_normal_shadowing(default_power, Gt, Gr, lambda, L, pathLossExp, std_db, d0, d);
        %Pr = log_normal_shadowing(0.2, 1, 1, 3e8/5e9, 1, 2.5, 0.1, 1, 1);
end
% white_noise_variance is used as N0 when calculating SNR
white_noise_variance = Pr / 1e6;    % SNR will be upper-bounded by 60 dB when d >= d0
max_SNR = db(Pr/white_noise_variance, 'power');
% carrier sense threthold is used to check if the channel is free to be taken for transmission
% we use Pr(when d=d0)+N0 so if there is a transmitter in distance d0 or multiple transmitter in longer distance,
% the channel will be regarded as busy.
cs_threshold = Pr + white_noise_variance;   % 0.1

% =========== MAC parameters ===========
Mode_AMPDU_AMSDU = 0; % 0: A-MPDU, 1: A-MSDU
basic_rate = R_data(MCS_ctrl,1,GI,1);
size_MAC_body = 1500*8;% MSDU = 1500 bytes % 802.11ac D2.0 P.51
size_MAC_header = (2+2+6+6+2+2+4+4)*8; % Frame Control+Duration+RxAddr+TxAddr+Sequence Control+Qos Control+HT Control+FCS
size_RTS = (2+2+6+6+4+4)*8; % Frame Control+Duration+RxAddr+TxAddr+HT Control+FCS
size_CTS = (2+2+6+4)*8; % Frame Control+Duration+RxAddr+FCS
size_ACK = size_CTS; % Normal ACK % size_BAR = 24*8; % Block ACK Request 
size_BA = 32*8; % Block ACK
size_NDP_Ann = (2+2+6+1+4+4)*8; % Frame Control+Duration+RxAddr+Sounding sequence+STA info1+STA infor+FCS
size_ReportPoll = (2+2+6+1+4)*8; %Frame Control+Duration+RxAddr+segment retransmission bitmap+FCS

% See specification 802.11ac D2.0 Table 22-5 and 22-12 and P.198
% T_VHTPHY = T_LEG_PREAMBLE+T_L-SIG+T_VHT-SIG-A+T_VHT_PREAMBLE+T_VHT-SIG-B
% T_LEG_PREAMBLE = T_L-STF+T_L-LTF
% T_VHT_PREAMBLE = T_VHT-STF+N_VHTLTF*T_VHT-LTF
T_VHTPHY = 40*1e-6; % second
T_VHT_LTF = 4*1e-6; % second
% 802.11ac P.68~70 VHT Compressed Beamforming Report field
Num_Angles = [  0,  0,  0,  0,  0,  0,  0,  0; ...
                2,  2,  0,  0,  0,  0,  0,  0; ...
                4,  6,  6,  0,  0,  0,  0,  0; ...
                6, 10, 12, 12,  0,  0,  0,  0; ...
                8, 14, 18, 20, 20,  0,  0,  0; ...
               10, 18, 24, 28, 30, 30,  0,  0; ...
               12, 22, 30, 36, 40, 42, 42,  0; ...
               14, 26, 36, 44, 50, 54, 56, 56]; % Num_Tx * Num_Rx
Num_Subcarriers = [  52,  30,  16; ...
                    108,  58,  30; ...
                    234, 122,  62; ...
                    468, 244, 124]; % Bandwidth types * Num_Group
MU_Num_psi = 5; MU_Num_phi = 7; % Another option: MU_Num_psi = 7; MU_Num_phi = 9;
% size_CVBFReport = (2+2+6+6+4)*8+(8*Num_Rx+Num_Subcarriers(BW, 1)*Num_Angles(Num_Tx, Num_Rx)*(MU_Num_psi+MU_Num_phi)/2); % a user of MU needs CSI bits which depend on Num_Tx, Num_Rx, BW
size_CVBFReport = (2+2+6+6+4)*8+(8*Num_Rx+1*Num_Angles(Num_Tx, Num_Rx)*(MU_Num_psi+MU_Num_phi)/2); % a user of MU needs CSI bits which depend on Num_Tx, Num_Rx, BW
CSITime = ceil(size_CVBFReport/basic_rate/SymbolTime(GI))*SymbolTime(GI);

NDPTime = 44*1e-6;
CompressedBeamformingTime = ceil(size_CVBFReport/basic_rate/SymbolTime(GI))*SymbolTime(GI);
NDPAnnTime =  T_VHTPHY + ceil((size_NDP_Ann / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
ReportPollTime =  T_VHTPHY + ceil((size_ReportPoll / basic_rate)/SymbolTime(GI))*SymbolTime(GI);

NDP = NDPTime + NDPAnnTime + SIFS; %like RTS
NDP_response = ReportPollTime + CompressedBeamformingTime + SIFS; %like CTS
%disp(['report poll ' num2str(ReportPollTime)]);
slotTime = 9*1e-6;
SIFS = 16*1e-6;
DIFS = SIFS + 2*slotTime;
aPHY_RX_START_Delay = 25*1e-6;
cca_time = 4*1e-6;
if (Mode_AMPDU_AMSDU == 0)
    ACK_tx_time = T_VHTPHY + ceil((size_BA / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
else
    ACK_tx_time = T_VHTPHY + ceil((size_ACK / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
end
CTS_tx_time = T_VHTPHY + ceil((size_CTS / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
RTS_tx_time = T_VHTPHY + ceil((size_RTS / basic_rate)/SymbolTime(GI))*SymbolTime(GI);

%disp(['RTS = ' num2str(RTS_tx_time) '  CTS =' num2str(CTS_tx_time) '  NDP= ' num2str(NDP) '  NDP response = ' num2str(NDP_response) ]);
CW_min = 4; % 15 = 2^4-1
CW_max = 10; % 1023 = 2^10-1
% Set dynamic parameters in simulation
backoff_counter = zeros(numSTAs, 1);
backoff_attempt = zeros(numSTAs, 1);
nav = []; for i=1:numSTAs, nav(i).start=0; nav(i).end=0; end
packet_id = zeros(numSTAs, 1); % id for next MAC or NET packet
pending_id = zeros(numSTAs, 1); % id of current transmitting MAC packet, used for timeout
% ========= Other parameters ==========
traffic_queue = []; for i=1:Num_AP, traffic_queue(i).list = []; traffic_queue(i).size = [];end % Traffic queue for AP because non-AP STA only transmits to the associated AP
mac_status = []; for i=1:numSTAs, mac_status(i) = 0; end % Distinguish whether a traffic going medium access control or not
queue_size = []; for i=1:numSTAs, queue_size(i) = 0; end % Size of each STA traffic queue
soundingperiod = 50*1e-3; % 50 ms
% Variables for gathering statics
statics_rv_bits = zeros(numSTAs,1); % total correct rv bits
DataSentSucceessed = zeros(numSTAs,1);
DataSent = zeros(numSTAs,1);
DataSentFailNotRx = zeros(numSTAs,1);
DataSentFailNoACK = zeros(numSTAs,1);
Interferencepkt = 0;
InterferenceDatapkt = 0;
InterferenceCtrlpkt = 0;
CoMPdone = 0;
CoMPfailbutsave = 0;
CoMPfail = 0;
DebugDataTime = zeros(numSTAs,1);
DebugBOTime = zeros(numSTAs,1);