%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [MCS, PER_mapping, Num_Aggregate] = PER_approximation(PacketErrorRate, SNR_record, pkt)
% Packet error rate is reference to 
% Simplifying the Configuration of 802.11 Wireless Networks with Effective SNR
%
%   Output: 
%           Modulation and coding scheme:
%           PER_mapping: debugging compared to PER.m
%           Num_Aggregate: the number of packets aggregation 
%   Input: 
%           PacketErrorRate: what is the packet error rate order wanted
%           SNR_record: Before starting to transmit a PPDU, the STA should calculate the SNR of user(s)
%           pkt: packet struct

global queue_Debug;
global BW GI R_data;
global Mode_AMPDU_AMSDU;
global size_MAC_body;
global traffic_queue queue_size;
global STA STA_Info;
% [BPSK 1/2, QPSK 1/2, QPSK 3/4, 16-QAM 1/2, 16-QAM 3/4, 64-QAM 2/3, 64-QAM 3/4, 64-QAM 5/6, 256-QAM 3/4, 256-QAM 5/6]	

if (length(SNR_record) ~= length(pkt.rv))
    error('Error using PER_approximation, length of SNR_record is not equal to length of pkt.rv');
end

i = pkt.tx;
j = pkt.rv;
MCS = -1*ones(length(SNR_record), 1);
PER_mapping = -1*ones(length(SNR_record), 1);
Num_Aggregate = -1*ones(length(SNR_record), 1);

% Retreive traffic_queue(i).size(j)
traffic_size = zeros(1, length(j));
for k=1:length(j)
    AP_index = STA(j(k), 3);
    traffic_size(k) = traffic_queue(AP_index).size(STA_Info(AP_index).associated_STA==j(k));
end
if queue_Debug
    disp(['  -Q: @ PER_approximation.m: ']);
    if (pkt.CoMP == 0)
        disp(['  -Q: STA ' num2str(i) ' has traffic_queue.size(' num2str(STA_Info(i).associated_STA) ') = [' num2str(traffic_queue(i).size) ']']);
        disp(['  -Q: traffic_queue.size(' num2str(j) ') = [' num2str(traffic_size) ']']);
    else % if (pkt.CoMP == 1)
        disp(['  -Q: STA ' num2str(i) ' has traffic_queue.size(' num2str(STA_Info(i).associated_STA) ') = [' num2str(traffic_queue(i).size) ']']);
        disp(['  -Q: STA ' num2str(STA_Info(i).CoMP_coordinator) ' has traffic_queue.size(' num2str(STA_Info(STA_Info(i).CoMP_coordinator).associated_STA) ') = [' num2str(traffic_queue(STA_Info(i).CoMP_coordinator).size) ']']);
        disp(['  -Q: traffic_queue.size(' num2str(j) ') = [' num2str(traffic_size) ']']);
    end
end
% BPSK 1/2
BER = qfunc(sqrt(2*SNR_record));
max_Aggrregate = ceil(R_data(1, BW, GI, 1)/R_data(1, BW, GI, 1));
if (STA(i, 3) == 0)
    Aggrregate = min(traffic_size.' , max_Aggrregate);
else % if (STA(tx, 3) ~= 0)
    Aggrregate = min(queue_size(i), max_Aggrregate);
end
if (Mode_AMPDU_AMSDU == 0)
    % A-MPDU, size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body;
else
    % A-MSDU, Num_Agg*size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body*Aggrregate;
end
Calc_PER = 1 - (1 - BER).^PacketLength;
if (any(Calc_PER <= PacketErrorRate))
    MCS(Calc_PER <= PacketErrorRate) = 1;
    Num_Aggregate(Calc_PER <= PacketErrorRate) = Aggrregate(Calc_PER <= PacketErrorRate);
    PER_mapping(Calc_PER <= PacketErrorRate) = Calc_PER(Calc_PER <= PacketErrorRate);
end

% QPSK 3/4
BER = qfunc(sqrt(SNR_record));
max_Aggrregate = ceil(R_data(3, BW, GI, 1)/R_data(1, BW, GI, 1));
if (STA(i, 3) == 0)
    Aggrregate = min(traffic_size.' , max_Aggrregate);
else % if (STA(tx, 3) ~= 0)
    Aggrregate = min(queue_size(i), max_Aggrregate);
end
if (Mode_AMPDU_AMSDU == 0)
    % A-MPDU, size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body;
else
    % A-MSDU, Num_Agg*size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body*Aggrregate;
end
Calc_PER = 1 - (1 - BER).^PacketLength;
if (any(Calc_PER <= PacketErrorRate))
    MCS(Calc_PER <= PacketErrorRate) = 3;
    Num_Aggregate(Calc_PER <= PacketErrorRate) = Aggrregate(Calc_PER <= PacketErrorRate);
    PER_mapping(Calc_PER <= PacketErrorRate) = Calc_PER(Calc_PER <= PacketErrorRate);
end

% 16-QAM 3/4
BER = 3/4*qfunc(sqrt(SNR_record/5));
max_Aggrregate = ceil(R_data(5, BW, GI, 1)/R_data(1, BW, GI, 1));
if (STA(i, 3) == 0)
    Aggrregate = min(traffic_size.' , max_Aggrregate);
else % if (STA(tx, 3) ~= 0)
    Aggrregate = min(queue_size(i), max_Aggrregate);
end
if (Mode_AMPDU_AMSDU == 0)
    % A-MPDU, size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body;
else
    % A-MSDU, Num_Agg*size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body*Aggrregate;
end
Calc_PER = 1 - (1 - BER).^PacketLength;
if (any(Calc_PER <= PacketErrorRate))
    MCS(Calc_PER <= PacketErrorRate) = 5;
    Num_Aggregate(Calc_PER <= PacketErrorRate) = Aggrregate(Calc_PER <= PacketErrorRate);
    PER_mapping(Calc_PER <= PacketErrorRate) = Calc_PER(Calc_PER <= PacketErrorRate);
end

% 64-QAM 5/6
BER = 7/12*qfunc(sqrt(SNR_record/21));
max_Aggrregate = ceil(R_data(8, BW, GI, 1)/R_data(1, BW, GI, 1));
if (STA(i, 3) == 0)
    Aggrregate = min(traffic_size.' , max_Aggrregate);
else % if (STA(tx, 3) ~= 0)
    Aggrregate = min(queue_size(i), max_Aggrregate);
end
if (Mode_AMPDU_AMSDU == 0)
    % A-MPDU, size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body;
else
    % A-MSDU, Num_Agg*size_MAC_body is calculated for packet error rate
    PacketLength = size_MAC_body*Aggrregate;
end
Calc_PER = 1 - (1 - BER).^PacketLength;
if (any(Calc_PER <= PacketErrorRate))
    MCS(Calc_PER <= PacketErrorRate) = 8;
    Num_Aggregate(Calc_PER <= PacketErrorRate) =  Aggrregate(Calc_PER <= PacketErrorRate);
    PER_mapping(Calc_PER <= PacketErrorRate) = Calc_PER(Calc_PER <= PacketErrorRate);
end

MCS(MCS == -1) = 1;
Num_Aggregate(Num_Aggregate == -1) = 1;
if queue_Debug
    disp(['  -Q: Num_Aggregate(' num2str(j) ') = [' num2str(Num_Aggregate.') ']']);
end
end

