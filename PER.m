%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 17.0
% Date      : 2013/9/16
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ PacketErrorRate ] = PER(MCS, PacketLength, SNR)
% Packet error rate is reference to 
% Simplifying the Configuration of 802.11 Wireless Networks with Effective SNR
%
%   Output: 
%           PacketErrorRate
%   Input: 
%           Modulation and coding scheme
%			PacketLength (bit)
%           SNR
global Mode_AMPDU_AMSDU;
global size_MAC_body;

% [BPSK 1/2, QPSK 1/2, QPSK 3/4, 16-QAM 1/2, 16-QAM 3/4, 64-QAM 2/3, 64-QAM 3/4, 64-QAM 5/6, 256-QAM 3/4, 256-QAM 5/6]	
if (MCS == 1) % BPSK
	BER = qfunc(sqrt(2*SNR));
%     SNR_r = qfuncinv(BER).^2/2
elseif (MCS == 2 || MCS == 3) % QPSK
	BER = qfunc(sqrt(SNR));
%     SNR_r = qfuncinv(BER).^2
elseif (MCS == 4 || MCS == 5) % 16-QAM
	BER = 3/4*qfunc(sqrt(SNR/5));
%     SNR_r = qfuncinv(BER * 4/3).^2*5
elseif (MCS == 6 || MCS == 7 || MCS == 8) % 64-QAM
	BER = 7/12*qfunc(sqrt(SNR/21));
%     SNR_r = qfuncinv(12/7*BER).^2*21
else % 256-QAM
	BER = 15/32*qfunc(sqrt(SNR/85));
%     SNR_r = qfuncinv(32/15*BER).^2*85
end

if (Mode_AMPDU_AMSDU == 0)
    % A-MPDU, size_MAC_body is calculated for packet error rate
    PacketErrorRate = 1 - (1 - BER)^size_MAC_body;
else
    % A-MSDU, PacketLength = Num_Aggr*size_MAC_body is calculated for packet error rate
    PacketErrorRate = 1 - (1 - BER)^PacketLength;
end
% BER_r = 1-(1-PacketErrorRate)^(1/PacketLength)