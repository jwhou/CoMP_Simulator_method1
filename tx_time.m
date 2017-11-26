%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 17.0
% Date      : 2013/9/16
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function txtime = tx_time(pkt)

global GI SymbolTime;
global Mode_AMPDU_AMSDU;
global size_MAC_body size_MAC_header size_RTS size_CTS size_ACK size_BA T_VHTPHY T_VHT_LTF NDPTime NDPAnnTime size_NDP_Ann CompressedBeamformingTime ReportPollTime size_ReportPoll size_CompressedBeamforming; 
global basic_rate SIFS size_CVBFReport num_msdu;

% The time unit is in second
switch pkt.type
    
    
	case 'NDP'
        NDPTime = 44*1e-6;
        NDPAnnTime =  T_VHTPHY + ceil((size_NDP_Ann / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
		txtime = NDPTime + NDPAnnTime + SIFS; %like RTS
    case 'NDP_response'
        CompressedBeamformingTime =  ceil(size_CVBFReport/basic_rate/SymbolTime(GI))*SymbolTime(GI);
        ReportPollTime =  T_VHTPHY + ceil((size_ReportPoll / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
        txtime =  ReportPollTime + CompressedBeamformingTime + SIFS; %like CTS
	case 'Data'
        txtime = T_VHTPHY + (length(pkt.rv)-1)*T_VHT_LTF;
        if (Mode_AMPDU_AMSDU == 0)
            % A-MPDU
            Delimeter = 4*8; % 4 Octects;
            FCS_size=4*8;
            Num_Aggr = pkt.size./ size_MAC_body/num_msdu;
            txtime =  txtime + ceil(max(((size_MAC_header+Delimeter+FCS_size)*Num_Aggr)/basic_rate+(pkt.size./pkt.rate))/SymbolTime(GI))*SymbolTime(GI);
        else
            % A-MSDU
            txtime = txtime + ceil(max((size_MAC_header+pkt.size)./pkt.rate)/SymbolTime(GI))*SymbolTime(GI);
        end
    case 'ACK'
        if (Mode_AMPDU_AMSDU == 0)
            txtime = T_VHTPHY + ceil((size_BA / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
        else
            txtime = T_VHTPHY + ceil((size_ACK / basic_rate)/SymbolTime(GI))*SymbolTime(GI);
        end
	otherwise
		disp(['tx_time: wrong packet type: ' pkt.type]);		
end

end

