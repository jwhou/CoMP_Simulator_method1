%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: Jing-Wen Hou
% Email     : aasss369@gmail.com
% Version   : 0.9
% Date      : 2017/11/25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Nsym = num_symbol(pkt_type)

global GI SymbolTime;
global Mode_AMPDU_AMSDU;
global size_MAC_body size_MAC_header size_RTS size_CTS size_ACK size_BA size_NDP_Ann size_ReportPoll size_CVBFReport; 
global basic_rate SIFS num_msdu;

% The time unit is in second
switch pkt_type
    
    case 'RTS'
        Nsym = ceil((size_RTS / basic_rate)/SymbolTime(GI));
    case 'CTS'
        Nsym = ceil((size_CTS / basic_rate)/SymbolTime(GI));
    case 'ACK'
        if (Mode_AMPDU_AMSDU == 0)
            Nsym = ceil((size_BA / basic_rate)/SymbolTime(GI));
        else
            Nsym = ceil((size_ACK / basic_rate)/SymbolTime(GI));
        end
    case 'NDP_Ann'
        Nsym = ceil((size_NDP_Ann / basic_rate)/SymbolTime(GI));
    case 'Report_P'
        Nsym = ceil((size_ReportPoll / basic_rate)/SymbolTime(GI));
    case 'Compressed_BF'
        Nsym = ceil((size_CVBFReport / basic_rate)/SymbolTime(GI));
	case 'Data'
        Nsym = 1000;
%         if (Mode_AMPDU_AMSDU == 0)
%             % A-MPDU
%             Delimeter = 4*8; % 4 Octects;
%             FCS_size=4*8;
%             Num_Aggr = pkt.size./ size_MAC_body/num_msdu;
%             txtime =  txtime + ceil(max(((size_MAC_header+Delimeter+FCS_size)*Num_Aggr)/basic_rate+(pkt.size./pkt.rate))/SymbolTime(GI))*SymbolTime(GI);
%         else
%             % A-MSDU
%             txtime = txtime + ceil(max((size_MAC_header+pkt.size)./pkt.rate)/SymbolTime(GI))*SymbolTime(GI);
%         end
	otherwise
		disp(['num_symbol: wrong packet type: ' pkt.type]);		
end

end

