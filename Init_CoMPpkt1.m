%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [CoMP_pkt] = Init_CoMPpkt1(old_pkt, i, t)
% Synchronize CoMP packet for each CoMP AP
%   Output: 
%           CoMP_pkt: update CoMP packet
%   Input: 
%           old_pkt: a STA state and packet detail
%           i: CoMP AP index
%           t: time for start initilizing CoMP packet
global Num_Tx Num_Rx;
global MCSAlgo per_order MCS;
global BW GI R_data;
global size_MAC_body SIFS CSITime ACK_tx_time CTS_tx_time RTS_tx_time NDP NDP_response;
global NDPTime NDPAnnTime ReportPollTime CompressedBeamformingTime;
global soundingperiod;
global STA STA_Info CoMP_Controller;
global Num_AP;
global RTS_method;
global traffic_queue;
global PHY_CH_module;
global num_msdu;
global MCS_ctrl;

CoMP_pkt = old_pkt;
CoMP_pkt.status = 0;  %0:no edge-user, 1:one edge user, 2:two edge user
CoMP_pkt.rv = [CoMP_Controller.information([i, STA_Info(i).CoMP_coordinator]).rv];
CoMP_pkt.size = [CoMP_Controller.information([i, STA_Info(i).CoMP_coordinator]).size];


if (CoMP_Controller.CoMP_tx_mode == 1) % CS/CB
    selfrvNum = length(CoMP_Controller.information(i).rv);
    cooprvNum = length(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).rv);
    selfrv = CoMP_Controller.information(i).rv;
    cooprv = CoMP_Controller.information(STA_Info(i).CoMP_coordinator).rv;
    selfAP = i;
    coopAP = STA_Info(i).CoMP_coordinator;
    maxEdgeSTA = Num_Tx/Num_Rx;
    selfCenterSTANum = 0;
    selfCenterrv = [];
    coopCenterSTANum = 0;
    coopCenterrv = [];
    selfEdgeSTANum = 0;
    selfEdgerv = [];
    coopEdgeSTANum = 0;
    coopEdgerv = [];
    for k=1:selfrvNum
        STA_index = selfrv(k);
        if (ismember(STA_index, STA_Info(coopAP).cover_STA) && selfEdgeSTANum < (maxEdgeSTA-1))
            selfEdgeSTANum = selfEdgeSTANum + 1;
            selfEdgerv = [selfEdgerv, STA_index];
        elseif (~ismember(STA_index, STA_Info(coopAP).cover_STA))
            selfCenterSTANum = selfCenterSTANum + 1;
            selfCenterrv = [selfCenterrv, STA_index];
        end
    end
    for k=1:cooprvNum
        STA_index = cooprv(k);
        if (ismember(STA_index, STA_Info(selfAP).cover_STA) && coopEdgeSTANum < (maxEdgeSTA-1))
           coopEdgeSTANum = coopEdgeSTANum + 1;
           coopEdgerv = [coopEdgerv, STA_index];
        elseif (~ismember(STA_index, STA_Info(selfAP).cover_STA))
           coopCenterSTANum = coopCenterSTANum + 1;
           coopCenterrv = [coopCenterrv, STA_index];
        end
    end
    SumEdgeSTANum = (selfEdgeSTANum + coopEdgeSTANum);
    longdec = 0;
    shortdec = 0;
    while SumEdgeSTANum > maxEdgeSTA
        if longdec > shortdec
            shortdec = shortdec + 1;
        else
            longdec = longdec + 1;
        end
        SumEdgeSTANum = SumEdgeSTANum - 1;
    end
    
    if selfEdgeSTANum > coopEdgeSTANum
        selfEdgeSTANum = selfEdgeSTANum - longdec;
        coopEdgeSTANum = coopEdgeSTANum - shortdec;
    elseif selfEdgeSTANum < coopEdgeSTANum
        selfEdgeSTANum = selfEdgeSTANum - shortdec;
        coopEdgeSTANum = coopEdgeSTANum - longdec;
    else
        selfEdgeSTANum = selfEdgeSTANum - longdec;
        coopEdgeSTANum = coopEdgeSTANum - longdec;
    end
    
    if ~isempty(selfEdgerv)
        selfEdgerv = selfEdgerv(1:selfEdgeSTANum);
    end
    if ~isempty(coopEdgerv)
        coopEdgerv = coopEdgerv(1:coopEdgeSTANum);
    end
    
    CenterAn = Num_Tx/Num_Rx - (selfEdgeSTANum + coopEdgeSTANum);
    
    if CenterAn <= selfCenterSTANum
        selfCenterrv = selfCenterrv(1:CenterAn);
    else
        extraCenterAn = CenterAn - selfCenterSTANum;
        temprv = [selfEdgerv, selfCenterrv];
        tempqueue = traffic_queue(i).list;
        for k=1:length(temprv)
            tempqueue = tempqueue(tempqueue ~= temprv(k));
        end
        for k=1:extraCenterAn
            newCenter = tempqueue(find(~ismember(tempqueue,STA_Info(coopAP).cover_STA),1));
            if isempty(newCenter)
                break;
            end
            selfCenterrv = [selfCenterrv, newCenter];
            tempqueue = tempqueue(tempqueue ~= newCenter);
        end
    end
    edgerv = [selfEdgerv,coopEdgerv];
    edgerv = sort(edgerv);
    CoMP_pkt.rv = [edgerv, selfCenterrv];
    
elseif (CoMP_Controller.CoMP_tx_mode == 0) % JP
    CoMP_pkt.rv = CoMP_pkt.rv(1:min(length(CoMP_pkt.rv.'), Num_Tx/Num_Rx*(1+length(STA_Info(i).CoMP_coordinator))));
    CoMP_pkt.size = CoMP_pkt.size.';
end

CoMP_pkt.type = 'Data';
CoMP_pkt.CoMP = 1;
CoMP_pkt.MU = 1; % old_pkt.MU possible be null, once doing coMP pkt.MU must be 1 
CoMP_pkt.CoMPGroup = CoMP_pkt.rv;
CoMP_pkt.CoMPinOrder = sort(CoMP_pkt.rv);

for k=1:length(CoMP_pkt.CoMPinOrder)
    STA_index = CoMP_pkt.CoMPinOrder(k);
    if (ismember(STA_index, STA_Info(i).cover_STA) && ismember(STA_index, STA_Info(STA_Info(i).CoMP_coordinator).cover_STA))
        CoMP_pkt.status = CoMP_pkt.status + 1;
    end
end

STA_Info(i).HelpedSTA = []; STA_Info(i).HelpedSTA = CoMP_pkt.rv(STA(CoMP_pkt.rv, 3) ~= i);
STA_Info(i).IntendSTA = []; STA_Info(i).IntendSTA = CoMP_pkt.rv(STA(CoMP_pkt.rv, 3) == i);


CoMP_pkt.sounding_index = zeros(1, length(CoMP_pkt.rv));
soundingNum = 0;   %Record Number of STA need to sounding

for j=1:length(CoMP_pkt.rv)
    %if (any(CoMP_pkt.rv(j) == STA_Info(sel_AP).cover_STA))
    CoMP_pkt.sounding_index(1, j) = ((t - STA_Info(i).CSI_TS(CoMP_pkt.rv(j)-Num_AP)) >= soundingperiod);
    %         if sel_AP == i
    %             soundingNum = soundingNum + 1;
    %         end
    %else
    %         continue;
    %end
end
%end
CoMP_pkt.sum_sounding = sum(CoMP_pkt.sounding_index(1,:)); %only cal self

switch PHY_CH_module
    case 'old'
        if (MCSAlgo == 1)
            % Dynamic MCS
            SNR_record = estimate_SNR(CoMP_pkt);
            [CoMP_pkt.MCS_index, ~, Num_Aggregate] = PER_approximation(per_order, SNR_record, CoMP_pkt);
            CoMP_pkt.rate = R_data(CoMP_pkt.MCS_index, BW, GI, 1);
            CoMP_pkt.size = size_MAC_body*Num_Aggregate; 
        elseif (MCSAlgo == 0)
            % Static MCS
            Num_Aggregate = CoMP_pkt.size/size_MAC_body;
            CoMP_pkt.MCS_index = MCS*ones(length(CoMP_pkt.rv),1);
            CoMP_pkt.rate = R_data(CoMP_pkt.MCS_index, BW, GI, 1);
            CoMP_pkt.size = size_MAC_body*Num_Aggregate;
        end
    case 'new'
        if (MCSAlgo == 1)
            % Dynamic MCS
            if CoMP_pkt.sum_sounding > 0
                CoMP_pkt.MCS_index = MCS_ctrl*ones(1, length(CoMP_pkt.CoMPGroup));
            else
                CoMP_pkt.MCS_index = STA_Info(i).MCS_record(CoMP_pkt.CoMPGroup - Num_AP);
            end
            CoMP_pkt.rate = R_data(CoMP_pkt.MCS_index, BW, GI, Num_Rx);
            [Num_Aggregate,num_msdu]=fc_packet_aggregation(CoMP_pkt.rate,i,CoMP_pkt.CoMPGroup);
            CoMP_pkt.size = size_MAC_body*num_msdu*Num_Aggregate;
        elseif (MCSAlgo == 0)
            % Static MCS
            if CoMP_pkt.sum_sounding > 0
                CoMP_pkt.MCS_index = MCS_ctrl*ones(1, length(CoMP_pkt.CoMPGroup));
            else
                CoMP_pkt.MCS_index = MCS*ones(1, length(CoMP_pkt.CoMPGroup));
            end
            CoMP_pkt.rate = R_data(CoMP_pkt.MCS_index, BW, GI, Num_Rx);
            [Num_Aggregate,num_msdu]=fc_packet_aggregation(CoMP_pkt.rate,i,CoMP_pkt.CoMPGroup);
            CoMP_pkt.size = size_MAC_body*num_msdu*Num_Aggregate;
        end
end

CoMP_pkt.txtime = tx_time(CoMP_pkt);
switch RTS_method
    case 0
        if(t - CoMP_Controller.information(i).lastsounding < soundingperiod && CoMP_Controller.information(i).lastsounding ~= -1)
            CoMP_Controller.information(i).readyRTS = 1;
            CoMP_pkt.nav = length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'Data';
        elseif (CoMP_pkt.sum_sounding > 0)
            CoMP_Controller.information(i).readyRTS = 0;
            CoMP_pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(CoMP_pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
            CoMP_pkt.type = 'NDP_Ann';
        else
            CoMP_Controller.information(i).readyRTS = 1;
            CoMP_pkt.nav = length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'Data';
        end
    case 1
        if(t - CoMP_Controller.information(i).lastsounding < soundingperiod && CoMP_Controller.information(i).lastsounding ~= -1)
            CoMP_pkt.nav = (length(STA_Info(i).CoMP_coordinator))*(SIFS+RTS_tx_time)+length(CoMP_pkt.rv)*(SIFS+CTS_tx_time) + tx_time(CoMP_pkt) +...
                length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'RTS';
        elseif (CoMP_pkt.sum_sounding > 0)
            CoMP_pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (soundingNum - 1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
            CoMP_pkt.type = 'NDP_Ann';
        elseif (sum(CoMP_pkt.sounding_index(2,:)) > 0 || CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS == 1)
            CoMP_pkt.nav = 0;
            CoMP_pkt.type = 'NDP_Ann';
        else
            CoMP_pkt.nav = (length(STA_Info(i).CoMP_coordinator))*(SIFS+RTS_tx_time)+length(CoMP_pkt.rv)*(SIFS+CTS_tx_time) + tx_time(CoMP_pkt) +...
                length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'RTS';
        end
    case 2
end


end

