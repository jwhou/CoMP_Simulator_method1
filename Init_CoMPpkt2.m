%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [CoMP_pkt] = Init_CoMPpkt2(old_pkt, i, t)
% Synchronize CoMP packet for each CoMP AP
%   Output: 
%           CoMP_pkt: update CoMP packet
%   Input: 
%           old_pkt: a STA state and packet detail
%           i: CoMP AP index
%           t: time for start initilizing CoMP packet
global powercontrol_Debug;
global cover_range default_power low_cover_range low_power;
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
global scheme

CoMP_pkt = old_pkt;
CoMP_pkt.status = 0;  %0:no edge-user, 1:one edge user, 2:two edge user

if CoMP_Controller.information(i).numInform == 0
    AP_list = [i, STA_Info(i).CoMP_coordinator];
    Antenna_list = Num_Tx*ones(1,length(AP_list));
    CoMP_Controller.information(STA_Info(i).CoMP_coordinator).numInform = 1;
    STA_list = [];
    if length(AP_list)==2
        temp_queue{1} = traffic_queue(AP_list(1)).list; 
        temp_queue{2} = traffic_queue(AP_list(2)).list;
        if (CoMP_Controller.CoMP_tx_mode == 1) % CS/CB
            k = 1;
            while(1)
                AP_index = AP_list(k);
                if all(Antenna_list == 0), break; end
                if isempty(temp_queue{1}) && isempty(temp_queue{2}), break; end
                if Antenna_list(k) > 0
                    temp_queue1 = temp_queue{k};
                    temp_queue2 = temp_queue{rem(k+2,2)+1};
                    First_STA = temp_queue1(1);
                    Temp_STA1 = [];
                    Temp_STA2 = [];
                    Remain_AP = AP_list(AP_list~=AP_index);
                    if (ismember(First_STA,STA_Info(Remain_AP).cover_STA) && all(Antenna_list > Num_Rx))
                        Temp_STA1 = First_STA;
                        Temp_STA2 = temp_queue2(find(ismember(temp_queue2,STA_Info(AP_index).cover_STA),1));
                        if isempty(Temp_STA2)
                            Temp_STA2 = temp_queue2(find(~ismember(temp_queue2,STA_Info(AP_index).cover_STA),1));
                            if isempty(Temp_STA2)
                                Antenna_list(k) = Antenna_list(k) - Num_Rx;
                                Antenna_list((rem(k+2,2)+1)) = Antenna_list((rem(k+2,2)+1)) - Num_Rx;
                            else
                                Antenna_list(k) = Antenna_list(k) - Num_Rx;
                                Antenna_list((rem(k+2,2)+1)) = Antenna_list((rem(k+2,2)+1)) - Num_Rx*2;
                            end
                        else
                            Antenna_list(k) = Antenna_list(k) - Num_Rx*2;
                            Antenna_list((rem(k+2,2)+1)) = Antenna_list((rem(k+2,2)+1)) - Num_Rx*2;
                        end
                    elseif(ismember(First_STA,STA_Info(Remain_AP).cover_STA) && Antenna_list(rem(k+2,2)+1) > Num_Rx)
                        Temp_STA1 = First_STA;
                        Temp_STA2 = temp_queue2(find(~ismember(temp_queue2,STA_Info(AP_index).cover_STA),1));
                        if isempty(Temp_STA2)
                            Antenna_list(k) = Antenna_list(k) - Num_Rx;
                            Antenna_list((rem(k+2,2)+1)) = Antenna_list((rem(k+2,2)+1)) - Num_Rx;
                        else
                            Antenna_list(k) = Antenna_list(k) - Num_Rx;
                            Antenna_list((rem(k+2,2)+1)) = Antenna_list((rem(k+2,2)+1)) - Num_Rx*2;
                        end
                    elseif(~isempty(First_STA) && ~ismember(First_STA,STA_Info(Remain_AP).cover_STA))
                        Temp_STA1 = First_STA;
                        Antenna_list(k) = Antenna_list(k) - Num_Rx;
                    end
                    if isempty(Temp_STA1)
                        temp_queue{k} = tempqueue1(tempqueue1 ~= First_STA);
                    else
                        STA_list = [STA_list Temp_STA1 Temp_STA2];
                        temp_queue{k} = temp_queue1(temp_queue1 ~= Temp_STA1);
                        if ~isempty(Temp_STA2)
                            temp_queue{rem(k+2,2)+1} = temp_queue2(temp_queue2 ~= Temp_STA2);
                        end
                    end
                end
                k = rem(k+2,2)+1;
            end
        elseif (CoMP_Controller.CoMP_tx_mode == 0) % JP
        end
        for k=1:length(AP_list)
            CoMP_Controller.information(AP_list(k)).rv = STA_list;
        end
    else
        temp_queue{1} = traffic_queue(AP_list(1)).list; 
        temp_queue{2} = traffic_queue(AP_list(2)).list;
        temp_queue{3} = traffic_queue(AP_list(3)).list;
        if (CoMP_Controller.CoMP_tx_mode == 1) % CS/CB
            k = 1;
            while(1)
                AP_index = AP_list(k);
                if all(Antenna_list == 0), break; end
                if isempty(temp_queue{1}) && isempty(temp_queue{2}) && isempty(temp_queue{3}), break; end
                if Antenna_list(k) > 0
                    temp_queue1 = temp_queue{k};
                    temp_queue2 = temp_queue{rem(k+3,3)+1};
                    temp_queue3 = temp_queue{rem(k+4,3)+1};
                    First_STA = temp_queue1(1);
                    Temp_STA1 = [];
                    Temp_STA2 = [];
                    Temp_STA3 = [];
                    Remain_AP_list = AP_list~=AP_index;
                    Remain_AP1 = Remain_AP_list(1);
                    Remain_AP2 = Remain_AP_list(2);
                    if (ismember(First_STA,STA_Info(Remain_AP1).cover_STA) && ismember(First_STA,STA_Info(Remain_AP2).cover_STA) && all(Antenna_list > 2 * Num_Rx))
                        Temp_STA1 = First_STA;
                        Temp_STA2 = temp_queue2(find(ismember(temp_queue2,STA_Info(AP_index).cover_STA) & ismember(temp_queue2,STA_Info(Remain_AP2).cover_STA),1));
                        Temp_STA3 = temp_queue3(find(ismember(temp_queue3,STA_Info(AP_index).cover_STA) & ismember(temp_queue3,STA_Info(Remain_AP1).cover_STA),1));
                        if isempty(Temp_STA2)
                            Temp_STA2 = temp_queue2(find(~ismember(temp_queue2,STA_Info(AP_index).cover_STA) & ~ismember(temp_queue2,STA_Info(Remain_AP2).cover_STA),1));
                            if isempty(Temp_STA2)
                                Antenna_list(k) = Antenna_list(k) - Num_Rx;
                                Antenna_list((rem(k+3,3)+1)) = Antenna_list((rem(k+3,3)+1)) - Num_Rx;
                                Antenna_list((rem(k+4,3)+1)) = Antenna_list((rem(k+4,3)+1)) - Num_Rx;
                            else
                                Antenna_list(k) = Antenna_list(k) - Num_Rx;
                                Antenna_list((rem(k+3,3)+1)) = Antenna_list((rem(k+3,3)+1)) - Num_Rx*2;
                                Antenna_list((rem(k+4,3)+1)) = Antenna_list((rem(k+4,3)+1)) - Num_Rx;
                            end
                        else
                            Antenna_list(k) = Antenna_list(k) - Num_Rx*2;
                            Antenna_list((rem(k+3,3)+1)) = Antenna_list((rem(k+3,3)+1)) - Num_Rx*2;
                            Antenna_list((rem(k+4,3)+1)) = Antenna_list((rem(k+4,3)+1)) - Num_Rx*2;
                        end
                        if isempty(Temp_STA3)
                            Temp_STA3 = temp_queue3(find(~ismember(temp_queue3,STA_Info(AP_index).cover_STA) & ~ismember(temp_queue3,STA_Info(Remain_AP1).cover_STA),1));
                            if isempty(Temp_STA3)
                                %Nothing has to do here.
                            else
                                Antenna_list((rem(k+4,3)+1)) = Antenna_list((rem(k+4,3)+1)) - Num_Rx;
                            end
                        else 
                            Antenna_list(k) = Antenna_list(k) - Num_Rx;
                            Antenna_list((rem(k+3,3)+1)) = Antenna_list((rem(k+3,3)+1)) - Num_Rx;
                            Antenna_list((rem(k+4,3)+1)) = Antenna_list((rem(k+4,3)+1)) - Num_Rx;
                        end
                    elseif(ismember(First_STA,STA_Info(Remain_AP1).cover_STA) && ismember(First_STA,STA_Info(Remain_AP2).cover_STA) && all(Antenna_list >  Num_Rx))
                        Temp_STA1 = First_STA;
                        if(Antenna_list(k) < 3*Num_Rx)
                            
                        else
                        end
                    end
                end
                k = rem(k+3,3)+1;
            end
        elseif (CoMP_Controller.CoMP_tx_mode == 0) % JP
        end
    end
else
    STA_list = CoMP_Controller.information(i).rv;
    CoMP_Controller.information(i).numInform = 0;
end

STA_edge = [];
STA_center = [];
STA_center_other = [];
for k=1:length(STA_list)
    STA_index = STA_list(k);
    if(ismember(STA_index, STA_Info(STA_Info(i).CoMP_coordinator).cover_STA) && ismember(STA_index, STA_Info(i).cover_STA))
        STA_edge = [STA_edge STA_index];
    elseif(ismember(STA_index, STA_Info(i).associated_STA))
        STA_center = [STA_center STA_index];
    else
        STA_center_other = [STA_center_other STA_index];
    end
end
if (powercontrol_Debug == 1 && isempty(STA_edge))
    CoMP_pkt.power = low_power;
    CoMP_pkt.cover_range = low_cover_range;
else
    CoMP_pkt.power = default_power;
    CoMP_pkt.cover_range = cover_range;
end
CoMP_pkt.type = 'Data';
CoMP_pkt.CoMP = 1;
CoMP_pkt.MU = 1; % old_pkt.MU possible be null, once doing coMP pkt.MU must be 1 
CoMP_pkt.rv = [sort(STA_edge) sort(STA_center)];
CoMP_pkt.CoMPGroup = [sort(STA_edge) sort(STA_center)];
CoMP_pkt.CoMPinOrder = [sort(STA_edge) sort(STA_center)];
CoMP_pkt.CoMPAll = [sort(STA_edge) sort(STA_center)];
% CoMP_pkt.CoMPAll = [sort(STA_edge) sort([STA_center STA_center_other])];

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
            if(scheme == 3)
                CoMP_pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(CoMP_pkt.CoMPAll)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
            else
                CoMP_pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(CoMP_pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
            end
            CoMP_pkt.type = 'NDP_Ann';
        else
            CoMP_Controller.information(i).readyRTS = 1;
            CoMP_pkt.nav = length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'Data';
        end
    case 1
        if(t - CoMP_Controller.information(i).lastsounding < soundingperiod && CoMP_Controller.information(i).lastsounding ~= -1)
            CoMP_Controller.information(i).readyRTS = 1;
            CoMP_pkt.nav = (length(STA_Info(i).CoMP_coordinator))*(SIFS+RTS_tx_time)+length(CoMP_pkt.rv)*(SIFS+CTS_tx_time)+SIFS+tx_time(CoMP_pkt) +...
                length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'RTS';
        elseif (CoMP_pkt.sum_sounding > 0)
            CoMP_Controller.information(i).readyRTS = 0;
            if(scheme == 3)
                CoMP_pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(CoMP_pkt.CoMPAll)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
            else
                CoMP_pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(CoMP_pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
            end
            CoMP_pkt.type = 'NDP_Ann';
        else
            CoMP_Controller.information(i).readyRTS = 1;
            CoMP_pkt.nav = (length(STA_Info(i).CoMP_coordinator))*(SIFS+RTS_tx_time)+length(CoMP_pkt.rv)*(SIFS+CTS_tx_time)+SIFS+tx_time(CoMP_pkt) +...
                length(CoMP_pkt.rv)*(SIFS+ACK_tx_time);
            CoMP_pkt.type = 'RTS';
        end
    case 2
end


end