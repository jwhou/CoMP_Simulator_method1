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
% action.m include PHY and MAC layers,
% Consecutive RTS CoMP prtocols

function [NewEvents] = action_CoMP_method1(event,PHY_CH_module,Nsubcarrier,channel_model,Bandwidth,Thermal_noise,tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,AI,select_user)
% parameter.m
global event_Debug detail_Debug MIMO_Debug queue_Debug powercontrol_Debug control_frame_Debug sounding_skipevent_Debug;
global Num_Tx Num_Rx;
global MCSAlgo per_order MCS MCS_ctrl;
global BW GI R_data freq;
global default_power;
global rmodel;
global size_MAC_body size_RTS size_CTS size_ACK size_BA size_ReportPoll size_CompressedBeamforming size_NDP_Ann size_CVBFReport;
global slotTime CSITime SIFS DIFS aPHY_RX_START_Delay cca_time;
global ACK_tx_time CTS_tx_time RTS_tx_time CW_min CW_max NDPTime NDPAnnTime ReportPollTime CompressedBeamformingTime;
global backoff_counter backoff_attempt;
global nav pending_id;
global traffic_queue mac_status queue_size soundingperiod;
global STA STA_Info CoMP_Controller;
global Num_AP Num_User;
global statics_rv_bits;
global DataSentSucceessed DataSent DataSentFailNotRx DataSentFailNoACK;
global Interferencepkt InterferenceDatapkt InterferenceCtrlpkt;
global CoMPfail CoMPfailbutsave CoMPdone;
global DebugDataTime DebugBOTime;
global RTS_method;
global lastsounding_enable;
global cover_range;
global Max_Report_P;
global num_msdu;
global spatial_stream;
global tx_interval; % added by jing-wen
global interference_queue; % added by jing-wen

NewEvents = [];

switch event.type
	case 'idle'
        % Generate data traffic event(idle) for each user within traffic arrival rate not equal to zero
		t = event.timer;
		i = event.STA_ID;
        % STA(i, 4) is the traffic arrival rate of STA i
        if (STA(i, 4) ~= 0)
            % STA i's traffic(STA(i, 4) ~= 0) arrival rate ~= 0
            TempEvent = event;
            TempEvent.timer = t + slotTime;
            TempEvent.type = 'idle';
            TempEvent.STA_ID = i;
            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
        end
        % STA(i,3) is an indicator to the associated AP index, 0 is an AP; otherwise, AP index
        if (STA(i, 3) == 0 && STA(i, 4) ~= 0)
            % DL, the AP selects user(s) to form the PPDU destinations
            if (STA(i, 4)*slotTime >= rand(1))
				TempEvent = event;
				TempEvent.timer =  t + slotTime;
				TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
				TempEvent.pkt.type = 'Data';
				TempEvent.pkt.tx = i;
				TempEvent.pkt.rv = STA_Info(i).associated_STA(randperm(STA_Info(i).num_ass));
				TempEvent.pkt.rv = TempEvent.pkt.rv(1);
				NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            end
        elseif (STA(i, 3) ~= 0 && STA(i, 4) ~= 0)
        % UL, the non-AP STA forms PPDU to its associated AP , Deleted by jingwen
        end
	case 'send_MAC'
		t = event.timer;
        i = event.STA_ID;
        % pkt information from idle state is not empty, thus store into STA i's traffic queue
        if (~isempty(event.pkt))
            % Trick: index helps running faster than aggrgation (e.g., A = [A, i]) with growing size
            queue_size(i) = queue_size(i) + 1;
            if (STA(i, 3) == 0)
                traffic_queue(i).size(event.pkt.rv == STA_Info(i).associated_STA) = traffic_queue(i).size(event.pkt.rv == STA_Info(i).associated_STA) + 1;
                traffic_queue(i).list(queue_size(i)) = event.pkt.rv;
            end
        else
            % pkt information is empty where last PPDU is succeed or failed, thus set STA i is not using medium
            % STA i will start next PPDU transmission if its traffic queue is not empty
            mac_status(i) = 0;
        end
        % First PPDU transmission and start next PPDU transmssion
        if (queue_size(i) ~= 0 && mac_status(i) == 0)
            if event_Debug, disp(['[' num2str(t, '%1.20f') ']: send_MAC @ STA ' num2str(i)]); end
            mac_status(i) = 1;
            % Fetch a PPDU pkt out of queue
            TempEvent = event;
            TempEvent.timer = t + cca_time;
            TempEvent.type = 'wait_for_channel';
            TempEvent.STA_ID = i;
            TempEvent.pkt.type = 'Data';
            TempEvent.pkt.power = default_power; % transmission power for current pkt
            TempEvent.pkt.id = new_id(i); % id for current packet
            TempEvent.pkt.tx = i;
            TempEvent.pkt.sum_sounding=0;
            if (STA(i, 3) == 0)
                num_ant = Num_Tx/spatial_stream;
                % Using FIFO to select users
                temp_queue = traffic_queue(i).list;
                select_rv = temp_queue(1);
                num_ant = num_ant - 1;
                temp_queue = temp_queue(temp_queue ~= select_rv);
                while num_ant ~= 0
                    newSTA = temp_queue(find(temp_queue,1));
                    if isempty(newSTA)
                        break;
                    end
                    select_rv = [select_rv, newSTA];
                    temp_queue = temp_queue(temp_queue ~= newSTA);
                    num_ant = num_ant - 1;
                end
                if any(ismember(select_rv, STA_Info(i).edge_STA))
                    TempEvent.pkt.power = default_power;
                    TempEvent.pkt.cover_range = cover_range;
                else
                    if (powercontrol_Debug == 1)
                        STA_distence = [];
                        for k=1:length(select_rv)
                            STA_index = select_rv(k);
                            STA_distence = [STA_distence, sqrt((STA(i, 1)-STA(STA_index, 1))^2+(STA(i, 2)-STA(STA_index, 2))^2)];
                        end
                        d = max(STA_distence);
                        TempEvent.pkt.power = log_normal_shadowingR(d);
                        TempEvent.pkt.cover_range = d;
                    else
                        TempEvent.pkt.power = default_power;
                        TempEvent.pkt.cover_range = cover_range;
                    end
                end
                TempEvent.pkt.rv = select_rv;
                if (length(select_rv) == 1)
                    TempEvent.pkt.MU = 0;
                else
                    TempEvent.pkt.MU = 1;
                end
                TempEvent.pkt.CoMP = 0;
                %==============channel module==============%
                switch PHY_CH_module
                    case 'old'
                        if (TempEvent.pkt.MU == 0 && TempEvent.pkt.CoMP == 0)
                            if (MCSAlgo == 1)
                                % Dynamic MCS
                                SNR_record = estimate_SNR(TempEvent.pkt);
                                [TempEvent.pkt.MCS_index, ~, Num_Aggregate] = PER_approximation(per_order, SNR_record, TempEvent.pkt);
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, 1);
                                TempEvent.pkt.size = size_MAC_body*Num_Aggregate;
                            elseif (MCSAlgo == 0)
                                % Static MCS
                                Num_Aggregate = ceil(R_data(MCS, BW, GI, 1)/R_data(MCS_ctrl, BW, GI, 1));
                                TempEvent.pkt.MCS_index = MCS;
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, 1);
                                TempEvent.pkt.size = size_MAC_body*Num_Aggregate;
                            end
                            
                            TempEvent.pkt.sounding_index = ((t - STA_Info(i).CSI_TS([TempEvent.pkt.rv]-Num_AP)) >= soundingperiod);
                            TempEvent.pkt.sum_sounding = sum(TempEvent.pkt.sounding_index);
                            switch RTS_method
                                case 0
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime;
                                    else
                                        TempEvent.pkt.nav = SIFS + ACK_tx_time;
                                    end
                                case 1
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime;
                                    else
                                        TempEvent.pkt.nav = SIFS + CTS_tx_time + SIFS +tx_time(TempEvent.pkt)+ SIFS + ACK_tx_time;
                                    end
                            end
                            
                        elseif (TempEvent.pkt.MU == 1 && TempEvent.pkt.CoMP == 0)
                            % Only AP could transmit MU-PPDU
                            if (MCSAlgo == 1)
                                % Dynamic MCS
                                SNR_record = estimate_SNR(TempEvent.pkt);
                                [TempEvent.pkt.MCS_index, ~, Num_Aggregate] = PER_approximation(per_order, SNR_record, TempEvent.pkt);
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, 1);
                                TempEvent.pkt.size = size_MAC_body*Num_Aggregate;
                            elseif (MCSAlgo == 0)
                                % Static MCS
                                Num_Aggregate = ones(length(TempEvent.pkt.rv), 1)*ceil(R_data(MCS, BW, GI, 1)/R_data(MCS_ctrl, BW, GI, 1));
                                TempEvent.pkt.MCS_index = MCS*ones(length(TempEvent.pkt.rv),1);
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, 1);
                                TempEvent.pkt.size = size_MAC_body*Num_Aggregate;
                            end
                            
                            TempEvent.pkt.sounding_index = ((t - STA_Info(i).CSI_TS([TempEvent.pkt.rv]-Num_AP)) >= soundingperiod);
                            TempEvent.pkt.sum_sounding = sum(TempEvent.pkt.sounding_index);
                            switch RTS_method
                                case 0
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
                                    else
                                        TempEvent.pkt.nav = length(TempEvent.pkt.rv)*(SIFS + ACK_tx_time);
                                    end
                                case 1
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
                                    else
                                        TempEvent.pkt.nav = length(TempEvent.pkt.rv)*(SIFS+CTS_tx_time)+SIFS+tx_time(TempEvent.pkt)+length(TempEvent.pkt.rv)*(SIFS+ACK_tx_time);
                                    end
                            end
                        end
                    case 'new'
                        if (TempEvent.pkt.MU == 0 && TempEvent.pkt.CoMP == 0)
                            TempEvent.pkt.sounding_index = ((t - STA_Info(i).CSI_TS([TempEvent.pkt.rv]-Num_AP)) >= soundingperiod);
                            TempEvent.pkt.sum_sounding = sum(TempEvent.pkt.sounding_index);
                            if (MCSAlgo == 1) 
                                % Dynamic MCS
                                if TempEvent.pkt.sum_sounding > 0
                                    TempEvent.pkt.MCS_index = MCS_ctrl;
                                else
                                    TempEvent.pkt.MCS_index = STA_Info(i).MCS_record(select_rv - Num_AP);
                                end
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, spatial_stream);
                                [Num_Aggregate,num_msdu]=fc_packet_aggregation(TempEvent.pkt.rate,i,select_rv);
                                TempEvent.pkt.size = size_MAC_body*num_msdu*Num_Aggregate;
                            elseif (MCSAlgo == 0)
                                % Static MCS
                                if TempEvent.pkt.sum_sounding > 0
                                    TempEvent.pkt.MCS_index = MCS_ctrl;
                                else
                                    TempEvent.pkt.MCS_index = MCS;
                                end
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, spatial_stream);
                                [Num_Aggregate,num_msdu]=fc_packet_aggregation(TempEvent.pkt.rate,i,select_rv);
                                TempEvent.pkt.size = size_MAC_body*num_msdu*Num_Aggregate;
                            end
                            switch RTS_method
                                case 0
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime;
                                    else
                                        TempEvent.pkt.nav = SIFS + ACK_tx_time;
                                    end
                                case 1
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime;
                                    else
                                        TempEvent.pkt.nav = SIFS + CTS_tx_time + SIFS +tx_time(TempEvent.pkt)+ SIFS + ACK_tx_time;
                                    end
                            end
                        elseif (TempEvent.pkt.MU == 1 && TempEvent.pkt.CoMP == 0)
                            TempEvent.pkt.sounding_index = ((t - STA_Info(i).CSI_TS([TempEvent.pkt.rv]-Num_AP)) >= soundingperiod);
                            TempEvent.pkt.sum_sounding = sum(TempEvent.pkt.sounding_index);
                            % Only AP could transmit MU-PPDU                   
                            if (MCSAlgo == 1) 
                                % Dynamic MCS
                                if TempEvent.pkt.sum_sounding > 0
                                    TempEvent.pkt.MCS_index = MCS_ctrl*ones(1, length(select_rv));
                                else
                                    TempEvent.pkt.MCS_index = STA_Info(i).MCS_record(select_rv - Num_AP);
                                end
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, spatial_stream);
                                [Num_Aggregate,num_msdu]=fc_packet_aggregation(TempEvent.pkt.rate,i,select_rv);
                                TempEvent.pkt.size = size_MAC_body*num_msdu*Num_Aggregate;
                            elseif (MCSAlgo == 0)
                                % Static MCS
                                if TempEvent.pkt.sum_sounding > 0
                                    TempEvent.pkt.MCS_index = MCS_ctrl*ones(1, length(select_rv));
                                else
                                    TempEvent.pkt.MCS_index = MCS*ones(1, length(TempEvent.pkt.rv));
                                end
                                TempEvent.pkt.rate = R_data(TempEvent.pkt.MCS_index, BW, GI, spatial_stream);
                                [Num_Aggregate,num_msdu]=fc_packet_aggregation(TempEvent.pkt.rate,i,select_rv);
                                TempEvent.pkt.size = size_MAC_body*num_msdu*Num_Aggregate;
                            end
                            switch RTS_method
                                case 0
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
                                    else
                                        TempEvent.pkt.nav = length(TempEvent.pkt.rv)*(SIFS + ACK_tx_time);
                                    end
                                case 1
                                    if TempEvent.pkt.sum_sounding > 0
                                        TempEvent.pkt.nav = SIFS + NDPTime + SIFS + CompressedBeamformingTime + (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
                                    else
                                        TempEvent.pkt.nav = length(TempEvent.pkt.rv)*(SIFS+CTS_tx_time)+SIFS+tx_time(TempEvent.pkt)+length(TempEvent.pkt.rv)*(SIFS+ACK_tx_time);
                                    end
                            end
                        end
                end
                %==============channel module==============%
                % STA i is an AP and already knows which APs are coordinators of CoMP transmission
                if (~isempty(STA_Info(i).CoMP_coordinator))
                    % A CoMP AP informs the CoMP_Controller to initialize CoMP transmission
                    % Otherwise, STA i is a non-AP STA or an AP without CoMP coordinators
                    CoMP_Controller.informer_index(i == CoMP_Controller.connector) = 1;
                    CoMP_Controller.information(i).rv = TempEvent.pkt.rv;
                    CoMP_Controller.information(i).size = size_MAC_body*Num_Aggregate.';
                end
            else %if (STA(i, 3) ~= 0)
                %for uplink deleted by jingwen
            end
            if detail_Debug, disp(['  -D: STA ' num2str(i) ' will reserve NAV = ' num2str(TempEvent.pkt.nav) '.']); end
            if (TempEvent.pkt.sum_sounding>0 && STA(i,3)==0)
                TempEvent.pkt.type = 'NDP_Ann';
            else
                if RTS_method == 0
                    TempEvent.pkt.type = 'Data';
                else
                    TempEvent.pkt.type = 'RTS';
                end
            end
            TempEvent.pkt.Group = TempEvent.pkt.rv;
            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
        else
			% Do nothing because STA i has no traffic in the traffic_queue(i) or already a PPDU in the medium
        end

    case 'wait_for_channel'
        t = event.timer;
        i = event.STA_ID;
        % STA(i, 6) is to represent STA i is in idle/transmit/receive
        % STA(i, 7) is on transmission index for protecting MU transmission if the first receiver fail receiving RTS
        % carrier_sense(i) == 0 is idle, otherwise, 1 is busy
        % t >= nav(i).end means STA i is not block by its NAV
        if (event.pkt.CoMP == 0 && ~isempty(STA_Info(i).CoMP_coordinator))
            if (all(CoMP_Controller.informer_index(STA_Info(i).CoMP_coordinator)))
                event.pkt = Init_CoMPpkt1(event.pkt, i, t);
                CoMP_Controller.information(i).pkt = event.pkt;
            end
        end
        if (STA(i, 3) == 0 && event.pkt.MU == 1 && event.pkt.CoMP == 1 && CoMP_Controller.information(i).numInform ~= 0)
            if (CoMP_Controller.information(i).reset == 1)
                CoMP_Controller.information(i).numInform = 0;
            elseif (strcmp(event.pkt.type, 'RTS') == 1)
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if (any(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS ~= 1))
                    TempEvent = event;
                    TempEvent.type = 'MUNDP_response';
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' cancel event ' num2str(event.pkt.type) '.']); end
                else
                    FirstCoMPAP = CoMP_Controller.information(i).startorder(1);
                    TempEvent = event;
                    TempEvent.type = 'send_PHY';
                    TempEvent.timer = CoMP_Controller.information(FirstCoMPAP).sendRTS_time(i);
                    TempEvent.pkt.nav = CoMP_Controller.information(FirstCoMPAP).nav(i);
                    TempEvent.pkt.sendDataTime = CoMP_Controller.information(FirstCoMPAP).sendData_time;
                    TempEvent.pkt.endTxTime = CoMP_Controller.information(FirstCoMPAP).endCoMPtransmission_time;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' wait to send ' num2str(event.pkt.type) '.']); end
                end
            elseif (strcmp(event.pkt.type, 'Data') == 1)
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if (CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS ~= 1)
                    TempEvent = event;
                    TempEvent.type = 'MUNDP_response';
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' cancel event ' num2str(event.pkt.type) '.']); end
            elseif (strcmp(event.pkt.type, 'NDP_Ann') == 1)
                FirstAP = CoMP_Controller.information(i).startorder(1);
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if CoMP_Controller.information(FirstAP).readyRTS == 1
                    if sounding_skipevent_Debug == 0
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' start sounding because STA ' num2str(FirstAP) ' readyRTS == 1']); end
                    else
                        TempEvent = event;
                        TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                            (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                        TempEvent.type = 'CBF_timeout';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' start sounding because STA ' num2str(FirstAP) ' readyRTS == 1']); end
                    end
                end
            end
        else
            if (STA(i, 6) == 0 && carrier_sense(i) == 0 && t >= nav(i).end && STA(i, 7) == 0)
                if event_Debug, disp(['[' num2str(t, '%1.20f') ']: wait_for_channel @ STA ' num2str(i)]); end
                % The STA is idle and the channel is free, so it wait DIFS
                TempEvent = event;
                TempEvent.timer = t + DIFS; DebugBOTime(i) = DebugBOTime(i) + DIFS;
                TempEvent.type = 'backoff_start';
                TempEvent.STA_ID = i;
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            else
                % STA i is not idle, wait_for_channel again
                TempEvent = event;
                TempEvent.timer = t + cca_time;
                TempEvent.type = 'wait_for_channel';
                TempEvent.STA_ID = i;
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            end
        end

    case 'backoff_start'
        % after DIFS start backoff
        t = event.timer;
        i = event.STA_ID;
        if (event.pkt.CoMP == 0 && ~isempty(STA_Info(i).CoMP_coordinator))
            if (all(CoMP_Controller.informer_index(STA_Info(i).CoMP_coordinator)))
                error "Impossible";
            end
        end
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: backoff_start @ STA ' num2str(i)]); end
        if (STA(i, 3) == 0 && event.pkt.MU == 1 && event.pkt.CoMP == 1 && CoMP_Controller.information(i).numInform ~= 0)
            if (CoMP_Controller.information(i).reset == 1)
                CoMP_Controller.information(i).numInform = 0;
            elseif (strcmp(event.pkt.type, 'RTS') == 1)
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if (CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS ~= 1)
                    TempEvent = event;
                    TempEvent.type = 'MUNDP_response';
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' cancel event ' num2str(event.pkt.type) '.']); end
                else
                    FirstCoMPAP = CoMP_Controller.information(i).startorder(1);
                    TempEvent = event;
                    TempEvent.type = 'send_PHY';
                    TempEvent.timer = CoMP_Controller.information(FirstCoMPAP).sendRTS_time(i);
                    TempEvent.pkt.nav = CoMP_Controller.information(FirstCoMPAP).nav(i);
                    TempEvent.pkt.sendDataTime = CoMP_Controller.information(FirstCoMPAP).sendData_time;
                    TempEvent.pkt.endTxTime = CoMP_Controller.information(FirstCoMPAP).endCoMPtransmission_time;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' wait to send ' num2str(event.pkt.type) '.']); end
                end
            elseif (strcmp(event.pkt.type, 'Data') == 1)
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if (CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS ~= 1)
                    TempEvent = event;
                    TempEvent.type = 'MUNDP_response';
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' cancel event ' num2str(event.pkt.type) '.']); end
            elseif (strcmp(event.pkt.type, 'NDP_Ann') == 1)
                FirstAP = CoMP_Controller.information(i).startorder(1);
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if CoMP_Controller.information(FirstAP).readyRTS == 1
                    if sounding_skipevent_Debug == 0
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' start sounding because STA ' num2str(FirstAP) ' readyRTS == 1']); end
                    else
                        TempEvent = event;
                        TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                            (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                        TempEvent.type = 'CBF_timeout';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' start sounding because STA ' num2str(FirstAP) ' readyRTS == 1']); end
                    end
                end
            end
        else
            if (STA(i, 6) == 0 && carrier_sense(i) == 0 && t >= nav(i).end && STA(i, 7) == 0)
                if (backoff_counter(i) == 0)
                    % Current traffic haven't decided a backoff counter yet.
                    backoff_attempt(i) = 0;
                    temp = min(backoff_attempt(i)+CW_min, CW_max);
                    backoff_counter(i) = floor((2^temp-1)*rand(1));
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' is idle and has backoff counter ' num2str(backoff_counter(i)) '.']); end
                    % Random pick backoff counter is zero, it means STA i immediately get access to the medium
                    if (backoff_counter(i) == 0)
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = i;
                        if (STA(i, 3) == 0 && event.pkt.MU == 1 && event.pkt.CoMP == 1)
                            if (CoMP_Controller.information(i).numInform == 0)
                                CoMP_Controller.information(i).startorder = [i, STA_Info(i).CoMP_coordinator];
                                CoMP_Controller.information(i).reset = 0;
                                % The first CoMP AP access to channel then inform other CoMP APs through CoMP_Controller
                                temp = STA_Info(i).CoMP_coordinator;
                                for k=1:length(temp)
                                    CoMP_Controller.information(temp(k)).numInform = 2;
                                    CoMP_Controller.information(temp(k)).reset = 0;
                                    CoMP_Controller.information(temp(k)).startorder = CoMP_Controller.information(i).startorder;
                                end
                                if CoMP_Controller.information(i).readyRTS == 1
                                    if all(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS) == 1
                                        if (strcmp(event.pkt.type, 'RTS') == 1)
                                            if RTS_method == 1
                                                all_pkt_nav = [];
                                                for k=1:length(CoMP_Controller.information(i).startorder)
                                                    AP_index = CoMP_Controller.information(i).startorder(k);
                                                    all_pkt_nav = [all_pkt_nav, CoMP_Controller.information(AP_index).pkt.nav];
                                                end
                                                event.pkt.nav = max(all_pkt_nav); %find out maximum pkt.nav
                                                for k=1:length(STA_Info(i).CoMP_coordinator)
                                                    CoMP_Controller.information(STA_Info(i).CoMP_coordinator(k)).numInform = 1;
                                                    CoMP_Controller.information(STA_Info(i).CoMP_coordinator(k)).startorder = [i, STA_Info(i).CoMP_coordinator];
                                                    CoMP_Controller.information(i).nav(STA_Info(i).CoMP_coordinator(k)) = event.pkt.nav - k * (SIFS + RTS_tx_time);
                                                    CoMP_Controller.information(i).sendRTS_time(STA_Info(i).CoMP_coordinator(k)) = t + k * (RTS_tx_time + SIFS);
                                                end
                                                CoMP_Controller.information(i).sendData_time = t+(length(STA_Info(i).CoMP_coordinator)+1)*(RTS_tx_time+SIFS)+...
                                                    length(event.pkt.rv)*(CTS_tx_time+SIFS);
                                                CoMP_Controller.information(i).endCoMPtransmission_time = t + event.pkt.nav + RTS_tx_time;
                                                TempEvent.pkt.endTxTime = CoMP_Controller.information(i).endCoMPtransmission_time;
                                                
                                                NewEvents = [TempEvent, NewEvents]; clear TempEvent;
                                                if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                                            else %RTS_method ==2
                                                %Not done here.
                                            end
                                        elseif (strcmp(event.pkt.type, 'Data') == 1)
                                            temp = CoMP_Controller.information(i).startorder;
                                            for k=1:length(temp)
                                                CoMPAP_index = temp(k);
                                                % send DATA
                                                TempEvent = event;
                                                TempEvent.timer = t;
                                                TempEvent.type = 'send_PHY';
                                                TempEvent.STA_ID = CoMPAP_index;
                                                TempEvent.pkt = CoMP_Controller.information(CoMPAP_index).pkt;
                                                % Creat a new id for the data packet
                                                TempEvent.pkt.id = new_id(CoMPAP_index);
                                                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                                
                                                if detail_Debug, disp(['  -D: STA ' num2str(CoMPAP_index) ' is ready to send ' num2str(event.pkt.type) '.']); end
                                            end
                                        else
                                            error 'error pkt.type, when backoff == 0 in CoMP mode';
                                        end
                                    else % STA_Info(i).CoMP_coordinator's readyRTS ~= 1
                                        TempEvent = event;
                                        TempEvent.type = 'MUNDP_response';
                                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                    end
                                else % CoMP_Controller.information(i).readyRTS ~= 1
                                    if sounding_skipevent_Debug == 0
                                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                    else
                                        TempEvent.type = 'CBF_timeout';
                                        TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                                            (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                    end
                                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                                end
                            end
                        elseif (STA(i, 3) == 0 && event.pkt.CoMP == 0)
                            if (sounding_skipevent_Debug == 1 && (strcmp(event.pkt.type, 'NDP_Ann') == 1))
                                TempEvent.type = 'CBF_timeout';
                                TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                                    (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                            else
                                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                            end
                        end
                    else
                        TempEvent = event;
                        TempEvent.timer = t + slotTime; DebugBOTime(i) = DebugBOTime(i) + slotTime;
                        TempEvent.type = 'backoff';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    end
                else
                    % Current traffic already has a backoff counter because it was blcok by others and resume backoff procedure.
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' resumes backoff and has backoff counter ' num2str(backoff_counter(i)) '.']); end
                    TempEvent = event;
                    TempEvent.timer = t + slotTime; DebugBOTime(i) = DebugBOTime(i) + slotTime;
                    TempEvent.type = 'backoff';
                    TempEvent.STA_ID = i;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
            else
                % The channel becomes busy during DIFS, wait until the channel is free
                if (STA(i, 6) ~= 0)
                    if (STA(i, 6) == 2)
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' senses not idle during DIFS because it is receiving.']); end
                    elseif (STA(i, 6) == 1)
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' senses not idle during DIFS because it is transmitting.']); end
                    end
                elseif (carrier_sense(i) ~= 0)
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' senses not idle during DIFS.']); end
                elseif (STA(i, 7) ~= 0)
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' is on a transmission.']); end
                else
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' defers medium access where NAV indicates not idle.']); end
                end
                TempEvent = event;
                TempEvent.timer = t + cca_time;
                TempEvent.type = 'wait_for_channel';
                TempEvent.STA_ID = i;
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            end
        end

    case 'backoff'
        t = event.timer;
        i = event.STA_ID;
        if (event.pkt.CoMP == 0 && ~isempty(STA_Info(i).CoMP_coordinator))
            if (all(CoMP_Controller.informer_index(STA_Info(i).CoMP_coordinator)))
                event.pkt = Init_CoMPpkt1(event.pkt, i, t);
                error "Impossible";
            end
        end
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: backoff @ STA ' num2str(i)]); end
        backoff_counter(i) = backoff_counter(i) - 1;
        if (STA(i, 3) == 0 && event.pkt.MU == 1 && event.pkt.CoMP == 1 && CoMP_Controller.information(i).numInform ~= 0)
            if (CoMP_Controller.information(i).reset == 1)
                CoMP_Controller.information(i).numInform = 0;
            elseif (strcmp(event.pkt.type, 'RTS') == 1)
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if (CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS ~= 1)
                    TempEvent = event;
                    TempEvent.type = 'MUNDP_response';
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' cancel event ' num2str(event.pkt.type) '.']); end
                else
                    FirstCoMPAP = CoMP_Controller.information(i).startorder(1);
                    TempEvent = event;
                    TempEvent.type = 'send_PHY';
                    TempEvent.timer = CoMP_Controller.information(FirstCoMPAP).sendRTS_time(i);
                    TempEvent.pkt.nav = CoMP_Controller.information(FirstCoMPAP).nav(i);
                    TempEvent.pkt.sendDataTime = CoMP_Controller.information(FirstCoMPAP).sendData_time;
                    TempEvent.pkt.endTxTime = CoMP_Controller.information(FirstCoMPAP).endCoMPtransmission_time;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' wait to send ' num2str(event.pkt.type) '.']); end
                end
            elseif (strcmp(event.pkt.type, 'Data') == 1)
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if (CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS ~= 1)
                    TempEvent = event;
                    TempEvent.type = 'MUNDP_response';
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' cancel event ' num2str(event.pkt.type) '.']); end
            elseif (strcmp(event.pkt.type, 'NDP_Ann') == 1)
                FirstAP = CoMP_Controller.information(i).startorder(1);
                backoff_counter(i) = 0;
                CoMP_Controller.information(i).numInform = 0;
                if CoMP_Controller.information(FirstAP).readyRTS == 1
                    if sounding_skipevent_Debug == 0
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' start sounding because STA ' num2str(FirstAP) ' readyRTS == 1']); end
                    else
                        TempEvent = event;
                        TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                            (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                        TempEvent.type = 'CBF_timeout';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if detail_Debug, disp(['  -D: STA ' num2str(i) ' start sounding because STA ' num2str(FirstAP) ' readyRTS == 1']); end
                    end
                end
            end
        else
            if (STA(i, 6) == 0 && carrier_sense(i) == 0 && t >= nav(i).end && STA(i, 7) == 0)
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' is idle and has backoff counter ' num2str(backoff_counter(i)) '.']); end
                if (backoff_counter(i) == 0)
                    TempEvent = event;
                    TempEvent.timer = t;
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = i;
                    if (STA(i, 3) == 0 && event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        if (CoMP_Controller.information(i).numInform == 0)
                            CoMP_Controller.information(i).startorder = [i, STA_Info(i).CoMP_coordinator];
                            CoMP_Controller.information(i).reset = 0;
                            % The first CoMP AP access to channel then inform other CoMP APs through CoMP_Controller
                            temp = STA_Info(i).CoMP_coordinator;
                            for k=1:length(temp)
                                CoMP_Controller.information(temp(k)).numInform = 2;
                                CoMP_Controller.information(temp(k)).startorder = CoMP_Controller.information(i).startorder;
                                CoMP_Controller.information(temp(k)).reset = 0;
                            end
                            if CoMP_Controller.information(i).readyRTS == 1
                                if all(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).readyRTS) == 1
                                    if (strcmp(event.pkt.type, 'RTS') == 1)
                                        if RTS_method == 1
                                            all_pkt_nav = [];
                                            for k=1:length(CoMP_Controller.information(i).startorder)
                                                AP_index = CoMP_Controller.information(i).startorder(k);
                                                all_pkt_nav = [all_pkt_nav, CoMP_Controller.information(AP_index).pkt.nav];
                                            end
                                            event.pkt.nav = max(all_pkt_nav); %find out maximum pkt.nav
                                            for k=1:length(STA_Info(i).CoMP_coordinator)
                                                CoMP_Controller.information(STA_Info(i).CoMP_coordinator(k)).numInform = 1;
                                                CoMP_Controller.information(STA_Info(i).CoMP_coordinator(k)).startorder = [i, STA_Info(i).CoMP_coordinator];
                                                CoMP_Controller.information(i).nav(STA_Info(i).CoMP_coordinator(k)) = event.pkt.nav - k * (SIFS + RTS_tx_time);
                                                CoMP_Controller.information(i).sendRTS_time(STA_Info(i).CoMP_coordinator(k)) = t + k * (RTS_tx_time + SIFS);
                                            end
                                            CoMP_Controller.information(i).sendData_time = t+(length(STA_Info(i).CoMP_coordinator)+1)*(RTS_tx_time+SIFS)+...
                                                length(event.pkt.rv)*(CTS_tx_time+SIFS);
                                            CoMP_Controller.information(i).endCoMPtransmission_time = t + event.pkt.nav + RTS_tx_time;
                                            TempEvent.pkt.endTxTime = CoMP_Controller.information(i).endCoMPtransmission_time;
                                            
                                            NewEvents = [TempEvent, NewEvents]; clear TempEvent;
                                            if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                                        else %RTS_method ==2
                                            %Not done here.
                                        end
                                    elseif (strcmp(event.pkt.type, 'Data') == 1)
                                        temp = CoMP_Controller.information(i).startorder;
                                        for k=1:length(temp)
                                            CoMPAP_index = temp(k);
                                            % send DATA
                                            TempEvent = event;
                                            TempEvent.timer = t;
                                            TempEvent.type = 'send_PHY';
                                            TempEvent.STA_ID = CoMPAP_index;
                                            TempEvent.pkt = CoMP_Controller.information(CoMPAP_index).pkt;
                                            % Creat a new id for the data packet
                                            TempEvent.pkt.id = new_id(CoMPAP_index);
                                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                            
                                            if detail_Debug, disp(['  -D: STA ' num2str(CoMPAP_index) ' is ready to send ' num2str(event.pkt.type) '.']); end
                                        end
                                    else
                                        error 'error pkt.type, when backoff == 0 in CoMP mode';
                                    end
                                else
                                    
                                    TempEvent = event;
                                    TempEvent.type = 'MUNDP_response';
                                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                end
                            else
                                if sounding_skipevent_Debug == 0
                                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                else
                                    TempEvent.type = 'CBF_timeout';
                                    TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                                        (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                                end
                                if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                            end
                        end
                    elseif (STA(i, 3) == 0 && event.pkt.CoMP == 0)
                        if (sounding_skipevent_Debug == 1 && (strcmp(event.pkt.type, 'NDP_Ann') == 1))
                            TempEvent.type = 'CBF_timeout';
                            TempEvent.timer = t + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                                (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime)+eps;
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                        else
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            if detail_Debug, disp(['  -D: STA ' num2str(i) ' is ready to send ' num2str(event.pkt.type) '.']); end
                        end
                    end
                else
                    % Ready to send the packet
                    TempEvent = event;
                    TempEvent.timer = t + slotTime; DebugBOTime(i) = DebugBOTime(i) + slotTime;
                    TempEvent.type = 'backoff';
                    TempEvent.STA_ID = i;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
            else
                % The channel becomes busy during backoff count-down
                if (backoff_counter(i) == 0)
                    % Start a new backoff counter when count-down is zero
                    if (backoff_attempt(i)+CW_min < CW_max)
                        backoff_attempt(i) = backoff_attempt(i) + 1;
                    end
                    temp = min(backoff_attempt(i)+CW_min, CW_max);
                    backoff_counter(i) = floor((2^temp-1)*rand(1));
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' senses not idle during backoff count-down and has backoff counter 0, reselect backoff counter ' num2str(backoff_counter(i)) '.']); end
                end
                TempEvent = event;
                TempEvent.timer = t + cca_time;
                TempEvent.type = 'wait_for_channel';
                TempEvent.STA_ID = i;
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            end
        end

    case 'send_PHY'
        t = event.timer;
        i = event.STA_ID;
%         if (strcmp(event.pkt.type, 'CTS') == 1)
%             event.pkt.rv = STA_Info(i).rxMU;
%             STA_Info(i).rxMU = [];   %move from recv_MAC case:RTS
%         end
        j = event.pkt.rv;

        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: send_PHY @ STA ' num2str(i)]); end
        if (STA(i, 3) == 0 && ~isempty(STA_Info(i).CoMP_coordinator))
            if (event.pkt.CoMP == 0 && CoMP_Controller.informer_index(i == CoMP_Controller.connector) == 1 && strcmp(event.pkt.type, 'NDP_Ann') == 1)
                CoMP_Controller.informer_index(i == CoMP_Controller.connector) = 0;
            end
        end

        switch event.pkt.type
            case 'NDP_Ann'
                txtime = NDPAnnTime;
            case 'NDP'
                txtime = NDPTime;
            case 'Compressed_BF'
                txtime = CompressedBeamformingTime;
            case 'Report_P'
                txtime = ReportPollTime;
            case 'RTS'
                txtime = RTS_tx_time;
            case 'CTS'
                txtime = CTS_tx_time;
            case 'ACK'
                txtime = ACK_tx_time;
            case 'Data'
                if (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    txtime = max(CoMP_Controller.information(i).pkt.txtime,CoMP_Controller.information(STA_Info(i).CoMP_coordinator).pkt.txtime);
                else
                    txtime = tx_time(event.pkt);
                end
            otherwise
                error (['send_PHY wrong txtime. STA ' num2str(i) ' is sending ' num2str(event.pkt.type)])
        end
        
            
        if (STA(i, 6) == 0  && (nav(i).start > (t+txtime) || nav(i).end < t) ) % idle and no nav

            if detail_Debug, disp(['  -D: STA ' num2str(i) ' is sending ' num2str(event.pkt.type) ' (' num2str(i) '->'  num2str(j) ')']); end
            % Branch current event into at least three parts:
            % I: transmitter timeout or MURTS_response or MUData_response + II: recv_PHY at receivers + III: send_PHY_finish at transmitter
            % Setting I:
            STA(i, 6) = 1; % Switch to transmit mode, assume RXTXturnaround time is zero
            STA(i, 5) = event.pkt.power; % STA(i, 5) is the power STA i uses
            tx_interval(i).start = t;
            tx_interval(i).end = t + txtime;
%             I = find(STA(:, 5)>0);
%             for k=1:length(I)
%                 tx1 = I(i);
%                 if any(tx1 == j), continue; end
%                 if any(tx1 == i), continue; end
%                 interference_queue(i).list = [interference_queue(i).list tx1];
%                 interference_queue(i).start = [interference_queue(i).start tx_interval(tx1).start];
%                 interference_queue(i).end = [interference_queue(i).end tx_interval(tx1).end];
%                 interference_queue(i).pkt_type = [interference_queue(i).pkt_type STA(tx1, 8)];
%                 
%                 interference_queue(tx1).list = [interference_queue(tx1).list i];
%                 interference_queue(tx1).start = [interference_queue(tx1).start tx_interval(i).start];
%                 interference_queue(tx1).end = [interference_queue(tx1).end tx_interval(i).end];
%                 interference_queue(tx1).pkt_type = [interference_queue(tx1).pkt_type STA(i, 8)];
%             end
            %disp([' STA power  ' num2str(STA(i,5))]);
            if (strcmp(event.pkt.type, 'Data') == 1)
                % STA(i, 8) is to store the transmitting pkt category of STA i
                STA(i, 8) = 2; % (0, 1, 2) = (None, RTS/CTS/ACK, Data)
                if (pending_id(i) > 0)
%                     error(['send_PHY: STA ' num2str(i) ' there is already a pending packet, cannot send a new DATA packet']);
                end
                pending_id(i) = event.pkt.id;
                if detail_Debug, disp(['  -D: The pending_id(STA ' num2str(i) ') = ' num2str(pending_id(i)) '(Data).']); end
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    % Set timeout timer for SU-PPDU Data
                    %disp([' STA power  at 0 0' num2str(STA(i,5))]);
                    TempEvent = event;
                    TempEvent.timer = t + txtime + SIFS + ACK_tx_time + 2*eps; % question: how to choose this timeout limit?
                    TempEvent.type = 'SUData_respnse';
                    TempEvent.STA_ID = i;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    %disp([' STA power  at 1 0' num2str(STA(i,5))]);
                    STA_Info(i).rxMU = [];
                    STA_Info(i).rxMU = zeros(1, length(event.pkt.Group));
                    % Set MUData_response for Data packet.
                    % 1. MU-PPDU, correctly receives MU CTS
                    % 2. SU-PPDU, only receives one CTS of MU
                    TempEvent = event;
                    TempEvent.timer = t + txtime + (SIFS + ACK_tx_time) * length(event.pkt.Group) + 2*eps;
                    TempEvent.type = 'MUData_response';
                    TempEvent.STA_ID = i;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    switch PHY_CH_module
                        case 'old'
                            % The multi-antenna AP send Data so calculate ZF precoding matrice for MU
                            selection_length = length(event.pkt.rv);
                            if (selection_length >= 1)
                                H = zeros(selection_length*Num_Rx, Num_Tx);
                                for k=1:selection_length
                                    H((k-1)*Num_Rx+1:k*Num_Rx, :) = STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:);
                                end
                                W = H'/(H*H');
                                if MIMO_Debug, disp('  -M: Channel H for precoder,'); disp(H); disp('  -M: Precoder H''/(H*H'') after calculating,'); disp(W); end
                                temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
                                W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
                                if MIMO_Debug, disp('  -M: Normalized precoder,'); disp(W); end
                                for k=1:selection_length
                                    if (MIMO_Debug && k == 1), disp(['  -M: Divide normalized precoder by sqrt(' num2str(selection_length) ') to equal power,']); end
                                    STA_Info(i).Precoding_Matrix(:,(event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx) = W(:,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(selection_length);
                                end
                                if MIMO_Debug,
                                    disp(W/sqrt(selection_length));
                                    % Verify receives signal strength of each intended STA
                                    for k=1:selection_length
                                        temp_H = STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx, :);
                                        disp(['  -M: STA ' num2str(event.pkt.rv(k)) ' should receive signal strength ' num2str(trace(temp_H*STA_Info(i).Precoding_Matrix*(temp_H*STA_Info(i).Precoding_Matrix)'))]);
                                    end
                                end
                            else
                                error(['send_PHY: STA ' num2str(i) ' it is impossible to transmit Data packet to NULL']);
                            end
                        case 'new'
                            % Nothing need to do here.
                    end
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    %disp([' STA power  at 1 1' num2str(STA(i,5))]);
                    STA_Info(i).rxMU = [];
                    STA_Info(i).rxMU = zeros(1, length(event.pkt.CoMPGroup));
                    % For checking a CoMP transmission
                    CoMP_Controller.information(i).doneData = 1;
                    if (any([CoMP_Controller.information(STA_Info(i).CoMP_coordinator).doneData] > 0) ~= 1)
                        TempEvent = event;
                        TempEvent.timer = t + txtime + (SIFS + ACK_tx_time) * max(length(event.pkt.CoMPGroup), length(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).pkt.CoMPGroup)) + 2*eps;
                        TempEvent.type = 'MUData_response';
                        TempEvent.STA_ID = CoMP_Controller.information(i).startorder;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    end
                    if MIMO_Debug, disp(['  -M: CoMP transmission send Data to ' num2str(event.pkt.rv)]); end
                    
                    switch PHY_CH_module
                        case 'old'
                            selection_length = length(event.pkt.rv);
                            if (selection_length == length(event.pkt.CoMPGroup) && CoMP_Controller.CoMP_tx_mode == 0) % Networked MIMO
                                CoMPAPs_length = length(CoMP_Controller.information(i).startorder);
                                H = zeros(selection_length*Num_Rx, CoMPAPs_length*Num_Tx);
                                for k=1:selection_length
                                    for l = 1:CoMPAPs_length
                                        CoMPAP_index = CoMP_Controller.information(i).startorder(l);
                                        temp_H = STA_Info(CoMPAP_index).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:);
                                        if (temp_H == zeros(Num_Rx, Num_Tx))
                                            temp_H = sqrt(LongTerm_recv_power(CoMPAP_index, event.pkt.rv(k), rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                                            % No CSI but restore temp CSI into STA_Info(i).Channel_Matrx, but STA_Info(i).TS do not record
                                            STA_Info(CoMPAP_index).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:) = temp_H;
                                        end
                                        H((k-1)*Num_Rx+1:k*Num_Rx, (l-1)*Num_Tx+1:l*Num_Tx) = temp_H;
                                    end
                                end
                                W = H'/(H*H');
                                if MIMO_Debug, disp('  -M: Channel H for precoder,'); disp(H); disp('  -M: Precoder H''/(H*H'') after calculating,'); disp(W); end
                                temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
                                W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
                                if MIMO_Debug, disp('  -M: Normalized precoder,'); disp(W); end
                                for k=1:selection_length
                                    for l=1:CoMPAPs_length
                                        CoMPAP_index = CoMP_Controller.information(i).startorder(l);
                                        if (MIMO_Debug && k == 1 && l == 1), disp(['  -M: Divide normalized precoder by sqrt(' num2str(length(STA_Info(CoMPAP_index).IntendSTA)) ') to equal power,']); end
                                        STA_Info(CoMPAP_index).Precoding_Matrix(:,(event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx) = W((l-1)*Num_Tx+1:l*Num_Tx,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(length(STA_Info(CoMPAP_index).IntendSTA));
                                    end
                                end
                                if MIMO_Debug,
                                    for l=1:CoMPAPs_length
                                        CoMPAP_index = CoMP_Controller.information(i).startorder(l);
                                        disp(STA_Info(CoMPAP_index).Precoding_Matrix);
                                    end
                                    % Verify receives signal strength of each intended STA
                                    for k=1:selection_length
                                        rv_signal = [];
                                        for l = 1:CoMPAPs_length
                                            CoMPAP_index = CoMP_Controller.information(i).startorder(l);
                                            temp_H = STA_Info(CoMPAP_index).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx, :);
                                            if (l == 1)
                                                rv_signal = temp_H*STA_Info(CoMPAP_index).Precoding_Matrix;
                                            else
                                                rv_signal = rv_signal + temp_H*STA_Info(CoMPAP_index).Precoding_Matrix;
                                            end
                                        end
                                        disp(['  -M: STA ' num2str(event.pkt.rv(k)) ' should receive signal strength ' num2str(trace(rv_signal*rv_signal'))]);
                                    end
                                end
                            elseif (selection_length == length(event.pkt.CoMPGroup) && CoMP_Controller.CoMP_tx_mode == 1) % Coordinated Scheduling/Coordinated Beamforming
                                H = zeros(selection_length*Num_Rx, Num_Tx);
                                for k=1:selection_length
                                    temp_H = STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:);
                                    if (temp_H == zeros(Num_Rx, Num_Tx))
                                        temp_H = sqrt(recv_power(i, event.pkt.rv(k), rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                                        % No CSI but restore temp CSI into STA_Info(i).Channel_Matrx, but STA_Info(i).TS do not record
                                        STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:) = temp_H;
                                    end
                                    H((k-1)*Num_Rx+1:k*Num_Rx, :) = temp_H;
                                end
                                W = H'/(H*H');
                                if MIMO_Debug, disp('  -M: Channel H for precoder,'); disp(H); disp('  -M: Precoder H''/(H*H'') after calculating,'); disp(W); end
                                temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
                                W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
                                if MIMO_Debug, disp('  -M: Normalized precoder,'); disp(W); end
                                for k=1:selection_length
                                    if (MIMO_Debug && k == 1), disp(['  -M: Divide normalized precoder by sqrt(' num2str(length(STA_Info(i).IntendSTA)) ') to equal power,']); end
                                    if (any(event.pkt.rv(k) == STA_Info(i).IntendSTA))
                                        STA_Info(i).Precoding_Matrix(:,(event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx) = W(:,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(length(STA_Info(i).IntendSTA));
                                    end
                                end
                                if MIMO_Debug,
                                    disp(STA_Info(i).Precoding_Matrix);
                                    % Verify receives signal strength of each intended STA
                                    for k=1:selection_length
                                        temp_H = STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx, :);
                                        disp(['  -M: STA ' num2str(event.pkt.rv(k)) ' should receive signal strength ' num2str(trace(temp_H*STA_Info(i).Precoding_Matrix*(temp_H*STA_Info(i).Precoding_Matrix)'))]);
                                    end
                                end
                            else % Networked MIMO and CS/CB fail => DL MU-MIMO
                                H = zeros(selection_length*Num_Rx, Num_Tx);
                                for k=1:selection_length
                                    temp_H = STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:);
                                    if (temp_H == zeros(Num_Rx, Num_Tx))
                                        temp_H = sqrt(recv_power(i, event.pkt.rv(k), rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                                        % No CSI but restore temp CSI into STA_Info(i).Channel_Matrx, but STA_Info(i).TS do not record
                                        STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx,:) = temp_H;
                                    end
                                    H((k-1)*Num_Rx+1:k*Num_Rx, :) = temp_H;
                                end
                                W = H'/(H*H');
                                if MIMO_Debug, disp('  -M: Channel H for precoder,'); disp(H); disp('  -M: Precoder H''/(H*H'') after calculating,'); disp(W); end
                                temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
                                W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
                                for k=1:selection_length
                                    if (MIMO_Debug && k == 1), disp(['  -M: Divide normalized precoder by sqrt(' num2str(selection_length) ') to equal power,']); end
                                    STA_Info(i).Precoding_Matrix(:,(event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx) = W(:,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(selection_length);
                                end
                                if MIMO_Debug,
                                    disp(STA_Info(i).Precoding_Matrix);
                                    % Verify receives signal strength of each intended STA
                                    for k=1:selection_length
                                        temp_H = STA_Info(i).Channel_Matrix((event.pkt.rv(k)-1)*Num_Rx+1:event.pkt.rv(k)*Num_Rx, :);
                                        disp(['  -M: STA ' num2str(event.pkt.rv(k)) ' should receive signal strength ' num2str(trace(temp_H*STA_Info(i).Precoding_Matrix*(temp_H*STA_Info(i).Precoding_Matrix)'))]);
                                    end
                                end
                            end
                        case 'new'
                            % Nothing need to do here.
                    end
                end
                if queue_Debug,
                    disp(['  -Q: @ action_CoMP.m: ']);
                    disp(['  -Q: STA ' num2str(i) ' transmits to STA ' num2str(j) ', and pkt.size = ' num2str(event.pkt.size.')]);
                    if (STA(i, 3) == 0)
                        disp(['  -Q: length(traffic_queue(' num2str(i) ').list) = ' num2str(length(traffic_queue(i).list)) ' = sum(traffic_queue(' num2str(i) ').size(:)) = ' num2str(queue_size(i)) ' = ' num2str(sum(traffic_queue(i).size))]);
                        disp(['  -Q: STA ' num2str(i) ' has traffic_queue.size(' num2str(STA_Info(i).associated_STA) ') = [' num2str(traffic_queue(i).size) ']']);
                    else %if (STA(i, 3) ~= 0)
                        disp(['  -Q: STA ' num2str(i) ' has queue_size(' num2str(i) ') = ' num2str(queue_size(i))]);
                    end
                end
                % Delete MSDUs from traffic_queue(i).list buffer
                if (STA(i, 3) == 0)
%                    pktsize_buf = event.pkt.size(event.pkt.size~=0);
                    pktsize_buf = event.pkt.size;
                    if (~isempty(pktsize_buf))
                        for k=1:length(j)
                            AP_index = STA(j(k), 3);
                            switch PHY_CH_module
                                case 'old'
                                    aggr = pktsize_buf(k)/size_MAC_body;
                                case 'new'
                                    aggr = pktsize_buf(k)/(size_MAC_body*num_msdu);
                            end
                            if (AP_index == i && aggr ~= 0)
                                traffic_queue(i).list(find(traffic_queue(i).list == j(k), aggr)) = [];
                                traffic_queue(i).size(STA_Info(i).associated_STA == j(k)) = traffic_queue(i).size(STA_Info(i).associated_STA == j(k)) - aggr;
                                queue_size(i) = queue_size(i) - aggr;
                            end
                        end
                    end
                else %if (STA(i, 3) ~= 0)
                    queue_size(i) = queue_size(i) - event.pkt.size/size_MAC_body;
                end
                if queue_Debug,
                    disp('  -Q: After deleteing MSDUs... ');
                    if (STA(i, 3) == 0)
                        disp(['  -Q: length(traffic_queue(' num2str(i) ').list) = ' num2str(length(traffic_queue(i).list)) ' = sum(traffic_queue(' num2str(i) ').size(:)) = ' num2str(queue_size(i)) ' = ' num2str(sum(traffic_queue(i).size))]);
                        disp(['  -Q: STA ' num2str(i) ' has traffic_queue.size(' num2str(STA_Info(i).associated_STA) ') = [' num2str(traffic_queue(i).size) ']']);
                    else %if (STA(i, 3) ~= 0)
                        disp(['  -Q: STA ' num2str(i) ' has queue_size(' num2str(i) ') = ' num2str(queue_size(i))]);
                    end
                end
            else % NDPAnn/NDP/Report/CBF/RTS/CTS/ACK packet
                STA(i, 8) = 1; % Store transmitting pkt category: (0, 1, 2) = (None, NDPAnn/NDP/Report/CBF/RTS/CTS/ACK, Data)
                if (strcmp(event.pkt.type, 'NDP_Ann') == 1)
                    if (pending_id(i) > 0)
%                         error(['send_PHY: STA ' num2str(i) ' there is already a pending packet, cannot send a new NDP_Ann packet']);
                    end
                    pending_id(i) = event.pkt.id;
                    if detail_Debug, disp(['  -D: The pending_id(STA ' num2str(i) ') = ' num2str(pending_id(i)) '(NDP_Ann).']); end
                    if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                        STA_Info(i).rxMU = [];
                        STA_Info(i).rxMU = zeros(1, length(event.pkt.rv));
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                        STA_Info(i).rxMU = [];
                        STA_Info(i).rxMU = zeros(1, length(event.pkt.rv));
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        STA_Info(i).rxMU = [];
                        STA_Info(i).rxMU = zeros(1, length(event.pkt.rv));
                    end
                elseif (strcmp(event.pkt.type, 'NDP') == 1 || strcmp(event.pkt.type, 'Report_P') == 1)
                    if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                        TempEvent = event;
                        TempEvent.timer = t + txtime + SIFS + CompressedBeamformingTime + eps;
                        TempEvent.type = 'CBF_timeout';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                        TempEvent = event;
                        TempEvent.timer = t + txtime + SIFS + CompressedBeamformingTime + eps;
                        TempEvent.type = 'CBF_timeout';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        TempEvent = event;
                        TempEvent.timer = t + txtime + SIFS + CompressedBeamformingTime + eps;
                        TempEvent.type = 'CBF_timeout';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    end
                elseif (strcmp(event.pkt.type, 'RTS') == 1)
                    if (pending_id(i) > 0)
%                         error(['send_PHY: STA ' num2str(i) ' there is already a pending packet, cannot send a new RTS packet']);
                    end
                    pending_id(i) = event.pkt.id;
                    if detail_Debug, disp(['  -D: The pending_id(STA ' num2str(i) ') = ' num2str(pending_id(i)) '(RTS).']); end
                    if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                        STA_Info(i).rxMU = [];
                        STA_Info(i).rxMU = zeros(1, length(event.pkt.rv));
                        TempEvent = event;
                        TempEvent.timer = t + txtime + SIFS + CTS_tx_time + 2*eps;
                        TempEvent.type = 'SURTS_response';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                        STA_Info(i).rxMU = [];
                        STA_Info(i).rxMU = zeros(1, length(event.pkt.rv));
                         % Set MURTS_response for RTS
                         TempEvent = event;
                         TempEvent.timer = t + txtime + SIFS + length(TempEvent.pkt.rv)*(SIFS+CTS_tx_time) + 2*eps;
                         TempEvent.type = 'MURTS_response';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        STA_Info(i).rxMU = [];
                        STA_Info(i).rxMU = zeros(1, length(event.pkt.rv));
                        % Set MURTS_response for RTS
                        temp = find(i == CoMP_Controller.information(i).startorder);
                        TempEvent = event;
                        TempEvent.timer = t + txtime + (length(CoMP_Controller.information(i).startorder)-temp)*(SIFS+RTS_tx_time) + length(TempEvent.pkt.rv)*(SIFS+CTS_tx_time) + 2*eps;
                        TempEvent.type = 'MURTS_response';
                        TempEvent.STA_ID = i;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    end
                end
            end
            % Setting II: All receivers should be considered, seperate two parts: II.1: receivers of matching RA + II.2: receivers of not matching RA
            % II.1: Setup receivers of matching RA
            Intend = zeros(1, length(j));
%             disp([' STA power  end' num2str(STA(i,5))]);
            for k=1:length(j)
                if (strcmp(event.pkt.type, 'Data') ~= 1 && ~any(j(k) == STA_Info(i).cover_STA))
                    continue;
                end
                if (strcmp(event.pkt.type, 'Data') == 1 && ~any(j(k) == STA_Info(i).associated_STA))
                    continue;
                end
                % Set up the receiver
                if (STA(j(k), 6) ~= 0 || overlap(t, t+txtime, nav(j(k)).start, nav(j(k)).end))
                    if (STA(j(k), 6) ~= 0)
                        if (STA(j(k), 6) == 2)
                            if detail_Debug, disp(['  -D: STA ' num2str(j(k)) ' is not ready to receive ' num2str(event.pkt.type) ' because it is receiving.']); end
                        elseif (STA(j(k), 6) == 1)
                            if detail_Debug, disp(['  -D: STA ' num2str(j(k)) ' is not ready to receive ' num2str(event.pkt.type) ' because it is transmitting.']); end
                        end
                    else
                        if detail_Debug, disp(['  -D: STA ' num2str(j(k)) ' is not ready to receive ' num2str(event.pkt.type) ' because it defers transmission where NAV indicates not idle.']); end
                    end
                else
                    Intend(k) = j(k);
                    STA(j(k), 6) = 2; % The receiver is switched to receiving mode
                    TempEvent = event;
                    if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                        TempEvent.timer = t + txtime;

                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                        TempEvent.timer = t + txtime;

                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        TempEvent.timer = t + txtime;
                        if (strcmp(event.pkt.type, 'Data') == 1)
                            TempEvent.pkt.tx = [i, STA_Info(i).CoMP_coordinator];%CoMP_Controller.connector;
                        end
                    end
                    TempEvent.type = 'recv_PHY';
                    TempEvent.STA_ID = j(k);
%                     disp([' intend timer ' num2str(TempEvent.timer)]);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
            end
            Intend(Intend == 0) = []; if detail_Debug, disp(['  -D: Intended receiver: STA [' num2str(Intend) ']']); end
            % II.2: Setup receivers of not matching RA
            NotIntend = zeros(1, Num_AP+Num_User);
            %disp([' STA power ' num2str(STA(i,5)) 'event.pkt.MU ' num2str(event.pkt.MU) 'event.pkt.CoMP' num2str(event.pkt.CoMP) ' pkt type ' num2str(event.pkt.type)]);
            for k=1:length(STA_Info(i).cover_STA)
                sel_STA = STA_Info(i).cover_STA(k);
                if ((sqrt((STA(i, 1)-STA(sel_STA, 1))^2+(STA(i, 2)-STA(sel_STA, 2))^2)) > event.pkt.cover_range),continue; end
                % Due to broadcast nature in wireless channel, every idle STA in this STA coverage may capture/sense this transmission
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    % E.g., send_PHY: pkt(i -> j)
                    % RTS(AP -> STA1)/CTS(STA1 -> AP)/Data(AP -> STA1)/ACK(STA1 -> AP)
                    if (STA(sel_STA, 6) ~= 0 || sel_STA == i || sel_STA == j), continue; end
                    if overlap(t, t+txtime, nav(sel_STA).start, nav(sel_STA).end), continue; end
                    NotIntend(sel_STA) = sel_STA;
                    STA(sel_STA, 6) = 2; % The receiver switches to receiving mode
                    TempEvent = event;
                    TempEvent.timer = t + txtime;
                    TempEvent.type = 'recv_PHY';
                    TempEvent.STA_ID = sel_STA;
%                     disp([' not intend timer ' num2str(TempEvent.timer)]);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0) % If a DL MU-MIMO transmission is on
                    % E.g., send_PHY: pkt(i -> j)
                    % RTS(AP -> STA1, STA2)/CTS(STA1 -> AP)/CTS(STA2 -> AP)/Data(AP -> STA1, STA2)/ACK(STA1 -> AP)/ACK(STA2 -> AP)
                    % RTS(AP -> STA1, STA2)/                CTS(STA2 -> AP)/Data(AP -> STA2)                      /ACK(STA2 -> AP)
                    % RTS(AP -> STA1, STA2)/CTS(STA1 -> AP)                /Data(AP -> STA1)      /ACK(STA1 -> AP)
                    if (any(sel_STA == event.pkt.Group))
                        %if (STA(sel_STA, 7) == 1)
                            % MU receive RTS from AP correctly would ignore inter-user's CTS or ACK
                            continue;
                        %end
                    end
                    if (STA(sel_STA, 6) ~= 0 || sel_STA == i || any(sel_STA == j)), continue; end
                    if overlap(t, t+txtime, nav(sel_STA).start, nav(sel_STA).end), continue; end
                    NotIntend(sel_STA) = sel_STA;
                    STA(sel_STA, 6) = 2; % The receiver switches to receiving mode
                    TempEvent = event;
                    TempEvent.timer = t + txtime;
                    TempEvent.type = 'recv_PHY';
                    TempEvent.STA_ID = sel_STA;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1) % If a CoMP transmission is on
                    % E.g., send_PHY: pkt(i -> j)
                    % RTS(AP1 -> STA2, STA6)/RTS(AP5 -> STA2, STA6)/CTS(STA2 -> AP1, AP5)/CTS(STA6 -> AP1, AP5)/ Data(AP1 -> STA2, STA6) and Data(AP5 -> STA2, STA6)/ ACK(STA2 -> AP1)/ACK(STA6 -> AP5)
                    if (any(sel_STA == event.pkt.CoMPGroup))
                        %if (STA(sel_STA, 7) == 1)
                            % MU receive RTS from AP correctly would ignore inter-user's CTS or ACK
                            continue;
                        %end
                    end
                    if (strcmp(event.pkt.type, 'CTS') == 1)
                        if (ismember(STA(sel_STA, 3),STA_Info(STA(i, 3)).CoMP_coordinator))
                            continue
                        end
                    end
                    if (strcmp(event.pkt.type, 'Data') == 1 || strcmp(event.pkt.type, 'RTS') == 1 || strcmp(event.pkt.type, 'CTS') == 1)
                        if (STA(i, 3) == 0)
                            if (any(sel_STA == STA_Info(i).CoMP_coordinator)), continue; end
                        elseif (STA(j, 3) == 0)
                            if (any(sel_STA == [STA_Info(j).CoMP_coordinator])), continue; end
                        end
                    end
                    if (STA(sel_STA, 6) ~= 0 || sel_STA == i || any(sel_STA == j)), continue; end
                    if overlap(t, t+txtime, nav(sel_STA).start, nav(sel_STA).end), continue; end
                    NotIntend(sel_STA) = sel_STA;
                    STA(sel_STA, 6) = 2; % The receiver switches to receiving mode
                    TempEvent = event;
                    if (strcmp(event.pkt.type, 'Data') == 1)
                            TempEvent.pkt.tx = [i, STA_Info(i).CoMP_coordinator];%CoMP_Controller.connector;
                    end
                    TempEvent.timer = t + txtime;
                    TempEvent.type = 'recv_PHY';
                    TempEvent.STA_ID = sel_STA;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
            end
            NotIntend(NotIntend == 0) = []; if detail_Debug, disp(['  -D: Not intended receiver: STA [' num2str(NotIntend) ']']); end
            % Setting III:
            TempEvent = event;
            if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                TempEvent.timer = t + txtime + eps;
            elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                TempEvent.timer = t + txtime + eps;
            elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                TempEvent.timer = t + txtime + eps;
            end
            TempEvent.type = 'send_PHY_finish';
            TempEvent.STA_ID = i;
            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
        else %  ~(STA(i, 6) == 0 && (nav(i).start > (t+txtime) || nav(i).end < t)) radio hardware is not idle or nav block
            if (STA(i, 6) ~= 0)
                if (STA(i, 6) == 2)
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' is not ready to send ' num2str(event.pkt.type) ' because it is receiving.']); end
                elseif (STA(i, 6) == 1)
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' is not ready to send ' num2str(event.pkt.type) ' because it is transmitting.']); end
                end
            else
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' is not ready to send ' num2str(event.pkt.type) ' because it defers transmission where NAV indicates not idle. (NAV.start, NAV.end) = (' num2str(nav(i).start) ', ' num2str(nav(i).end) ')']); end
            end
            % Since the STA status is already checked at MAC layer, it must be due to NAV virtual carrier sense
            % I am a hiddent STA: physical carrier sense is okay, but blocked by virtual carrier sense

            if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                % Drop current traffic and initialize another one traffic
                TempEvent = event;
                TempEvent.timer = t;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                TempEvent = event;
                TempEvent.timer = t;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                if (strcmp(event.pkt.type, 'Report_P') == 1 || strcmp(event.pkt.type, 'NDP') == 1 || strcmp(event.pkt.type, 'NDP_Ann') == 1)
                    temp = CoMP_Controller.information(i).startorder;
                    if detail_Debug, disp(['  -D: STA ' num2str(temp) ' sounding fail, so reset and back to send_MAC.']); end
                    for k=1:length(temp)
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'send_MAC';
                        TempEvent.STA_ID = temp(k);
                        TempEvent.pkt = [];
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        if (CoMP_Controller.information(temp(k)).numInform == 1)
                            CoMP_Controller.information(temp(k)).numInform = 0;
                        end
                        CoMP_Controller.information(temp(k)).reset = 1;
                        CoMP_Controller.information(temp(k)).startorder = [];
                        CoMP_Controller.informer_index(temp(k)) = 0;
                        backoff_counter(temp(k)) = 0;
                        STA_Info(temp(k)).rxMU = [];
                    end
                        
                elseif (strcmp(event.pkt.type, 'RTS') == 1)
                    TempEvent = event;
                    TempEvent.timer = t;
                    TempEvent.type = 'MURTS_response';
                    TempEvent.STA_ID = i;
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (strcmp(event.pkt.type, 'Data') == 1)
                    CoMP_Controller.information(i).doneData = -1;
                    if (sum([CoMP_Controller.information([i,STA_Info(i).CoMP_coordinator]).doneData]) == ((length(STA_Info(i).CoMP_coordinator)+1)*(-1)) )
                        temp = CoMP_Controller.information(i).startorder;
                        for k=1:length(temp)
                            TempEvent = event;
                            TempEvent.timer = t;
                            TempEvent.type = 'send_MAC';
                            TempEvent.STA_ID = temp(k);
                            TempEvent.pkt = [];
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            CoMP_Controller.information(temp(k)).doneData = 0;
                            CoMP_Controller.information(temp(k)).startorder = [];
                            CoMP_Controller.information(temp(k)).lastsounding = -1;
                            CoMP_Controller.informer_index(temp(k)) = 0;
                            STA_Info(temp(k)).rxMU = [];
                        end
                    end
                else
                % STA i sending CTS or ACK or Compressed_BF is triggered by correctly
                % received NDP or Report_P or RTS or Data, thus do nothing here
                end
            end
        end

    case 'send_PHY_finish'
        t = event.timer;
        i = event.STA_ID;
        j = event.pkt.rv;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: send_PHY_finish @ STA ' num2str(i)]); end
        if detail_Debug, disp(['  -D: STA ' num2str(i) ' finishes sending ' num2str(event.pkt.type) ' to STA '  num2str(j) '.']); end
        if (STA(i, 6) ~= 1)
            error(['send_PHY_finish:  ' num2str(i) ' should be in transmission mode']);
        end
        STA(i, 5) = 0; % The transmitter close tranmitting power
        STA(i, 6) = 0; % The transmitter go back to idle after all STAs who can hear finish receiving
        STA(i, 8) = 0; % The transmitter close packet category stored in sned_PHY for tracing and SNR calculation
        if strcmp(event.pkt.type, 'Data') % If send_PHY(Data) sends precoded MU-PPDU then close precoder
            DebugDataTime(i) = DebugDataTime(i) + tx_time(event.pkt);
            if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                DataSent(i) = DataSent(i) + 1;
                STA_Info(i).Precoding_Matrix(STA_Info(i).Precoding_Matrix ~= 0) = 0;
            elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                DataSent(i) = DataSent(i) + length(j);
                STA_Info(i).Precoding_Matrix(STA_Info(i).Precoding_Matrix ~= 0) = 0;
            elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                if (length(j) == length(event.pkt.CoMPGroup))
                    DataSent(i) = DataSent(i) + length(STA_Info(i).IntendSTA);
                else
                    DataSent(i) = DataSent(i) + length(j);
                end
                STA_Info(i).Precoding_Matrix(STA_Info(i).Precoding_Matrix ~= 0) = 0;
            end
        elseif strcmp(event.pkt.type, 'NDP_Ann')
            TempEvent = event;
            TempEvent.type = 'send_PHY';
            STA_Info(i).soundingqueue = event.pkt.rv;
            TempEvent.timer = t + SIFS - eps;
            TempEvent.pkt.nav = 0;
            TempEvent.pkt.rv = STA_Info(i).soundingqueue(1);
            TempEvent.pkt.type = 'NDP';
            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            STA_Info(i).nextsounding = STA_Info(i).soundingqueue(1);
            STA_Info(i).report_ptimes = 0;
        end
    case 'CBF_timeout'
        t = event.timer;
        i = event.STA_ID;
        if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
            if (isempty(STA_Info(i).soundingqueue))
                [MCS_index,~] = fc_return_MCS_Int(t,per_order,i,event.pkt.Group, Num_Tx,spatial_stream,freq,...
                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_MAC_body,AI,'MAC_body');
                if sounding_skipevent_Debug == 1
                    STA_Info(i).CSI_TS(event.pkt.Group - Num_AP) = t - eps;
                end
                STA_Info(i).MCS_record(event.pkt.Group - Num_AP) = MCS_index;
                TempEvent = event;
                TempEvent.timer = t;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            else
                if (STA_Info(i).nextsounding == STA_Info(i).soundingqueue(1))
                    STA_Info(i).report_ptimes = STA_Info(i).report_ptimes + 1;
                else
                    STA_Info(i).nextsounding = STA_Info(i).soundingqueue(1);
                    STA_Info(i).report_ptimes = 1;
                end

                if STA_Info(i).report_ptimes > Max_Report_P
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' fail to get CBF than ' num2str(Max_Report_P) ' times to STA '  num2str(STA_Info(i).nextsounding) '.']); end
                    TempEvent = event;
                    TempEvent.timer = t + length(STA_Info(i).soundingqueue)*(ReportPollTime + SIFS + CompressedBeamformingTime) - ReportPollTime - eps;
                    TempEvent.type = 'send_MAC';
                    TempEvent.STA_ID = i;
                    TempEvent.pkt = [];
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    STA_Info(i).rxMU = [];
                    STA_Info(i).soundingqueue = [];
                elseif ~isempty(STA_Info(i).soundingqueue)
                    TempEvent = event;
                    TempEvent.type = 'send_PHY';
                    TempEvent.timer = t + SIFS - eps;
                    TempEvent.pkt.nav = SIFS + CompressedBeamformingTime;
                    TempEvent.pkt.type = 'Report_P';
                    TempEvent.pkt.rv = STA_Info(i).soundingqueue(1);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                else
                    error 'What is the case in CBF_timeout?';
                end
            end

        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
            
            if (isempty(STA_Info(i).soundingqueue))
                [MCS_index,~] = fc_return_MCS_Int(t,per_order,i,event.pkt.Group, Num_Tx,spatial_stream,freq,...
                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_MAC_body,AI,'MAC_body');
                if sounding_skipevent_Debug == 1
                    for k=1:length(event.pkt.Group)
                        sum_STA = length(event.pkt.Group);
                        select_STA = event.pkt.Group(k);
                        STA_Info(i).CSI_TS(select_STA - Num_AP) = t-(sum_STA-k)*(SIFS+ReportPollTime+SIFS+CompressedBeamformingTime)-eps;
                    end
                end
                STA_Info(i).MCS_record(event.pkt.Group - Num_AP) = MCS_index;
                TempEvent = event;
                TempEvent.timer = t;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            else
                if (STA_Info(i).nextsounding == STA_Info(i).soundingqueue(1))
                    STA_Info(i).report_ptimes = STA_Info(i).report_ptimes + 1;
                else
                    STA_Info(i).nextsounding = STA_Info(i).soundingqueue(1);
                    STA_Info(i).report_ptimes = 1;
                end

                if STA_Info(i).report_ptimes > Max_Report_P
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' fail to get CBF than ' num2str(Max_Report_P) ' times to STA '  num2str(STA_Info(i).nextsounding) '.']); end
                    TempEvent = event;
                    TempEvent.timer = t + length(STA_Info(i).soundingqueue)*(ReportPollTime + SIFS + CompressedBeamformingTime) - ReportPollTime - eps;
                    TempEvent.type = 'send_MAC';
                    TempEvent.STA_ID = i;
                    TempEvent.pkt = [];
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    STA_Info(i).rxMU = [];
                    STA_Info(i).soundingqueue = [];
                elseif ~isempty(STA_Info(i).soundingqueue)
                    TempEvent = event;
                    TempEvent.type = 'send_PHY';
                    TempEvent.timer = t + SIFS - eps;
                    TempEvent.pkt.nav = SIFS + CompressedBeamformingTime;
                    TempEvent.pkt.type = 'Report_P';
                    TempEvent.pkt.rv = STA_Info(i).soundingqueue(1);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                else
                    error 'What is the case in CBF_timeout?';
                end
            end
            
        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
            while (~isempty(STA_Info(i).soundingqueue))
                if (any(STA_Info(i).soundingqueue(1) == STA_Info(i).cover_STA))
                    break;
                else
                    STA_Info(i).soundingqueue(1) = [];
                end
            end
            if (isempty(STA_Info(i).soundingqueue))
                [MCS_index,~] = fc_return_MCS_Int(t,per_order,i,event.pkt.CoMPGroup, Num_Tx,spatial_stream,freq,...
                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_MAC_body,AI,'MAC_body');
                if sounding_skipevent_Debug == 1
                    for k=1:length(event.pkt.CoMPGroup)
                        sum_STA = length(event.pkt.CoMPGroup);
                        select_STA = event.pkt.CoMPGroup(k);
                        STA_Info(i).CSI_TS(select_STA - Num_AP) = t-(sum_STA-k)*(SIFS+ReportPollTime+SIFS+CompressedBeamformingTime)-eps;
                    end
                end
                STA_Info(i).MCS_record(event.pkt.CoMPGroup - Num_AP) = MCS_index;
                TempEvent = event;
                TempEvent.timer = t + eps;
                TempEvent.type = 'MUNDP_response';
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                NextAP = i;
                while (CoMP_Controller.information(NextAP).startorder(end) ~= NextAP )
                    NextAP = CoMP_Controller.information(NextAP).startorder(find(NextAP == CoMP_Controller.information(NextAP).startorder)+1);
                    if(CoMP_Controller.information(NextAP).readyRTS ~= 1)
                        if sounding_skipevent_Debug == 0
                            TempEvent = event;
                            TempEvent.type = 'send_PHY';
                            TempEvent.STA_ID = NextAP;
                            TempEvent.timer = t + SIFS - eps;
                            TempEvent.pkt = CoMP_Controller.information(NextAP).pkt;
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            break;
                        else
                            TempEvent = event;
                            TempEvent.type = 'CBF_timeout';
                            TempEvent.STA_ID = NextAP;
                            TempEvent.timer = t + SIFS + NDPAnnTime + SIFS + NDPTime + SIFS + CompressedBeamformingTime +...
                                            (length(TempEvent.pkt.rv)-1)*(SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);
                            TempEvent.pkt = CoMP_Controller.information(NextAP).pkt;
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            break;
                        end
                    end
                end
            else
                if (STA_Info(i).nextsounding == STA_Info(i).soundingqueue(1))
                    STA_Info(i).report_ptimes = STA_Info(i).report_ptimes + 1;
                else
                    STA_Info(i).nextsounding = STA_Info(i).soundingqueue(1);
                    STA_Info(i).report_ptimes = 1;
                end
                if STA_Info(i).report_ptimes > Max_Report_P
                    if detail_Debug, disp(['  -D: STA ' num2str(i) ' fail to get CBF than ' num2str(Max_Report_P) ' times to STA '  num2str(STA_Info(i).nextsounding) '.']); end
                    temp = CoMP_Controller.information(i).startorder;
                    for k=1:length(temp)
                        TempEvent = event;
                        TempEvent.timer = t + length(STA_Info(i).soundingqueue)*(ReportPollTime + SIFS + CompressedBeamformingTime) - ReportPollTime - eps;
                        TempEvent.type = 'send_MAC';
                        TempEvent.STA_ID = temp(k);
                        TempEvent.pkt = [];
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        STA_Info(temp(k)).rxMU = [];
                        CoMP_Controller.information(temp(k)).numInform = 0;
                        CoMP_Controller.information(temp(k)).startorder = [];
                        CoMP_Controller.information(temp(k)).readyRTS = 0;
                    end
                    STA_Info(i).soundingqueue = [];
                elseif ~isempty(STA_Info(i).soundingqueue)
                    TempEvent = event;
                    TempEvent.type = 'send_PHY';
                    TempEvent.timer = t + SIFS - eps;
                    TempEvent.pkt.nav = length(STA_Info(i).soundingqueue)*(ReportPollTime + SIFS + CompressedBeamformingTime) - ReportPollTime;
                    TempEvent.pkt.type = 'Report_P';
                    TempEvent.pkt.rv = STA_Info(i).soundingqueue(1);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                else
                    error 'What is the case in CBF_timeout?';
                end
            end
            
            
        end
    
    case 'MUNDP_response'
        t = event.timer;
        i = event.STA_ID;
        if detail_Debug, disp(['  -D: STA ' num2str(i) ' is in MUNDP_response with ' num2str(event.pkt.type) ' and it readyRTS = ' num2str(CoMP_Controller.information(i).readyRTS) '.']); end
        
        if (event.pkt.MU == 1 && event.pkt.CoMP == 1 && CoMP_Controller.information(i).reset ~= 1)
            CoMP_Controller.information(i).numInform = 1;
            pending_id(i) = 0;
            if (CoMP_Controller.information(i).numInform == 1 && all(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).numInform == 1))
                % CoMP_Controller had checked all receiving Compressed_BF corresponding to CoMP AP
                temp = [i, STA_Info(i).CoMP_coordinator];
                for k=1:length(temp)
                    CoMPAP_index = temp(k);
                    % back to send_MAC
                    TempEvent = event;
                    TempEvent.timer = t;
                    TempEvent.type = 'send_MAC';
                    TempEvent.STA_ID = CoMPAP_index;
                    TempEvent.pkt = [];
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    if lastsounding_enable
                        CoMP_Controller.information(CoMPAP_index).lastsounding = t;
                    end
                    STA_Info(CoMPAP_index).rxMU = [];
                    STA_Info(CoMPAP_index).rxMU = zeros(1,2);
                end
                for k=1:length(temp)
                    CoMP_Controller.information(temp(k)).numInform = 0;
                    CoMP_Controller.information(temp(k)).startorder = [];
                    CoMP_Controller.information(temp(k)).readyRTS = 0;
                end
            end
        else
            if detail_Debug, disp(['  -D: STA ' num2str(i) ' had been reseted, then it in MUNDP_response now.']); end
            %error('event.pkt.MU == 0 or event.pkt.CoMP == 0 do not have MUNDP_response state');
        end
    case 'SURTS_response'
        t = event.timer;
        i = event.STA_ID;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: SURTS_response @ STA ' num2str(i)]); end
        if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
            % remove pending id for RTS
            pending_id(i) = 0;
            if (any(STA_Info(i).rxMU)) % rx any CTS from MU
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' receives CTS from STA [' num2str(STA_Info(i).rxMU) '].']); end
                % send DATA
                TempEvent = event;
                TempEvent.timer = t - 2*eps + SIFS;
                TempEvent.type = 'send_PHY';
                TempEvent.STA_ID = i;
                TempEvent.pkt.size = event.pkt.size.*(STA_Info(i).rxMU == event.pkt.Group)';
                TempEvent.pkt.rv = STA_Info(i).rxMU;
                TempEvent.pkt.type = 'Data';
                % Creat a new id for the data packet
                TempEvent.pkt.id = new_id(i);
                TempEvent.pkt.nav = 0; % not necessary because RTS and CTS already did so
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            else
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' do not receive any CTS correctly.']); end
                TempEvent = event;
                TempEvent.timer = t + slotTime;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            end
        else
            error('event.pkt.MU == 1 or event.pkt.CoMP == 1 do not have SURTS_response state');
        end
    case 'MURTS_response'
        t = event.timer;
        i = event.STA_ID;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: MURTS_response @ STA ' num2str(i)]); end
        if (event.pkt.MU == 1 && event.pkt.CoMP == 0)
            % remove pending id for RTS
            pending_id(i) = 0;
            if (any(STA_Info(i).rxMU)) % rx any CTS from MU
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' receives CTS from STA [' num2str(STA_Info(i).rxMU) '].']); end
                % send DATA
                TempEvent = event;
                TempEvent.timer = t - 2*eps + SIFS;
                TempEvent.type = 'send_PHY';
                TempEvent.STA_ID = i;
                TempEvent.pkt.size = event.pkt.size.*(STA_Info(i).rxMU == event.pkt.Group)';
                if (any(STA_Info(i).rxMU == 0))
                    STA_Info(i).rxMU(STA_Info(i).rxMU == 0) = [];
                    TempEvent.pkt.rv = STA_Info(i).rxMU;
                else
                    TempEvent.pkt.rv = STA_Info(i).rxMU;
                end
                TempEvent.pkt.type = 'Data';
                % Creat a new id for the data packet
                TempEvent.pkt.id = new_id(i);
                TempEvent.pkt.nav = 0; % not necessary because RTS and CTS already did so
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            else
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' do not receive any CTS correctly.']); end
                STA(event.pkt.rv, 7) = 0; % if someone received RTS correctly, then its STA(i, 7) was on, so that close the MU transmission. Check RTS of recv_MAC
                TempEvent = event;
                TempEvent.timer = t + slotTime;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            end
        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
            CoMP_Controller.information(i).numInform = 1;
            % CoMP_Controller collect information of receiving CTS
            CoMP_Controller.information(i).rxSTA_index = ismember(STA_Info(i).IntendSTA, STA_Info(i).rxMU).*STA_Info(i).IntendSTA;
            % remove pending id for RTS
            pending_id(i) = 0;

            if (all(STA_Info(i).rxMU == 0))
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' do not receive any CTS correctly.']); end
            else % rx any CoMP MU CTS
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' receives any CTS correctly from [' num2str(STA_Info(i).rxMU) '].']); end
            end
            if (CoMP_Controller.information(i).numInform == 1 && all(CoMP_Controller.information(STA_Info(i).CoMP_coordinator).numInform == 1))
                % CoMP_Controller had checked all receiving CTS corresponding to CoMP AP
                if (all([CoMP_Controller.information([i,STA_Info(i).CoMP_coordinator]).rxSTA_index])) % All CTS+CSI are received by CoMP APs
                    for k=1:length(CoMP_Controller.information(i).startorder)
                        CoMPAP_index = CoMP_Controller.information(i).startorder(k);
                        % send DATA
                        TempEvent = event;
                        TempEvent.timer = t - 2*eps + SIFS ;
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = CoMPAP_index;
                        TempEvent.pkt.tx = CoMPAP_index;
                        TempEvent.pkt.rv = CoMP_Controller.information(CoMPAP_index).pkt.CoMPGroup;
                        TempEvent.pkt.CoMPGroup = CoMP_Controller.information(CoMPAP_index).pkt.CoMPGroup;
                        TempEvent.pkt.size = CoMP_Controller.information(CoMPAP_index).pkt.size;
                        TempEvent.pkt.type = 'Data';
                        % Creat a new id for the data packet
                        TempEvent.pkt.id = new_id(CoMPAP_index);
                        TempEvent.pkt.nav = 0; % not necessary because RTS and CTS already did so
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    end
                else % Partial CTS+CSI lost
                    flag = 0;
                    for k=1:length(CoMP_Controller.information(i).startorder)
                        CoMPAP_index = CoMP_Controller.information(i).startorder(k);
                        if (any(CoMP_Controller.information(CoMP_Controller.information(i).startorder(k)).rxSTA_index))
                            % send DATA
                            TempEvent = event;
                            TempEvent.timer = t - 2*eps + SIFS ;
                            TempEvent.type = 'send_PHY';
                            TempEvent.STA_ID = CoMPAP_index;
                            TempEvent.pkt.tx = CoMPAP_index;
                            %TempEvent.pkt.rv = CoMP_Controller.information(CoMPAP_index).rxSTA_index;
                            %TempEvent.pkt.size = TempEvent.pkt.size.*ismember(event.pkt.CoMPGroup, TempEvent.pkt.rv)';
                            %TempEvent.pkt.rv(TempEvent.pkt.rv == 0) = [];
                            TempEvent.pkt.rv = CoMP_Controller.information(CoMPAP_index).pkt.CoMPGroup;
                            TempEvent.pkt.CoMPGroup = CoMP_Controller.information(CoMPAP_index).pkt.CoMPGroup;
                            TempEvent.pkt.size = CoMP_Controller.information(CoMPAP_index).pkt.size;
                            TempEvent.pkt.type = 'Data';
                            TempEvent.pkt.id = new_id(CoMPAP_index);
                            TempEvent.pkt.nav = 0; % not necessary because RTS and CTS already did so
                            if detail_Debug, disp(['  -D: STA ' num2str(CoMP_Controller.information(i).startorder) ' fail doing CoMP, thus STA ' num2str(TempEvent.pkt.tx) ' transmit Data to STA ' num2str(TempEvent.pkt.rv) '.']); end
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            flag = 1;
%                             break;
                        else
                            CoMP_Controller.information(i).doneData = -1;
                        end
                    end
                    if (flag == 0)
                        % CoMP transmission failed
                        CoMPfail = CoMPfail + 1;
                        for k=1:length(CoMP_Controller.information(i).startorder)
                            TempEvent = event;
                            TempEvent.timer = t + slotTime;
                            TempEvent.type = 'send_MAC';
                            TempEvent.STA_ID = CoMP_Controller.information(i).startorder(k);
                            TempEvent.pkt = [];
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                            CoMP_Controller.information(CoMP_Controller.information(i).startorder(k)).doneData = 0;
                        end
                        STA(event.pkt.rv, 7) = 0; % if someone received RTS correctly, then its STA(i, 7) was on, so that close the MU transmission. Check RTS of recv_MAC
                        CoMP_Controller.informer_index(CoMP_Controller.information(i).startorder) = 0;
                        temp = [i, STA_Info(i).CoMP_coordinator];
                        for k=1:length(temp)
                            CoMP_Controller.information(temp(k)).startorder = 0;
                        end
                    else
                        CoMPfailbutsave = CoMPfailbutsave + 1;
                    end
                end
                temp = [i, STA_Info(i).CoMP_coordinator];
                for k=1:length(temp)
                    CoMP_Controller.information(temp(k)).numInform = 0;
                    CoMP_Controller.information(temp(k)).rxSTA_index = 0;
                    STA_Info(temp(k)).rxMU = [];
                end
            end
        end
        
    case 'SUData_respnse'
        t = event.timer;
        i = event.STA_ID;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: timeout_' num2str(event.pkt.type) ' @ STA ' num2str(i)]); end
        if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
            if (pending_id(i) == event.pkt.id) % Not acknowledgement yet
                % Remove pending_id for RTS timeout or Data timeout
                pending_id(i) = 0;
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' drop current traffic and will start next traffic']); end
                TempEvent = event;
                TempEvent.timer = t;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i;
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            else
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' had received any response for ' num2str(event.pkt.type) '.' ]); end
            end
        else
            error('event.pkt.MU == 1 do not have timeout state');
        end
        STA_Info(i).rxMU = [];
        
    case 'MUData_response'
        t = event.timer;
        i = event.STA_ID;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: MUData_response @ STA ' num2str(i)]); end
        if (event.pkt.MU == 1 && event.pkt.CoMP == 0)
            STA(event.pkt.Group, 7) = 0;
            % remove pending id for Data
            pending_id(i) = 0;
            if (any(STA_Info(i).rxMU)) % rx any ACK from MU
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' receives any ACK correctly from STA [' num2str(STA_Info(i).rxMU) '].']); end
            else
                if detail_Debug, disp(['  -D: STA ' num2str(i) ' do not receive any ACK correctly.']); end
            end
            % Go to send_MAC check STA traffic queue
            TempEvent = event;
            TempEvent.timer = t;
            TempEvent.type = 'send_MAC';
            TempEvent.STA_ID = i;
            TempEvent.pkt = [];
            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
            STA_Info(i).rxMU = [];
        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
            % CoMP_Controller collect all reciving ACK and tell AP go to idle
            % remove pending id for Data, i is a vector
            STA(event.pkt.CoMPGroup, 7) = 0;
            CoMPdone = CoMPdone + 1;
            pending_id(i) = 0;
            for k=1:length(i)
                if (any(STA_Info(i(k)).rxMU)) % rx any ACK from MU
                    if detail_Debug, disp(['  -D: STA ' num2str(i(k)) ' receives any ACK correctly from STA [' num2str(STA_Info(i(k)).rxMU) '].']); end
                else
                    if detail_Debug, disp(['  -D: STA ' num2str(i(k)) ' do not receive any ACK correctly.']); end
                end
            end
            for k=1:length(i)
                % Go to send_MAC check STA traffic queue
                TempEvent = event;
                TempEvent.timer = t;
                TempEvent.type = 'send_MAC';
                TempEvent.STA_ID = i(k);
                TempEvent.pkt = [];
                NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                CoMP_Controller.information(i(k)).startorder = [];
                CoMP_Controller.information(i(k)).lastsounding = -1;
                CoMP_Controller.information(i(k)).doneData = 0;
                CoMP_Controller.information(i(k)).readyRTS = 0;
                CoMP_Controller.informer_index(i(k)) = 0;
                CoMP_Controller.information(i(k)).numInform = 0;
                STA_Info(i(k)).rxMU = [];
            end
        end
    case 'recv_PHY'
        t = event.timer;
        i = event.pkt.tx;
        j = event.STA_ID;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: recv_PHY @ STA ' num2str(j)]); end
        if detail_Debug, disp(['  -D: STA ' num2str(j) ' is receiveng ' num2str(event.pkt.type) ' from STA ' num2str(i) '.']); end
        if STA(j, 6) ~= 2
            error(['recv_PHY: STA ' num2str(j) ' is not in receiving mode']);
        end
        STA(j, 6) = 0; % The receiver switches back to idle mode
        if ((t > nav(j).start) && (t < nav(j).end))
            % This has already been checked when sending but nav may be changed during transmission, so double check
            if detail_Debug, disp(['  -D: STA '  num2str(j) ' has packet virtual collision.']); end
        else
            switch PHY_CH_module
                case 'old'
                    [pr0, pr, snr] = recv_phy(i, j, rmodel);
                    if (strcmp(event.pkt.type, 'Data') == 1)
                        if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                            Prob = PER(event.pkt.MCS_index, event.pkt.size, snr);
                            %Prob = 0;
                        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                            if (isempty(event.pkt.MCS_index(j==event.pkt.Group)))
                                Prob = 1;
                            else
                                Prob = PER(event.pkt.MCS_index(j==event.pkt.Group), event.pkt.size(j==event.pkt.Group), snr);
                                %Prob = 0;
                            end
                        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                            if (isempty(event.pkt.MCS_index(j==event.pkt.CoMPGroup)))
                                Prob = 1;
                            else
                                %    Prob = 0;
                                Prob = PER(event.pkt.MCS_index(j==event.pkt.CoMPGroup), event.pkt.size(j==event.pkt.CoMPGroup), snr);
                                %dBB = db(snr, 'power');disp(['STA ' num2str(j) ' receives ' num2str(snr) '(' num2str(dBB) ') => PER = ' num2str(Prob) ' select MCS' num2str(event.pkt.MCS_index(j==event.pkt.Group)) '.']);
                            end
                        end
                    elseif (control_frame_Debug == 1)
                        Prob = 0;
                    elseif (strcmp(event.pkt.type, 'NDP_Ann') == 1)
                        Prob = PER(MCS_ctrl, size_NDP_Ann, snr);
                    elseif (strcmp(event.pkt.type, 'NDP') == 1)
                        Prob = 0;
                    elseif (strcmp(event.pkt.type, 'Compressed_BF') == 1)
                        Prob = PER(MCS_ctrl, size_CVBFReport, snr);
                    elseif (strcmp(event.pkt.type, 'Report_P') == 1)
                        Prob = PER(MCS_ctrl, size_ReportPoll, snr);
                    elseif (strcmp(event.pkt.type, 'ACK') == 1)
                        Prob = PER(MCS_ctrl, size_BA, snr);
                    elseif (strcmp(event.pkt.type, 'RTS') == 1)
                        Prob = PER(MCS_ctrl, size_RTS, snr);
                    elseif (strcmp(event.pkt.type, 'CTS') == 1)
                        Prob = PER(MCS_ctrl, size_CTS, snr);
                    end
                case 'new'
                    if (strcmp(event.pkt.type, 'Data') == 1)
                        if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                            [pr,snr, Prob] = fc_recv_phy(t,1,event.pkt.MCS_index, i,j, spatial_stream,spatial_stream,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,event.pkt.size,AI,event.pkt.type);
                            %Prob = 0;
                        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                            if (isempty(event.pkt.MCS_index(j==event.pkt.Group)))
                                Prob = 1;
                                pr = 1;
                            else
                                [pr,snr, Prob] = fc_recv_phy(t,find(j==event.pkt.Group),event.pkt.MCS_index(j==event.pkt.Group),i,j,length(event.pkt.Group)*spatial_stream,spatial_stream,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,event.pkt.size(j==event.pkt.Group),AI,event.pkt.type);
                                %Prob = 0;
                            end
                        elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                            if (isempty(event.pkt.MCS_index(j==event.pkt.CoMPGroup)))
                                Prob = 1;
                                pr = 1;
                            else
                                [pr,snr, Prob] = fc_recv_phy(t,find(j==event.pkt.CoMPGroup),event.pkt.MCS_index(j==event.pkt.CoMPGroup),i(i==STA(j,3)),j,length(event.pkt.Group)*spatial_stream,spatial_stream,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,event.pkt.size(j==event.pkt.CoMPGroup),AI,event.pkt.type);
                                %    Prob = 0;
                                %dBB = db(snr, 'power');disp(['STA ' num2str(j) ' receives ' num2str(snr) '(' num2str(dBB) ') => PER = ' num2str(Prob) ' select MCS' num2str(event.pkt.MCS_index(j==event.pkt.Group)) '.']);
                            end
                        end
                    elseif (control_frame_Debug == 1)
                        pr = 0;
                        Prob = 0;
                    elseif (strcmp(event.pkt.type, 'NDP_Ann') == 1)
                        [pr,snr, Prob] = fc_recv_phy(t,1,MCS_ctrl, i,j, 1,1,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_NDP_Ann,AI,event.pkt.type);
                    elseif (strcmp(event.pkt.type, 'NDP') == 1)
                        pr = 0;
                        Prob = 0;
                    elseif (strcmp(event.pkt.type, 'Compressed_BF') == 1)
                        [pr,snr, Prob] = fc_recv_phy(t,1,MCS_ctrl, i,j, 1,1,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_CVBFReport,AI,event.pkt.type);
                    elseif (strcmp(event.pkt.type, 'Report_P') == 1)
                        [pr,snr, Prob] = fc_recv_phy(t,1,MCS_ctrl, i,j, 1,1,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_ReportPoll,AI,event.pkt.type);
                    elseif (strcmp(event.pkt.type, 'ACK') == 1)
                        [pr,snr, Prob] = fc_recv_phy(t,1,MCS_ctrl, i,j, 1,1,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_BA,AI,event.pkt.type);
                    elseif (strcmp(event.pkt.type, 'RTS') == 1)
                        [pr,snr, Prob] = fc_recv_phy(t,1,MCS_ctrl, i,j, 1,1,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_RTS,AI,event.pkt.type);
                    elseif (strcmp(event.pkt.type, 'CTS') == 1)
                        [pr,snr, Prob] = fc_recv_phy(t,1,MCS_ctrl, i,j, 1,1,freq,...
                                                    Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
                                                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_CTS,AI,event.pkt.type);
                    end
            end

            if (rand(1) < Prob) % Error packet
                if detail_Debug, disp(['  -D: STA ' num2str(j) ' cannot hear ' num2str(event.pkt.type) ' from STA ' num2str(i) '.']); end
                if (any(j == event.pkt.rv))
                    if (event.pkt.CoMP == 0)
                        if (pr ~= 0)
                            if (strcmp(event.pkt.type, 'Data') == 1)
                                Interferencepkt = Interferencepkt + 1;
                            end
                        end
                        if (strcmp(event.pkt.type, 'Data') == 1)
                            if (pr ~= 0)
                                InterferenceDatapkt = InterferenceDatapkt + 1;
                            end
                            DataSentFailNotRx(i) = DataSentFailNotRx(i) + 1;
                        else
                            if (pr ~= 0)
                                InterferenceCtrlpkt = InterferenceCtrlpkt + 1;
                            end
                            if (strcmp(event.pkt.type, 'ACK') == 1)
                                DataSentFailNoACK(j) = DataSentFailNoACK(j) + 1;
                            end
                        end

                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        % Due to CoMP transmission, CoMP APs consecutively send RTS.
                        % If a STA receives any RTS should response CTS.
                        % Thus, a STA do not receive the last RTS but receives other RTS should also go to recv_MAC.
%                         if (strcmp(event.pkt.type, 'RTS') == 1 && i == CoMP_Controller.information(i).startorder(end) && ~isempty(STA_Info(j).rxMU))
%                             TempEvent = event;
%                             TempEvent.timer = t;
%                             TempEvent.type = 'recv_MAC';
%                             TempEvent.STA_ID = j;
%                             NewEvents = [NewEvents, TempEvent]; clear TempEvent;
%                         end
                        if (pr ~= 0)
                            if (strcmp(event.pkt.type, 'Data') == 1 && CoMP_Controller.information(STA(j,3)).doneData ~= -1)
                                Interferencepkt = Interferencepkt + 1;
                            end
                        end
                        if (strcmp(event.pkt.type, 'Data') == 1 && CoMP_Controller.information(STA(j,3)).doneData ~= -1)
                            if (pr ~= 0)
                                InterferenceDatapkt = InterferenceDatapkt + 1;
                            end
                            if (event.pkt.CoMP == 1)
                                DataSentFailNotRx(STA(j, 3)) = DataSentFailNotRx(STA(j, 3)) + 1;
                            end
                        else
                            if (pr ~= 0)
                                InterferenceCtrlpkt = InterferenceCtrlpkt + 1;
                            end
                            if (strcmp(event.pkt.type, 'ACK') == 1)
                                DataSentFailNoACK(j) = DataSentFailNoACK(j) + 1;
                            end
                        end
                    end
                end
            else
                if (any(j == event.pkt.rv))
                    % The STA receives CTS also CSI, so generate channel matrix
                    if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                        if (strcmp(event.pkt.type, 'Data') == 1)
                             switch PHY_CH_module
                                    case 'old'
                                        STA_Info(i).SNR_record(j) = snr;
                                 case 'new'
                             end
                        end
                        if (strcmp(event.pkt.type,'Compressed_BF') == 1)
                            if (STA(j, 3) == 0) % A STA i sends CTS to the AP j and the AP j receives correctly
                                switch PHY_CH_module
                                    case 'old'
                                        STA_Info(j).Channel_Matrix((i-1)*Num_Rx+1:i*Num_Rx, :) = sqrt(recv_power(i, j, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                                    case 'new'
%                                         [MCS_index,~] = fc_return_MCS(per_order,j,i, Num_Tx,spatial_stream,freq,...
%                                                      Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
%                                                      tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_MAC_body,AI);
%                                         MCS_index = 1;
%                                         STA_Info(j).MCS_record(i-Num_AP) = MCS_index;
                                        
                                end
                                STA_Info(j).CSI_TS(i-Num_AP) = t;
                                if MIMO_Debug, disp(['  -M: Channel Matrix(CSI), H' num2str(i) num2str(j) ' = ']); disp(STA_Info(j).Channel_Matrix((i-1)*Num_Rx+1:i*Num_Rx, :)); end
                            end
                        end
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'recv_MAC';
                        TempEvent.STA_ID = j;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                        if (strcmp(event.pkt.type,'Compressed_BF') == 1)
                            if (STA(j, 3) == 0) % A STA i sends CTS to the AP j and the AP j receives correctly
                                switch PHY_CH_module
                                    case 'old'
                                        STA_Info(j).Channel_Matrix((i-1)*Num_Rx+1:i*Num_Rx, :) = sqrt(recv_power(i, j, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                                    case 'new'
%                                         if ((t - STA_Info(j).CSI_TS(i-Num_AP)) < soundingperiod && skip_cal_MCS_sp_Debug)
%                                             MCS_index = STA_Info(j).MCS_record(i - Num_AP);
%                                         else
%                                             [MCS_index,~] = fc_return_MCS(per_order,j,i, Num_Tx,spatial_stream,freq,...
%                                                 Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
%                                                 tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_MAC_body,AI);
%                                             MCS_index=1;
%                                         end
%                                         STA_Info(j).MCS_record(i-Num_AP) = MCS_index;
                                end
                                STA_Info(j).CSI_TS(i-Num_AP) = t;
                                if MIMO_Debug, disp(['  -M: Channel Matrix(CSI), H' num2str(i) num2str(j) ' = ']); disp(STA_Info(j).Channel_Matrix((i-1)*Num_Rx+1:i*Num_Rx, :)); end
                            end
                        end
                        TempEvent = event;
                        TempEvent.timer = t;
                        TempEvent.type = 'recv_MAC';
                        TempEvent.STA_ID = j;
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                        if (strcmp(event.pkt.type,'Compressed_BF') == 1)
                            if (STA(j, 3) == 0) % A STA i sends CTS to the AP j and the AP j receives correctly
                                %all_CoMP_tx = sort(j, STA_Info(j).CoMP_coordinator);
                                %if (event.pkt.sounding_index(j==all_CoMP_tx, i==event.pkt.CoMPGroup) == 1)
                                switch PHY_CH_module
                                    case 'old'
                                        STA_Info(j).Channel_Matrix((i-1)*Num_Rx+1:i*Num_Rx, :) = sqrt(recv_power(i, j, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                                    case 'new'
%                                         if STA(i, 3) == j
%                                             if ((t - STA_Info(j).CSI_TS(i-Num_AP)) < soundingperiod && skip_cal_MCS_sp_Debug)
%                                                 MCS_index = STA_Info(j).MCS_record(i - Num_AP);
%                                             else
%                                                 [MCS_index,~] = fc_return_MCS(per_order,j,i, Num_Tx,spatial_stream,freq,...
%                                                 Nsubcarrier,default_power,channel_model,Bandwidth,Thermal_noise,...
%                                                 tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,size_MAC_body,AI);
%                                             end
%                                         else
%                                             MCS_index = 1;
%                                         end
%                                         STA_Info(j).MCS_record(i-Num_AP) = MCS_index;
                                end
                                STA_Info(j).CSI_TS(i-Num_AP) = t;
                                if MIMO_Debug, disp(['  -M: Channel Matrix(CSI), H' num2str(i) num2str(j) ' = ']); disp(STA_Info(j).Channel_Matrix((i-1)*Num_Rx+1:i*Num_Rx, :)); end
                                %end
                            end
                        end
                        if ( (strcmp(event.pkt.type, 'Data') ~= 1) || ((strcmp(event.pkt.type, 'Data') == 1) && (CoMP_Controller.information(STA(j,3)).doneData ~= -1) ) ) 
                            TempEvent = event;
                            TempEvent.timer = t;
                            TempEvent.type = 'recv_MAC';
                            TempEvent.STA_ID = j;
                            NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                        end
                    end

                elseif (event.pkt.nav > 0)    % this packet is not for me, but use its nav
                    if (nav(j).start < t)
                        nav(j).start = t;
                    end
                    if (nav(j).end < (t+event.pkt.nav))
                        nav(j).end = t + event.pkt.nav;
                    end
                    if detail_Debug, disp(['  -D: STA ' num2str(j) ' set NAV.start = ' num2str(nav(j).start) ' NAV.end = ' num2str(nav(j).end)]); end
                end
            end
        end
    case 'recv_MAC'
        t = event.timer;
        i = event.pkt.tx;
        j = event.STA_ID;
        if event_Debug, disp(['[' num2str(t, '%1.20f') ']: recv_MAC @ STA ' num2str(j)]); end
        if detail_Debug, disp(['  -D: STA ' num2str(j) ' receives ' num2str(event.pkt.type) ' from ' num2str(i) '.']); end
        switch (event.pkt.type)
            case 'NDP_Ann'
                TempEvent = event;
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    STA_Info(j).rxMU = [];
                    STA_Info(j).rxMU = i;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    STA_Info(j).rxMU = [];
                    STA_Info(j).rxMU = i;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA_Info(j).rxMU = [];
                    STA_Info(j).rxMU = i;
                end
            case 'NDP'
                % Send back a CTS
                TempEvent = event;
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    TempEvent.timer = t + SIFS;
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    TempEvent.pkt.type = 'Compressed_BF';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = event.pkt.nav - (SIFS + CompressedBeamformingTime);
                    %disp(['  pkt.tx = ' num2str(TempEvent.pkt.tx) '  pkt.rx = ' num2str(TempEvent.pkt.rv)  '  STA_ID = ' num2str(TempEvent.STA_ID)]);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    STA(j, 7) = 1; % Involve MU transmission
                    TempEvent.timer = t + SIFS;
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    TempEvent.pkt.type = 'Compressed_BF';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = event.pkt.nav - (SIFS + CompressedBeamformingTime);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA(j, 7) = 1; % Involve CoMP MU transmission
                    %if (i == STA_Info(j).rxMU)
                        TempEvent.timer = t + SIFS;
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = j;
                        TempEvent.pkt.type = 'Compressed_BF';
                        TempEvent.pkt.tx = j;
                        %TempEvent.pkt.rv = STA_Info(j).rxMU;
                        TempEvent.pkt.rv = i;
                        TempEvent.pkt.nav = (length(TempEvent.pkt.CoMPGroup)-1) * (SIFS + ReportPollTime + SIFS + CompressedBeamformingTime);

                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    %end

                end

            case 'Report_P'
                TempEvent = event;
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    STA(j, 7) = 1;
                    TempEvent.timer = t + SIFS;

                    TempEvent.type = 'send_PHY';

                    TempEvent.STA_ID = j;
                    TempEvent.pkt.type = 'Compressed_BF';
                    TempEvent.pkt.tx = j;
                    %TempEvent.pkt.rv = STA_Info(j).rxMU;  %for ingnor NDP_Ann failure
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = event.pkt.nav - (SIFS + CompressedBeamformingTime);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    STA(j, 7) = 1;
                    TempEvent.timer = t + SIFS;

                    TempEvent.type = 'send_PHY';

                    TempEvent.STA_ID = j;
                    TempEvent.pkt.type = 'Compressed_BF';
                    TempEvent.pkt.tx = j;
                    %TempEvent.pkt.rv = STA_Info(j).rxMU;  %for ingnor NDP_Ann failure
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = event.pkt.nav - (SIFS + CompressedBeamformingTime);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;

                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA(j, 7) = 1; % Involve CoMP MU transmission
                    %if (i == STA_Info(j).rxMU)
                        TempEvent.timer = t + SIFS;

                        TempEvent.type = 'send_PHY';

                        TempEvent.STA_ID = j;
                        TempEvent.pkt.type = 'Compressed_BF';
                        TempEvent.pkt.tx = j;
                        %TempEvent.pkt.rv = STA_Info(j).rxMU;
                        TempEvent.pkt.rv = i;
                        TempEvent.pkt.nav = event.pkt.nav - (SIFS + CompressedBeamformingTime);
                        %TempEvent.pkt.nav = 0;

                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    %end
                end

            case 'Compressed_BF'
                TempEvent = event;
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    STA_Info(j).rxMU = [STA_Info(j).rxMU, i];
                    STA_Info(j).soundingqueue = STA_Info(j).soundingqueue(2:length(STA_Info(j).soundingqueue));
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    STA_Info(j).rxMU = [STA_Info(j).rxMU, i];
                    STA_Info(j).soundingqueue = STA_Info(j).soundingqueue(2:length(STA_Info(j).soundingqueue));
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA_Info(j).rxMU = [STA_Info(j).rxMU, i];
                    STA_Info(j).soundingqueue = STA_Info(j).soundingqueue(2:length(STA_Info(j).soundingqueue));
                end
            case 'RTS'
                TempEvent = event;
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    TempEvent.timer = t + SIFS;
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    TempEvent.pkt.type = 'CTS';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = STA(j,3);
                    TempEvent.pkt.nav = TempEvent.pkt.nav - (SIFS + CTS_tx_time);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    STA(j, 7) = 1; % Involve MU transmission
                    temp = find(j == event.pkt.Group);
                    TempEvent.timer = t + SIFS + (temp-1)*(CTS_tx_time+SIFS);                    
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    TempEvent.pkt.type = 'CTS';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = event.pkt.nav - temp*(SIFS+CTS_tx_time);
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA(j, 7) = 1; % Involve CoMP MU transmission
%                     if( i == CoMP_Controller.information(i).startorder(1) || length(STA_Info(j).rxMU) > length(STA_Info(i).CoMP_coordinator))
%                         STA_Info(j).rxMU = [];
%                     end
%                     STA_Info(j).rxMU = [STA_Info(j).rxMU, i];
                    %if (i == CoMP_Controller.information(i).startorder(end))
%                     if( length(STA_Info(j).rxMU) == 1)
%                         temp = find(j == event.pkt.CoMPinOrder);
                    if (STA(j,3) == i)  %only reply its associate AP
                        temp = find(j == event.pkt.rv);
                        numorder = find(i == CoMP_Controller.information(i).startorder);
                        TempEvent.timer = t + (length(CoMP_Controller.information(i).startorder) - numorder)*(SIFS + RTS_tx_time)+ SIFS + (temp-1)*(CTS_tx_time+SIFS);
                        TempEvent.type = 'send_PHY';
                        TempEvent.STA_ID = j;
                        TempEvent.pkt.type = 'CTS';
                        TempEvent.pkt.tx = j;
                        %                         TempEvent.pkt.rv = STA_Info(j).rxMU;
                        TempEvent.pkt.rv = STA(j,3);
                        TempEvent.pkt.nav = event.pkt.nav - ((length(CoMP_Controller.information(i).startorder) - numorder)*(SIFS+RTS_tx_time)) - (temp*(SIFS+CTS_tx_time));
                        NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    end
                        %STA_Info(j).rxMU = []; %move to sendPHY
%                     end
                end

            case 'CTS'
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    STA_Info(j).rxMU = STA_Info(j).rxMU + (i == event.pkt.Group).*event.pkt.Group;
                    if detail_Debug, disp(['  -D: Receiver flag: STA [' num2str(STA_Info(j).rxMU) ']']); end
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    STA_Info(j).rxMU = STA_Info(j).rxMU + (i == event.pkt.Group).*event.pkt.Group;
                    if detail_Debug, disp(['  -D: Receiver flag: STA [' num2str(STA_Info(j).rxMU) ']']); end
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA_Info(j).rxMU = STA_Info(j).rxMU + (i == event.pkt.CoMPGroup).*event.pkt.CoMPGroup;
                    if detail_Debug, disp(['  -D: Receiver flag: STA [' num2str(STA_Info(j).rxMU) ']']); end
                end
                
            case 'Data'
                % send back an ACK
                TempEvent = event;
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    TempEvent.timer = t + SIFS;
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    % keep the data size, rate, and id the same as DATA packet
                    TempEvent.pkt.type = 'ACK';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = 0; % not necessary because RTS CTS already did so
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    temp = find(j == event.pkt.Group);
                    TempEvent.timer = t + SIFS + (temp - 1) * (ACK_tx_time + SIFS);
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    % keep the data size, rate, and id the same as DATA packet
                    TempEvent.pkt.type = 'ACK';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = i;
                    TempEvent.pkt.nav = 0; % not necessary because RTS CTS already did so
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    temp = find(j == event.pkt.CoMPGroup);
                    TempEvent.type = 'send_PHY';
                    TempEvent.STA_ID = j;
                    % keep the data size, rate, and id the same as DATA packet
                    TempEvent.pkt.type = 'ACK';
                    TempEvent.pkt.tx = j;
                    TempEvent.pkt.rv = STA(j, 3);
                    TempEvent.timer = t + SIFS + (temp - 1) * (ACK_tx_time + SIFS);
                    %if(RTS_method ~= 0)
                        TempEvent.pkt.nav = 0; % not necessary because RTS CTS already did so
                    %end
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                end
            case 'ACK'
                if detail_Debug, disp(['  -D: The pending_id(STA ' num2str(j) ') = ' num2str(pending_id(j)) ', event.pkt.id = ' num2str(event.pkt.id)]); end
                if (event.pkt.MU == 0 && event.pkt.CoMP == 0)
                    if pending_id(j) ~= event.pkt.id
                        if detail_Debug, disp(['  -D: The received CTS id ' num2str(event.pkt.id) ' does not match the pending RTS id ' num2str(pending_id(j)) '.']); end
                        % probably this CTS is in response to an earlier RTS
                        return;
                    end
                    % remove pending id for DATA
                    pending_id(j) = 0;
                    % Go to send_MAC check STA traffic queue
                    TempEvent = event;
                    TempEvent.timer = t;
                    TempEvent.type = 'send_MAC';
                    TempEvent.STA_ID = j;
                    TempEvent.pkt = [];
                    NewEvents = [NewEvents, TempEvent]; clear TempEvent;
                    % Gathering statics
                    DataSentSucceessed(j) = DataSentSucceessed(j) + 1;
                    statics_rv_bits(i) = statics_rv_bits(i) + event.pkt.size;
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 0)
                    if pending_id(j) ~= event.pkt.id
                        if detail_Debug, disp(['  -D: The received CTS id ' num2str(event.pkt.id) ' does not match the pending RTS id ' num2str(pending_id(j)) '.']); end
                        % probably this CTS is in response to an earlier RTS
                        return;
                    end
                    STA_Info(j).rxMU = STA_Info(j).rxMU + (i == event.pkt.Group).*event.pkt.Group;
                    if detail_Debug, disp(['  -D: Receiver flag: STA [' num2str(STA_Info(j).rxMU) ']']); end
                    % Gathering statics
                    DataSentSucceessed(j) = DataSentSucceessed(j) + 1;
                    statics_rv_bits(i) = statics_rv_bits(i) + event.pkt.size(i == event.pkt.Group);
                elseif (event.pkt.MU == 1 && event.pkt.CoMP == 1)
                    STA_Info(j).rxMU = STA_Info(j).rxMU + (i == event.pkt.CoMPGroup).*event.pkt.CoMPGroup;
                    if detail_Debug, disp(['  -D: Receiver flag: STA [' num2str(STA_Info(j).rxMU) ']']); end
                    DataSentSucceessed(j) = DataSentSucceessed(j) + 1;
                    statics_rv_bits(i) = statics_rv_bits(i) + event.pkt.size(i == event.pkt.CoMPGroup);
                end
            otherwise
                error(['recv_MAC: Undefined MAC packet type: ' event.pkt.type]);
        end
    otherwise
        error(['action_CoMP: Undefined event type: ' event.type]);
end

end