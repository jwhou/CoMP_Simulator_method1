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
% A CoMP simulation template
clear; clc;
disp(['Start simulator @ ' num2str(datestr(now))]);
global Num_AP Num_User;
global STA STA_Info CoMP_Controller;
global soundingperiod;
global statics_rv_bits;
global DataSentSucceessed DataSent DataSentFailNotRx DataSentFailNoACK;
global Interferencepkt InterferenceDatapkt InterferenceCtrlpkt; 
global CoMPdone CoMPfail CoMPfailbutsave;
global DebugDataTime DebugBOTime;
global sounding_Debug;
global RTS_method;
global lastsounding_enable;

% scheme_quantity = [1,2,3];
scheme_quantity = [2]; % testing by shuyu 
RTS_method = 0; %testing by jing-wen  0: no RTS, 1: Typical RTS, 2: RTS Simultaneously
lastsounding_enable = 0; %testing by jing-wen  0: no record, 1: record
run_quantity = [1];
envAPmodel = -2; % [-2:specail case, -1: Fixed AP location, 0: Square grid, 1:Hexagonal grid, 2: PPP]
AP_distance = 90; 
envUsermodel = -2; % [-2:special case, -1: Fixed users, 0: Users around AP, 1:Hexagonal grid, 2: PPP]
tho = [0.00005, 0.0005]; % tho_AP, tho_user
Area = [500, 500]; % areaX, areaY
average_Troughput = zeros(length(run_quantity), length(scheme_quantity));
% ======================== Block 1 ========================
% Block1: user inputs settings for simulation
maxSimTime = 1;%1000*9*1e-6;
traffic_type = 0;
traffic_rate = 0;
% Initialize random number generator
rng(sum(100*clock),'twister'); %psuedo randomizer
% ====================== Block 1 end ======================
for ind0=1:length(run_quantity)
    run = run_quantity(ind0);
    %disp(['  - Run ' num2str(run)]);
    [Num_AP, NumUserVec, XY_vector, Heterogeneous,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,wall_start,wall_end] = BuildEnvironment(envAPmodel, AP_distance, envUsermodel, tho, Area);
    Num_User = sum(NumUserVec);
    for ind1=1:length(scheme_quantity)
        scheme = scheme_quantity(ind1);
        % ======================== Block 2 ========================
        % Block2: do parameters settings and initialize CoMP_Controller for simulation
        parameter;
        % ============= Parse wall data & calculate room range ========== %
        [wall_index] = fc_check_input_data(Npkt,Npkt_length,AI,MCS_in_beamtracking,...
            tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,maxSimTime,wall_start,wall_end);
        [wall_mix,HOV,wall_x_range,wall_y_range] = fc_parse_wall_data(wall_index,wall_start,wall_end);
        room_range = (wall_x_range(2)-wall_x_range(1))+(wall_y_range(2)-wall_y_range(1));
        % ============= Parse wall data & calculate room range ========== %
        [STA_Info, STA] = Init_STAInfo(Num_AP, NumUserVec, XY_vector, traffic_type, traffic_rate, envUsermodel);% use some global parameters from parmeter.m        numSTAs = Num_AP + Num_User;
        STA = [STA, zeros(numSTAs, 4)]; % [location x, location y, associated AP, arrival rate, transmitting power, state(0:idle ; 1:transmit ; 2:receive), on transmission, 0:Null 1:RTS/CTS/ACK 2:Data]
        DrawEnvironment(Num_AP, Num_User, XY_vector, envAPmodel, envUsermodel, tho, Area);
        if (scheme == 1)
            disp(['======== Scheme: DL MU-MIMO @ '  num2str(datestr(now))]);
            fid = fopen(['outputs_traffic', num2str(traffic_type),'_MU.txt'],'a');
            CoMP_Controller = [];
        elseif (scheme == 2)
            disp(['======== Scheme: CS/CB @ '  num2str(datestr(now))]);
            fid = fopen(['outputs_traffic', num2str(traffic_type),'_CSCB.txt'],'a');
            CoMP_Controller = CoMP_Pair_Algo(Num_AP, Num_User, XY_vector, 1, envAPmodel, tho, Area);
        elseif (scheme == 3)
            disp(['======== Scheme: Joint Process @ '  num2str(datestr(now))]);
            fid = fopen(['outputs_traffic', num2str(traffic_type),'_JP.txt'],'a');
            CoMP_Controller = CoMP_Pair_Algo(Num_AP, Num_User, XY_vector, 0, envAPmodel, tho, Area);
        end
        % ====================== Block 2 end ======================

        % ======================== Block 3 ========================
        % Block3: Initialize Event_list for simulation and a while loop time based event runner 
        Event_list = [];
        Init_event_size = 1;
        for i=1:numSTAs
            if (STA(i, 3) ~= -1)
                Event_list(Init_event_size).timer = 0;
                Event_list(Init_event_size).type = 'idle';
                Event_list(Init_event_size).STA_ID = i;
                Event_list(Init_event_size).pkt = [];
                Init_event_size = Init_event_size + 1;
            end
        end
        tstart = clock;
        % Run the simulation
        % Time based event driven simulation loop
        while 1
            if (isempty(Event_list))
                break;
            end
            [min_timer, min_index] = min([Event_list(:).timer]);
            if isempty(min_timer) 
               break;
            end
            if min_timer > maxSimTime
               break;
            end
            % Get new events from executing the latest 'action'
            NewEvents = action_CoMP_method1(Event_list(min_index),PHY_CH_module,Nsubcarrier,channel_model,Bandwidth,Thermal_noise,tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,AI,select_user);
            
            Event_list(min_index) = []; % Delete the latest event which has been just executed.
            Event_list = [NewEvents, Event_list]; % Append new events generated by executing the latest event
        end
        fprintf(fid, '======== Run: %d, Running time:%g s ========\r\n', run, etime(clock, tstart));
        % ====================== Block 3 end ======================
        
        % ======================== Block 4 ========================
        % Block4: outputs simulation results
        % Observation suggestion by Prof. Chao and JK
        Data_t = sum(DebugDataTime(1:Num_AP));
        fprintf(fid, 'Simulation time: %f (Data transmission time: %f)\r\n', maxSimTime, Data_t);
        %fprintf(fid, 'Simulation time: %f\r\n', maxSimTime);
        fprintf(fid, 'Sounding period: %f\r\n', soundingperiod);
        fprintf(fid, 'Number of Data packet sent in the Network Topology: %d,\r\n(Received correctly: %d, Received failed: %d, (Drop: %d, No ACK: %d))\r\n', sum(DataSent), sum(DataSentSucceessed), sum(DataSentFailNotRx)+sum(DataSentFailNoACK), sum(DataSentFailNotRx), sum(DataSentFailNoACK));
        fprintf(fid, 'Number of doing CoMP: %d, (Not enough CSI: %d, No Intended CSI: %d)\r\n', CoMPdone, CoMPfailbutsave, CoMPfail);
        fprintf(fid, 'Number of collided packet: %d, (Data: %d, Control: %d)\r\n', Interferencepkt, InterferenceDatapkt, InterferenceCtrlpkt);
        fprintf(fid, '\r\n');
        for i=1:Num_AP
            if (~isempty(STA_Info(i).CoMP_coordinator))
                fprintf(fid, 'AP%d coordinates with AP%d\r\n', i, STA_Info(i).CoMP_coordinator);
            end
        end
        fprintf(fid, '\r\nData packets sent distribution:\r\n');
        for i=1:Num_AP
            if (STA(i, 3) ~= -1)
                fprintf(fid, 'AP %3d(%5d/%5d) ', i, DataSentSucceessed(i), DataSent(i));
            end
        end
        fprintf(fid, '\r\n');
        for i=1:max([STA_Info(1:Num_AP).num_ass])
            for j=1:Num_AP
                if (STA(j, 3) ~= -1)
                    if (i > STA_Info(j).num_ass)
                        fprintf(fid, '                    ');
                        continue;
                    else
                        sel_STA = STA_Info(j).associated_STA(i);
                        fprintf(fid, 'STA%3d(%5d/%5d) ', sel_STA, DataSentSucceessed(sel_STA), DataSent(sel_STA));
                    end
                end
            end
            fprintf(fid, '\r\n');
        end
        fprintf(fid, '\r\nRecieved bits distribution:\r\n');
        for i=1:Num_AP
            if (STA(i, 3) ~= -1)
                fprintf(fid, 'AP %3d(%11d) ', i, statics_rv_bits(i));
            end
        end
        fprintf(fid, '\r\n');
        for i=1:max([STA_Info(1:Num_AP).num_ass])
            for j=1:Num_AP
                if (STA(j, 3) ~= -1)
                    if (i > STA_Info(j).num_ass)
                        fprintf(fid, '                    ');
                        continue;
                    else
                        sel_STA = STA_Info(j).associated_STA(i);
                        fprintf(fid, 'STA%3d(%11d) ', sel_STA, statics_rv_bits(sel_STA));
                    end
                end
            end
            fprintf(fid, '\r\n');
        end
        fprintf(fid, '\r\nThroughputs (Mbps) of each STA:\r\n');
        separate_statics = statics_rv_bits / maxSimTime / 1e6;
        EachAP_observation = zeros(1, Num_AP);
        for i=1:Num_AP
            if (STA(i, 3) ~= -1)
                fprintf(fid, 'AP %3d: %9.3f ', i, separate_statics(i));
                EachAP_observation(i) = separate_statics(i) + sum(separate_statics(STA_Info(i).associated_STA));
            end
        end
        fprintf(fid, '\r\n');
        for i=1:max([STA_Info(1:Num_AP).num_ass])
            for j=1:Num_AP
                if (STA(j, 3) ~= -1)
                    if (i > STA_Info(j).num_ass)
                        fprintf(fid, '                  ');
                        continue;
                    else
                        sel_STA = STA_Info(j).associated_STA(i);
                        fprintf(fid, 'STA%3d: %9.3f ', sel_STA, separate_statics(sel_STA));
                    end
                end
            end
            fprintf(fid, '\r\n');
        end
        for i=1:Num_AP
            if (STA(i, 3) ~= -1)
                fprintf(fid, 'BSS%3d: %9.3f ', i, EachAP_observation(i));
            end
        end
        edge_throughput = [];
        center_throughput = [];
        edge_CoMP_throughput = []; %for CoMP mode
        num_center = 0;
        num_edge = 0;
        num_edgecomp = 0;
        for i=1:Num_AP
            for j=1:length(STA_Info(i).associated_STA)
                sel_STA = STA_Info(i).associated_STA(j);
                if ismember(sel_STA, STA_Info(i).center_STA)
                    center_throughput = [center_throughput, separate_statics(sel_STA)];
                    num_center = num_center + 1;
                else
                    if (scheme == 2)
                        if ~isempty(STA_Info(i).CoMP_coordinator)
                            if ismember(sel_STA, STA_Info(STA_Info(i).CoMP_coordinator).cover_STA)
                                edge_CoMP_throughput = [edge_CoMP_throughput, separate_statics(sel_STA)];
                                num_edgecomp = num_edgecomp + 1;
                            else
                                edge_throughput = [edge_throughput, separate_statics(sel_STA)];
                                num_edge = num_edge + 1;
                            end
                        else
                            edge_throughput = [edge_throughput, separate_statics(sel_STA)];
                            num_edge = num_edge + 1;
                        end
                    else
                        edge_throughput = [edge_throughput, separate_statics(sel_STA)];
                        num_edge = num_edge + 1;
                    end
                end
            end
        end
        if (scheme == 2)
            fprintf(fid, '\r\n Center_throughput = %9.3f \r STA Average = %9.3f', sum(center_throughput), sum(center_throughput)/num_center);
            fprintf(fid, '\r\n Edge_CoMP_throughput = %9.3f \r STA Average = %9.3f', sum(edge_CoMP_throughput), sum(edge_CoMP_throughput)/num_edgecomp);
            fprintf(fid, '\r\n Edge_throughput = %9.3f \r STA Average = %9.3f', sum(edge_throughput), sum(edge_throughput)/num_edge);
        else
            fprintf(fid, '\r\n Center_throughput = %9.3f \r STA Average = %9.3f', sum(center_throughput), sum(center_throughput)/num_center);
            fprintf(fid, '\r\n Edge_throughput = %9.3f \r STA Average = %9.3f', sum(edge_throughput), sum(edge_throughput)/num_edge);
        end
        fprintf(fid, '\r\n=> Throughputs of the Network topology: %f Mbps\r\n\r\n', sum(EachAP_observation));
        disp (['Throughputs : ', num2str(sum(EachAP_observation))]);
        average_Troughput(ind0, ind1) = sum(EachAP_observation);
        fclose(fid);
    end
end
if (max(run_quantity) > 1)
    average_Troughput = sum(average_Troughput)/max(run_quantity);
end
h = figure(3);
% set(h, 'Visible', 'off')
clf(h)
hold on
if (envAPmodel == 2) 
    if (envUsermodel == 2)
        title(['£l_A_P = ', num2str(tho(1)), ' (1/m^2) and £l_u_s_e_r = ', num2str(tho(2)), ' (1/m^2)'])
    elseif (envUsermodel == 0)
        title({['£l_A_P = ', num2str(tho(1)), ' in ', num2str(Area(1)), 'x', num2str(Area(2)), ' (m^2)'],...
            ['£l_u_s_e_r = ', num2str(tho(2)), ' (1/m^2) in each AP cover area']})
    end
elseif (envAPmodel == -1) 
    if (envUsermodel == 2)
        title(['£l_u_s_e_r = ', num2str(tho(2)), ' (1/m^2)'])
    elseif (envUsermodel == 0)
        title(['£l_u_s_e_r = ', num2str(tho(2)), ' (1/m^2) in each AP cover area'])
    end
end
xlabel('Scheme')
ylabel('Average Throughput (Mbps)')
bar(average_Troughput, 'BarWidth', 0.2)
set(gca,'XTick',1:scheme);
set(gca,'XTickLabel', {'DL MU-MIMO', 'CS/CB' ,'JP'});
hold off
% ====================== Block 4 end ======================
disp(['Finish simulator @ ' num2str(datestr(now))]);
