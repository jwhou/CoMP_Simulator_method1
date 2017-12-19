% Given TX, RX, target PER, this function returns the suggested MCS to
% guaranteed that PER < target PER
% The MCS mode selection algorithm can be found on Sony's research notebook
% Vol. 17, pp. 10.
% output: MCS index (1, 2, 3, ..., 8)
%         for example, "MCS index = 1" means "MCS mode 0"
%
% input:
%   target_PER: the MCS mode is chosen to satisfy this target PER
%   tx
%   rv
%   N_tx: number of TX antennas
%   N_rx: number of RX antennas
%   freq: center frequency (Hz)
%   Nsubcarrier: number of subcarriers
%   Pt: trnasmisiion power (Watt)
%   channel_model: WiFi channel mode A, B, C, D, E or F
%   Bandwidth
%   Thermal_noise
%   tx_ant_gain: transmite antenna gain (RF smart antenna, not BF or RF beamforming)
%   reflection_times: maximum order of reflection
%   wall_mix,wall_index,HOV,room_range
%   pkt_size_in_bits

function [MCS_index,snr_dB] = fc_return_MCS_Int(t,target_PER,tx,rv,N_tx,N_rx,freq,...
    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range, L,tempindex_AI,pkt_type)

global STA STA_Info;
global per_order;
global GI SymbolTime;
global control_intf_skip_Debug;
global tx_interval interference_queue;
global num_msdu;

tx_x_pos=STA(tx, 1);
tx_y_pos=STA(tx, 2);
rx_x_pos=[STA(rv, 1)'];
rx_y_pos=[STA(rv, 2)'];
N_tx = N_rx*length(rv);
Nsym = num_symbol(pkt_type,L);
L = L*num_msdu;
signal_power=0;
WiFi_standard='80211ac';
time = tx_interval(tx).end;
tempindex_1=11;
tempindex_2=11;
tempindex_3=11;
tempindex_4=11;
tempindex_5=11;
tempindex_6=11;
tempindex_7=11;
tempindex_8=11;
temp_AI=[tempindex_AI(tempindex_1) tempindex_AI(tempindex_2) tempindex_AI(tempindex_3) tempindex_AI(tempindex_4) ...
    tempindex_AI(tempindex_5) tempindex_AI(tempindex_6) tempindex_AI(tempindex_7) tempindex_AI(tempindex_8)];
AI = temp_AI(1:N_tx);
Ant_type=3;
%1:2.4GHz Simulated Antenna Pattern
%2:2.4GHz Measured Antenna Pattern
%3:5GHz Antenna Pattern
Npkt=30;
%training packets

% <----- Paramaters for the matched filter -----
Tow_max = 400;  % Maximum value of delay in ns
Dens = 10;    % Definition of the baseband signals (resolution: upsample factor)
Roll_factor = 0.3;    % Roll-Off factor of square root-raised consine filter
SRRC_filter = rcosine(Bandwidth,Bandwidth*Dens,'fir/sqrt',Roll_factor);
[~,offset]=max(SRRC_filter);
% ------------------------------------------------->

% ============================================================== %
% The gain from the smart antenna design (at TX) in normal scale %
% ============================================================== %
temp=fc_AI_pattern(AI(1),Ant_type);
ant_gain=zeros(N_tx,length(temp.gain));
for ind_ant=1:1:N_tx
    pattern=fc_AI_pattern(AI(ind_ant),Ant_type);
    ant_gain(ind_ant,:)=pattern.gain;
end
% ============================================================== %

% ===== Environment parameters ===== %
Temperature=298;          % Temperature in k (273+25)
Thermal_noise = 1*1.38*10^(-23)*Temperature*Bandwidth;
%     Bandwidth=40*10^6;           % Bandwidth in Hz
Samping_rate_expansion_factor=fc_give_channel_sampling_rate_expantion_factor(Bandwidth);
% ===== Environment parameters ===== %

for k=1:1:length(rv)
    % ============== Retrieving AoDs and Traveling distances ============ %
    % [AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
    %     channel_type,collision_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range);
    [AoAAoD,Ncluster{k},distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
        reflection_times,wall_mix,wall_index,HOV,tx_x_pos,tx_y_pos,rx_x_pos(k),rx_y_pos(k),room_range);
    % ======================= End of retrieve =========================== %
    
    % =================================================================== %
    % Calculate in count part when there are a lot of rays in the system  %
    % For example, model type B only need 2 cluster. So we only want to   %
    % calculate 2 cluster with their distance, antenna gain, path loss and%
    % angle of departure (AoD).                                           %
    % ====================================================================%
    LOS = 0;
    
    % <----- Ncluster = 0? -----
    if Ncluster{k} == 0
        error('Error in fc_give_MIMO_SM_PER().m, Ncluster==0')
    end
    % ----- Ncluster = 0? ----->
    
    Ant_gain_in_count=zeros(1,Ncluster{k});
    AoD_in_count=zeros(1,Ncluster{k});
    AoD_in_floor{k}=zeros(1,Ncluster{k});
    Distance_in_count = distance(1:1:Ncluster{k});
    
    for q=1:1:Ncluster{k}
        AoD_in_count(1,q)=AoAAoD(1,index(1,q));
        temp_AoD = floor(AoAAoD(1,index(1,q)));
        AoD_in_floor{k}(1,q) = temp_AoD;
        AoD_index=temp_AoD+1;
        Ant_gain_in_count(1,q) = ant_gain(1,AoD_index);
    end
    %PL_dB_in_count=fc_path_loss(channel_type,Distance_in_count,freq,LOS,Npkt*Nsym); % Path loss effect of each cluster in dB
    PL_dB_in_count{k}=fix_fc_path_loss(channel_model,Distance_in_count,freq,LOS,Npkt*Nsym,Ncluster{k}); % Path loss effect of each cluster in dB
    % =========== End of getting the imformation of in count ray ======== %
    
    % ================ Get Angular Spread for each main path ============ %
    angular{k}=fc_cal_angular_spread(wall_mix,wall_index,HOV,tx_x_pos,tx_y_pos,rx_x_pos(k),rx_y_pos(k),...
        channel_model,AoD_in_count,wall_in_count,Ncluster{k},reflection_times,collision_in_count);
    % ======================== End of getting AS ======================== %
end

t1 = zeros(N_rx,N_rx); %receive correlation
t2 = zeros(N_rx,N_rx); %transmit correlation
t3 = 0; %receive correlation normalize
t4 = 0; %transmit correlation normalize
t5 = 0; %Singal

for k=1:length(rv)
    T1{k} = t1;
    T2{k} = t2;
    T3{k} = t3;
    T4{k} = t4;
    T5{k} = [];
end
for ind_sym=1:1:Nsym
    for index_f=1:1:Nsubcarrier
        T5{k}{ind_sym}{index_f} = t5;
    end
end


SINR = zeros(length(rv),Nsym,Nsubcarrier);
for k=1:length(rv), pr{k}=zeros(Nsym,Nsubcarrier); end

for ind_sym=1:1:Nsym
    h_temp=zeros(N_tx*N_rx,Nsubcarrier);
    for k=1:1:length(rv)
        % generating the channel coefficients in time and frequency domain
        h_time=fc_generate_h(Nsubcarrier,N_tx,N_rx,channel_model,AoD_in_floor{k},...
            Samping_rate_expansion_factor,PL_dB_in_count{k},ant_gain,Ncluster{k},angular{k});
        for ind=1:1:N_tx*N_rx
            col=ind+((k-1)*N_tx*N_rx);
            h_temp(col,:)=fft(h_time(ind,:),Nsubcarrier)/sqrt(Nsubcarrier);
        end
    end
    H_freq = zeros(N_rx*length(rv),N_tx,Nsubcarrier);
    H_new =zeros(N_rx*length(rv),N_tx,Nsubcarrier);
    M = zeros(N_tx, N_rx*length(rv), Nsubcarrier);
    
    for index_f=1:1:Nsubcarrier
        H_freq(:,:,index_f)=reshape(h_temp(:,index_f),N_rx*length(rv),N_tx);
        for j=1:length(rv)
            all_stream = 1:1:N_tx;
            own_stream = (j-1)*N_rx+1:1:(j-1)*N_rx+N_rx;
            nown_stream = all_stream(~ismember(all_stream, own_stream));
            M(:,own_stream,index_f) = null(H_freq(nown_stream,:,index_f));
        end
        STA_Info(tx).Channel_record{ind_sym}{index_f} = H_freq(:,:,index_f);
        H_new(:,:,index_f)=H_freq(:,:,index_f)*M(:,:,index_f);
        STA_Info(tx).Precoder_record{ind_sym}{index_f} = M(:,:,index_f);
        
        for j=1:length(rv)
            H_user = H_new((j-1)*N_rx+1:1:(j-1)*N_rx+N_rx,(j-1)*N_rx+1:1:(j-1)*N_rx+N_rx,index_f);
            H_receive = H_user*H_user';
            H_transmit = H_user'*H_user;
            T1{j} = T1{j} + H_receive; %receive
            T2{j} = T2{j} + H_transmit; %transmit
            T3{j} = T3{j} + trace(H_receive); %receive
            T4{j} = T4{j} + trace(H_transmit); %transmit
            T5{j}{ind_sym}{index_f} = trace(H_transmit); %SNR
        end
    end
    for k=1:length(rv)
        for j=1:length(interference_queue(tx).list)
            tx1 = interference_queue(tx).list(j);
            if(time > interference_queue(tx).start(j) && time <= interference_queue(tx).end(j))
                if ~isempty(STA_Info(tx).CoMP_coordinator)
                    if (ismember(tx1, STA_Info(tx).CoMP_coordinator))
                        if(ismember(tx1, STA_Info(rv(k)).cover_STA))
                            continue;
                        end
                    end
                end
                if tx1 == rv, continue; end
                if any(tx1 == tx), continue; end
                if (interference_queue(tx).pkt_type(j) == 2) % tx1 transmits Data pkt
                    if (STA(tx1, 3) == 0) % tx1 is an AP
                        temp_N_tx = length(STA_Info(tx1).Precoder_record{1}(:,1,1));
                        pr{k}(ind_sym,:) = pr{k}(ind_sym,:) + fc_cal_interference(ind_sym,tx1,rv(k),temp_N_tx,N_rx,Nsym,freq,...
                            Nsubcarrier,STA(tx1,5),channel_model,Bandwidth,Thermal_noise,...
                            tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
                    else % tx1 is a non-AP STA
                        pr{k}(ind_sym,:) = pr{k}(ind_sym,:) + fc_cal_interference(ind_sym,tx1,rv(k),1,N_rx,Nsym,freq,...
                            Nsubcarrier,STA(tx1,5),channel_model,Bandwidth,Thermal_noise,...
                            tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
                    end
                elseif (control_intf_skip_Debug == 0) % tx transmits RTS/CTS/ACK pkt, do not care # of antennas
                    pr{k}(ind_sym,:) = pr{k}(ind_sym,:) + fc_cal_interference(ind_sym,tx1,rv(k),1,N_rx,Nsym,freq,...
                        Nsubcarrier,STA(tx1,5),channel_model,Bandwidth,Thermal_noise,...
                        tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
                end
            end
        end
    end
    for k=1:length(rv)
        for index_f=1:1:Nsubcarrier
            SINR(k,ind_sym,index_f) = (1/(N_tx*N_rx))*Pt*T5{k}{ind_sym}{index_f}/(Thermal_noise+pr{k}(ind_sym,index_f));
        end
    end
    time = time - SymbolTime(GI);
end
SINR_per_f = zeros(length(rv),ind_sym);
final_SINR = zeros(1,length(rv));
for k=1:length(rv)
    for ind_sym=1:1:Nsym
        SINR_per_f(k,ind_sym) = sum(SINR(k,ind_sym,:))/Nsubcarrier;
    end
    final_SINR(k) = sum(SINR_per_f(k,:))/Nsym;
end

snr_dB = zeros(1,length(rv));
for k=1:1:length(rv)
    MCS_init = 7;
    snr_dB(k)=10*log10(final_SINR(k));
    gamma = final_SINR(k);
    R_corr = abs(T1{k})/(abs((T3{k}))/N_rx); % receive spatial correlation matrix
    S_corr = abs(T2{k})/(abs((T4{k}))/(1/length(rv)*N_tx)); % transmit spatial correlation matrix
    eig_Value = eig(R_corr).';
    [modulation,code_rate] = fix_fc_return_modulation_code_rate(WiFi_standard,MCS_init);
    BER = fc_MIMO_CPEP(WiFi_standard,modulation,code_rate,(1/length(rv))*N_tx,N_rx,gamma,eig_Value,R_corr,S_corr);
    if BER > 1
        warning ('BER > 1');
        BER = 1;
    end
    PER = 1-(1-BER)^L;
    MCS_index(k) = MCS_init;
    while PER > per_order
        %display('find the best beam');
        if MCS_init > 0
            MCS_init = MCS_init - 1;
            [modulation,code_rate] = fix_fc_return_modulation_code_rate(WiFi_standard,MCS_init);
            BER = fc_MIMO_CPEP(WiFi_standard,modulation,code_rate,(1/length(rv))*N_tx,N_rx,gamma,eig_Value,R_corr,S_corr);
            if BER > 1
                warning ('BER > 1');
                BER = 1;
            end
            PER = 1-(1-BER)^L;
            MCS_index(k) = MCS_init;
        else
            break;
        end
    end
    MCS_index(k) = MCS_index(k) + 1;   %to cope with our simulator MCS_index  8->7
end

end