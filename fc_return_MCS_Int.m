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

function [MCS_index,snr_dB] = fc_return_MCS_Int(target_PER,tx,rv,N_tx,N_rx,freq,...
    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range, L,tempindex_AI)

global STA STA_Info;
global per_order;

tx_x_pos=STA(tx, 1);
tx_y_pos=STA(tx, 2);
rx_x_pos=[STA(rv, 1)'];
rx_y_pos=[STA(rv, 2)'];
N_tx = N_rx*length(rv);
signal_power=0;
WiFi_standard='80211ac';
Nsym=10^3;
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

t1 = zeros(N_rx,N_rx); %receive correlation
t2 = zeros(N_rx,N_rx); %transmit correlation
t3 = 0; %receive correlation normalize
t4 = 0; %transmit correlation normalize
t5 = 0; %SNR

for k=1:length(rv)
    T1{k} = t1;
    T2{k} = t2;
    T3{k} = t3;
    T4{k} = t4;
    T5{k} = t5;
end
    
for ind_sym=1:1:Nsym
    for k=1:1:length(rv)
        % ============== Retrieving AoDs and Traveling distances ============ %
        % [AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
        %     channel_type,collision_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range);
        [AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
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
        if Ncluster == 0
            error('Error in fc_give_MIMO_SM_PER().m, Ncluster==0')
        end
        % ----- Ncluster = 0? ----->
        
        Ant_gain_in_count=zeros(1,Ncluster);
        AoD_in_count=zeros(1,Ncluster);
        AoD_in_floor=zeros(1,Ncluster);
        Distance_in_count = distance(1:1:Ncluster);
        
        for q=1:1:Ncluster
            AoD_in_count(1,q)=AoAAoD(1,index(1,q));
            temp_AoD = floor(AoAAoD(1,index(1,q)));
            AoD_in_floor(1,q) = temp_AoD;
            AoD_index=temp_AoD+1;
            Ant_gain_in_count(1,q) = ant_gain(1,AoD_index);
        end
        %PL_dB_in_count=fc_path_loss(channel_type,Distance_in_count,freq,LOS,Npkt*Nsym); % Path loss effect of each cluster in dB
        PL_dB_in_count=fix_fc_path_loss(channel_model,Distance_in_count,freq,LOS,Npkt*Nsym,Ncluster); % Path loss effect of each cluster in dB
        % =========== End of getting the imformation of in count ray ======== %
        
        % ================ Get Angular Spread for each main path ============ %
        angular=fc_cal_angular_spread(wall_mix,wall_index,HOV,tx_x_pos,tx_y_pos,rx_x_pos(k),rx_y_pos(k),...
            channel_model,AoD_in_count,wall_in_count,Ncluster,reflection_times,collision_in_count);
        % ======================== End of getting AS ======================== %
        
        
        % generating the channel coefficients in time and frequency domain
        h_time=fc_generate_h(Nsubcarrier,N_tx,N_rx,channel_model,AoD_in_floor,...
            Samping_rate_expansion_factor,PL_dB_in_count,ant_gain,Ncluster,angular);
        
        %h_temp=zeros(size(h_time));
        for ind=1:1:N_tx*N_rx
            col=ind+((k-1)*N_tx*N_rx);
            h_temp(col,:)=fft(h_time(ind,:),Nsubcarrier)/sqrt(Nsubcarrier);
        end
    end
    H_freq=zeros(N_rx*length(rv),N_tx,Nsubcarrier);
    M = zeros(N_tx, N_rx*length(rv), Nsubcarrier);
    for index_f=1:1:Nsubcarrier
        H_freq(:,:,index_f)=reshape(h_temp(:,index_f),N_rx*length(rv),N_tx);
        for j=1:length(rv)
            all_stream = 1:1:N_tx;
            own_stream = (j-1)*N_rx+1:1:(j-1)*N_rx+N_rx;
            nown_stream = all_stream(~ismember(all_stream, own_stream));
            M(:,own_stream,index_f) = null(H_freq(nown_stream,:,index_f));
        end
%        M(:,:,index_f)=[ null(H_freq(3:6,:,index_f)) null(H_freq([1 2 5 6],:,index_f)) null(H_freq(1:4,:,index_f))];
        H_new(:,:,index_f)=H_freq(:,:,index_f)*M(:,:,index_f);
        for j=1:length(rv)
            H_user = H_new((j-1)*N_rx+1:1:(j-1)*N_rx+N_rx,(j-1)*N_rx+1:1:(j-1)*N_rx+N_rx,index_f);
            T1{j} = T1{j} + H_user*H_user'; %receive
            T2{j} = T2{j} + H_user'*H_user; %transmit
            T3{j} = T3{j} + trace(H_user*H_user'); %receive
            T4{j} = T4{j} + trace(H_user'*H_user); %transmit
            T5{j} = T5{j} + trace(H_user'*H_user); %SNR
        end
    end
    STA_Info(tx).Precoder_record{ind_sym} = M;
end

for k=1:1:length(rv)
    pr = 0;
    I = find(STA(:, 5)>0);
    for i=1:length(I)
        tx1 = I(i);
        if tx1 == rv, continue; end
        if any(tx1 == tx), continue; end
        if (STA(tx1, 8) == 2) % tx1 transmits Data pkt
            if (STA(tx1, 3) == 0) % tx1 is an AP
                temp_N_tx = length(STA_Info(tx1).Precoder_record{1}(:,1,1));
                pr = pr + fc_cal_interference(tx1,rv(k),temp_N_tx,N_rx,freq,...
                    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
                    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
            else % tx1 is a non-AP STA
                %Donot have uplink
            end
        else % tx transmits RTS/CTS/ACK pkt, do not care # of antennas
            pr = pr + fc_cal_interference(tx1,rv(k),1,N_rx,freq,...
                Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
                tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
        end
    end
    MCS_init = 7;
    T5{k} = T5{k}/Nsym;
    %SNR=(1/Nsubcarrier)*Pt*abs(t5)/(Nt*Nr*Thermal_noise);  %abs is needed?
    T5{k} = (1/Nsubcarrier)*Pt*T5{k}/(N_tx*N_rx*Thermal_noise)+pr;
    snr_dB(k)=10*log10(T5{k});
    gamma = T5{k};
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