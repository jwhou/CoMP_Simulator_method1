% Given the simulated room and the TX/RX position, this function returns
% the received signal power and interference

function [pr,snr_dB,final_PER] = fc_recv_phy(order,MCS,tx,rv,N_tx,N_rx,freq,...
    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range, pkt_size_in_bits,tempindex_AI, pkt_type)


global STA STA_Info;
global CoMP_Controller;
global control_intf_skip_Debug

tx_x_pos=STA(tx, 1);
tx_y_pos=STA(tx, 2);
rx_x_pos=STA(rv, 1);
rx_y_pos=STA(rv, 2);
WiFi_standard='80211ac';
L = pkt_size_in_bits;
MCS_int=MCS-1; %decrease 1 for cope with simulator
Nsym = num_symbol(pkt_type);  % number of OFDM symbols per packet
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
%隨著天線數增加
Ant_type=3;
%1:2.4GHz Simulated Antenna Pattern
%2:2.4GHz Measured Antenna Pattern
%3:5GHz Antenna Pattern
Npkt=1;
%Npkt=30;
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

% ============== Retrieving AoDs and Traveling distances ============ %
% [AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
%     channel_type,collision_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range);
[AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
    reflection_times,wall_mix,wall_index,HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
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
angular=fc_cal_angular_spread(wall_mix,wall_index,HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,...
    channel_model,AoD_in_count,wall_in_count,Ncluster,reflection_times,collision_in_count);
% ======================== End of getting AS ======================== %

if (strcmp(pkt_type, 'Data') == 1)
    for ind_sym=1:1:Nsym
        H_new = zeros(N_tx,N_tx,Nsubcarrier);
        for index_f=1:1:Nsubcarrier
            H_new(:,:,index_f) = STA_Info(tx).Channel_record{ind_sym}(:,:,index_f)*STA_Info(tx).Precoder_record{ind_sym}(:,:,index_f);
            H_user = H_new((order-1)*N_rx+1:1:(order-1)*N_rx+N_rx,(order-1)*N_rx+1:1:(order-1)*N_rx+N_rx,index_f);
            H_receive = H_user*H_user';
            H_transmit = H_user'*H_user;
            t1 = t1 + H_receive; %receive
            t2 = t2 + H_transmit; %transmit
            t3 = t3 + trace(H_receive); %receive
            t4 = t4 + trace(H_transmit); %transmit
            t5 = t5 + trace(H_transmit); %SNR
        end
    end
else
    for ind_sym=1:1:Nsym
        % generating the channel coefficients in time and frequency domain
        h_time=fc_generate_h(Nsubcarrier,N_tx,N_rx,channel_model,AoD_in_floor,...
            Samping_rate_expansion_factor,PL_dB_in_count,ant_gain,Ncluster,angular);
        
        h_temp=zeros(size(h_time));
        for ind=1:1:N_tx*N_rx
            h_temp(ind,:)=fft(h_time(ind,:),Nsubcarrier)/sqrt(Nsubcarrier);
        end
        
        H_freq=zeros(N_rx,N_tx,Nsubcarrier);
        for index_f=1:1:Nsubcarrier
            H_freq(:,:,index_f)=reshape(h_temp(:,index_f),N_rx,N_tx);
            t1 = t1 + H_freq(:,:,index_f)*H_freq(:,:,index_f)'; %receive
            t2 = t2 + H_freq(:,:,index_f)'*H_freq(:,:,index_f); %transmit
            t3 = t3 + trace(H_freq(:,:,index_f)*H_freq(:,:,index_f)'); %receive
            t4 = t4 + trace(H_freq(:,:,index_f)'*H_freq(:,:,index_f)); %transmit
            t5 = t5 + trace(H_freq(:,:,index_f)'*H_freq(:,:,index_f)); %SNR
        end
    end
end

pr = 0;
I = find(STA(:, 5)>0);
for i=1:length(I)
    tx1 = I(i);
    if ~isempty(STA_Info(tx).CoMP_coordinator)
        if (ismember(tx1, STA_Info(tx).CoMP_coordinator))
            if(ismember(tx1, STA_Info(rv).cover_STA))
                continue;
            end
        end
    end
    if tx1 == rv, continue; end
    if any(tx1 == tx), continue; end
    if (STA(tx1, 8) == 2) % tx1 transmits Data pkt
        if (STA(tx1, 3) == 0) % tx1 is an AP
            temp_N_tx = length(STA_Info(tx1).Precoder_record{1}(:,1,1));
            pr = pr + fc_cal_interference(tx1,rv,temp_N_tx,N_rx,Nsym,freq,...
                Nsubcarrier,STA(tx1,5),channel_model,Bandwidth,Thermal_noise,...
                tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
        else % tx1 is a non-AP STA
            %Donot have uplink
        end
    elseif (control_intf_skip_Debug == 0) % tx transmits RTS/CTS/ACK pkt, do not care # of antennas
        pr = pr + fc_cal_interference(tx1,rv,1,N_rx,Nsym,freq,...
            Nsubcarrier,STA(tx1,5),channel_model,Bandwidth,Thermal_noise,...
            tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI);
    end
end
t5 = t5/Nsym;
%SNR=(1/Nsubcarrier)*Pt*abs(t5)/(Nt*Nr*Thermal_noise);  %abs is needed?
%t5 = (1/Nsubcarrier)*Pt*t5*(1/(N_tx*N_rx))/(Thermal_noise+pr);
t5 = (1/(Nsubcarrier*N_rx))*Pt*t5*(1/(N_tx))/(Thermal_noise+pr);
snr_dB=10*log10(t5);
gamma = t5;
R_corr = abs(t1)/(abs((t3))/N_rx); % receive spatial correlation matrix
S_corr = abs(t2)/(abs((t4))/(N_rx)); % transmit spatial correlation matrix
eig_Value = eig(R_corr).';
[modulation,code_rate] = fix_fc_return_modulation_code_rate(WiFi_standard,MCS_int);
BER = fc_MIMO_CPEP(WiFi_standard,modulation,code_rate,N_rx,N_rx,gamma,eig_Value,R_corr,S_corr);
if BER > 1
    warning ('BER > 1');
    BER = 1;
end
PER = 1-(1-BER)^L;
final_PER = PER;
end