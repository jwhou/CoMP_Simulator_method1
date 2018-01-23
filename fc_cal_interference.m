%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: Jing-Wen Hou
% Email     : aasss369@gmail.com
% Version   : 0.9
% Date      : 2017/12/8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pr] = fc_cal_interference(ind_sym,tx,rv,N_tx,N_rx,Nsym,freq,...
    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range,tempindex_AI )

global STA STA_Info;

tx_x_pos=STA(tx, 1);
tx_y_pos=STA(tx, 2);
rx_x_pos=STA(rv, 1);
rx_y_pos=STA(rv, 2);
WiFi_standard='80211ac';
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

% generating the channel coefficients in time and frequency domain
h_time=fc_generate_h(Nsubcarrier,N_tx,N_rx,channel_model,AoD_in_floor,...
    Samping_rate_expansion_factor,PL_dB_in_count,ant_gain,Ncluster,angular);

h_temp=zeros(size(h_time));
for ind=1:1:N_tx*N_rx
    h_temp(ind,:)=fft(h_time(ind,:),Nsubcarrier)/sqrt(Nsubcarrier);
end

H_new=zeros(N_rx,N_tx,Nsubcarrier);
H_freq=zeros(N_rx,N_tx,Nsubcarrier);
t5 = zeros(1,Nsubcarrier);
pr = zeros(1,Nsubcarrier);
for index_f=1:1:Nsubcarrier
    H_freq(:,:,index_f)=reshape(h_temp(:,index_f),N_rx,N_tx);
    if N_tx == 1
        t5(index_f) = trace(H_freq(:,:,index_f)'*H_freq(:,:,index_f)); %SNR
    else
        H_new(:,:,index_f)=H_freq(:,:,index_f)*STA_Info(tx).Precoder_record{ind_sym}{index_f};
        t5(index_f) = trace(H_new(:,:,index_f)'*H_new(:,:,index_f)); %SNR
    end
    %pr(index_f) = (Pt*t5(index_f))/(N_tx*N_rx);
    pr(index_f) = (Pt*abs(t5(index_f)))/(N_rx);
end

end


