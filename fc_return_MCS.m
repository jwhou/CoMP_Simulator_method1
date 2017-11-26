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

function [output,snr_dB] = fc_return_MCS(target_PER,tx,rv,N_tx,N_rx,freq,...
    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range, pkt_size_in_bits,tempindex_AI)

% function output = fc_return_MCS(target_PER,N_tx,N_rx,...
%                     Nsubcarrier,Bandwidth,Thermal_noise,...
%                     pkt_size_in_bits)

                                                
global STA;

for num_rv=1:length(rv)
    tx_x_pos=STA(tx, 1);
    tx_y_pos=STA(tx, 2);
    rx_x_pos=STA(rv(1,num_rv), 1);
    rx_y_pos=STA(rv(1,num_rv), 2);
    signal_power=0;
    WiFi_standard='80211ac';
    MCS_int=5;
    tempindex_1=11;
    tempindex_2=11;
%     tempindex_3=11;
%     tempindex_4=11;
%     AI=[tempindex_AI(tempindex_1) tempindex_AI(tempindex_2) tempindex_AI(tempindex_3) tempindex_AI(tempindex_4)];
    AI=[tempindex_AI(tempindex_1) tempindex_AI(tempindex_2)];
    %隨著天線數增加
    Ant_type=3;
    %1:2.4GHz Simulated Antenna Pattern
    %2:2.4GHz Measured Antenna Pattern
    %3:5GHz Antenna Pattern
    Npkt=30;
    %training packets
    PER = zeros(1,8);
    [PER(MCS_int),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_int,N_rx,Pt,freq,Bandwidth,...
    Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
    HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
    
% ===== MCS mode selection algorithm =====
    if PER(MCS_int) < target_PER
        MCS_temp = 7;
        [PER(MCS_temp),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_temp,N_rx,Pt,freq,Bandwidth,...
         Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
         HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
    
        if PER(MCS_temp) < target_PER
            MCS_temp = 8;
            [PER(MCS_temp),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_temp,N_rx,Pt,freq,Bandwidth,...
            Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
            HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
            if PER(MCS_temp) < target_PER
                final_MCS = 8;
            else
                final_MCS = 7;
            end
        else % PER(7) > target_PER
            MCS_temp = 6;
            [PER(MCS_temp),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_temp,N_rx,Pt,freq,Bandwidth,...
            Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
            HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
            if PER(MCS_temp) < target_PER
                final_MCS = 6;
            else % PER(6) > target_PER
                final_MCS = 5;
            end
        end
    else % PER(5) > target_PER
    MCS_temp = 3;
    [PER(MCS_temp),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_temp,N_rx,Pt,freq,Bandwidth,...
            Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
            HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
        if PER(MCS_temp) < target_PER
            MCS_temp = 4;
            [PER(MCS_temp),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_temp,N_rx,Pt,freq,Bandwidth,...
            Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
            HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
            if PER(MCS_temp) < target_PER
                final_MCS = 4;
            else % PER(4) > target_PER
                final_MCS = 3;
            end
        else % PER(3) > target_PER
            MCS_temp = 2;
            [PER(MCS_temp),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_temp,N_rx,Pt,freq,Bandwidth,...
            Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
            HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range);
            if PER(2) < target_PER
                final_MCS = 2;
            else % PER(2) > target_PER
                final_MCS = 1;
            end
        end     
    end
    output(1,num_rv) = final_MCS;
 end
end