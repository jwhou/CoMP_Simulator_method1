% Given the simulated room and the TX/RX position, this function returns
% the received signal power and interference

function [snr_dB,final_PER] = fc_recv_phy(MCS,tx,rv,N_tx,N_rx,freq,...
    Nsubcarrier,Pt,channel_model,Bandwidth,Thermal_noise,...
    tx_ant_gain,reflection_times,wall_mix,wall_index,HOV,room_range, pkt_size_in_bits,tempindex_AI, pkt_type)


    global STA;

    if (STA(tx, 5) <= 0)
        error('send_PHY: transmission power is zero');
    end

     %interference_power=0;

    for num_rv=1:length(rv)
        tx_x_pos=STA(tx, 1);
        tx_y_pos=STA(tx, 2);
        rx_x_pos=STA(rv(1,num_rv), 1);
        rx_y_pos=STA(rv(1,num_rv), 2);
        signal_power=0;
        WiFi_standard='80211ac';
        MCS_int=MCS;
        tempindex_1=11;
        tempindex_2=11;
%         tempindex_3=11;
        AI=[tempindex_AI(tempindex_1) tempindex_AI(tempindex_2)];
        %隨著天線數增加
        Ant_type=3;
        %1:2.4GHz Simulated Antenna Pattern
        %2:2.4GHz Measured Antenna Pattern
        %3:5GHz Antenna Pattern
        Npkt=1;
        %Npkt=30;
        %training packets
        PER = zeros(1,8);
        [PER(MCS_int),snr_dB(1,num_rv)] = fix_fc_give_MIMO_SM_NtNr_recv_phy(WiFi_standard,MCS_int,N_rx,Pt,freq,Bandwidth,...
        Npkt,pkt_size_in_bits,AI,Ant_type,channel_model,Nsubcarrier,reflection_times,wall_mix,wall_index,...
        HOV,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,room_range,pkt_type);
        final_PER = PER(MCS_int);

    end

end