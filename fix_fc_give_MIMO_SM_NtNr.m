%{
Author:      Sony, Ju-Chia Wang
Date:        2017/10/20
Input:       WiFi_standard: it is a "string", and it must be '80211n' or '80211ac';
             MCS_mode: 0~9  if WiFi_standard == 80211ac
                       0~31 if WiFi_standard == 80211n
             Nt: number of transmit antennas
             Pt: transmit power in Watt
             freq: center carrier frequency (Hz)
             Bandwidth (Hz)
             Npkt: number of training packets
             L: pkt size in bits
Output:      
Description: % In this function, we follow Matthew McKay and Robert Heath's work to
             % analyze the approximated BER for 2x2 MIMO spatial multiplexing.
             % This function returns the PER and SNR in dB scale
             % Matthew. R. McKay, I. B. Collings, A. Forenza, R. W. Health, 
             % "Multiplexing/beamforming switching for coded MIMO in spatially correlated 
             % channels based on closed-form BER approximations," IEEE Trans. on 
             % Vehicular Technology, vol. 56, no. 5, Sept. 2007 
            
             % providing the codeword pairwise error probability (C-PEP) for spatial multiplexing
             % this function follows the equation (46) in Matthew McKay's work, and is
             % for the case of Nt x Nr MIMO

%}

function [PER,snr_dB] = fix_fc_give_MIMO_SM_NtNr(WiFi_standard,MCS_mode,Nt,Pt,freq,Bandwidth,...
    Npkt,L,AI,Ant_type,channel_type,Nsubcarrier,reflection_times,wall_mix,wall_index,...
    HOV,tx,ty,rx,ry,room_range)

    Nr=Nt;
    Nsym=10^3;  % number of OFDM symbols per packet
    % number of OFDM symdol, which is also the number of channel realizations
    % this is used for generating the channels and calculate the AVERAGE
    % channel spatial correlation
    [modulation,code_rate] = fix_fc_return_modulation_code_rate(WiFi_standard,MCS_mode);

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
    ant_gain=zeros(Nt,length(temp.gain));
    for ind_ant=1:1:Nt
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

    % =================== End of initialzing parameter ================== %

    % ============== Retrieving AoDs and Traveling distances ============ %
    % [AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
    %     channel_type,collision_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range);
    [AoAAoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
            reflection_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range);
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

    t1=zeros(Nr,Nt); %receive correlation 
    t2=zeros(Nr,Nt); %transmit correlation
    t3=0; %receive correlation normalize
    t4=0; %transmit correlation normalize
    t5=0; %SNR

    Distance_in_count = distance(1:1:Ncluster);

    for q=1:1:Ncluster
        AoD_in_count(1,q)=AoAAoD(1,index(1,q));
        temp_AoD = floor(AoAAoD(1,index(1,q)));
        AoD_in_floor(1,q) = temp_AoD;
        AoD_index=temp_AoD+1;
        Ant_gain_in_count(1,q) = ant_gain(1,AoD_index);
    end

    %PL_dB_in_count=fc_path_loss(channel_type,Distance_in_count,freq,LOS,Npkt*Nsym); % Path loss effect of each cluster in dB
    PL_dB_in_count=fix_fc_path_loss(channel_type,Distance_in_count,freq,LOS,Npkt*Nsym,Ncluster); % Path loss effect of each cluster in dB
    % =========== End of getting the imformation of in count ray ======== %

    % ================ Get Angular Spread for each main path ============ %
    angular=fc_cal_angular_spread(wall_mix,wall_index,HOV,tx,ty,rx,ry,...
        channel_type,AoD_in_count,wall_in_count,Ncluster,reflection_times,collision_in_count);
    % ======================== End of getting AS ======================== %


    % =============================================================== %
    % Calculating the h cluster of the receving signal, and compare   %
    % with standard rate(bits/sec/Hz). If over the standard rate,     %
    % the packet will be count as error packet.                       %
    % h_time (Nt*Nr,Nsubcarrier): time domain channel coefficients    %
    % H_freq (Nt,Nr,Nsubcarrier): freq domain channel coefficients    %
    % =============================================================== %
    %capacity=zeros(Nsym,Nsubcarrier); % MIMO capacity: one stream only, no multiplexing gain
    %sm_capacity=zeros(Nsym,Nsubcarrier); % MIMO capacity: spatial multiplexing over parallel channels

    for ind_sym=1:1:Nsym
        % generating the channel coefficients in time and frequency domain
        h_time=fc_generate_h(Nsubcarrier,Nt,Nr,channel_type,AoD_in_floor,...
            Samping_rate_expansion_factor,PL_dB_in_count,ant_gain,Ncluster,angular);

        h_temp=zeros(size(h_time));
        for ind=1:1:Nt*Nr
            h_temp(ind,:)=fft(h_time(ind,:),Nsubcarrier)/sqrt(Nsubcarrier);
        end
        H_freq=zeros(Nr,Nt,Nsubcarrier);

        for index_f=1:1:Nsubcarrier
            H_freq(:,:,index_f)=reshape(h_temp(:,index_f),Nr,Nt);
            %capacity(ind_sym,index_f) = capacity(ind_sym,index_f) + log2(det(eye(Nr) + Pr*H_freq(:,:,index_f)*H_freq(:,:,index_f)'/(Nt*Thermal_noise)));
            %eigV = eigs(H_freq(:,:,index_f));
            %for ind=1:1:length(eigV)
            %    sm_capacity(ind_sym,index_f) = sm_capacity(ind_sym,index_f) + log2(1+Pr*eigV(ind)^2/(Nt*Thermal_noise));
            %end
            t1(:,:)=t1(:,:)+H_freq(:,:,index_f)*H_freq(:,:,index_f)'; %receive 
            t2(:,:)=t2(:,:)+H_freq(:,:,index_f)'*H_freq(:,:,index_f); %transmit
            t3=t3+trace(H_freq(:,:,index_f)*H_freq(:,:,index_f)'); %receive
            t4=t4+trace(H_freq(:,:,index_f)'*H_freq(:,:,index_f)); %transmit
            t5=t5+trace(H_freq(:,:,index_f)'*H_freq(:,:,index_f)); %SNR
        end
        % --- each antenna's channel coefficients are different --->
        % ===== generating the channel coefficients in time and frequency domain =====
    end
    t5=t5/Nsym;
    SNR=(1/Nsubcarrier)*Pt*abs(t5)/(Nt*Nr*Thermal_noise);
    snr_dB=10*log10(SNR);
    % ================ End of counting the error packet ================= %

    gamma=SNR;
    R_corr=abs(t1(:,:))/(abs((t3))/Nr); % receive spatial correlation matrix
    S_corr=abs(t2(:,:))/(abs((t4))/Nt); % transmit spatial correlation matrix
    eig_Value=eig(R_corr).';

    % < ----- Return the codeword pairwise error probability (C-PEP) for 
    % spatial multiplexing as BER --------------------------------------
    BER = fc_MIMO_CPEP(WiFi_standard,modulation,code_rate,Nt,Nr,gamma,eig_Value,R_corr,S_corr);
    if BER > 1
        warning ('BER > 1');
        BER = 1;
    end
    % ------------------------------------------------------------------------>
    PER = 1-(1-BER)^L;
end