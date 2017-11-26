%{
Author:      Wen-Pin Hsu, SONY
Date:        2016/10/25
Input:
    % Nsubcarrier: number of subcarriers
    % N_tx: # of TX antennas
    % N_rx: # of RX antennas
    % channel_type: channel model A/B/C/D/E/F
    % AoD: it's an vector and each element represent the AoD of each cluster
    % exp_fac: samping rate expansion factor
    % PL_dB: PL_dB(1,n) denotes the path loss of cluster n in dB
    % gain: TX directional antenna gain in normal scale and its size is Nt*(pattern resolution)
            gain(i,:) denotes the i-th antenna's pattern
            
Output:
    h_time: time-domain channel coefficients and its size is (Nt*Nr)xNsubcarrier
      e.g., if Nt=Nr=2, h_time = [  h11(0) h11(1) h11(2) ...
                                    h21(0) h21(1) h21(2) ...
                                    h12(0) h12(1) h12(2) ...
                                    h22(0) h22(1) h22(2) ... ]

Description: This is a function to generate the time-domain channel coefficients for 802.11 2.4GHz and 5GHz channels

%}

function h_time=fc_generate_h(Nsubcarrier,N_tx,N_rx,channel_type,AoD,...
    exp_fac,PL_dB,gain,Ncluster,angular)

pathloss=10.^(PL_dB/20); % path loss for each cluster in normal scale
% ant_gain=10.^(ant_gain_dB/20); % antenna gain for each cluster in normal scale

h_time=zeros(N_tx*N_rx,Nsubcarrier);
t=exp_fac;
% ===== model A =====
if channel_type==1
    index=1;
    for ind1=1:1:N_tx
        for ind2=1:1:N_rx
            h_time(index,1)= (1/sqrt(2))*(randn(1)+j*randn(1))*sqrt(gain(ind1,AoD(1,1)))/pathloss(1,1);
            index=index+1;
        end
    end
end
% ===== model A =====

% ============================== model B ================================ %
if channel_type==2
    index=1;
    for ind1=1:1:N_tx
        for ind2=1:1:N_rx
            % <---------- cluster 1, decaying slope = 5.4dB/10ns, AS=14.4 ----------> %
            if Ncluster >= 1
                decay_vec1=(0:1:4*t)*5.4/t; % decaying vector in dB for cluster 1
                %decay_vec1=[0:1:4*t]*5.4/t; % decaying vector in dB for cluster 1
                decay_vec1_amp=10.^(decay_vec1/20); % the amplitute decaying factor
                
                cluster1=fc_cal_cluster(...
                    AoD,decay_vec1,decay_vec1_amp,channel_type,gain(ind1,:),pathloss,1,angular(3,1));
                
                h_time(index,1:1:(4*t+1)) = h_time(index,1:1:(4*t+1)) + cluster1;
            end
            % <---------------------- end of model B cluster 1 ---------------------> %
            
            % <---------- cluster 2, decaying slope = 3.1dB/10ns, AS=25.4 ----------> %
            if Ncluster >= 2
                decay_vec2=(0:1:(9-3)*t)*3.1/t+3.2; % decaying vector in dB for cluster 2
                %decay_vec2=[0:1:(9-3)*t]*3.1/t+3.2; % decaying vector in dB for cluster 2
                decay_vec2_amp=10.^(decay_vec2/20); % the amplitute decaying factor
                
                cluster2=fc_cal_cluster(...
                    AoD,decay_vec2,decay_vec2_amp,channel_type,gain(ind1,:),pathloss,2,angular(3,2));
                
                h_time(index,((3-1)*t+1):1:((9-1)*t+1)) = h_time(index,((3-1)*t+1):1:((9-1)*t+1)) + cluster2;
            end
            % <---------------------- end of model B cluster 2 ---------------------> %
            index=index+1;
        end
    end
end
% =========================== end of model B ============================ %

% ============================== model C ================================ %
if channel_type==3 % model C
    index=1;
    for ind1=1:1:N_tx
        for ind2=1:1:N_rx
            h_time1=zeros(1,Nsubcarrier);
            h_time2=zeros(1,Nsubcarrier);
            % <---------- cluster 1, decaying slope = 2.2dB/10ns, AS=24.6 ----------> %
            if Ncluster >= 1
                decay_vec1=(0:1:9*t)*2.2/t; % decaying vector in dB for cluster 1
                %decay_vec1=[0:1:9*t]*2.2/t; % decaying vector in dB for cluster 1
                decay_vec1_amp=10.^(decay_vec1/20); % the amplitute decaying factor
                
                cluster1=fc_cal_cluster(...
                    AoD,decay_vec1,decay_vec1_amp,channel_type,gain(ind1,:),pathloss,1,angular(3,1));
                
                h_time1(1,1:1:(9*t+1)) = cluster1;
            end
            % <---------------------- end of model C cluster 1 ---------------------> %
            
            % <---------- cluster 2, decaying slope = 2.2dB/10ns, AS=22.4 ----------> %
            if Ncluster >= 2
                decay_vec2=(0:1:(14-7)*t)*2.2/t+5.0; % decaying vector in dB for cluster 2
                %decay_vec2=[0:1:(14-7)*t]*2.2/t+5.0; % decaying vector in dB for cluster 2
                decay_vec2_amp=10.^(decay_vec2/20); % the amplitute decaying factor
                
                cluster2=fc_cal_cluster(...
                    AoD,decay_vec2,decay_vec2_amp,channel_type,gain(ind1,:),pathloss,2,angular(3,2));
                
                h_time2(1,(60*t/10+1):1:(60*t/10+1+4*t-1)) = cluster2(1,1:1:4*t);
                h_time2(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster2(1,(4*t+1):1:(4*t+1+t-1));
                h_time2(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster2(1,(5*t+1):1:(5*t+1+t-1));
                h_time2(1,(170*t/10+1):1:(170*t/10+1+t-1)) = cluster2(1,(6*t+1):1:(6*t+1+t-1));
                h_time2(1,(200*t/10)+1) = cluster2(1,(7*t+1));
            end
            % <---------------------- end of model C cluster 2 ---------------------> %
            h_time(index,:)=h_time1+h_time2;
            index=index+1;
        end
    end
end
% =========================== end of model C ============================ %

% ============================== model D ================================ %
if channel_type==4
    index=1;
    for ind1=1:1:N_tx
        for ind2=1:1:N_rx
            h_time1=zeros(1,Nsubcarrier);
            h_time2=zeros(1,Nsubcarrier);
            h_time3=zeros(1,Nsubcarrier);
            % <-------------------- cluster 1, AoD AS = 27.4 -----------------------> %
            if Ncluster >= 1
                ori_decay1=[0 0.9 1.7 2.6 3.5 4.3 5.2 6.1 6.9 7.8 9.0 11.1 13.7 16.3 19.3 23.2];
                decay_vec1=fc_atten_of_each_cluster(ori_decay1,t);
                decay_vec1_amp=10.^(decay_vec1/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec1,decay_vec1_amp,channel_type,gain(ind1,:),pathloss,1,angular(3,1));
                
                h_time1(1,1:1:(9*t)) = cluster(1,1:1:9*t);
                h_time1(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster(1,(9*t+1):1:(9*t+1+t-1));
                h_time1(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster(1,(10*t+1):1:(10*t+1+t-1));
                h_time1(1,(170*t/10+1):1:(170*t/10+1+t-1)) = cluster(1,(11*t+1):1:(11*t+1+t-1));
                h_time1(1,(200*t/10+1):1:(200*t/10+1+t-1)) = cluster(1,(12*t+1):1:(12*t+1+t-1));
                h_time1(1,(240*t/10+1):1:(240*t/10+1+t-1)) = cluster(1,(13*t+1):1:(13*t+1+t-1));
                h_time1(1,(290*t/10+1)) = cluster(1,(14*t+1));
            end
            % <---------------------- end of model D cluster 1 ---------------------> %
            
            % <-------------------- cluster 2, AoD AS = 32.1 -----------------------> %
            if Ncluster >= 2
                ori_decay2=[6.6 9.5 12.1 14.7 17.4 21.9 25.5];
                decay_vec2=fc_atten_of_each_cluster(ori_decay2,t);
                decay_vec2_amp=10.^(decay_vec2/20); % the amplitute decaying factor
                
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec2,decay_vec2_amp,channel_type,gain(ind1,:),pathloss,2,angular(3,2));
                
                
                h_time2(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time2(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time2(1,(170*t/10+1):1:(170*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                h_time2(1,(200*t/10+1):1:(200*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time2(1,(240*t/10+1):1:(240*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time2(1,(290*t/10+1):1:(290*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time2(1,(340*t/10+1)) = cluster(1,(6*t+1));
            end
            % <---------------------- end of model D cluster 2 ---------------------> %
            
            % <-------------------- cluster 3, AoD AS = 36.8 -----------------------> %
            if Ncluster >= 3
                ori_decay3=[18.8 23.2 25.2 26.7];
                decay_vec3=fc_atten_of_each_cluster(ori_decay3,t);
                decay_vec3_amp=10.^(decay_vec3/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec3,decay_vec3_amp,channel_type,gain(ind1,:),pathloss,3,angular(3,3));
                
                h_time3(1,(240*t/10+1):1:(240*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time3(1,(290*t/10+1):1:(290*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time3(1,(340*t/10+1):1:(340*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                h_time3(1,(390*t/10+1)) = cluster(1,(3*t+1));
            end
            % <---------------------- end of model D cluster 3 ---------------------> %
            h_time(index,:)=h_time1+h_time2+h_time3;
            index=index+1;
        end
    end
end
% =========================== end of model D ============================ %

% ============================== model E ================================ %
if channel_type==5
    index=1;
    for ind1=1:1:N_tx
        for ind2=1:1:N_rx
            h_time1=zeros(1,Nsubcarrier);
            h_time2=zeros(1,Nsubcarrier);
            h_time3=zeros(1,Nsubcarrier);
            h_time4=zeros(1,Nsubcarrier);
            
            % <-------------------- cluster 1, AoD AS = 36.1 -----------------------> %
            if Ncluster >= 1
                ori_decay1=[2.6 3.0 3.5 3.9 4.5 5.6 6.9 8.2 9.8 11.7 13.9 16.1 18.3 20.5 22.9];
                decay_vec1=fc_atten_of_each_cluster(ori_decay1,t);
                decay_vec1_amp=10.^(decay_vec1/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec1,decay_vec1_amp,channel_type,gain(ind1,:),pathloss,1,angular(3,1));
                h_time1(1,1:1:(3*t)) = cluster(1,1:1:3*t);
                h_time1(1,(50*t/10+1):1:(50*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time1(1,(80*t/10+1):1:(80*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time1(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time1(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster(1,(6*t+1):1:(6*t+1+t-1));
                h_time1(1,(180*t/10+1):1:(180*t/10+1+t-1)) = cluster(1,(7*t+1):1:(7*t+1+t-1));
                h_time1(1,(230*t/10+1):1:(230*t/10+1+t-1)) = cluster(1,(8*t+1):1:(8*t+1+t-1));
                h_time1(1,(280*t/10+1):1:(280*t/10+1+t-1)) = cluster(1,(9*t+1):1:(9*t+1+t-1));
                h_time1(1,(330*t/10+1):1:(330*t/10+1+t-1)) = cluster(1,(10*t+1):1:(10*t+1+t-1));
                h_time1(1,(380*t/10+1):1:(380*t/10+1+t-1)) = cluster(1,(11*t+1):1:(11*t+1+t-1));
                h_time1(1,(430*t/10+1):1:(430*t/10+1+t-1)) = cluster(1,(12*t+1):1:(12*t+1+t-1));
                h_time1(1,(490*t/10+1)) = cluster(1,(13*t+1));
            end
            % <---------------------- end of model E cluster 1 ---------------------> %
            
            % <-------------------- cluster 2, AoD AS = 42.5 -----------------------> %
            if Ncluster >= 2
                ori_decay2=[1.8 3.2 4.5 5.8 7.1 9.9 10.3 14.3 14.7 18.7 19.9 22.4];
                decay_vec2=fc_atten_of_each_cluster(ori_decay2,t);
                decay_vec2_amp=10.^(decay_vec2/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec2,decay_vec2_amp,channel_type,gain(ind1,:),pathloss,2,angular(3,2));
                
                h_time2(1,(50*t/10+1):1:(50*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time2(1,(80*t/10+1):1:(80*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time2(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                h_time2(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time2(1,(180*t/10+1):1:(180*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time2(1,(230*t/10+1):1:(230*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time2(1,(280*t/10+1):1:(280*t/10+1+t-1)) = cluster(1,(6*t+1):1:(6*t+1+t-1));
                h_time2(1,(330*t/10+1):1:(330*t/10+1+t-1)) = cluster(1,(7*t+1):1:(7*t+1+t-1));
                h_time2(1,(380*t/10+1):1:(380*t/10+1+t-1)) = cluster(1,(8*t+1):1:(8*t+1+t-1));
                h_time2(1,(430*t/10+1):1:(430*t/10+1+t-1)) = cluster(1,(9*t+1):1:(9*t+1+t-1));
                h_time2(1,(490*t/10+1):1:(490*t/10+1+t-1)) = cluster(1,(10*t+1):1:(10*t+1+t-1));
                h_time2(1,(560*t/10+1)) = cluster(1,(11*t+1));
            end
            % <---------------------- end of model E cluster 2 ---------------------> %
            
            % <---------------------- cluster 3, AoD AS = 38 -----------------------> %
            if Ncluster >= 3
                ori_decay3=[7.9 9.6 14.2 13.8 18.6 18.1 22.8];
                decay_vec3=fc_atten_of_each_cluster(ori_decay3,t);
                decay_vec3_amp=10.^(decay_vec3/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec3,decay_vec3_amp,channel_type,gain(ind1,:),pathloss,3,angular(3,3));
                
                h_time3(1,(180*t/10+1):1:(180*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time3(1,(230*t/10+1):1:(230*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time3(1,(280*t/10+1):1:(280*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                h_time3(1,(330*t/10+1):1:(330*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time3(1,(380*t/10+1):1:(380*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time3(1,(430*t/10+1):1:(430*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time3(1,(490*t/10+1)) = cluster(1,(6*t+1));
            end
            % <---------------------- end of model E cluster 3 ---------------------> %
            
            % <---------------------- cluster 4, AoD AS = 38.7 ---------------------> %
            if Ncluster >= 4
                ori_decay4=[20.6 20.5 20.7 24.6];
                decay_vec4=fc_atten_of_each_cluster(ori_decay4,t);
                decay_vec4_amp=10.^(decay_vec4/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec4,decay_vec4_amp,channel_type,gain(ind1,:),pathloss,4,angular(3,4));
                
                h_time4(1,(490*t/10+1):1:(490*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time4(1,(560*t/10+1):1:(560*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                if 640*t/10<Nsubcarrier
                    h_time4(1,(640*t/10+1):1:(640*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                    h_time4(1,(730*t/10+1)) = cluster(1,(3*t+1));
                end
            end
            % <---------------------- end of model E cluster 4 ---------------------> %
            
            h_time(index,:)=h_time1+h_time2+h_time3+h_time4;
            index=index+1;
        end
    end
end
% =========================== end of model E ============================ %

% ============================== model F ================================ %
if channel_type==6
    index=1;
    for ind1=1:1:N_tx
        for ind2=1:1:N_rx
            h_time1=zeros(1,Nsubcarrier);
            h_time2=zeros(1,Nsubcarrier);
            h_time3=zeros(1,Nsubcarrier);
            h_time4=zeros(1,Nsubcarrier);
            h_time5=zeros(1,Nsubcarrier);
            h_time6=zeros(1,Nsubcarrier);
            
            % <--------------------- cluster 1, AoD AS = 41.6 ----------------------> %
            if Ncluster >= 1
                ori_decay1=[3.3 3.6 3.9 4.2 4.6 5.3 6.2 7.1 8.2 9.5 11.0 12.5 14.3 16.7 19.9];
                decay_vec1=fc_atten_of_each_cluster(ori_decay1,t);
                decay_vec1_amp=10.^(decay_vec1/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec1,decay_vec1_amp,channel_type,gain(ind1,:),pathloss,1,angular(3,1));
                
                h_time1(1,1:1:(3*t)) = cluster(1,1:1:3*t);
                h_time1(1,(50*t/10+1):1:(50*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time1(1,(80*t/10+1):1:(80*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time1(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time1(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster(1,(6*t+1):1:(6*t+1+t-1));
                h_time1(1,(180*t/10+1):1:(180*t/10+1+t-1)) = cluster(1,(7*t+1):1:(7*t+1+t-1));
                h_time1(1,(230*t/10+1):1:(230*t/10+1+t-1)) = cluster(1,(8*t+1):1:(8*t+1+t-1));
                h_time1(1,(280*t/10+1):1:(280*t/10+1+t-1)) = cluster(1,(9*t+1):1:(9*t+1+t-1));
                h_time1(1,(330*t/10+1):1:(330*t/10+1+t-1)) = cluster(1,(10*t+1):1:(10*t+1+t-1));
                h_time1(1,(400*t/10+1):1:(400*t/10+1+t-1)) = cluster(1,(11*t+1):1:(11*t+1+t-1));
                h_time1(1,(490*t/10+1):1:(490*t/10+1+t-1)) = cluster(1,(12*t+1):1:(12*t+1+t-1));
                h_time1(1,(600*t/10+1)) = cluster(1,(13*t+1));
            end
            % <---------------------- end of model F cluster 1 ---------------------> %
            
            % <--------------------- cluster 2, AoD AS = 55.2 ----------------------> %
            if Ncluster >= 2
                ori_decay2=[1.8 2.8 3.5 4.4 5.3 7.4 7.0 10.3 10.4 13.8 15.7 19.9];
                decay_vec2=fc_atten_of_each_cluster(ori_decay2,t);
                decay_vec2_amp=10.^(decay_vec2/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec2,decay_vec2_amp,channel_type,gain(ind1,:),pathloss,2,angular(3,2));
                
                h_time2(1,(50*t/10+1):1:(50*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time2(1,(80*t/10+1):1:(80*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time2(1,(110*t/10+1):1:(110*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                h_time2(1,(140*t/10+1):1:(140*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time2(1,(180*t/10+1):1:(180*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time2(1,(230*t/10+1):1:(230*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time2(1,(280*t/10+1):1:(280*t/10+1+t-1)) = cluster(1,(6*t+1):1:(6*t+1+t-1));
                h_time2(1,(330*t/10+1):1:(330*t/10+1+t-1)) = cluster(1,(7*t+1):1:(7*t+1+t-1));
                h_time2(1,(400*t/10+1):1:(400*t/10+1+t-1)) = cluster(1,(8*t+1):1:(8*t+1+t-1));
                h_time2(1,(490*t/10+1):1:(490*t/10+1+t-1)) = cluster(1,(9*t+1):1:(9*t+1+t-1));
                h_time2(1,(600*t/10+1):1:(600*t/10+1+t-1)) = cluster(1,(10*t+1):1:(10*t+1+t-1));
                if 730*t/10<Nsubcarrier
                    h_time2(1,(730*t/10+1)) = cluster(1,(11*t+1));
                end
            end
            % <---------------------- end of model F cluster 2 ---------------------> %
            
            % <--------------------- cluster 3, AoD AS = 47.4 ----------------------> %
            if Ncluster >= 3
                ori_decay3=[5.7 6.7 10.4 9.6 14.1 12.7 18.5];
                decay_vec3=fc_atten_of_each_cluster(ori_decay3,t);
                decay_vec3_amp=10.^(decay_vec3/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec3,decay_vec3_amp,channel_type,gain(ind1,:),pathloss,3,angular(3,3));
                
                h_time3(1,(180*t/10+1):1:(180*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time3(1,(230*t/10+1):1:(230*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time3(1,(280*t/10+1):1:(280*t/10+1+t-1)) = cluster(1,(2*t+1):1:(2*t+1+t-1));
                h_time3(1,(330*t/10+1):1:(330*t/10+1+t-1)) = cluster(1,(3*t+1):1:(3*t+1+t-1));
                h_time3(1,(400*t/10+1):1:(400*t/10+1+t-1)) = cluster(1,(4*t+1):1:(4*t+1+t-1));
                h_time3(1,(490*t/10+1):1:(490*t/10+1+t-1)) = cluster(1,(5*t+1):1:(5*t+1+t-1));
                h_time3(1,(600*t/10+1)) = cluster(1,(6*t+1));
            end
            % <---------------------- end of model F cluster 3 ---------------------> %
            
            % <--------------------- cluster 4, AoD AS = 27.2 ----------------------> %
            if Ncluster >= 4
                ori_decay4=[8.8 13.3 18.7];
                decay_vec4=fc_atten_of_each_cluster(ori_decay4,t);
                decay_vec4_amp=10.^(decay_vec4/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec4,decay_vec4_amp,channel_type,gain(ind1,:),pathloss,4,angular(3,4));
                
                h_time4(1,(400*t/10+1):1:(400*t/10+1+t-1)) = cluster(1,1:1:t);
                h_time4(1,(490*t/10+1):1:(490*t/10+1+t-1)) = cluster(1,(t+1):1:(t+1+t-1));
                h_time4(1,(600*t/10+1)) = cluster(1,(2*t+1));
            end
            % <---------------------- end of model F cluster 4 ---------------------> %
            
            % <--------------------- cluster 5, AoD AS = 33.0 ----------------------> %
            if Ncluster >= 5
                ori_decay5=[12.9 14.2];
                decay_vec5=fc_atten_of_each_cluster(ori_decay5,t);
                decay_vec5_amp=10.^(decay_vec5/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec5,decay_vec5_amp,channel_type,gain(ind1,:),pathloss,5,angular(3,5));
                
                h_time5(1,(600*t/10+1):1:(600*t/10+1+t-1)) = cluster(1,1:1:t);
                if 730*t/10<Nsubcarrier
                    h_time5(1,(730*t/10+1)) = cluster(1,(t+1));
                end
            end
            % <---------------------- end of model F cluster 5 ---------------------> %
            
            % <--------------------- cluster 6, AoD AS = 38.0 ----------------------> %
            if Ncluster >= 6
                ori_decay6=[16.3 21.2];
                decay_vec6=fc_atten_of_each_cluster(ori_decay6,t);
                decay_vec6_amp=10.^(decay_vec6/20); % the amplitute decaying factor
                
                cluster=fc_cal_cluster(...
                    AoD,decay_vec6,decay_vec6_amp,channel_type,gain(ind1,:),pathloss,6,angular(3,6));
                
                if 880*t/10<Nsubcarrier
                    h_time6(1,(880*t/10+1):1:(880*t/10+1+t-1)) = cluster(1,1:1:t);
                    h_time6(1,(1050*t/10+1)) = cluster(1,(t+1));
                end
            end
            % <---------------------- end of model F cluster 6 ---------------------> %
            
            h_time(index,:)=h_time1+h_time2+h_time3+h_time4+h_time5+h_time6;
            index=index+1;
        end
    end
end
% =========================== end of model F ============================ %
end