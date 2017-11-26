function [ Y ] = fix_fc_path_loss(channel_type,d,freq,LOS,Npkt,Ncluster)

% This is the path loss model for IEEE 802.11 2GHz and 5GHz channels.
% This function returns the path loss in dB

% channel_type: channel model type, A/B/C/D/E/F
% d: traveling distance
% freq: center carrier frequency
% LOS: if LOS==1, there is LoS path
%          if LOS==0, there are NLoS paths only
% Npkt: number of pkts, for calculating the average large scaling fading

lambda=3*10^8/freq; % wavelength of the center frequency 

if channel_type==1 % model A
    d_BP=5;
elseif channel_type==2 % model B
    d_BP=5;
elseif channel_type==3 % model C
    d_BP=5;
elseif channel_type==4 % model D
    d_BP=10;
elseif channel_type==5 % model E
    d_BP=20;
elseif channel_type==6% model F
    d_BP=30;
else % model else(debug use this)
    d_BP=30;
end

PathLoss = zeros(1,Ncluster);
for i = 1:1:Ncluster
    if d(i) <= d_BP
    %     PathLoss=20*log10(d);
        PathLoss(i)=20*log10(4*pi*d(i)/lambda);
    else
    %     PathLoss=20*log10(d_BP)+35*log10(d/d_BP);
        PathLoss(i)=20*log10(4*pi*d_BP/lambda)+35*log10(d(i)/d_BP);
    end
end

if LOS==1
    temp_v=3*randn(1,Npkt);
else % only NLOS
    if channel_type<=2
        temp_v=normrnd(0,10^(.4),1,Npkt);
    elseif channel_type<=4
        temp_v=normrnd(0,10^(.5),1,Npkt);
    elseif channel_type<=6
        temp_v=10^(.6)*randn(1,Npkt);
    end
end

%temp=sum(temp_v)/length(temp_v);
%Shadowing=temp*ones(1,length(d));
Shadowing=sum(temp_v)/length(temp_v);

% Y = PathLoss + shadowing fading. 
Y=PathLoss+Shadowing;