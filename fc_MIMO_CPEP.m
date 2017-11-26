%{
Author:      Sony, Ju-Chia Wang
Date:        2017/10/20
Input:       % WiFi_standard, it is a "string" == "80211n" or "80211ac"
             
Output:      
Description: 
%}

function [Y]=fc_MIMO_CPEP(WiFi_standard,modulation,code_rate,Nt,Nr,gamma,eig_Value,R_corr,S_corr)

d_free=0; % Hamming distance, which depends on the modulation
K_c=0;    % (n,k) convolutional code

if Nt>Nr
    error('Nt has to less than or equal to Nr')
end

R_det=det(R_corr);
S_det=det(S_corr);
% % S_corr_inv = S_corr^(-1);

if strcmp(modulation,'BPSK') ==1 % BPSK
    P_M=1;
    eps_M=4;
elseif strcmp(modulation,'QPSK') ==1 % QPSK
    P_M=1;
    eps_M=2;
elseif strcmp(modulation,'16QAM') ==1 % 16QAM
    P_M=[3/4, 1/4];
    eps_M=[0.4, 1.6];
else
    P_M=[7/12, 1/4, 1/12, 1/12];
    eps_M=[0.0952, 0.3810, 0.8571, 1.5238];
end

C_ell = fc_return_auxiliary_constant_Cell(Nt,eig_Value);
if code_rate==1/2
    d_free=10;
    K_c=1;
elseif code_rate==2/3
    d_free=7;
    K_c=2;
elseif code_rate==3/4
    d_free=6;
    K_c=3;
elseif code_rate==5/6
    d_free=5;
    K_c=5;
end


Y=0;
for d=d_free:1:d_free+100
    %<----- calculating the nominator of eq. (46) -----
    t1=zeros(1,Nr-Nt+1);
    t2=zeros(1,length(P_M));
    t3=zeros(1,Nt);
    for k=1:1:Nt
        for i=1:1:length(P_M)
    %         t1 = t1 + P_M(1,i)*(1+gamma*eps_M(1,i)*R_det*S_det/4)^(-1);
            for ell=1:1:Nr-Nt+1    
                temp1=[ S_corr(:,1:(k-1)),S_corr(:,(k+1):Nt)];
                temp2=[temp1(1:(k-1),:);temp1((k+1):Nt,:)];
                S_corr_inv = det(temp2)/det(S_corr);
                t1(ell) = (gamma*eps_M(1,i)/8/S_corr_inv)^ell*C_ell(ell);
            end
            t2(i) = P_M(1,i)*(1+sum(t1))^(-1);
            t1=zeros(1,Nr-Nt+1);
        end
        t3(k)=sum(t2);
        t2=zeros(1,length(P_M));
    end
    nomi=sum(t3)^(d+1/2); % the nominator of eq. (46)
    % ----- calculating the nominator of eq. (46) ----->

    % <----- calculating the denominator of eq. (46) -----
    u1_up=zeros(1,Nr-Nt+1);
    u1_down=zeros(1,Nr-Nt+1);
    u2=zeros(1,length(P_M));
    u3=zeros(1,Nt);
    for k=1:1:Nt
        for i=1:1:length(P_M)
    %         t1 = t1 + P_M(1,i)*(1+gamma*eps_M(1,i)*R_det*S_det/4)^(-2)*(gamma*eps_M(1,i)*R_det*S_det/4);
            for ell=1:1:Nr-Nt+1
                temp1=[ S_corr(:,1:(k-1)),S_corr(:,(k+1):Nt)];
                temp2=[temp1(1:(k-1),:);temp1((k+1):Nt,:)];
                S_corr_inv = det(temp2)/det(S_corr);
                u1_up(ell) = ell*(gamma*eps_M(1,i)/8/S_corr_inv)^ell*C_ell(ell);
                u1_down(ell) = (gamma*eps_M(1,i)/8/S_corr_inv)^ell*C_ell(ell);
            end
            u2(i) = P_M(1,i)*sum(u1_up)/((1+sum(u1_down))^2);
            u1_up=zeros(1,Nr-Nt+1);
            u1_down=zeros(1,Nr-Nt+1);
        end
        u3(k)=sum(u2);
        u2=zeros(1,length(P_M));
    end
    denomi=sqrt(sum(u3)); % the denominator of eq. (46)
    % ----- calculating the denominator of eq. (46) ----->
    Y=Y+nomi/denomi*(1/2/Nt^d/sqrt(pi*d));
end
Y=Y/K_c;