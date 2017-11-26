clc
clear all
load('1(+x+y)2(+y)_theta45.mat')
g_leq1=NaN*zeros(size(gain));
g_leq2=NaN*zeros(size(gain));
g_leq_half=NaN*zeros(size(gain));
[M,dir]=max(gain);
temp=zeros(size(gain));
for i=1:1:length(gain)
    if gain(1,i)>=max(gain)/2
        g_leq_half(1,i)=gain(1,i);
        temp(1,i)=1;
    end
    
    if gain(1,i)>=1
        g_leq1(1,i)=gain(1,i);
        if gain(1,i)>=2
            g_leq2(1,i)=gain(1,i);
        end
    end
end
BW=sum(temp);

figure;
polar(phi,5*ones(size(gain)),'k-')
hold on
h1=polar(phi,gain,'k-')
% h2=polar(phi,g_leq1,'b-')
% h3=polar(phi,g_leq2,'r-')
h4=polar(phi,g_leq_half,'b-')
set(h1,'linewidth',3.0)
% set(h2,'linewidth',3.0)
% set(h3,'linewidth',3.0)
set(h4,'linewidth',3.0)
title(['1(+x+y) 2(+y), max=',num2str(M),', \phi=',num2str(dir),'^o, BW=',num2str(BW),'^o'],'fontsize',16)