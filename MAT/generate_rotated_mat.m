clc
clear all

load('1(0)2(+y)_theta45.mat')
polar(phi,gain);
gain2=zeros(size(gain));
gain_dB2=zeros(size(gain));
for i=1:1:length(gain)
    t1 = 45+i;
    t2 = 45-i;
%     t1 = 135+i;
%     t2 = 135-i;
    if t1 > 360
        t1=t1 - 360;
    end
    if t1 <= 0
        t1=t1 + 360;
    end
    if t2 > 360
        t2=t2 - 360;
    end
    if t2 <= 0
        t2=t2 + 360;
    end
    gain2(1,t1) = gain(1,t2);
    gain_dB2(1,t1) = gain_dB(1,t2);
end
gain2(1,361)=gain(1,89);
gain_dB2(1,361)=gain_dB(1,89);
% gain2(1,361)=gain(1,269);
% gain_dB2(1,361)=gain_dB(1,269);

gain=gain2;
gain_dB=gain_dB2;

% <=======================

% =======================>

figure;
polar(phi,5*ones(size(gain)),'k-')
hold on
polar(phi,gain,'b-')
title(['1(-y)2(0)'],'fontsize',16)
save('1(-y)2(0)_theta45.mat')