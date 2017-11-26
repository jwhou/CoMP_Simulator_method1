function output = fc_atten_of_each_cluster(ori_decay,t)

% Given the attenuation of each ray/tap in one cluster in [1] (t=1, i.e. sampling=10ns) 
% and the channel sampleing rate expansion factor (t, decided by the bandwidth), 
% this function returns a vector of the attenuation of each ray/tap by [2] (interpolation).
% 
% [1]: V. Erceg, et. al. "TGn Channel Models", Doc. IEEE 802.11-03/940r4, 2004-05-10
% [2]: G. Breit, et. al. "TGac Channel Models Addendum Supporting Material.", 
%                                                                                           Doc. IEEE 802.11-09/569r0, 2009-05
% ori_decay: by [1] the attenuation of each ray/tap in one cluster 
% t: channel sampling rate expansion factor
%   t=1 if BW <= 40MHz
%   t=2 if 40MHz <= BW <= 80MHz
%   t=4 if 80MHz <= BW <= 160MHz
%   t=8 if BW >= 160MHz

y=upsample(ori_decay,t);
if t==1
    output=ori_decay;
elseif t==2
    output=y(1,1:1:length(y)-1);
    for i=2:2:length(y)-2
        output(1,i)=(y(1,i-1)+y(1,i+1))/2;
    end
elseif t==4
    output=y(1,1:1:length(y)-3);
     for i=3:4:length(output)-1
         output(1,i)=(y(1,i-2)+y(1,i+2))/2;
     end
     for i=2:2:length(output)-1
         output(1,i)=(output(1,i-1)+output(1,i+1))/2;
     end
else % t==8
    output=y(1,1:1:length(y)-7);
    for i=5:8:length(output)-3
        output(1,i)=(y(1,i-4)+y(1,i+4))/2;
    end
    for i=3:4:length(output)-1
        output(1,i)=(output(1,i-2)+output(1,i+2))/2;
    end
    for i=2:2:length(output)-1
        output(1,i)=(output(1,i-1)+output(1,i+1))/2;
    end
end
end
