%{
Author:      SONY, Wen-Pin Hsu
Date:        2016/5/20
Input:       AoD      
             decay_vec
             decay_vec_amp
             AS
             gain: one antenna's pattern in normal scale and its size is 1x(pattern resolution)
             pathloss
             cluster_index: cluster num(type B cluster "1")
                When cluster_index=n, "AoD(1,cluster_index)" denotes the AoD of n-th cluster
                                      and "pathloss(1,cluster_index)" denotes path loss of n-th cluster in dB
             new_AS: delete collision AS
Output:      cluster
Description: Part 1: 
             The idea is to let the power summation of each cluster equals 
             to 1. In other words, "h_cluster/sqrt(sum(total_pow))" represents 
             the channel coefficients whose sum of h.conj(h) equals to 1.
             More information in details can be found in the research notebook,
             vol. 15, pp 102

             Part 2: 
             Check the new AS after collision. If there is the difference,
             we will randomly delete part of cluster ray
      
%}
function [cluster] = fc_cal_cluster(...
    AoD,decay_vec,decay_vec_amp,channel_type,gain,pathloss,cluster_index,new_AS)
    width=length(decay_vec)-1;
    AS=fc_AS_table(channel_type,cluster_index); % find table to get AS
    % part 1
    cluster=(1/sqrt(2))*(randn(1,length(decay_vec))+1i*randn(1,length(decay_vec)))./decay_vec_amp;
    %cluster=(1/sqrt(2))*(randn(1,length(decay_vec))+j*randn(1,length(decay_vec)))./decay_vec_amp;
    total_pow = cluster.*conj(cluster); % power summation of the rays in cluster
    half_AS=floor(AS/2);
    if (AoD(1,cluster_index)-half_AS<=0)||(AoD(1,cluster_index)+half_AS>361)
        if (AoD(1,cluster_index)-half_AS<=0)
            G1=[gain((AoD(1,cluster_index)-half_AS+360):1:360),gain(1:1:(AoD(1,cluster_index)+half_AS))];
        end
        if (AoD(1,cluster_index)+half_AS>361)
            G1=[gain((AoD(1,cluster_index)-half_AS):1:360),gain(1:1:(AoD(1,cluster_index)+half_AS-360))];
        end
    else
        G1=gain( (AoD(1,cluster_index)-half_AS):1:(AoD(1,cluster_index)+half_AS) );
    end
    pow_gain = sum(G1);
    cluster = cluster/sqrt(sum(total_pow))/pathloss(1,cluster_index)*sqrt(pow_gain/sum(gain(1:1:360)));
    % the idea is to let the power summation of each cluster equals to 1, In
    % other words, "h_cluster1/sqrt(sum(total_pow1))" represents the
    % channel coefficients whose sum of h.conj(h) equals to 1.
    % More information in details can be found in the research notebook,
    % vol. 15, pp 102
    
    % part 2
    % ===================== check AS cluser index ======================= %
    if new_AS < AS
        count=round((AS-new_AS)*width/AS);% delete this num ray
        randomarray=randperm(width);%random number with no repeat
        for i=1:1:count
           cluster(randomarray(i)+1)=0;% main path is the first
        end
    end
    % ===================== check AS cluser index ======================= %
end

