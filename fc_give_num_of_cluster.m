%{
Author:      SONY, Wen-Pin Hsu
Date:        2016/3/17
Input:       channel_type
             count
Output:      num_of_cluster
Description: Given the type of channel model, this function returns the number of
             clusters for the give type of channel model
             channel type  A  B  C  D  E  F
             # of cluster  1  2  2  3  4  6
                      mode 7: print all lines
%}
function num_of_cluster=fc_give_num_of_cluster(channel_type,count)

if channel_type == 1
    num_of_cluster = 1;
elseif channel_type == 2
    num_of_cluster = 2;
elseif channel_type == 3
    num_of_cluster = 2;
elseif channel_type == 4
    num_of_cluster = 3;
elseif channel_type == 5
    num_of_cluster = 4;
elseif channel_type == 6 
    num_of_cluster = 6;
else
    num_of_cluster = 7;
end
% =========== check error ========= % 
if num_of_cluster > count
    num_of_cluster=count;
end
% ================================== %
end