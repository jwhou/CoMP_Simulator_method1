%{
Author:      Wen-Pin Hsu
Date:        2016/5/12
Input:       channel_type
             cluster_num
Output:      angular_spread
Description: Given the type of channel model, this function returns the number of
             clusters for the give type of channel model
             ----------------------------------
             |channel type  A  B  C  D  E  F  | 
             |# of cluster  1  2  2  3  4  6  | 
             |         mode 7: print all lines|
             ---------------------------------- 
            channel type   1st   2nd   3rd   4th   5th   6th
                 A          -     -     -     -     -     - 
                 B         14.4  25.4   -     -     -     -
                 C         24.6  22.4   -     -     -     -
                 D         27.4  32.1  36.8   -     -     -
                 E         36.1  42.5  38.0  38.7   -     -
                 F         41.6  55.2  47.4  27.2  33.0  38.0
new_index = [ 2 4 6 0 0 0 0 0 0 ]         %cluster_num = new_index(i)
=====> AS = [ 0 AS1 0 AS2 0 AS3 0 0 0 0 ] %AS(new_index(i))=angular_spread
%}
function [angular_spread] = fc_AS_table(channel_type,cluster_num)
    table=[  0     0     0     0     0     0; 
            14.4  25.4   0     0     0     0;
            24.6  22.4   0     0     0     0;
            27.4  32.1  36.8   0     0     0;
            36.1  42.5  38.0  38.7   0     0;
            41.6  55.2  47.4  27.2  33.0  38.0];
    if channel_type == 7
        channel_type = 6;
    end
    angular_spread=table(channel_type,cluster_num);
end

