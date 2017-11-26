%{
Author:      Wen-Pin Hsu
Date:        2016/5/31
Input:       wall_mix
             wall_index: Number of wall
             HOV       : 0: Horizon
                         1: Vertical
                         2: Slash
             tx is the term "x_of_transmitter" in the Main function
             ty is the term "y_of_transmitter" in the Main function
             rx is the term "x_of_receiver" in the Main function
             ry is the term "y_of_receiver" in the Main function
             channel_type
             AoD_in_count
             wall_in_count
             Ncluster
             collision_times
             collision_in_count
Output:      angular
             
Description: According the AoD angle of the main path in AoD_in_count
             cluster, we check the Angular Spread (AS) when there are some
             angle is collision by the other wall
Example:     AoD_in_count=
             [51.8  306.5  53.5  133.06  304.42 229.3]
             wall_in_count=
             [ 0     1      8      5      1      1  ]
             [ 0     0      0      0      8      5  ]
             Ncluster = 6
             precision = 0.04
             AS = [41.6   55.2   47.4   27.2   33.0   38.0 ]
             angular=
   AS_start  [ 33.7  278.9   45.61  119.46  292.04  213.7]
   AS_end    [ 65.89 334.1   65.89  146.66  312.1   248.3]
   AS_diff   [ 32.19  55.2   20.28   27.2    20.06   34.6]
   main_AoD  [ 51.8  306.5   53.5   133.06  304.42  229.3]
%}
function [angular] = fc_cal_angular_spread(wall_mix,wall_index,HOV,tx,ty,rx,ry,...
    channel_type,AoD_in_count,wall_in_count,Ncluster,collision_times,collision_in_count)
    
    angular=zeros(4,Ncluster);
    angular(4,:)=AoD_in_count(:);
    precision = 0.03;
    
    new_wall_in_count=zeros(collision_times+1,Ncluster);
    
    for num=1:1:Ncluster
        new_wall_in_count(1:1:collision_times,num)=wall_in_count(1:1:collision_times,num);
        % get Angular Spread from table
        AS=fc_AS_table(channel_type,num);
        Txangle=AoD_in_count(num);
        angular(1,num)=Txangle;
        angular(2,num)=Txangle;
        
        
        
        % =============================================================== %
        % Checking the AS of the main path's collision wall. If same as   %
        % cluster 'wall_in_count(i)' => correct, else wrong.              %
        % =============================================================== %
        % check AS_start
        for angle = Txangle-precision : -precision : Txangle-AS/2
            % Increase or decrease (IoD)
            % if IoD = 0, decrease angle, calculate angular(1,num) => AS_start
            % if IoD = 1, decrease angle, calculate angular(2,num) => AS_end
            IoD=0;
            [angular,flag]=fc_cal_AS_sub(angular,angle,collision_times,tx,ty,rx,ry,...
                wall_index,HOV,wall_mix,new_wall_in_count,collision_in_count,num,IoD);
            if flag == 1
                break;
            end
        end % end of for angle = ...
        % check AS_end
        for angle = Txangle+precision : precision : Txangle+AS/2
            % Increase or decrease (IoD)
            % if IoD = 0, decrease angle, calculate angular(1,num) => AS_start
            % if IoD = 1, decrease angle, calculate angular(2,num) => AS_end
            IoD=1;
            [angular,flag]=fc_cal_AS_sub(angular,angle,collision_times,tx,ty,rx,ry,...
                wall_index,HOV,wall_mix,new_wall_in_count,collision_in_count,num,IoD);
            if flag == 1
                break;
            end
        end
        
        % ================ End of checking angular angle ================ %
        
        % let the angle in 0 ~ 359 
        if angular(1,num) >= 360
            angular(1,num)=angular(1,num)-360;
        elseif angular(1,num) < 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
            angular(1,num)=angular(1,num)+360;
        end
        if angular(2,num) >= 360
            angular(2,num)=angular(2,num)-360;
        elseif angular(2,num) < 0
            angular(2,num)=angular(2,num)+360;
        end
        if Txangle < 0
            angular(4,num)=Txangle+360;
        else
            angular(4,num)=Txangle;
        end
        % calculate difference of start and end
        if abs(angular(2,num)-angular(1,num)) >= 180
            angular(3,num)=abs(abs(angular(2,num)-angular(1,num))-360);
        else
            angular(3,num)=abs(angular(2,num)-angular(1,num));
        end
    end
end

