%{
Author:      Wen-Pin Hsu
Date:        2016/6/4
Input:       angular: Initial angular array
             angle: heading angle
             collision_times: one reflection or two
             tx is the term "x_of_transmitter" in the Main function
             ty is the term "y_of_transmitter" in the Main function
             rx is the term "x_of_receiver" in the Main function
             ry is the term "y_of_receiver" in the Main function
             wall_index: Number of wall
             HOV       : 0: Horizon
                         1: Vertical
                         2: Slash
             wall_mix  : wall data
             new_wall_in_count: wall_in_count plus one zero's row
             collision_in_count: collision times
             num: Ncluster id
             IoD: Increase or decrease
Output:      angular
             flag   : If flag = 1, collision to the wrong wall
                    : If flag = 0, correct
             
Description: This is the fc_cal_angular_spread's sub function.
             According the AoD angle of the main path in AoD_in_count
             cluster, we check the Angular Spread (AS) when there are some
             angle is collision by the other wall
Example:     wall_in_count=[0 1 8 5]
             new_wall_in_count= [0 1 8 5]
                                [0 0 0 0] <- add one zero row for calculate
%}
function [angular,flag] = fc_cal_AS_sub(angular,angle,collision_times,tx,ty,rx,ry,...
                wall_index,HOV,wall_mix,new_wall_in_count,collision_in_count,num,IoD)

    j=0;
    if angle >= 360
        heading = angle-360;
    elseif angle < 0
        heading = angle+360;
    else
        heading = angle;
    end
    new_tx=tx;
    new_ty=ty;
    flag=0;
    while j <= collision_times && flag == 0
        slope=tan(deg2rad(heading));
        collision_point=zeros(wall_index,2);
        new_heading=zeros(wall_index,1);
        distance=zeros(wall_index,1);
        for i=1:1:wall_index
            % =========================================================== %
            % case 1: heading angle = 0      case 2: heading angle = 90   %
            % case 3: heading angle = 180    case 4: heading angle = 270  %
            % case 5:  0 < heading angle <  90                            % 
            % case 6: 90 < heading angle < 180                            %
            % case 7:180 < heading angle < 270                            %
            % case 8:270 < heading angle < 360                            %
            % =========================================================== %
            if heading == 0 && HOV(i) == 1 % case 1
                if wall_mix(i,1) > new_tx
                    if new_ty >= wall_mix(i,2) && new_ty <= wall_mix(i,3)
                        if wall_mix(i,1) < rx && new_wall_in_count(j+1,num) == 0
                            flag=1;
                            break;% break for function
                        else
                            distance(i)=abs(wall_mix(i,1)-new_tx);
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=new_ty;
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=180;
                            end
                        end
                    end
                end
            elseif heading == 90 && HOV(i) == 0 % case 2
                if wall_mix(i,1) > new_ty
                    if new_tx >= wall_mix(i,2) && new_tx <= wall_mix(i,3)
                        if wall_mix(i,1) < ry && new_wall_in_count(j+1,num) == 0
                            flag=1;
                            break;% break for function
                        else
                            distance(i)=abs(wall_mix(i,1)-new_ty);
                            collision_point(i,1)=new_tx;
                            collision_point(i,2)=wall_mix(i,1);
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=270;
                            end
                        end
                    end
                end
            elseif heading == 180 && HOV(i) == 1 % case 3
                if wall_mix(i,1) < new_tx
                    if new_ty >= wall_mix(i,2) && new_ty <= wall_mix(i,3)
                        if wall_mix(i,1) > rx && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=abs(new_tx-wall_mix(i,1));
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=new_ty;
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=0;
                            end
                        end
                    end
                end
            elseif heading == 270 && HOV(i) == 0 % case 4
                if wall_mix(i,1) < new_ty
                    if new_tx >= wall_mix(i,2) && new_tx <= wall_mix(i,3)
                        if wall_mix(i,1) > ry && new_wall_in_count(j+1,num) == 0
                            flag=1;
                            break;% break for function
                        else
                            distance(i)=abs(new_ty-wall_mix(i,1));
                            collision_point(i,1)=new_tx;
                            collision_point(i,2)=wall_mix(i,1);
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=90;
                            end
                        end
                    end
               end
            elseif heading < 90 && heading > 0 % case 5
                if wall_mix(i,1) > new_ty && HOV(i) == 0 % horizon
                    x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                    if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                        if wall_mix(i,1) < ry && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                            collision_point(i,1)=x;
                            collision_point(i,2)=wall_mix(i,1);
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=360-heading;
                            end
                        end
                    end
                elseif wall_mix(i,1) > new_tx && HOV(i) == 1 % vertical
                    y=new_ty+slope*(wall_mix(i,1)-new_tx);
                    if y >= wall_mix(i,2) && y <= wall_mix(i,3)
%                         rx=30;%=========================================================================================
                        if wall_mix(i,1) < rx && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=y;
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=180-heading;
                            end
                        end
                    end
                end
            elseif heading < 180 && heading > 90 % case 6
                if wall_mix(i,1) > new_ty && HOV(i) == 0 % horizon
                    x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                    if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                        if wall_mix(i,1) < ry && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                            collision_point(i,1)=x;
                            collision_point(i,2)=wall_mix(i,1);
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=360-heading;
                            end
                        end
                    end
                elseif wall_mix(i,1) < new_tx && HOV(i) == 1 % vertical
                    y=new_ty+slope*(wall_mix(i,1)-new_tx);
                    if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                        if wall_mix(i,1) > rx && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=y;
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=180-heading;
                            end
                        end
                    end
                end 
            elseif heading < 270 && heading > 180 % case 7
                if wall_mix(i,1) < new_ty && HOV(i) == 0 % horizon
                    x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                    if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                        if wall_mix(i,1) > ry && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                            collision_point(i,1)=x;
                            collision_point(i,2)=wall_mix(i,1);
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=360-heading;
                            end
                        end
                    end
                elseif wall_mix(i,1) < new_tx && HOV(i) == 1 % vertical
                    y=new_ty+slope*(wall_mix(i,1)-new_tx);
                    if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                        if wall_mix(i,1) > rx && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=y;
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=540-heading;
                            end
                        end
                    end
                end 
            elseif heading < 360 && heading > 270 % case 8
                if wall_mix(i,1) < new_ty && HOV(i) == 0 % horizon
                    x=(wall_mix(i,1)-new_ty)/slope+new_tx;
%                     ry=90;%=========================================================================================
                    if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                        if wall_mix(i,1) > ry && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                            collision_point(i,1)=x;
                            collision_point(i,2)=wall_mix(i,1);
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=360-heading;
                            end
                        end
                    end
                elseif wall_mix(i,1) > new_tx && HOV(i) == 1 % vertical
                    y=new_ty+slope*(wall_mix(i,1)-new_tx);
                    if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                        if wall_mix(i,1) < rx && new_wall_in_count(j+1,num) == 0
                            flag = 1;
                            break;
                        else
                            distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=y;
                            if j == collision_in_count(num)-1
                                new_heading(i,1)=...
                                    fc_cal_heading(collision_point(i,1),collision_point(i,2),rx,ry);
                            else
                                new_heading(i,1)=540-heading;
                            end
                        end
                    end
                end 
            end % end of if heading == 0 && HOV(i) == 1
        end % end of for i=1:1:wall_index
                
        if flag == 1 % collision to wall at this angle
            break; % break while func
        elseif flag == 0 && new_wall_in_count(j+1,num) == 0 % correct ray
            if IoD == 0 % write in angular_start
                angular(1,num)=angle;
            elseif IoD == 1 % write in angular_end
                angular(2,num)=angle;
            end
            break; % break while func and continue for angle...
        else % flag == 0 but wall_in_count(j+1,num) ~= 0
            % choose the smallest distance
            [new_distance,index]=sort(distance);
            min_distance =0;
            temp_wall_index=0;
            for i=1:1:wall_index
                if new_distance(i) == 0
                    continue;
                else 
                    min_distance = new_distance(i);
                    temp_wall_index=index(i);
                    break;
                end
            end
            if min_distance == 0 % need to collision but not
                flag = 1;
                break; % break while
            else %min_distance ~= 0
                if new_wall_in_count(j+1,num) ~= temp_wall_index
                    flag = 1;
                    break; % break while
                else
                    heading = new_heading(temp_wall_index);
                    j=j+1;
                    new_tx=collision_point(temp_wall_index,1);
                    new_ty=collision_point(temp_wall_index,2);
                end
            end
        end
    end % end of while j <= collision_times
end % end of function

