%{
Author:      Wen-Pin Hsu
Date:        2016/6/28
Input:       wall_start : 
                [wall_1_start_x wall_1_start_y; wall_2_start_x wall_2_start_y ...]
             wall_end   : 
                [wall_1_end_x   wall_1_end_y  ; wall_2_end_x   wall_2_end_y   ...]
             wall_index: Number of wall
             HOV       : 0: Horizon
                         1: Vertical
                         2: Slash
             tx is the term "x_of_transmitter" in the Main function
             ty is the term "y_of_transmitter" in the Main function
             rx is the term "x_of_receiver" in the Main function
             ry is the term "y_of_receiver" in the Main function
             room_range is using on precision
Output:      AoD(1,:) means AoD to reflectors of each cluster (azimuth)
             AoD(2,:) means AoA from reflectors of each cluster (azimuth)
             AoD(3,:) traveling distance of each cluster
             
Description: Computing the angle of departure (AoD), angle of arrival (AoA).
             We use every angle to check which angle is accept with RX
             greedly
             research book P?
             NOTE: The reflection from the roof/ceiling is not taken into account
Example:     wall_start = [ 0   0 ]   wall_end = [500  0 ]
                          [500  0 ]              [500 500]
                          [ 0  500]              [500 500]
                          [ 0   0 ]              [ 0  500]
             wall_mix   = [ 0   0  500]
                          [500  0  500]             
                          [500  0  500]
                          [0    0  500]
             wall_index: 4
             HOV:       [0 1 0 1]
%}
% function [AoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(...
%     reflection_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range)

function [AoD,Ncluster,distance,index,wall_in_count,collision_in_count]=fc_cal_main_path(reflection_times,wall_mix,wall_index,HOV,tx,ty,rx,ry,room_range)

    % =================================================================== %
    % According the collision times and room range, we can decide a good  %
    % precision number to improve the algorithm speed.                    %
    % =================================================================== %
  
    %disp('in fc_cal_main_path.m')
   % [tx,ty,rx,ry]
    if reflection_times == 2
        if room_range > 1000
            precision = 0.04;
        elseif room_range > 800
            precision = 0.04;
        elseif room_range > 400
            precision = 0.06;
        else
            precision = 0.08;
        end
    elseif reflection_times == 1
        if room_range > 800
            precision = 0.04;
        elseif room_range > 400
            precision = 0.08;
        else
            precision = 0.08;
        end
    else
        precision = 0.01;
    end
    % =================== end of calculate precision ==================== %
%     column_size=(160/precision)
    column_size=floor(160/precision);
    %column_size=8/precision;
    if wall_index > 5 && precision >= 0.02
        precision = max(precision-0.02,precision/2);
    end
    accept_main_path_num  = 0;% number of accepting main path
    collision_count_record= zeros(1,column_size);
    collision_wall_record = zeros(column_size,reflection_times);
    distance_record=zeros(1,column_size);
    AOD=zeros(column_size,1);

    for angle=0:precision:360-precision
        j=0;
        heading=angle;% ray heading angle
        new_tx=tx;
        new_ty=ty;
        temp_count_record = 0;
        temp_wall_record  = zeros(1,reflection_times);
        temp_distance_record = 0;
        

        while j<=reflection_times
            slope=tan(deg2rad(heading));
            LOS_distance=0;
            if heading == 0
                if rx >= new_tx && ry == new_ty
                    LOS_distance=rx-new_tx;
                end
            elseif heading == 90 % slope will infinite
                if rx == new_tx && ry > new_ty% accept and calculate distance
                    LOS_distance=ry-new_ty;
                end
            elseif heading == 180
                if rx <= new_tx && ry == new_ty
                    LOS_distance=new_tx-rx;
                end
            elseif heading == 270 % slope will infinite
                if rx == new_tx && ry < new_ty% accept and calculate distance
                    LOS_distance=new_ty-ry;
                end
            else
                if (heading < 90 && heading > 0 && rx >= new_tx && ry >= new_ty) ||...
                   (heading <180 && heading >90 && rx <= new_tx && ry >= new_ty) ||...
                   (heading <270 && heading >180 && rx <= new_tx && ry <= new_ty) ||...
                   (heading <360 && heading >270 && rx >= new_tx && ry <= new_ty)
                    % need to double check because of the precision
                    % use rx to check ry is on the line or not
                    y=new_ty+slope*(rx-new_tx);
                    if round(ry*10) == round(y*10)
                        LOS_distance=sqrt((rx-new_tx)^2+(ry-new_ty)^2);
                    end

                    % use ry to check rx is on the line or not
                    x=(ry-new_ty)/slope+new_tx;
                    if round(rx*10) == round(x*10)
                        LOS_distance=sqrt((rx-new_tx)^2+(ry-new_ty)^2);
                    end
                end
            end
            %
            % check the wall collision with the line or not
            collision_point=zeros(wall_index,2);
            new_heading=zeros(wall_index,1);
            distance=zeros(wall_index,1);
            for i=1:1:wall_index
                % only check vertical line or horizon line when heading 
                % angle is 0 90 180 270
                if heading == 0 && HOV(i) == 1
                    % x level > tx check collision to wall or not
                    if wall_mix(i,1) > new_tx
                        if new_ty >= wall_mix(i,2) && new_ty <= wall_mix(i,3)
                            distance(i)=abs(wall_mix(i,1)-new_tx);
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=new_ty;
                            new_heading(i,1)=180;
                        end
                    end
                elseif heading == 90 && HOV(i) == 0
                    if wall_mix(i,1) > new_ty
                        if new_tx >= wall_mix(i,2) && new_tx <= wall_mix(i,3)
                            distance(i)=abs(wall_mix(i,1)-new_ty);
                            collision_point(i,1)=new_tx;
                            collision_point(i,2)=wall_mix(i,1);
                            new_heading(i,1)=270;
                        end
                    end
                elseif heading == 180 && HOV(i) == 1
                    if wall_mix(i,1) < new_tx
                        if new_ty >= wall_mix(i,2) && new_ty <= wall_mix(i,3)
                            distance(i)=abs(new_tx-wall_mix(i,1));
                            collision_point(i,1)=wall_mix(i,1);
                            collision_point(i,2)=new_ty;
                            new_heading(i,1)=0;
                        end
                    end
                elseif heading == 270 && HOV(i) == 0
                    if wall_mix(i,1) < new_ty
                        if new_tx >= wall_mix(i,2) && new_tx <= wall_mix(i,3)
                            distance(i)=abs(new_ty-wall_mix(i,1));
                            collision_point(i,1)=new_tx;
                            collision_point(i,2)=wall_mix(i,1);
                            new_heading(i,1)=90;
                        end
                    end
                else% other heading
                    if heading < 90 && heading > 0 
                        if wall_mix(i,1) > new_ty && HOV(i) == 0 % horizon
                            x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                            if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                                distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                                collision_point(i,1)=x;
                                collision_point(i,2)=wall_mix(i,1);
                                new_heading(i,1)=360-heading;
                            end
                        elseif wall_mix(i,1) > new_tx && HOV(i) == 1 % vertical
                            y=new_ty+slope*(wall_mix(i,1)-new_tx);
                            if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                                distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                                collision_point(i,1)=wall_mix(i,1);
                                collision_point(i,2)=y;
                                new_heading(i,1)=180-heading;
                            end
                        end
                    elseif heading < 180 && heading > 90
                        if wall_mix(i,1) > new_ty && HOV(i) == 0 % horizon
                            x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                            if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                                distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                                collision_point(i,1)=x;
                                collision_point(i,2)=wall_mix(i,1);
                                new_heading(i,1)=360-heading;
                            end
                        elseif wall_mix(i,1) < new_tx && HOV(i) == 1 % vertical
                            y=new_ty+slope*(wall_mix(i,1)-new_tx);
                            if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                                distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                                collision_point(i,1)=wall_mix(i,1);
                                collision_point(i,2)=y;
                                new_heading(i,1)=180-heading;
                            end
                        end 
                    elseif heading < 270 && heading > 180
                        if wall_mix(i,1) < new_ty && HOV(i) == 0 % horizon
                            x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                            if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                                distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                                collision_point(i,1)=x;
                                collision_point(i,2)=wall_mix(i,1);
                                new_heading(i,1)=360-heading;
                            end
                        elseif wall_mix(i,1) < new_tx && HOV(i) == 1 % vertical
                            y=new_ty+slope*(wall_mix(i,1)-new_tx);
                            if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                                distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                                collision_point(i,1)=wall_mix(i,1);
                                collision_point(i,2)=y;
                                new_heading(i,1)=540-heading;
                            end
                        end 
                    elseif heading < 360 && heading > 270
                        if wall_mix(i,1) < new_ty && HOV(i) == 0 % horizon
                            x=(wall_mix(i,1)-new_ty)/slope+new_tx;
                            if x >= wall_mix(i,2) && x <= wall_mix(i,3)
                                distance(i)=sqrt((x-new_tx)^2+(wall_mix(i,1)-new_ty)^2);
                                collision_point(i,1)=x;
                                collision_point(i,2)=wall_mix(i,1);
                                new_heading(i,1)=360-heading;
                            end
                        elseif wall_mix(i,1) > new_tx && HOV(i) == 1 % vertical
                            y=new_ty+slope*(wall_mix(i,1)-new_tx);
                            if y >= wall_mix(i,2) && y <= wall_mix(i,3)
                                distance(i)=sqrt((wall_mix(i,1)-new_tx)^2+(y-new_ty)^2);
                                collision_point(i,1)=wall_mix(i,1);
                                collision_point(i,2)=y;
                                new_heading(i,1)=540-heading;
                            end
                        end 
                    end % end of if heading < 90 && heading > 0
                end % end of if heading == 0 && HOV(i) == 1
            end % end of for i=1:1:wall_index
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
            % compare distance for min_distance & LOS_distance
            if min_distance == 0 && LOS_distance == 0
                % Do not have any chance and break while loop;
                break;
            elseif (min_distance == 0 && LOS_distance ~= 0) || ...
                    (min_distance ~= 0 && LOS_distance ~= 0 && min_distance >= LOS_distance)
                accept_main_path_num=accept_main_path_num+1;
                if j == 0 % LOS and don't need to collision to any wall
                    collision_wall_record(accept_main_path_num,j+1)=0;
                    collision_count_record(accept_main_path_num)=0;
                    distance_record(accept_main_path_num)=LOS_distance;
                    AOD(accept_main_path_num,1)=angle;
                else
                    for k=1:1:length(temp_wall_record)
                        collision_wall_record(accept_main_path_num,k)=temp_wall_record(k);
                    end  
                    collision_count_record(accept_main_path_num)=j;
                    distance_record(accept_main_path_num)=temp_distance_record+LOS_distance;
                    AOD(accept_main_path_num,1)=angle;
                end
                break;
            elseif (min_distance ~= 0 && LOS_distance == 0) || ...
                    (min_distance ~= 0 && LOS_distance ~= 0 && min_distance < LOS_distance)
                % reflection procedure
                % change heading angle
                % change j
                % change new_tx, new_ty
                % plus distance
                % save collision wall # this round
                heading = new_heading(temp_wall_index);
                j=j+1;% WHY
                new_tx=collision_point(temp_wall_index,1);
                new_ty=collision_point(temp_wall_index,2);
                temp_count_record = temp_count_record+1;
                temp_wall_record(j)=temp_wall_index;
                temp_distance_record = temp_distance_record+distance(temp_wall_index);
                temp_count_record=temp_count_record+1;
            end % end of if min_distance == 0 && LOS_distance == 0
        end % end of while
    end % end of for
    
    %k=1;
    flag=0;
    count=0;
    % Get AoD's column number
    for k = 1:1:accept_main_path_num
    %while distance_record(k) ~= 0
        % if this flag == 1, the collision_wall_record(k,:)=collision_wall_record(k,:)
        record_flag = 1;
        temp_index = 1;
        while temp_index <= reflection_times
            if collision_wall_record(k+1,temp_index) ~= collision_wall_record(k,temp_index) ||...
                k ==  accept_main_path_num
                record_flag = 0;
            %else 
            %    record_flag = 0;
            end
            temp_index=temp_index+1;
        end
        if flag == 0 % first one
            if record_flag == 1
            %if abs(distance_record(k+1)-distance_record(k)) < precision
            %if abs(AOD(k+1,1)-AOD(k,1)) < 2 * precision
                flag = 2; % same
            else 
                count=count+1;
                flag = 1; % different
            end
        elseif flag == 1 % k-1 ~= k 
            if record_flag == 1
            %if abs(distance_record(k+1)-distance_record(k)) < precision
            %if abs(AOD(k+1,1)-AOD(k,1)) < 2 * precision
                flag = 2;
            else
                count=count+1;
                flag = 1;
            end
        elseif flag == 2 %k-1 == k
            if record_flag == 1
            %if abs(distance_record(k+1)-distance_record(k)) < precision
            %if abs(AOD(k+1,1)-AOD(k,1)) < 2 * precision
                flag = 2;
            else % save if different
                count=count+1;
                flag = 1;
            end
        end
        %k=k+1;
    end
    %count
    %k=1;
    start_AOD=0;
    start_distance=0;
    flag=0;
    index=0;
    if count == 0 % no any ray
        AoD=zeros(5,1);
        collision_wall=zeros(reflection_times,1);
        collision_count=zeros(1,1);
    else
        AoD=zeros(5,count);
        collision_wall=zeros(reflection_times,count);
        collision_count=zeros(1,count);
    end
    
    for k = 1:1:accept_main_path_num
    %while distance_record(k) ~= 0
        % if this flag == 1, the collision_wall_record(k,:)=collision_wall_record(k,:)
        record_flag = 1;
        temp_index = 1;
        while temp_index <= reflection_times
            if collision_wall_record(k+1,temp_index) ~= collision_wall_record(k,temp_index) ||...
                    k ==  accept_main_path_num
                record_flag = 0;
            %else 
            %    record_flag = 0;
            end
            temp_index=temp_index+1;
        end
        if flag == 0 % first one
            if record_flag == 1
            %if abs(distance_record(k+1)-distance_record(k)) < precision
            %if abs(AOD(k+1,1)-AOD(k,1)) < 2 * precision
                start_AOD = AOD(k,1);
                start_distance=distance_record(k);
                flag = 2; % same
            else 
                index=index+1;
                AoD(1,index)=AOD(k,1);
                AoD(3,index)=distance_record(k);
                for i=1:1:reflection_times
                    collision_wall(i,index)=collision_wall_record(k,i);
                end
                collision_count(1,index)=collision_count_record(k);
                flag = 1; % different
            end
        elseif flag == 1 % k-1 ~= k 
            if record_flag == 1
            %if abs(distance_record(k+1)-distance_record(k)) < precision
            %if abs(AOD(k+1,1)-AOD(k,1)) < 2 * precision
                start_AOD = AOD(k,1);
                start_distance=distance_record(k);
                flag = 2;
            else
                index=index+1;
                AoD(1,index)=AOD(k,1);
                AoD(3,index)=distance_record(k);
                for i=1:1:reflection_times
                    collision_wall(i,index)=collision_wall_record(k,i);
                end
                collision_count(1,index)=collision_count_record(k);
                flag = 1;
            end
        elseif flag == 2 %k-1 == k
            if record_flag == 1
            %if abs(distance_record(k+1)-distance_record(k)) < precision
            %if abs(AOD(k+1,1)-AOD(k,1)) < 2 * precision
                flag = 2;
            else % save if different
                index=index+1;
                AoD(1,index)=(start_AOD+AOD(k,1))/2;
                AoD(3,index)=(start_distance+distance_record(k))/2;
                for i=1:1:reflection_times
                    collision_wall(i,index)=collision_wall_record(k,i);
                end
                collision_count(1,index)=collision_count_record(k);
                flag = 1;
            end
        end
        %k=k+1;
    end
    % ========================= debug parameter ========================= %
    %AOD
    %collision_count_record
    %collision_wall_record
    %distance_record
    
    %AoD
    %collision_wall
    % ========================= debug parameter ========================= %
    
    % === According the cluster type A to F, get number of the cluster == %
    Ncluster = fc_give_num_of_cluster(2,count); 
    %Ncluster = min(6,count);
    % =================================================================== %
    
    % ============ Sort the distance from small to large ================ %
    [distance,index]=sort(AoD(3,:));
    % ======================== End of sorting =========================== %
    %index
    wall_in_count=zeros(reflection_times,Ncluster);
%     collision_wall
%     size(collision_wall)
%     index
    for i=1:1:reflection_times
        wall_in_count(i,1:1:Ncluster)=collision_wall(i,index(1:1:Ncluster));
    end
    
    collision_in_count(1,1:1:Ncluster)=collision_count(1,index(1:1:Ncluster));
    %wall_in_count
    %collision_in_count
end % end of function