%{
Author:      Wen-Pin Hsu
Date:        2016/6/7
Input:       wall_index: Number of wall
             wall_start
             wall_end

             tx is the term "x_of_transmitter" in the Main function
             ty is the term "y_of_transmitter" in the Main function

Output:      wall_mix: wall input data
             HOV       : 0: Horizon
                         1: Vertical
                         2: Slash
             wall_x_range = [min_x max_x]
             wall_y_range = [min_y max_y]
             
Description: 
       part1:First, we want to let the input in the regular type 
             wall_1_start_x <= wall_1_end_x
             wall_1_start_y <= wall_1_end_y
             Second, we want to check the line is horizon or vertical
             if is horizon line, HOV(wall_index) = 0
             if is vertical line, HOV(wall_index) = 1
             if is slash line, HOV(wall_index) = 2
       part2:Then we want to calculate the room size, so we check the
             wall's maximum and minimum of x and y 
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
function [wall_mix,HOV,wall_x_range,wall_y_range] = fc_parse_wall_data(wall_index,wall_start,wall_end)

HOV=zeros(1,wall_index);
wall_mix=zeros(wall_index,3);
wall_x_range=[10000 -10000];% wall x level [ min max ]
wall_y_range=[10000 -10000];% wall y level [ min max ]

% ============================== Part 1 ================================= %
for i=1:1:wall_index
    % horizon line
    if wall_start(i,1) ~= wall_end(i,1) &&...
       wall_start(i,2) == wall_end(i,2)
        % change if input wall_x_start > wall_x_end
        if wall_start(i,1) > wall_end(i,1)
            temp = wall_end(i,1);
            wall_end(i,1)=wall_start(i,1);
            wall_start(i,1)=temp;
        end
        wall_mix(i,1)=wall_start(i,2);% y level
        wall_mix(i,2)=wall_start(i,1);% x start
        wall_mix(i,3)=wall_end(i,1);  % x end
        HOV(i)=0;
    % vertical line    
    elseif wall_start(i,1) == wall_end(i,1) &&...
       wall_start(i,2) ~= wall_end(i,2)
        % change if input wall_y_start > wall_y_end
        if wall_start(i,2) > wall_end(i,2)
            temp = wall_end(i,2);
            wall_end(i,2)=wall_start(i,2);
            wall_start(i,2)=temp;
        end
        wall_mix(i,1)=wall_start(i,1);% x level
        wall_mix(i,2)=wall_start(i,2);% y start
        wall_mix(i,3)=wall_end(i,2);  % y end
        HOV(i)=1;
    % slash
    elseif wall_start(i,1) ~= wall_end(i,1) &&...
        wall_start(i,2) ~= wall_end(i,2)
        HOV(i)=2;
    end
end
% =========================== End of part 1 ============================= %

% =============================== Part 2 ================================ % 
for i=1:1:wall_index
    if HOV(i)==0 % horizon
        % check y level
        if wall_mix(i,1) > wall_y_range(2)
            wall_y_range(2) = wall_mix(i,1);
        end
        if wall_mix(i,1) < wall_y_range(1)
            wall_y_range(1) = wall_mix(i,1);
        end
        % check x start & x end
        if wall_mix(i,2) < wall_x_range(1)
            wall_x_range(1) = wall_mix(i,2);
        end
        if wall_mix(i,3) > wall_x_range(2)
            wall_x_range(2) = wall_mix(i,3);
        end 
    else % vertical
        % check x level
        if wall_mix(i,1) > wall_x_range(2)
            wall_x_range(2) = wall_mix(i,1);
        end
        if wall_mix(i,1) < wall_x_range(1)
            wall_x_range(1) = wall_mix(i,1);
        end
        % check y start & y end
        if wall_mix(i,2) < wall_y_range(1)
            wall_y_range(1) = wall_mix(i,2);
        end
        if wall_mix(i,3) > wall_y_range(2)
            wall_y_range(2) = wall_mix(i,3);
        end
    end
end
% =========================== End of part 2 ============================= %
end % end of function

