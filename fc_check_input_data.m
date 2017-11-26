%{
Author:      Wen-Pin Hsu
Date:        2016/6/13
Input:       All initialize parameter
Output:      wall_index
Description: In this function, we what to check if there are some input
             error. If error occurs, there are some error message and break
             the process
%}
function [wall_index] = fc_check_input_data(Npkt,Npkt_length,AI,...
    MCS_in_beamtracking,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,...
    time,wall_start,wall_end)


if Npkt <= 0
    display('Npkt Error in fc_input_initialize_parameter.m');
    quit cancel;
end
if Npkt_length <= 0
    display('Npkt Length Error in fc_input_initialize_parameter.m');
    quit cancel;
end
for i=1:1:length(AI)
    if AI(i) < -5 || AI(i) > 5
        display('AI Input Error in fc_input_initialize_parameter.m');
        quit cancel;
    end
end
if MCS_in_beamtracking ~= 5
    display('MCS Mode for beamtracking is not setting to 5 in fc_input_initialize_parameter.m');
    quit cancel;
end
%if MCS_in_cutting > 9 || MCS_in_cutting < 0
%    display('MCS Mode for cutting Error in fc_input_initialize_parameter.m');
%    quit cancel;
%end
if length(tx_x_pos) ~= length(tx_y_pos)
    display('Tx Input Error in fc_input_initialize_parameter.m');
    quit cancel;
end
if length(rx_x_pos) ~= length(rx_y_pos)
    display('Rx Input Error in fc_input_initialize_parameter.m');
    quit cancel;
end
if time <= 0
    display('Time Setting Error in fc_input_initialize_parameter.m');
    quit cancel;
end
if length(wall_start) ~= length(wall_end)
    display('wall Input Error in fc_input_initialize_parameter.m');
    quit cancel;
end

wall_index=length(wall_start);
break_flag=0;


for i=1:1:wall_index
    if wall_start(i,1) == wall_end(i,1) &&...
       wall_start(i,2) == wall_end(i,2)
        errmsg1 = 'Input Wall Index ';
        errmsg2 = int2str(i);
        errmsg3 = ' start and end are the same point in fc_input_initialize_parameter.m';
        errmsg  = [errmsg1 errmsg2 errmsg3];
        display(errmsg);
        break_flag=1;
        break;
    end
end
if break_flag == 1
    quit cancel;
end


end % end of function

