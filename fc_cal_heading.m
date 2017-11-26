%{
Author:      Wen-Pin Hsu
Date:        2016/6/1
Input:       tx
             ty
             rx
             ry
Output:      heading
Description: Given a Tx position Rx position, the function can output the
             heading angle from Tx to Rx (0-359)
%}
function [heading] = fc_cal_heading(tx,ty,rx,ry)
    if ty == ry % Tx & Rx horizon
        if tx <= rx % Rx at right side
            heading=0;
        else
            heading=180;
        end
    elseif tx == rx
        if ty <= ry % Rx at upper side
            heading=90;
        else
            heading=270;
        end
    else % at 1-4 quadrant
        angle=rad2deg(atan((ry-ty)/(rx-tx)));
        if angle > 0 % 1 or 3 quadrant
            if ry > ty && rx > tx % 1 quadrant
                heading = angle;
            else % 3 quadrant 
                heading = angle+180;
            end
        else % 2 or 4 quadrant
%             rx
%             ry
%             tx
%             ty
            if ry > ty && rx < tx % 2 quadrant
                heading = angle+180;
            else % 4 quadrant
                heading = angle+360;
            end
        end
    end
end

