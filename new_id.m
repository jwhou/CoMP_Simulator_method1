%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [newid] = new_id(i)
% return a new id for node i

global Num_AP Num_User packet_id;

newid = 0;
if i<=0 || i>(Num_AP + Num_AP * Num_User), return; end
packet_id(i) = packet_id(i) + 1;
newid = packet_id(i);

return;
