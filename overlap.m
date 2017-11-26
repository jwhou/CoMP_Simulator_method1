%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [out] = overlap(a, b, c, d)

% check if [a, b] and [c, d] overlap

out = 0;
if (a>c && a<d), out = 1; return; end
if (b>c && b<d), out = 1; return; end
if (c>a && c<b), out = 1; return; end
if (d>a && d<b), out = 1; return; end

return;    