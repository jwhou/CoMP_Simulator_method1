%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References: 
%	1.http://wireless-matlab.sourceforge.net/
% Reverse Version by jing-wen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Pt = friisR(Pr, Gt, Gr, lambda, L, d)

% Friis free space propagation model:
%        Pt * Gt * Gr * (lambda^2)
%  Pr = --------------------------
%        (4 *pi * d)^2 * L

M = lambda / (4 * pi * d);
Pt = (L * Pr) / (Gt * Gr * (M * M));

return;