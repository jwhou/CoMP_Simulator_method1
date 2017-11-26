%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References: 
%	1.http://wireless-matlab.sourceforge.net/
% Reverse Version by jing-wen
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Pt = log_normal_shadowingR (d)
global Gt Gr L d0 pathLossExp freq;
global Pr_edge;  %added by jing-wen

%remove random term baout powerLoss_db
lambda = 3e8 / freq;
avg_db = -10.0 * pathLossExp * log10(d/d0);
powerLoss_db = avg_db;
Pr0 = Pr_edge / (10^(powerLoss_db/10));
Pt = friisR(Pr0, Gt, Gr, lambda, L, d0);

return;