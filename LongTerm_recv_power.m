%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Pr] = LongTerm_recv_power(tx, rv, rmodel)
% send packet at PHY layer
global STA;
global Gt Gr freq L ht hr pathLossExp std_db d0;
global default_power;
lambda = 3e8 / freq;
Pt = default_power;

d = sqrt((STA(tx, 1)-STA(rv, 1))^2+(STA(tx, 2)-STA(rv, 2))^2);
d(d == 0) = 0.001;
switch rmodel
    case 'friis'
        Pr = friis(Pt, Gt, Gr, lambda, L, d);
    case 'tworay'
        [Pr, crossover_dist] = tworay(Pt, Gt, Gr, lambda, L, ht, hr, d);
    case 'shadowing'
        Pr = log_normal_shadowing(Pt, Gt, Gr, lambda, L, pathLossExp, std_db, d0, d);
end

return;
