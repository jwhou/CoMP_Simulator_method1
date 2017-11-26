%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function busy = carrier_sense(rv)
% check if the channel is busy

global STA;
global rmodel white_noise_variance cs_threshold;

Pr = 0;
I = find(STA(:, 5)>0);
for i=1:length(I)
   tx1 = I(i);
   if tx1 == rv, continue; end
   Pr = Pr + recv_power(tx1, rv, rmodel);
end
% N0 = abs(random('norm', 0, white_noise_variance));
N0 = white_noise_variance;

% Pr+N0

if (Pr+N0) > cs_threshold %Pr + white_noise_variance;
    busy = 1;
else
    busy = 0;
end

return;