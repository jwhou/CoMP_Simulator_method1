function [sampling_rate]=fc_give_channel_sampling_rate_expantion_factor(Bandwidth)

% By "Breit, G. er al. "TGac Channel Model Addendum." Doc. IEEE802.11-09/0308r1212",
% this function returns the channel sampling rate expansion factor based on
% the given bandwidth.
% The wider bandwidth, the faster sampling rate should be

switch Bandwidth
    case 40*10^6
        sampling_rate=1;
    case 80*10^6
        sampling_rate=2;
    case 160*10^6
        sampling_rate=4;
    case 320*10^6
        sampling_rate=8;
    case 640*10^6
        sampling_rate=16;
    otherwise
        sampling_rate=32;
end

end