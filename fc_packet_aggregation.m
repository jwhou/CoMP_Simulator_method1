%{
Author:      Cheng-ming Chuang
Date:        2017/8/27
input:       max_aggregated_packets
             data_queue
output:      max_send_packets(after packets aggregation)
             sort_packets
Description: Packets aggregation
%}
function [num_aggregation,num_msdu] = fc_packet_aggregation(packet_send_rate,tx,select_rv)
        
global traffic_queue queue_size basic_rate size_MAC_body size_MAC_header;
global STA STA_Info;
% ======================================================================= %
% As possibile as to aggregate packets                                    %
% (max_send_packets <= max_aggregated_packets)                            %
% ======================================================================= %
i = tx;
j = select_rv;
max_ppdu_size = 5.484e-3; %5.484ms
num_aggregation = -1*ones(length(j),1);
time_mpdu = -1*ones(length(j), 1);
mpdu_size = 11454*8; %MPDU size (bit)
FCS_size=4*8;
traffic_size = zeros(1, length(j));

for k=1:length(j)
    if (STA(i, 3) == 0)  
        AP_index = STA(j(k), 3); 
        traffic_size(k) = traffic_queue(AP_index).size(STA_Info(AP_index).associated_STA==j(k));
    end
end
num_msdu = floor((mpdu_size-FCS_size-size_MAC_header)/size_MAC_body); %1 MPDU裡有多少個MSDU
time_mpdu = ((size_MAC_header+FCS_size)/basic_rate)+(num_msdu*size_MAC_body./packet_send_rate); %傳1 MPDU所需花的時間
num_aggregation = floor((max_ppdu_size./time_mpdu));
if(num_aggregation<1)
    num_aggregation=1;
end

if (STA(i, 3) == 0)
   num_aggregation = min(traffic_size.' , num_aggregation);
else % if (STA(tx, 3) ~= 0)
   num_aggregation = min(queue_size(i), num_aggregation);
end
% ======================= End of aggregated packet ====================== %
end