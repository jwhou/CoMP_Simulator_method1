%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [STA_Info, STA] = Init_STAInfo(Num_AP, NumUserVec, XY_location, traffic_type, traffic_rate, envUsermodel)
% Outputs:
%   STA_Info: a structure is used to record infomation of association STA
%   STA: a (Num_AP+Num_User)*3 matrix records location x, location y, associated AP
% Inputs:
%   Num_AP: the number of APs in the network
%   Num_User: the number of users in the network
%   XY_location: APs+users (x,y)
global Num_Tx Num_Rx;
global all_STA_distance;
global cover_range;
global slotTime;
global soundingperiod;
global traffic_queue;
Num_User = sum(NumUserVec);
numSTAs = Num_AP+Num_User;
% The distances of all STAs to all STAs
for i=1:numSTAs
    X = XY_location(i, 1);
    Y = XY_location(i, 2);
    all_STA_distance(:, i) = sqrt((X - XY_location(:, 1)).^2 + (Y - XY_location(:, 2)).^2);
end

STA = [XY_location, zeros(numSTAs, 2)];
% Initialize the structure of each STA, STA_Info
if (envUsermodel == 0), 
    AP_index = 1;
    index_MAP = [];
    for AP_index=1:length(NumUserVec)
        if (NumUserVec(AP_index) ~= 0)
            index_MAP = [index_MAP, ones(1,NumUserVec(AP_index))*AP_index];
        end
    end
end
for i=1:numSTAs
    STA_Info(i).index = i; % STA ID
    STA_Info(i).X = XY_location(i,1); % STA X location
    STA_Info(i).Y = XY_location(i,2); % STA Y location
    STA_Info(i).IsAP = 0; % 1: AP STA; 0: user STA
    STA_Info(i).CoMP_coordinator = []; % AP STA might have CoMP transmission partner, otherwise null
    STA_Info(i).active = 0; % The STA is active if it has association with other STA
    % Association algorithm based on distance matrix
    % a user finds the nearest AP for association and in the coverage.
    STA_Info(i).cover_STA = [];
    STA_Info(i).edge_STA = [];    % Added by jing-wen
    STA_Info(i).center_STA = [];  % Added by jing-wen
    STA_Info(i).num_ass = 0; % The number of association for current STA
    STA_Info(i).associated_STA = [];
    STA_Info(i).ableCoMP_STA = [];
    STA_Info(i).Channel_Matrix = [];
    STA_Info(i).Precoding_Matrix = [];
    % For each STA, assume it know who can hear it
    STA_Info(i).cover_STA = (all_STA_distance(i,:) <= cover_range) .* (1:numSTAs);
    STA_Info(i).cover_STA(STA_Info(i).cover_STA == 0) = []; % Clear no cover STA
    STA_Info(i).cover_STA(STA_Info(i).cover_STA == i) = []; % Clear self
    if (i <= Num_AP)
        STA_Info(i).IsAP = 1;
        STA_Info(i).PER_record = ones(1, Num_User); %added by jing-wen
        STA_Info(i).MCS_record = ones(1, Num_User); %added by jing-wen
        STA_Info(i).Channel_Matrix = zeros(numSTAs*Num_Rx, Num_Tx);
        STA_Info(i).Precoding_Matrix = zeros(Num_Tx, numSTAs*Num_Rx);
        STA_Info(i).SNR_record = zeros(1, numSTAs);
        STA_Info(i).CSI_TS = -1*soundingperiod*ones(1, Num_User); % Time stamp of the CSI
    else
        STA_Info(i).SNR_record = zeros(1, Num_AP);
        STA_Info(i).CSI_TS = [];
        traffic_queue(i).list = 0;
        if (envUsermodel == 0)
            % Trace here as a user STA view
            AP_index = index_MAP(i-Num_AP);
            STA(i, 3) = AP_index;
            % A user STA's association should update STA_Info structure of the associated AP  
            STA_Info(AP_index).num_ass = STA_Info(AP_index).num_ass + 1; 
            STA_Info(AP_index).associated_STA(STA_Info(AP_index).num_ass) = i;
            traffic_queue(AP_index).size(STA_Info(AP_index).num_ass) = 0;
            % A user STA's association should also update STA_Info structure of itself
            STA_Info(i).num_ass = 1;
            STA_Info(i).associated_STA(STA_Info(i).num_ass) = AP_index;
        else
            % Trace here as a user STA view
            [value, index] = min(all_STA_distance(1:Num_AP,i));
            if (value <= cover_range)
                STA(i, 3) = index;
                % A user STA's association should update STA_Info structure of the associated AP  
                STA_Info(index).num_ass = STA_Info(index).num_ass + 1; 
                STA_Info(index).associated_STA(STA_Info(index).num_ass) = i;
                traffic_queue(index).size(STA_Info(index).num_ass) = 0;
                % A user STA's association should also update STA_Info structure of itself
                STA_Info(i).num_ass = 1;
                STA_Info(i).associated_STA(STA_Info(i).num_ass) = index;
            end
        end
    end
end

for i=1:Num_AP    %added by jing-wen for check edge or center STA
    for k=1:length(STA_Info(i).associated_STA)
        AP_index = STA_Info(i).associated_STA(k);
        if sum(STA_Info(AP_index).cover_STA <= Num_AP) > 1
            STA_Info(i).edge_STA = [STA_Info(i).edge_STA, AP_index];
        else
            STA_Info(i).center_STA = [STA_Info(i).center_STA, AP_index];
        end
    end
end

for i=1:numSTAs
    if (~isempty(STA_Info(i).associated_STA))
        STA_Info(i).active = 1;
    else
        STA(i, 3) = -1;
    end
    % Generate traffic arrival rate lambda, arrivals/second
    if (traffic_type == 0) % DL
        if (STA(i, 3) ~= -1 && STA(i, 3) == 0)
            if (traffic_rate ~= 0) 
                STA(i, 4) = traffic_rate;
            else
                STA(i, 4) = (1/slotTime);%/(STA_Info(i).num_ass+1);
            end
        end
    elseif (traffic_type == 1) % UL
        if (STA(i, 3) ~= -1 && STA(i, 3) ~= 0)
            if (traffic_rate ~= 0) 
                STA(i, 4) = traffic_rate;
            else
                STA(i, 4) = 1/slotTime;
            end
        end
    elseif (traffic_type == 2) % only AP has sturate traffic and assign non-AP STA traffic 
        if (STA(i, 3) ~= -1 && STA(i, 3) == 0) % AP
            STA(i, 4) = 1/slotTime;
        elseif (STA(i, 3) ~= -1 && STA(i, 3) ~= 0) % non-AP STA
            STA(i, 4) = traffic_rate;
        end
    elseif (traffic_type == 3) % all STAs have saturate traffic
        if (STA(i, 3) ~= -1)
            if (traffic_rate ~= 0) 
                STA(i, 4) = traffic_rate;
            else
                STA(i, 4) = 1/slotTime;
            end
        end
    end
end

end

