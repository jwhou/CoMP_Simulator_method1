%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function CoMP_Controller = CoMP_Pair_Algo(Num_AP, Num_User, XY_location, CoMP_Mode, AP_distribution, density_array, area_density)
% Inputs:
%   Num_AP: the number of APs in the network
%   Num_User: the number of users in the network
%   XY_location: APs+users (x,y)
%   CoMP_Mode: Networked MIMO is 0, Coordinated Scheduling/Coordinated Beamforming is 1
%   AP_distribution: the AP distribution models 
%                    [-1: Fixed AP location, 0: Square grid, 1:Hexagonal grid, 2: PPP]
%   density_array = [lambda_AP, lambda_User]; [number of AP per m^2 area if AP_distribution = 2, number of AP per m^2 area if User_distribution = 2]
%   area_array = [area_X, area_Y]; unit is meter if AP_distribution | User_distribution = 2

global all_STA_distance;
global def_CoMP_L cover_range;
global STA STA_Info;
global ClusterSize;

CoMP_Controller = [];

numSTAs = Num_AP+ Num_User;
h = figure(2);
clf(h)
set(h, 'Visible', 'on')
hold on
% Illustrate the environment
counter = 0;
active_AP_array = [];
for i=1:numSTAs
    if (i <= Num_AP)
        scatter(XY_location(i, 1), XY_location(i, 2), '^', 'filled', 'k');
        if (STA(i, 3) ~= -1)
            if (AP_distribution == -1 || AP_distribution == -2)
                text(XY_location(i, 1)-4, XY_location(i, 2)-5, num2str(i, 'AP%d'), 'FontSize', 7);
            elseif (AP_distribution == 2)
                text(XY_location(i, 1)-8, XY_location(i, 2)-10, num2str(i, 'AP%d'), 'FontSize', 7);
            end
            counter = counter + 1;
            gplot_node(counter, :) = [XY_location(i, 1), XY_location(i, 2)];
            gplot_node_ind(counter) = i;
            active_AP_array(counter) = i;
        end
    end
end

if (isempty(def_CoMP_L))
    def_CoMP_L = 0;
end
if (counter ~= 0)
    [m, ~] = size(gplot_node);
    % The distances of all active APs to all active APs
    distance_temp = all_STA_distance(gplot_node_ind, gplot_node_ind); 
    adjMatrix = zeros(m, m);
    temp = (distance_temp <= def_CoMP_L) - eye(m,m);
    tril_adjM_elements = tril(temp);
    Num_tril_elements = sum(sum(tril_adjM_elements));
    for i=1:Num_tril_elements
        if (sum(sum(tril_adjM_elements)) <= 0)
            break;
        end
        [r, c] = ind2sub(size(tril_adjM_elements), find(tril_adjM_elements==1));
        distance_buf = distance_temp(tril_adjM_elements == 1);
        [~, min_ind] = min(distance_buf);
        AP1_ind = r(min_ind); AP1 = active_AP_array(AP1_ind);
        AP2_ind = c(min_ind); AP2 = active_AP_array(AP2_ind);
        STA_Info(AP1).CoMP_coordinator = AP2; STA_Info(AP2).CoMP_coordinator = AP1;
%         def_JPuser_D = 40
%         % Classify JP users for AP1 from associated AP1 users
%         STA_Info(AP1).ableCoMP_STA = STA_Info(AP1).associated_STA(max([all_STA_distance(AP1, STA_Info(AP1).associated_STA);all_STA_distance(AP2, STA_Info(AP1).associated_STA)]) <= def_JPuser_D);
%         % Classify JP users for AP2 from associated AP2 users
%         STA_Info(AP2).ableCoMP_STA = STA_Info(AP2).associated_STA(max([all_STA_distance(AP1, STA_Info(AP2).associated_STA);all_STA_distance(AP2, STA_Info(AP2).associated_STA)]) <= def_JPuser_D);
        tril_adjM_elements(AP1_ind, :) = 0;tril_adjM_elements(:, AP1_ind) = 0; % Symmetric property
        tril_adjM_elements(:, AP2_ind) = 0;tril_adjM_elements(AP2_ind, :) = 0;
        adjMatrix(AP1_ind, AP2_ind) = 1;adjMatrix(AP2_ind, AP1_ind) = 1;
    end
    gplot(adjMatrix, gplot_node, 'r-');
    set(findall(gcf,'type','line'),'LineWidth',2)
    CoMP_Controller.connector = [1:Num_AP];
    CoMP_Controller.CoMP_tx_mode = CoMP_Mode; 
    CoMP_Controller.informer_index = zeros(1, length(CoMP_Controller.connector));
    for i=1:Num_AP
        CoMP_Controller.information(i).numInform = 0;
        CoMP_Controller.information(i).startorder = 0;
        CoMP_Controller.information(i).lastsounding = -1;   %new
        CoMP_Controller.information(i).waitsounding = 0;    %new
        CoMP_Controller.information(i).readyRTS = 0;        %new
        CoMP_Controller.information(i).doneData = 0;        %new
        CoMP_Controller.information(i).reset = 0;        %new
        CoMP_Controller.information(i).doneSounding = 0;    %new for method 2
    end
end
if (AP_distribution == -1 || AP_distribution == -2)
    max_x_unit = max(XY_location(1:Num_AP, 1))+cover_range;
    max_y_unit = max(XY_location(1:Num_AP, 2))+cover_range;
    axis([0, max_x_unit, 0, max_y_unit])
    title('CoMP pairs in a network topology')
elseif (AP_distribution == 2)
    axis([0, area_density(1), 0, area_density(2)])
    title(['CoMP pairs with £l_A_P = ', num2str(density_array(1)), ' (1/m^2) in ', num2str(area_density(1)), 'x', num2str(area_density(2)), ' (m^2)'])
end

xlabel('X-Axis Position (m)')
ylabel('Y-Axis Position (m)')
hold off
% saveas(h, 'CoMP Sectors.png')
clear h;
end

