%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DrawEnvironment(Num_AP, Num_User, XY_location,  AP_distribution, User_distribution, density_array, area_density)
% Inputs:
%   Num_AP: the number of APs in the network
%   Num_User: the number of users in the network
%   XY_location: APs+users (x,y)
%   AP_distribution: the AP distribution models 
%                    [-1: Fixed AP location, 0: Square grid, 1:Hexagonal grid, 2: PPP]
%   User_distribution: the user distribution models
%                      [-1: Fixed users, 0: Users around AP, 2: PPP]
%   density_array = [lambda_AP, lambda_User]; [number of AP per m^2 area if AP_distribution = 2, number of AP per m^2 area if User_distribution = 2]
%   area_array = [area_X, area_Y]; unit is meter if AP_distribution | User_distribution = 2

global STA;
global cover_range;

h = figure(1);
clf(h)
% set(h, 'Visible', 'off')
hold on
% Illustrate the environment
numSTAs = Num_AP+ Num_User;
colorMAP = linspace(1,100,Num_AP);
size = 36; % default
for i=1:numSTAs
    if (i <= Num_AP)
        scatter(XY_location(i, 1), XY_location(i, 2), size, colorMAP(i), '^', 'filled', 'MarkerEdgeColor', 'k');
        if (STA(i, 3) ~= -1)
            if (AP_distribution == -1)
                text(XY_location(i, 1)-4, XY_location(i, 2)-5, num2str(i, 'AP%d'), 'FontSize', 7);
            elseif (AP_distribution == 2)
                text(XY_location(i, 1)-8, XY_location(i, 2)-11, num2str(i, 'AP%d'), 'FontSize', 6);
            end
        end
    else
        if (STA(i, 3) ~= -1)
            scatter(XY_location(i, 1), XY_location(i, 2), size-16, colorMAP(STA(i,3)), 'filled', 'MarkerEdgeColor', 'k');
            if (AP_distribution == -1)
                text(XY_location(i, 1)-6, XY_location(i, 2)-5, num2str(i, 'STA%d'), 'FontSize', 7);
            elseif (AP_distribution == 2)
                text(XY_location(i, 1)-12, XY_location(i, 2)-11, num2str(i, 'STA%d'), 'FontSize', 6);
            end
        else
            scatter(XY_location(i, 1), XY_location(i, 2), size-16, 'k');
        end
    end
end
if (Num_AP >= 3)
    [XV YV] = voronoi(XY_location(1:Num_AP, 1), XY_location(1:Num_AP,2)); plot(XV,YV,'b-')
end
if (AP_distribution == -1) % fixed AP
    if (User_distribution == -1)
        max_x_unit = ceil(max(XY_location(:, 1))/cover_range)*cover_range;
        max_y_unit = ceil(max(XY_location(:, 2))/cover_range)*cover_range;
        axis([0, max_x_unit, 0, max_y_unit])
    else
        max_x_unit = max(XY_location(1:Num_AP, 1))+cover_range;
        max_y_unit = max(XY_location(1:Num_AP, 2))+cover_range;
        axis([0, max_x_unit, 0, max_y_unit])
        set(gca, 'XTick', 0:50:max_x_unit)
        set(gca, 'YTick', 0:cover_range:max_y_unit)
        if (User_distribution == 0)
            title(['£l_u_s_e_r = ', num2str(density_array(2)), ' (1/m^2) in each AP cover area'])
        elseif (User_distribution == 2)
            title(['£l_u_s_e_r = ', num2str(density_array(2)), ' (1/m^2)'])   
        end
    end
elseif (AP_distribution == 2) 
    if (User_distribution == 0)
        max_x_unit = ceil(max(XY_location(:, 1))/cover_range)*cover_range;
        max_y_unit = ceil(max(XY_location(:, 2))/cover_range)*cover_range;
        min_x_unit = floor(min(XY_location(:, 1))/cover_range)*cover_range;
        min_y_unit = floor(min(XY_location(:, 2))/cover_range)*cover_range;
        title({['£l_A_P = ', num2str(density_array(1)), ' in ', num2str(area_density(1)), 'x', num2str(area_density(2)), ' (m^2)'],...
            ['£l_u_s_e_r = ', num2str(density_array(2)), ' (1/m^2) in each AP cover area']})
        axis([min_x_unit, max_x_unit, min_y_unit, max_y_unit])
    elseif (User_distribution == 2)
        axis([0, area_density(1), 0, area_density(2)])
%         % Because APs and Users have shifted (+cover_range, +cover_range), need to set the gca
%         axis([cover_range, area_density(1)+cover_range, cover_range, area_density(2)+cover_range])
%         set(gca, 'XTick', cover_range:cover_range:area_density(1)+cover_range)
%         set(gca, 'YTick', cover_range:cover_range:area_density(2)+cover_range)
%         set(gca, 'XTickLabel', 0:cover_range:area_density(1))
%         set(gca, 'YTickLabel', 0:cover_range:area_density(2))
        title(['£l_A_P = ', num2str(density_array(1)), ' (1/m^2) and £l_u_s_e_r = ', num2str(density_array(2)), ' (1/m^2)'])
    end
end
xlabel('X-Axis Position (m)')
ylabel('Y-Axis Position (m)')
hold off
% saveas(h, ['Network Topology.png'])
clear h;

end

