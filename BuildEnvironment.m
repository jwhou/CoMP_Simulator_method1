%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Num_AP, NumUser, XY_location, Heterogeneous,tx_x_pos,tx_y_pos,rx_x_pos,rx_y_pos,wall_start,wall_end] = BuildEnvironment(AP_distribution, AP_distance, User_distribution, density_array, area_array, Heterogeneous)
% Outputs:
%   Num_AP: the number of APs in the network
%   NumUser: the number of users vector of each AP if User_distribution = -1
%            the number of users in the network 
%   XY_location: APs+users (x,y)
% Inputs:
%   AP_distribution: the AP distribution models 
%                    [-1: Fixed AP location, 0: Square grid, 1:Hexagonal grid, 2: PPP]
%   AP_distance: It is for AP_distribution = 0 | 1
%   User_distribution: the user distribution models
%                      [-1: Fixed users, 0: Users around AP, 2: PPP]
%   density_array = [lambda_AP, lambda_User]; 
%                   [# AP per m^2 area if AP_distribution = 2, # user per m^2 area if User_distribution = 0 | 2]
%   area_array = [area_X, area_Y]; unit is meter if AP_distribution = User_distribution = 2
%   Heterogeneous = [1:802.11ac + 802.11ad , 2: 802.11ad + LTE , 3:802.11ad ]
global shift;
global cover_range;

cover_range = 50;
R = cover_range; % please fill cover_range form parameter.m
lambda_AP = density_array(1);
lambda_User = density_array(2);
area_X = area_array(1);
area_Y = area_array(2);
Heterogeneous = 1; % [1:802.11ac + 802.11ad , 2: 802.11ad + LTE , 3:802.11ad ]

if(AP_distribution == -2)
    Num_AP = 4;
    AP_location = [
        40,40; % AP1
         100,40; % AP2
        40,100; % AP3
        100,100; % AP4
        ];
    if (User_distribution == -2)
        AP_STA_dis = 30;
        User_location = [
            AP_location(1,1)+AP_STA_dis*cosd(105),AP_location(1,2)+AP_STA_dis*sind(105);
            AP_location(1,1)+AP_STA_dis*cosd(225),AP_location(1,2)+AP_STA_dis*sind(225);
            AP_location(1,1)+AP_STA_dis*cosd(345),AP_location(1,2)+AP_STA_dis*sind(345);
            
            AP_location(2,1)+AP_STA_dis*cosd(75),AP_location(2,2)+AP_STA_dis*sind(75);
            AP_location(2,1)+AP_STA_dis*cosd(195),AP_location(2,2)+AP_STA_dis*sind(195);
            AP_location(2,1)+AP_STA_dis*cosd(315),AP_location(2,2)+AP_STA_dis*sind(315);
            
            AP_location(3,1)+AP_STA_dis*cosd(15),AP_location(3,2)+AP_STA_dis*sind(15);
            AP_location(3,1)+AP_STA_dis*cosd(135),AP_location(3,2)+AP_STA_dis*sind(135);
            AP_location(3,1)+AP_STA_dis*cosd(255),AP_location(3,2)+AP_STA_dis*sind(255);
            
            AP_location(4,1)+AP_STA_dis*cosd(45),AP_location(4,2)+AP_STA_dis*sind(45);
            AP_location(4,1)+AP_STA_dis*cosd(165),AP_location(4,2)+AP_STA_dis*sind(165);
            AP_location(4,1)+AP_STA_dis*cosd(285),AP_location(4,2)+AP_STA_dis*sind(285);
%             40,50;
%             50,40;
%             50,60;
%             60,50;
%             
%             95,50;
%             105,40;
%             105,60
%             115,50;
%             
%             40,110;
%             50,100;
%             50,120;
%             60,110;
%             
%             95,110;
%             105,100;
%             105,120
%             115,110;
            ];
          NumUser = 12;
    end


elseif (AP_distribution == -1)
%     Num_AP =2;
%     AP_location = [100,100; 40+100,100;];
     Num_AP = 1;
     AP_location = [100,100;];
    if (User_distribution == -1)
        shift2 = 10;
        shift = 40;
        User_location = [
        
             100+shift2*cos(pi/4),100+shift2*sin(pi/4);
             100+shift2,100;
             100+shift2*cos(pi/4),100-shift2*sin(pi/4);
             100-shift2,100;
%  
%             shift+100-shift2*cos(pi/4),100+shift2*sin(pi/4);
%             shift+100-shift2,100;
%             shift+100-shift2*cos(pi/4),100-shift2*sin(pi/4);
%             shift+100+shift2,100;
            ];
%           NumUser = 2;
         NumUser = 4;%debuger
    elseif (User_distribution == 0)
        if (lambda_User == 0)
            error('Please check tho_user ~= 0');
        end
        NumUser_Vector = random('Poisson', pi*R^2*lambda_User*ones(Num_AP,1));

        User_location = []; 
        for i=1:Num_AP
            r = R*sqrt(rand(NumUser_Vector(i),1));
            theta = 2*pi*rand(NumUser_Vector(i),1);
            User_location = [User_location; zeros(NumUser_Vector(i),1)+AP_location(i,1)+r.*cos(theta), zeros(NumUser_Vector(i),1)+AP_location(i,2)+r.*sin(theta)];
        end
        NumUser = NumUser_Vector;

    elseif (User_distribution == 2)
        if (lambda_User == 0)
            error('Please check tho_user ~= 0 and Area ~= 0');
        end
        area_X = max(AP_location(1:Num_AP, 1))+R;
        area_Y = max(AP_location(1:Num_AP, 2))+R;
        NumUser_Vector = random('Poisson', lambda_User*area_X*area_Y);
        User_point = rand(NumUser_Vector, 2);
        User_location = [User_point(:,1)*area_X, User_point(:,2)*area_Y];
        NumUser = NumUser_Vector;
    else
         error('No this kind of (AP, user) distribution, please see BuildEnvironment.m');
    end
elseif (AP_distribution == 2) % Poisson point process for varied number of APs (PPP model)
        if (lambda_AP == 0 || area_X == 0 || area_Y == 0)
            error('Please check tho_AP ~= 0 and Area ~= 0');
        end
        % Generate 2D poisson process
        Num_AP = random('Poisson', area_X*area_Y*lambda_AP); % The number of points to generate
        AP_point = rand(Num_AP, 2);
        AP_X = AP_point(:,1)*area_X;
        AP_Y = AP_point(:,2)*area_Y;
        AP_location = [AP_X, AP_Y];
        if (User_distribution == 0)
            if (lambda_User == 0)
                error('Please check tho_user ~= 0');
            end
            NumUser_Vector = random('Poisson', pi*R^2*lambda_User*ones(Num_AP,1));

            User_location = []; 
            for i=1:Num_AP
                r = R*sqrt(rand(NumUser_Vector(i),1));
                theta = 2*pi*rand(NumUser_Vector(i),1);
                User_location = [User_location; zeros(NumUser_Vector(i),1)+AP_location(i,1)+r.*cos(theta), zeros(NumUser_Vector(i),1)+AP_location(i,2)+r.*sin(theta)];
            end
            NumUser = NumUser_Vector;
        elseif (User_distribution == 2) % Poisson point process for varied number of users (PPP model)
            if (lambda_User == 0 || area_X == 0 || area_Y == 0)
                error('Please check tho_user ~= 0 and Area ~= 0');
            end
            NumUser_Vector = random('Poisson', lambda_User*area_X*area_Y);
            User_point = rand(NumUser_Vector, 2);
            User_location = [User_point(:,1)*area_X, User_point(:,2)*area_Y];
            NumUser = NumUser_Vector;
        else
             error('No this kind of (AP, user) distribution, please see BuildEnvironment.m');
        end
else
    if (AP_distribution == 0) % Square grid for fixed number of APs
        if (AP_distance == 0)
            error('Please check AP_distance ~= 0');
        end
        Num_AP = 9;
        Num_AP = floor(Num_AP);
        TurningPoint = ceil(sqrt(Num_AP));
        [Y, X] = meshgrid(1:TurningPoint);
        X = X*AP_distance; [r,c] = size(X);
        AP_X = reshape(X, r*c, 1);
        Y = Y*AP_distance;
        AP_Y = reshape(Y, r*c, 1);
        AP_location = [AP_X(1:Num_AP)+R,AP_Y(1:Num_AP)+R];
    elseif (AP_distribution == 1) % Hexagonal grid for fixed number of APs
        if (AP_distance == 0)
            error('Please check AP_distance ~= 0');
        end
        Num_AP = 25;
        Num_AP = floor(Num_AP);
        TurningPoint = ceil(sqrt(Num_AP));
        [Y, X] = meshgrid(1:TurningPoint);
        X = X*AP_distance; [r,c] = size(X);
        Y = Y*AP_distance*sqrt(3)/2;
        AP_Y = reshape(Y, r*c, 1);
        n = size(X,1);
        X_shift = repmat([0, 1/2]*AP_distance,[n, ceil(n/2)]);
        X = X + X_shift(1:n,1:n);
        AP_X = reshape(X, r*c, 1);
        AP_location = [AP_X(1:Num_AP)+R,AP_Y(1:Num_AP)+R];
        if (User_distribution == 1)
            UserR = 10;
            User_location = [];
            for k=1:Num_Tx
                angle = k*(2*pi)/Num_Tx;
                NewUser = [AP_X(1:Num_AP)+R+UserR*cos(angle),AP_Y(1:Num_AP)+R+UserR*sin(angle)];
                User_location = [User_location; NewUser];
            end
            NumUser = Num_AP * Num_Tx;
        elseif (User_distribution == 2)
            if (lambda_User == 0)
                error('Please check tho_user ~= 0 and Area ~= 0');
            end
            area_X = max(AP_location(1:Num_AP, 1))+R;
            area_Y = max(AP_location(1:Num_AP, 2))+R;
            NumUser_Vector = random('Poisson', lambda_User*area_X*area_Y);
            User_point = rand(NumUser_Vector, 2);
            User_location = [User_point(:,1)*area_X, User_point(:,2)*area_Y];
            NumUser = NumUser_Vector;
        else
            error('No this kind of (AP, user) distribution, please see BuildEnvironment.m');
        end
    else   
        error('No this kind of AP distribution, please see BuildEnvironment.m');
    end
end
tx_x_pos = AP_location(:,1);
tx_y_pos = AP_location(:,2);
rx_x_pos = User_location(:,1);
rx_y_pos = User_location(:,2);

XY_location = [AP_location; User_location];
%  wall_start = [0 0;140 0; 140 80; 0 80];
%  wall_end = [140 0; 140 80; 0 80; 0 0];
%  wall_start = [0 0;80 0; 80 80; 0 80];
%  wall_end = [80 0; 80 80; 0 80; 0 0];
  wall_start = [0 0;140 0; 140 140; 0 140];
 wall_end = [140 0; 140 140; 0 140; 0 0];
%wall_start=[0   0;ceil(max(XY_location(:, 1))/cover_range)*cover_range 0;ceil(max(XY_location(:, 1))/cover_range)*cover_range ceil(max(XY_location(:, 2))/cover_range)*cover_range;0 ceil(max(XY_location(:, 2))/cover_range)*cover_range];
%wall_end  =[ceil(max(XY_location(:, 1))/cover_range)*cover_range  0;ceil(max(XY_location(:, 1))/cover_range)*cover_range ceil(max(XY_location(:, 2))/cover_range)*cover_range;0 ceil(max(XY_location(:, 2))/cover_range)*cover_range;0 0];
end

