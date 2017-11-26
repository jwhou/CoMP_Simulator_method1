%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 17.0
% Date      : 2013/9/16
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [SNR_vector] = estimate_SNR(pkt)
% Calculating SNR in dynamic MCS mode, only an AP STA calls
%   Output: 
%           SNR_vector correspond to pkt.rv
%   Input: 
%           pkt: all pkt information
global Num_Tx Num_Rx; 
global STA STA_Info CoMP_Controller;
global rmodel white_noise_variance;

SNR_vector = zeros(length(pkt.rv), 1);
i = pkt.tx; % i always be an AP index
if (pkt.MU == 0 && pkt.CoMP == 0)
    % There is only one index in pkt.rv
    H = zeros(Num_Rx, Num_Tx);
    temp = STA_Info(i).Channel_Matrix((pkt.rv-1)*Num_Rx+1:pkt.rv*Num_Rx,:);
    if (temp == zeros(Num_Rx, Num_Tx))
        SNR_vector = LongTerm_recv_power(i, pkt.rv, rmodel);
    else
        H = temp;
        SNR_vector = trace(H*H');
    end
elseif (pkt.MU == 1 && pkt.CoMP == 0)
    selection_length = length(pkt.rv);
    H = zeros(selection_length*Num_Rx, Num_Tx);
    longtermrvpower = zeros(selection_length, 1);
    flag = 0;
    for k=1:selection_length
        temp = STA_Info(i).Channel_Matrix((pkt.rv(k)-1)*Num_Rx+1:pkt.rv(k)*Num_Rx,:);
        if (temp == zeros(Num_Rx, Num_Tx))
            flag = 1;
        else
            H((k-1)*Num_Rx+1:k*Num_Rx, :) = temp;
        end
        longtermrvpower(k) = LongTerm_recv_power(i, pkt.rv(k), rmodel)/selection_length;
    end
    if (flag == 0)
        W = H'/(H*H');
        temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
        W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
        for k=1:selection_length
            W(:,(k-1)*Num_Rx+1:k*Num_Rx) = W(:,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(selection_length);
        end
        for k=1:selection_length
            temp_H = H((k-1)*Num_Rx+1:k*Num_Rx, :);
            SNR_vector(k) = trace(temp_H*W*(temp_H*W)');
        end
    else
        SNR_vector = longtermrvpower;
    end
elseif (pkt.MU == 1 && pkt.CoMP == 1)
    selection_length = length(pkt.rv);
    longtermrvpower = zeros(selection_length, 1);
    if (CoMP_Controller.CoMP_tx_mode == 0) % Networked MIMO
        CoMPAPs = [i, STA_Info(i).CoMP_coordinator];
        CoMPAPs_length = length(CoMPAPs);
        H = zeros(selection_length*Num_Rx, CoMPAPs_length*Num_Tx);
        flag = 0;
        for k=1:selection_length
            for l = 1:CoMPAPs_length
                CoMPAP_index = CoMPAPs(l);
                temp = STA_Info(CoMPAP_index).Channel_Matrix((pkt.rv(k)-1)*Num_Rx+1:pkt.rv(k)*Num_Rx,:);
                if (temp == zeros(Num_Rx, Num_Tx))
                    flag = 1;
                else
                    H((k-1)*Num_Rx+1:k*Num_Rx, (l-1)*Num_Tx+1:l*Num_Tx) = temp;
                end
                longtermrvpower(k) = longtermrvpower(k) + LongTerm_recv_power(CoMPAP_index, pkt.rv(k), rmodel)/selection_length;
            end
        end
        if (flag == 0)
            W = H'/(H*H');
            temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
            W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
            for k=1:selection_length
                for l=1:CoMPAPs_length
                    CoMPAP_index = CoMPAPs(l);
                    % STA_Info(CoMPAP_index).IntendSTA could be NULL, a small bug here
                    W((l-1)*Num_Tx+1:l*Num_Tx,(k-1)*Num_Rx+1:k*Num_Rx) = W((l-1)*Num_Tx+1:l*Num_Tx,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(length(STA_Info(CoMPAP_index).IntendSTA));
                end
            end
            for k=1:selection_length
                temp_H = H((k-1)*Num_Rx+1:k*Num_Rx, :);
                SNR_vector(k) = trace((temp_H*W)*(temp_H*W)');
            end
        else
            for k=1:selection_length
                SNR_vector = longtermrvpower;
            end
        end 
    elseif (CoMP_Controller.CoMP_tx_mode == 1) % Coordinated Scheduling/Coordinated Beamforming
        %CoMPAPs = [i, STA_Info(i).CoMP_coordinator];
        CoMPAPs = i;
        CoMPAPs_length = length(CoMPAPs);
        All_W = zeros(CoMPAPs_length*Num_Tx, selection_length*Num_Rx);
        for l=1:CoMPAPs_length
            flag = 0;
            CoMPAP_index = CoMPAPs(l);
            H = zeros(selection_length*Num_Rx, Num_Tx);
            for k=1:selection_length
                temp = STA_Info(CoMPAP_index).Channel_Matrix((pkt.rv(k)-1)*Num_Rx+1:pkt.rv(k)*Num_Rx, :);
                if (temp == zeros(Num_Rx, Num_Tx))
                    flag = 1;
                    break;
                else
                    H((k-1)*Num_Rx+1:k*Num_Rx, :) = temp;
                end
            end
            if (flag == 0)
                W = H'/(H*H');
                temp = diag(W'*W).'; % diag output a colum vector, transpose it to row vector
                W = bsxfun(@rdivide, W, sqrt(temp)); % Normalize precoding matrix
                for k=1:selection_length
                    %if (STA(pkt.rv(k), 3) == CoMPAP_index)
                        % STA_Info(CoMPAP_index).IntendSTA could be NULL, a small bug here
                        All_W((l-1)*Num_Tx+1:l*Num_Tx,(k-1)*Num_Rx+1:k*Num_Rx) = W(:,(k-1)*Num_Rx+1:k*Num_Rx)/sqrt(length(STA_Info(CoMPAP_index).IntendSTA));   
                    %end
                end
            end
        end
        if (flag == 0)
            for k=1:selection_length
                temp_H = zeros(Num_Rx, CoMPAPs_length*Num_Tx);
                for l=1:CoMPAPs_length
                    CoMPAP_index = CoMPAPs(l);
                    temp_H(:, (l-1)*Num_Tx+1:l*Num_Tx) = STA_Info(CoMPAP_index).Channel_Matrix((pkt.rv(k)-1)*Num_Rx+1:pkt.rv(k)*Num_Rx,:);
                end
                SNR_vector(k) = trace(temp_H*All_W*(temp_H*All_W)');
            end
        else
            for k=1:selection_length
                SNR_vector(k) = 0;
            end
        end
    end
end
N0 = white_noise_variance; % N0 = abs(random('norm', 0, white_noise_variance));
if (isnan(SNR_vector))
%     SNR_vector = zeros(length(pkt.rv), 1);
    error('Hi');
else
    SNR_vector = real(SNR_vector)/N0;
end

end

