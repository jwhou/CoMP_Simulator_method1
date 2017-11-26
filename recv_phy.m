%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author	: TaHsiang Kuan
% Email     : kuanken.cs96@g2.nctu.edu.tw
% Version   : 18.0
% Date      : 2013/9/24
% References: 
%	1.http://wireless-matlab.sourceforge.net/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Pr0, Pr, SNR] = recv_phy(tx, rv, rmodel)
% send packet at PHY layer
global power_Debug MIMO_Debug;
global STA STA_Info CoMP_Controller;
global white_noise_variance;
global Num_Tx Num_Rx;

% Thinking:
% 1. tx is intended to rx
% 2. tx is not intended rx

% Pr is interference term
Pr = 0;
% Pr0 is signal term
Pr0 = 0;
if (length(tx) > 1) % CoMP transmission, and this must be sending Data
    if (any(STA(tx, 3) ~= 0))
        error('all transmitters should be CoMP APs');
    end
    for i=1:length(tx)
        if (STA(tx(i), 5) ~= 0) % CoMP AP has power
            if (STA(tx(i), 8) == 2) % it must send Data
                if power_Debug, disp(['  -P: Original recv_power(' num2str(tx(i)) ', ' num2str(rv) ', rmodel) = ' num2str(recv_power(tx(i), rv, rmodel))]); end
                if (CoMP_Controller.CoMP_tx_mode == 1) % Coordinated Scheduling/Coordinated Beamforming
                    if (any(rv == STA_Info(tx(i)).IntendSTA))
                        H = STA_Info(tx(i)).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx, :);
                        if MIMO_Debug, disp(['  -M: Channel to CoMP AP' num2str(tx(i))]); disp(H); end
                        for j=1:length(STA_Info(tx(i)).IntendSTA)
                            W = STA_Info(tx(i)).Precoding_Matrix(:, (STA_Info(tx(i)).IntendSTA(j)-1)*Num_Rx+1:STA_Info(tx(i)).IntendSTA(j)*Num_Rx);
                            if (rv == STA_Info(tx(i)).IntendSTA(j))
                                if MIMO_Debug, disp(['  -M: Precoder to STA' num2str(rv)]); disp(W); end
                                Pr0 = Pr0 + trace((H*W)*(H*W)');
                                if power_Debug, disp([' -P: calculating Pr0 = ' num2str(Pr0)]); end
                            else
                                if MIMO_Debug, disp(['  -M: Precoder to STA' num2str(rv)]); disp(W); end
                                Pr = Pr + trace((H*W)*(H*W)'); % Although I know intra-cell interference is nulled out, I still add them to interference term
                                if power_Debug, disp(['  -P: calculating Pr = ' num2str(Pr)]); end
                            end
                        end
                    elseif (any(rv == STA_Info(tx(i)).HelpedSTA))
                        H = STA_Info(tx(i)).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx, :);
                        W = STA_Info(tx(i)).Precoding_Matrix;
                        if MIMO_Debug 
                            disp(['  -M: Channel to CoMP AP' num2str(tx(i))]); disp(H); 
                            disp(['  -M: Precoder to STA' num2str(rv)]); disp(W); 
                        end
                        Pr = Pr + trace((H*W)*(H*W)'); % Although I know inter-cell interference is nulled out, I still add them to interference term
                        if power_Debug, disp(['  -P: calculating Pr = ' num2str(Pr)]); end
                    else % Other rv, neither tx(i) intended STA nor tx(i) helped STA
                        W = STA_Info(tx(i)).Precoding_Matrix;
                        if (STA(rv, 3) == 0) % rv is an AP
                            H = sqrt(recv_power(tx(i), rv, rmodel))*(randn(Num_Tx, Num_Tx)+1i*randn(Num_Tx, Num_Tx))/sqrt(2);
                        else % rv is a non-AP STA
                            H = STA_Info(tx(i)).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx, :);
                            if (H == zeros(Num_Rx, Num_Tx))
                                H = sqrt(recv_power(tx(i), rv, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                            end
                        end
                        Pr0 = Pr0 + trace((H*W)*(H*W)');
                        if power_Debug, disp(['  -P: calculating Pr0 = ' num2str(Pr0)]); end
                    end
                else %if (CoMP_Controller.CoMP_tx_mode == 0) % Networked MIMO
                    if (any(rv == STA_Info(tx(i)).IntendSTA) || any(rv == STA_Info(tx(i)).HelpedSTA))
                        H = STA_Info(tx(i)).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx, :);
                        W = STA_Info(tx(i)).Precoding_Matrix(:, (rv-1)*Num_Rx+1:rv*Num_Rx);
                        W_interference = STA_Info(tx(i)).Precoding_Matrix;
                        W_interference(:, (rv-1)*Num_Rx+1:rv*Num_Rx) = zeros(Num_Tx, Num_Rx);
                        if MIMO_Debug 
                            disp(['  -M: Channel to CoMP AP' num2str(tx(i))]); disp(H); 
                            disp(['  -M: Precoder to STA' num2str(rv)]); disp(W);
                            disp(['  -M: Interference precoder to STA' num2str(rv)]); disp(W_interference); 
                        end
                        Pr0 = Pr0 + (H*W);
                        Pr = Pr + trace((H*W_interference)*(H*W_interference)'); % Although I know intra-cell and inter-cell interference are nulled out, I still add them to interference term
                    else % Other rv, neither tx(i) intended STA nor tx(i) helped STA
                        W = STA_Info(tx(i)).Precoding_Matrix;
                        if (STA(rv, 3) == 0) % rv is an AP
                            H = sqrt(recv_power(tx(i), rv, rmodel))*(randn(Num_Tx, Num_Tx)+1i*randn(Num_Tx, Num_Tx))/sqrt(2);
                        else % rv is a non-AP STA
                            H = STA_Info(tx(i)).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx, :);
                            if (H == zeros(Num_Rx, Num_Tx))
                                H = sqrt(recv_power(tx(i), rv, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                            end
                        end
                        Pr0 = Pr0 + trace((H*W)*(H*W)');
                        if power_Debug, disp(['  -P: calculating Pr0 = ' num2str(Pr0)]); end
                    end
                end
            end
        end
    end
    if (CoMP_Controller.CoMP_tx_mode == 0)
        Pr0 = trace(Pr0*Pr0');
        Pr = trace(Pr*Pr');
        if power_Debug, disp(['  -P: calculating Pr0 = ' num2str(Pr0)]); disp(['- calculating Pr = ' num2str(Pr)]); end
    end
else % single transmitter
    if (STA(tx, 5) <= 0 ) 
        disp(['error tx ' num2str(STA(tx,5))  '6 = ' num2str(STA(tx,6)) '8 = ' num2str(STA(tx,8))]);
        error('send_PHY: transmission power is zero ·F');
    end
    if power_Debug, disp(['  -P: Original recv_power(' num2str(tx) ', ' num2str(rv) ', rmodel) = ' num2str(recv_power(tx, rv, rmodel))]); end
    if (STA(tx, 8) == 2) % the transmitter transmits Data pkt
        if (STA(tx, 3) == 0) % the transmitter is an AP
            if (floor(Num_Tx/Num_Rx) > 1) % the transmitter has multiple antennas
                % An AP has multiple antennas could send precoded Data either SU-PPDU or MU-PPDU
                W = STA_Info(tx).Precoding_Matrix;
                if (STA(rv, 3) == 0) % the reciever is an AP
                    H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Tx, Num_Tx)+1i*randn(Num_Tx, Num_Tx))/sqrt(2);
                    if (all(W == 0))
                        Pr0 = trace(H*H');
                    else
                        Pr0 = trace((H*W)*(H*W)');
                    end
                else % the reciever is a non-AP STA
                    H = STA_Info(tx).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx,:);
                    if (H == zeros(Num_Rx, Num_Tx))
                        H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);    
                    end
                    if (all(W == 0))
                        Pr0 = trace(H*H'); % In the beginning, only one user is served, no precoding matrix
                    else
                        Pr0 = trace((H*W)*(H*W)');
                    end
                end
                if power_Debug, disp(['  -P: Precoder of STA ' num2str(tx) ',']); disp(W); end
                if power_Debug, disp(['  -P: H' num2str(rv)  num2str(tx) ' = ']); disp(H); end      
            else % the transmitter only has one antenna
                % An AP has only one antenna
                if (STA(rv, 3) == 0) % the reciever is an AP
                    H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Tx, Num_Tx)+1i*randn(Num_Tx, Num_Tx))/sqrt(2);
                    Pr0 = trace(H*H');
                else % the reciever is a non-AP STA
                    H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                    Pr0 = trace(H*H');
                end
                if power_Debug, disp(['  -P: H' num2str(rv)  num2str(tx) ' = ']); disp(H); end
            end
        else % the transmitter is a non-AP STA and send Data packet
            if (STA(rv, 3) == 0) % the reciever is an AP
                H = STA_Info(rv).Channel_Matrix((tx-1)*Num_Rx+1:tx*Num_Rx, :);
                if (H == zeros(Num_Rx, Num_Tx))
                    H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx,Num_Tx)+1i*randn(Num_Rx,Num_Tx))/sqrt(2);
                end
                Pr0 = trace(H*H');
                if power_Debug, disp(['  -P: H' num2str(rv)  num2str(tx) ' = ']); disp(H); end
            else % the reciever is a non-AP STA
                H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx)+1i*randn(Num_Rx))/sqrt(2);
                Pr0 = trace(H*H');
            end
        end
    else % the transmitter transmits RTS/CTS/ACK pkt
%         % Maximum ratio combining technique for receiving control frames
%         if (STA(tx, 3) == 0 && STA(rv, 3) == 0)
%             H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Tx)+1i*randn(Num_Tx))/sqrt(2);
%         elseif (STA(tx, 3) == 0 && STA(rv, 3) ~= 0)
%             H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
%         elseif (STA(tx, 3) ~= 0 && STA(rv, 3) == 0)
%             H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Tx, Num_Rx)+1i*randn(Num_Tx, Num_Rx))/sqrt(2);
%         else
%             H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx)+1i*randn(Num_Rx))/sqrt(2);
%         end
        H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx)+1i*randn(Num_Rx))/sqrt(2);
        Pr0 = trace(H*H');
    end    
end

I = find(STA(:, 5)>0);
for i=1:length(I)
    tx1 = I(i);
    if tx1 == rv, continue; end
    if any(tx1 == tx), continue; end   
    if (STA(tx1, 8) == 2) % tx1 transmits Data pkt
        if (STA(tx1, 3) == 0) % tx1 is an AP
            if (floor(Num_Tx/Num_Rx) > 1) % tx1 has multiple antennas
                W = STA_Info(tx1).Precoding_Matrix;
                if (STA(rv, 3) == 0) % rv is an AP
                    H = sqrt(recv_power(tx1, rv, rmodel))*(randn(Num_Tx,Num_Tx)+1i*randn(Num_Tx,Num_Tx))/sqrt(2);
                    temp_Pr = trace((H*W)*(H*W)');
                else % rv is a non-AP STA
                    H = STA_Info(tx1).Channel_Matrix((rv-1)*Num_Rx+1:rv*Num_Rx,:);
                    if (H == zeros(Num_Rx, Num_Tx))
                        H = sqrt(recv_power(tx1, rv, rmodel))*(randn(Num_Rx,Num_Tx)+1i*randn(Num_Rx,Num_Tx))/sqrt(2);
                    end
                    temp_Pr = trace((H*W)*(H*W)');
                end
            else % tx1 only has one antenna
                % An AP has only one antenna
                if (STA(rv, 3) == 0) % rv is an AP
                    H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Tx, Num_Tx)+1i*randn(Num_Tx, Num_Tx))/sqrt(2);
                    temp_Pr = trace(H*H');
                else % rv is a non-AP STA
                    H = sqrt(recv_power(tx, rv, rmodel))*(randn(Num_Rx, Num_Tx)+1i*randn(Num_Rx, Num_Tx))/sqrt(2);
                    temp_Pr = trace(H*H');
                end
            end
        else % tx1 is a non-AP STA
            if (STA(rv, 3) == 0) % rv is an AP
                H = STA_Info(rv).Channel_Matrix((tx1-1)*Num_Rx+1:tx1*Num_Rx, :);
                if (H == zeros(Num_Rx, Num_Tx))
                    H = sqrt(recv_power(tx1, rv, rmodel))*(randn(Num_Rx,Num_Tx)+1i*randn(Num_Rx,Num_Tx))/sqrt(2);
                end
                temp_Pr = trace(H*H');
            else % rv is a non-AP STA
                H = (randn(1)+1i*randn(1))/sqrt(2);
                temp_Pr = recv_power(tx1, rv, rmodel)*trace(H*H');
            end
        end
    else % tx transmits RTS/CTS/ACK pkt, do not care # of antennas
        H = (randn(Num_Rx)+1i*randn(Num_Rx))/sqrt(2);
        temp_Pr = recv_power(tx1, rv, rmodel)*trace(H*H');
    end
    Pr = Pr + temp_Pr;
end

N0 = white_noise_variance; % N0 = abs(random('norm', 0, white_noise_variance));
SNR = real(Pr0)/(real(Pr)+N0);
if power_Debug, disp(['  -P: Received power = ' num2str(Pr0) ',  Interference = ' num2str(Pr) ', Noise = ' num2str(N0) ' =>  SNR = ' num2str(SNR) ' = '  num2str(db(SNR, 'power')) 'dB']); end

return;
