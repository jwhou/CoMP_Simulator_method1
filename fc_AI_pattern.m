%{
Author:      Ju-Chia Wang
Date:        2016/3/22
Input:       AI
Output:      The antenna pattern that the AI(input) matches
Description: The function accoding every AI to match antenna pattern
             (Write in a fc_AI_pattern.docx in detail)
p.s. Not including all possible AI now.Only has the results from Chia-shan. 
%}
function pattern=fc_AI_pattern(AI,Ant_type)

if Ant_type==1 %for 2.4GHz Simulated Antenna Pattern
    switch AI
        case -1%Beam B
            pattern=load('MAT/1(-y)2(-x-y)_theta45.mat');
        case 1%Beam A
            pattern=load('MAT/1(+x+y)2(+y)_theta45.mat');
        case 2%Beam F
            pattern=load('MAT/1(0)2(+y)_theta45.mat');
        case 3%Beam E
            pattern=load('MAT/1(0)2(+x+y)_theta45.mat');
        case 4%Beam D
            pattern=load('MAT/1(0)2(+x)_theta45.mat');
        case 5%Beam C
            pattern=load('MAT/1(0)2(+x-y)_theta45.mat');
        case -2%Beam J
            pattern=load('MAT/1T(-y)2(0)_theta45.mat');
        case -3%Beam I
            pattern=load('MAT/1T(-x-y)2(0)_theta45.mat');
        case -4%Beam H
            pattern=load('MAT/1T(-x)2(0)_theta45.mat');
        case -5%Beam G
            pattern=load('MAT/1T(-x+y)2(0)_theta45.mat');
        case 0%Omni
            %note need to replace 2.4GHz omni beam%%%%%%%%%%%%%%%%%%
            pattern=load('MAT/2.4Omni.mat');
        otherwise    
    end
elseif Ant_type==2 %for 2.4GHz Measured Antenna Pattern
    switch AI
        case -1%Beam B
            pattern=load('Beam/Beam_B_meas.mat');
        case 1%Beam A
            pattern=load('Beam/Beam_A_meas.mat');
        case 2%Beam F
            pattern=load('Beam/Beam_F_meas.mat');
        case 3%Beam E
            pattern=load('Beam/Beam_E_meas.mat');
        case 4%Beam D
            pattern=load('Beam/Beam_D_meas.mat');
        case 5%Beam C
            pattern=load('Beam/Beam_C_meas.mat');
        case -2%Beam J
            pattern=load('Beam/Beam_J_meas.mat');
        case -3%Beam I
            pattern=load('Beam/Beam_I_meas.mat');
        case -4%Beam H
            pattern=load('Beam/Beam_H_meas.mat');
        case -5%Beam G
            pattern=load('Beam/Beam_G_meas.mat');
        case 0%Omni
            %note need to replace 2.4GHz omni beam%%%%%%%%%%%%%%%
            pattern=load('AI/omni_5GHz_sim.mat');
        otherwise    
    end
else %for 5 GHz Simulated Antenna Pattern
    switch AI
        case -1%Beam B
            pattern=load('AI/BeamB_5GHz_sim.mat');
        case 1%Beam A
            pattern=load('AI/BeamA_5GHz_sim.mat');
        case 2%Beam F
            pattern=load('AI/BeamF_5GHz_sim.mat');
        case 3%Beam E
            pattern=load('AI/BeamE_5GHz_sim.mat');
        case 4%Beam D
            pattern=load('AI/BeamD_5GHz_sim.mat');
        case 5%Beam C
            pattern=load('AI/BeamC_5GHz_sim.mat');
        case -2%Beam J
            pattern=load('AI/BeamJ_5GHz_sim.mat');
        case -3%Beam I
            pattern=load('AI/BeamI_5GHz_sim.mat');
        case -4%Beam H
            pattern=load('AI/BeamH_5GHz_sim.mat');
        case -5%Beam G
            pattern=load('AI/BeamG_5GHz_sim.mat');
        case 0%Omni
            pattern=load('AI/omni_5GHz_sim.mat');
        otherwise  
    end   
end%end of Ant_type==1

end