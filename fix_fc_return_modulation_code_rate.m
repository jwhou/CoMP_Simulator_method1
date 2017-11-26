function [modulation,code_rate] = fc_return_modulation_code_rate(WiFi_standard,MCS_mode)

% Given the WiFi_standard (11n or 11ac) and MCS mode, this function returns the modulation and code rate
% Input:
%     WiFi_standard: it is a "string", and it must be '80211n' or '80211ac';
%     MCS_mode: 0~9  if WiFi_standard == 80211ac
%               0~31 if WiFi_standard == 80211n
% % Output:
%     modulation: it is a "string"
%     code_rate: convolutional code rate
%       802.11ac     
%              ------------------------------------------------
%              | VHT-MCS Index | Modulation |  R  | data_rate |
%              |       0       |    BPSK    | 1/2 |    0.5    | 
%              |       1       |    QPSK    | 1/2 |     1     |
%              |       2       |    QPSK    | 3/4 |    1.5    |
%              |       3       |   16-QAM   | 1/2 |     2     |
%              |       4       |   16-QAM   | 3/4 |     3     |
%              |       5       |   64-QAM   | 2/3 |     4     |
%              |       6       |   64-QAM   | 3/4 |    4.5    |
%              |       7       |   64-QAM   | 5/6 |     5     |
%              |       8       |  256-QAM   | 3/4 |     6     |
%              |       9       |  256-QAM   | 5/6 |    20/3   |
%              ------------------------------------------------
%       802.11n
%         ------------------------------------------------
%         | HT-MCS Index  | # of streams | Modulation |  R  | data_rate |         | HT-MCS Index  | # of streams | Modulation |  R  | data_rate |
%         |       0       |       1      |    BPSK    | 1/2 |    0.5    |         |       8       |       2      |    BPSK    | 1/2 |    0.5    | 
%         |       1       |       1      |    QPSK    | 1/2 |     1     |         |       9       |       2      |    QPSK    | 1/2 |     1     |
%         |       2       |       1      |    QPSK    | 3/4 |    1.5    |         |       10      |       2      |    QPSK    | 3/4 |    1.5    |
%         |       3       |       1      |   16-QAM   | 1/2 |     2     |         |       11      |       2      |   16-QAM   | 1/2 |     2     |
%         |       4       |       1      |   16-QAM   | 3/4 |     3     |         |       12      |       2      |   16-QAM   | 3/4 |     3     |
%         |       5       |       1      |   64-QAM   | 2/3 |     4     |         |       13      |       2      |   64-QAM   | 2/3 |     4     |
%         |       6       |       1      |   64-QAM   | 3/4 |    4.5    |         |       14      |       2      |   64-QAM   | 3/4 |    4.5    |
%         |       7       |       1      |   64-QAM   | 5/6 |     5     |         |       15      |       2      |   64-QAM   | 5/6 |     5     |
%         ------------------------------------------------

modulation=('');
code_rate=0;

if strcmp(WiFi_standard,'80211n')==1
    switch MCS_mode
    case 0
        modulation=('BPSK');
        code_rate=1/2;
    case 1
        modulation=('QPSK');
        code_rate=1/2;
    case 2
        modulation=('QPSK');
        code_rate=3/4;
    case 3
        modulation=('16QAM');
        code_rate=1/2;
    case 4
        modulation=('16QAM');
        code_rate=3/4;
    case 5
        modulation=('64QAM');
        code_rate=2/3;
    case 6
        modulation=('64QAM');
        code_rate=3/4;
    case 7
        modulation=('64QAM');
        code_rate=5/6;
    case 8
        modulation=('BPSK');
        code_rate=1/2;
    case 9
        modulation=('QPSK');
        code_rate=1/2;
    case 10
        modulation=('QPSK');
        code_rate=3/4;
    case 11
        modulation=('16QAM');
        code_rate=1/2;
    case 12
        modulation=('16QAM');
        code_rate=3/4;
    case 13
        modulation=('64QAM');
        code_rate=2/3;
    case 14
        modulation=('64QAM');
        code_rate=3/4;
    case 15
        modulation=('64QAM');
        code_rate=5/6;
    case 16
        modulation=('BPSK');
        code_rate=1/2;
    case 17
        modulation=('QPSK');
        code_rate=1/2;
    case 18
        modulation=('QPSK');
        code_rate=3/4;
    case 19
        modulation=('16QAM');
        code_rate=1/2;
    case 20
        modulation=('16QAM');
        code_rate=3/4;
    case 21
        modulation=('64QAM');
        code_rate=2/3;
    case 22
        modulation=('64QAM');
        code_rate=3/4;
    case 23
        modulation=('64QAM');
        code_rate=5/6;
    case 24
        modulation=('BPSK');
        code_rate=1/2;
    case 25
        modulation=('QPSK');
        code_rate=1/2;
    case 26
        modulation=('QPSK');
        code_rate=3/4;
    case 27
        modulation=('16QAM');
        code_rate=1/2;
    case 28
        modulation=('16QAM');
        code_rate=3/4;
    case 29
        modulation=('64QAM');
        code_rate=2/3;
    case 30
        modulation=('64QAM');
        code_rate=3/4;
    case 31
        modulation=('64QAM');
        code_rate=5/6;
    %otherwise
    %    Error('Error in fc_return_modulation_code_rate().m, where MCS mode > 15(n)')
    end

else if strcmp(WiFi_standard,'80211ac')==1
    switch MCS_mode
    case 0
        modulation=('BPSK');
        code_rate=1/2;
    case 1
        modulation=('QPSK');
        code_rate=1/2;
    case 2
        modulation=('QPSK');
        code_rate=3/4;
    case 3
        modulation=('16QAM');
        code_rate=1/2;
    case 4
        modulation=('16QAM');
        code_rate=3/4;
    case 5
        modulation=('64QAM');
        code_rate=2/3;
    case 6
        modulation=('64QAM');
        code_rate=3/4;
    case 7
        modulation=('64QAM');
        code_rate=5/6;
    case 8
        modulation=('256QAM');
        code_rate=3/4;
    case 9
        modulation=('256QAM');
        code_rate=5/6;
   case 10
        modulation=('BPSK');
        code_rate=1/2;
    case 11
        modulation=('QPSK');
        code_rate=1/2;
    case 12
        modulation=('QPSK');
        code_rate=3/4;
    case 13
        modulation=('16QAM');
        code_rate=1/2;
    case 14
        modulation=('16QAM');
        code_rate=3/4;
    case 15
        modulation=('64QAM');
        code_rate=2/3;
    case 16
        modulation=('64QAM');
        code_rate=3/4;
    case 17
        modulation=('64QAM');
        code_rate=5/6;
    case 18
        modulation=('256QAM');
        code_rate=3/4;
    case 19
        modulation=('256QAM');
        code_rate=5/6;
    otherwise
        error('Error in fc_return_modulation_code_rate().m, where MCS mode > 19(ac)');
    end
    
    else
        error('Error. "WiFi_standard" must be 80211n or 80211ac')
    end
end