function Y=fc_return_auxiliary_constant_Cell(Nt,eig_Value)

% [1]. M. KcKay, et. al., "multiplexing/beamforming switching for coded
% MIMO in spatially correlated channels based on closed-form BER
% approximations", TVT 2007
% Based on [1], this function returns the auxiliary constants, Cell which are
% given in the appendix. Be aware of that this returns a vector, for ell=1,2,..., Nr-Nt+1
% ,instead of a value for a given ell

% Nt: number of transmit antenna
% eig_Value: a row vertor, the eigenvalue vector of the receive correlation matrix
Nr=length(eig_Value); % number of receive antenna
Y=zeros(1,Nr-Nt+1);

for ell=1:1:Nr-Nt+1
%     ele_matrix = nchoosek(eig_Value,ell);
    ele_matrix = nchoosek(eig_Value,Nt-1);
    nchoosek_ind=nchoosek([1:1:Nr],Nt-1); % this matrix is used to identify which of those eigenvalues are choosen in each row of ele_matrix
                                          % it is needed when some of the
                                          % eigenvalues are equal to each other
    
    for i=1:1:size(ele_matrix,1)
        % <----- generating the sub-eigenvalue vector -----
        % the definition of the "sub-eigenvalue vector is given in the
        % research note vol. 18, pp. 142
        sub_eig_Value = zeros(1,length(eig_Value)-(Nt-1));
%         sub_eig_Value = zeros(1,length(eig_Value)-ell);
        index=1;
        for j=1:1:Nr
%             if prod(ele_matrix(i,:)-eig_Value(j))~=0
            % The above "if" works only if every eigenvalue is different.
            % It fails if some of the eigenvalues are equal to each other,
            % and so we need the following "if"
            if prod(nchoosek_ind(i,:)-j)~=0
                sub_eig_Value(1,index) = eig_Value(j);
                index=index+1;
            end
        end
        % ----- generating the sub-eigenvalue vector ----->
%         ele_matrix(i,:)
%         sub_eig_Value
%         ell
        if sub_eig_Value == 0
            temp = 1;
        else
            temp = sum(prod(nchoosek(sub_eig_Value,ell),2));
        end
        Y(ell) = Y(ell) + prod(ele_matrix(i,:))*temp;
    end
end