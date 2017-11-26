function y = qfuncinv(x)
	if (nargin < 1); 
		usage('qfuncinv(x)'); 
	end
	
	y = sqrt(2)*erfcinv(2*x);
end