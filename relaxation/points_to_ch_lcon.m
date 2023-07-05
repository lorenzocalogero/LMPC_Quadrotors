function [A,b] = points_to_ch_lcon(V,tol)

if nargin<2
	tol = 1e-10;
end

V = V';
k = convhulln(V);
c = mean(V(unique(k),:));
V = bsxfun(@minus,V,c);
A = nan(size(k,1),size(V,2));
dim = size(V,2);
ee = ones(size(k,2),1);
rc = 0;

for ix = 1:size(k,1)
	F = V(k(ix,:),:);
	if lindep(F,tol) == dim
		rc=rc+1;
		A(rc,:)=F\ee;
	end
end

A = A(1:rc,:);
b = ones(size(A,1),1);
b = b+A*c';

% Eliminate duplicate constraints
[A,b] = rownormalize(A,b);
[~,I] = unique(round([A,b]*1e6),'rows');

A=A(I,:);
b=b(I);

end

function [A,b]=rownormalize(A,b)
% Modifies A,b data pair so that norm of rows of A is either 0 or 1

if isempty(A)
	return
end

normsA = sqrt(sum(A.^2,2));
idx = (normsA > 0);
A(idx,:) = bsxfun(@rdivide,A(idx,:),normsA(idx));
b(idx) = b(idx)./normsA(idx);

end

function [r,idx,Xsub]=lindep(X,tol)

if ~nnz(X) % X has no non-zeros and hence no independent columns
	Xsub=[]; idx=[];
	return
end

[~, R, E] = qr(X,0);
diagr = abs(diag(R));

% Rank estimation
r = find(diagr >= tol*diagr(1), 1, 'last');

if nargout>1
	idx=sort(E(1:r));
	idx=idx(:);
end

if nargout>2
	Xsub=X(:,idx);
end

end