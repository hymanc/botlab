function y = blkrep(x,k)
% Y = BLKREP(X,K)
%   Creates a block diagonal matrix Y, composed of
%   K copies of X.

y = [];
[p2,m2] = size(x);
for ii=1:k
    [p1,m1] = size(y);
    y = [y zeros(p1,m2); 
         zeros(p2,m1) x];
end
