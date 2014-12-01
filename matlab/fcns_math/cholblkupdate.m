function T = cholblkupdate(A,T,fn,Nv);
%function T = cholblkupdate(A,T,fn,Nv);  

Nf = length(A)/Nv; % number of feature blocks
ii = [1:Nv] + (fn-1)*Nv;
for fni=fn:Nf;
  k  = ii(1)-1;
  T(ii,ii) = chol( A(ii,ii) - T(1:k,ii)'*T(1:k,ii) );
  jj = ii+Nv;
  for fnj=fni+1:Nf;
    T(ii,jj) = T(ii,ii)' \ ( A(ii,jj) - T(1:k,ii)'*T(1:k,jj) );    
    jj = jj+Nv;
  end;
  ii = ii+Nv;
end;
