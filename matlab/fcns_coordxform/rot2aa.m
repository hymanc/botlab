function r = rot2aa(R)
%ROT2AA rotation matrix to axis-angle representation.
%   r = ROT2AA(R) converts a [3 x 3] orthonormal rotation matrix R to a 
%   [3 x 1] axis-angle vector r.
%
%   This algorithm is based upon Pennec 97 IJCV Appendix A.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            op          Created and written.
%    09-20-2003      rme         Renamed to rot2aa.m 

theta = acos((trace(R)-1)/2);
small = 1e-10;
if (theta > small) && (pi-theta > small)
  Sn = (R-R')/(2*sin(theta));
  n = [Sn(3,2); Sn(1,3); Sn(2,1)];
  r = theta*n;
elseif theta <= small
  Sr = 0.5*(1+theta^2/6)*(R-R');
  r = [Sr(3,2); Sr(1,3); Sr(2,1)];
else %theta close to pi
  fprintf(1,'Angle close to pi!\n')
  %  N2 = eye(3) + 1/(2*(1-cos(theta)))*(R+R'-2*eye(3));
  %  n = (diag(N2)).^0.5;
  %  signN = sign(R+R');
  Sn = (R-R')/(2*sin(theta));
  n = [Sn(3,2); Sn(1,3); Sn(2,1)];
  r = theta*n;
end
  

  
