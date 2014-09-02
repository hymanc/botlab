function R = aa2rot(r)
%AA2ROT axis-angle to rotation matrix representation.
%   R = AA2ROT(r) converts a [3 x 1] axis-angle vector r to a [3 x 3]
%   orthonormal rotation matrix R.
%
%   This algorithm is based upon Pennec 97 IJCV Appendix A.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            op          Created and written.
%    09-20-2003      rme         Renamed to aa2rot.m
  
theta = norm(r);
small = 1e-10;
if theta > small
  n = r/theta;
  Sn = [  0   -n(3)  n(2); 
	 n(3)   0   -n(1);
	-n(2)  n(1)    0 ];
  R = eye(3) + sin(theta)*Sn + (1-cos(theta))*(Sn*Sn);
else
  Sr = [  0   -r(3)  r(2); 
	 r(3)    0  -r(1); 
	-r(2)  r(1)    0 ];
  R = eye(3) + (1-theta^2/6)*Sr + (0.5-theta^2/24)*(Sr*Sr);
end
