function q = rph2quat(rph)
%RPH2QUAT converts Euler RPH to unit quaternion.
%   q = RPH2QUAT(rph) returns a [4 x 1] unit quaternion
%   equivalent to the [3 x 1] Euler xyz representation.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-30-2007      rme         Created and written.

R = rotxyz(rph);

aa = rot2aa(R);

q = aa2quat(aa);
