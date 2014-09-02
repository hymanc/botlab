function [t,J] = dm2trans(b)
%DM2TRANS direction-magnitude to translation vector.
%
% t = DM2TRANS(b) converts a direction-magnitude [3 x 1] vector 
% (i.e. b = [azimuth, elevation, magnitude]') to a [3 x 1]
% translation vector (i.e. t = [tx ty tz]').
%
% [t,J] = DM2TRANS(b) also returns the Jacobian of this transformation
% which is useful for propogating 1st order covariances e.g.
% Cov_t = J*Cov_b*J';
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            op          Created and written.
%    09-20-2003      rme         Rewrote jacobian_b2t and renamed parent
%                                function to dm2trans.m


% convert direction-magnitude to translation vector
t = b2t(b);
if nargout > 1
  J = jacobian_b2t(b);
end

%**************************************************
function t = b2t(b)

azim = b(1);
elev = b(2);
mag  = b(3);

tz = mag*sin(elev);
tx = mag*cos(elev)*cos(azim);
ty = mag*cos(elev)*sin(azim);

t = [tx; ty; tz];

%**************************************************
function J = jacobian_b2t(b);

ca = cos(b(1)); sa = sin(b(1));
ce = cos(b(2)); se = sin(b(2));
mag = b(3);

% analytical expression for jacobian matrix
J = [-mag*ce*sa, -mag*se*ca, ce*ca; ...
      mag*ce*ca, -mag*se*sa, ce*sa; ...
          0,      mag*ce,    se];
