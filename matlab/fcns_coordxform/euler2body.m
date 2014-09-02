function [pqr,J] = euler2body(rph_dot,rph)
%EULER2BODY converts Euler angular rates to body frame rates.
%   PQR = EULER2BODY(RPH_DOT,RPH) converts Euler angular rates RPH_DOT to
%   body frame angular rates PQR given Euler angles RPH.
%
%   [PQR,J] = EULER2BODY(RPH_DOT,RPH) also returns the [3 x 6] Jacobian J
%   associated with the transformation where J = [J_rph_dot, J_rph].
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-10-2004      rme         Created and Written

cr = cos(rph(1)); sr = sin(rph(1));
cp = cos(rph(2)); sp = sin(rph(2));

% transformation matrix which converts Euler angular rates rph_dot to
% body frame angular rates pqr
T = [1,    0,     -sp; ...
     0,   cr,   sr*cp; ...
     0,  -sr,   cr*cp];

% body frame angular rates
pqr = T*rph_dot;

if nargout == 2
  % compute the Jacobian associated with the transformation
  dTdr = [0,    0,       0; ...
	  0,  -sr,   cr*cp; ...
	  0,  -cr,  -sr*cp];
  
  dTdp = [0,    0,     -cp; ...
	  0,    0,  -sr*sp; ...
	  0,    0,  -cr*sp];

  J = zeros(3,6);
  J(:,1:3) = T;             % deriv w.r.t rph_dot
  J(:,4)   = dTdr*rph_dot;  % deriv w.r.t. r
  J(:,5)   = dTdp*rph_dot;  % deriv w.r.t. p
 %J(:,6)   = [0,0,0]';      % deriv w.r.t. h
end
