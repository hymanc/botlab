function [rph_dot,J] = body2euler(pqr,rph)
%BODY2EULER converts body frame angular rates to Euler rates.
%   RPH_DOT = BODY2EULER(PQR,RPH) converts body frame angular rates PQR to
%   Euler angular rates RPH_DOT given Euler angles RPH.
%
%   [RPH_DOT,J] = BODY2EULER(PQR,RPH) also returns the [3 x 6] Jacobian J
%   associated with the transformation where J = [J_pqr, J_rph].
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-10-2004      rme         Created and Written
%    11-02-2004      rme         Modified to use secant of pitch

cr = cos(rph(1)); sr = sin(rph(1));
sp = sin(rph(2)); tp = tan(rph(2)); kp = sec(rph(2));

% transformation matrix which converts body frame
% angular rates pqr to local-level Euler angular rates rph_dot
T = [1,  sr*tp,  cr*tp; ...
     0,     cr,    -sr; ...
     0,  sr*kp,  cr*kp];

% Euler angular rates
rph_dot = T*pqr;

if nargout == 2
  % compute the Jacobian associated with the transformation
  dTdr = [0,  cr*tp,  -sr*tp; ...
	  0,    -sr,     -cr; ...
	  0,  cr*kp,  -sr*kp];
  
  dTdp = [0,   sr*kp^2,   cr*kp^2; ...
	  0,         0,         0; ...
	  0,  sr*kp*tp,  cr*kp*tp];

  J = zeros(3,6);
  J(:,1:3) = T;         % deriv w.r.t pqr
  J(:,4)   = dTdr*pqr;  % deriv w.r.t. r
  J(:,5)   = dTdp*pqr;  % deriv w.r.t. p
 %J(:,6)   = [0,0,0]';  % deriv w.r.t. h
end
