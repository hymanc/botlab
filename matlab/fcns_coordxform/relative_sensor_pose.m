function [X_a1b2,Jt] = relative_sensor_pose(X_lv1,X_lv2,X_va,X_vb)
%RELATIVE_SENOSOR_POSE calculates 6 DOF relative pose coordinate transformation.
%   [X_a1a2,Jt] = RELATIVE_SENSOR_POSE(X_lv1,X_lv2,X_va) computes the 6 DOF
%   relative pose vector X_a1a2 between sensor 'a' at time t2 with respect
%   to sensor 'a' at time t1.  X_lv1 and X_lv2 are vehicle poses in the
%   local-level reference frame at time t1 and t2 respectively.  X_va is the
%   static sensor to vehicle pose vector.  Jt is the total Jacobian of the
%   transformation, i.e. Jt = d(X_a1a2)/d(X_lv1,X_lv2,X_va).
%
%   [X_a1b2,Jt] = RELATIVE_SENSOR_POSE(X_lv1,X_lv2,X_va,X_vb) computes the 6 DOF
%   relative pose vector X_a1b2 between sensor 'b' at time t2 with
%   respect to sensor 'a' at time t1.  The Jacobian of this
%   transformation is given by Jt = d(X_a1b2)/d(X_lv1,X_lv2,X_va,X_vb).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-13-2004      rme         Created and written.

NARGIN  = nargin;
NARGOUT = nargout;

% check input arguments
if NARGIN == 3
  X_vb = X_va;
end

% pose of sensor frame 'a' at time t1  w.r.t local-level
[X_la1,J_lv1_va] = head2tail(X_lv1,X_va);

% pose of sensor frame 'b' at time t2  w.r.t local-level
[X_lb2,J_lv2_vb] = head2tail(X_lv2,X_vb);

% relative pose of sensor frame 'b' at time t2 w.r.t. 
% sensor frame 'a' at time t1
[X_a1b2,J_la1_lb2] = tail2tail(X_la1,X_lb2);

% check output arguments
if NARGOUT == 2 % compute total Jacobian
  if NARGIN == 4
    % partial derivative of [X_la1,X_lb2] w.r.t. inputs [X_lv1,X_lv2,X_va,X_vb]
    Ji = [J_lv1_va(:,1:6), zeros(6),        J_lv1_va(:,7:12), zeros(6); ...
	  zeros(6),        J_lv2_vb(:,1:6), zeros(6), 	      J_lv2_vb(:,7:12)];
  else % NARGIN == 3 and X_va is idential to X_vb
    % partial derivative of [X_la1,X_lb2] w.r.t. inputs [X_lv1,X_lv2,X_va]    
    Ji = [J_lv1_va(:,1:6), zeros(6),        J_lv1_va(:,7:12); ...
	  zeros(6),        J_lv2_vb(:,1:6), J_lv2_vb(:,7:12)];
  end
  % the total Jacobian of X_a1b2 w.r.t. to the inputs is calculated via
  % chain-rule as:
  Jt = J_la1_lb2 * Ji;
end
