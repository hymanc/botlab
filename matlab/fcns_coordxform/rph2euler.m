function [euler_rph] = rph2euler(inclinometer_rph)
% RPH2EULER Computes Euler angles from inclinometer attitude data.
%    [EULER_RPH] = RPH2EULER(RPH) computes the Euler angle roll,
%    pitch, and heading used for coordinate transformation matrices
%    from pendulum measured inclinometer roll/pitch data.  The
%    input, RPH, and output, EULER_RPH, are vectors of 
%    [roll,pitch,heading].  All angles are in radians.
%
%    Note: the RDI DVL measures inclinometer roll/pitch.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-02-2002      rme         Created from LLW rph_to_rot_faster.m

% Note on roll/pitch values from RDI dvl from "RDI Coordinate
% Transformations" publication, April 23 2001 LLW
% 
% RDI Tilt1 is pendulum pitch which is NOT gimbal pitch
% RDI Tilt2 is pendulum roll which is also gimbal roll
% 
% gimbal pitch = arctan(tan(Tilt 1)*cos(Tilt 2))
% gimbal pitch = arctan(tan(pendulum pitch)*cos(roll))
% gimbal roll  = tilt roll = Tilt2

% inclinometer roll and heading are unaffected  
euler_rph = inclinometer_rph;

% raw instrument pitch, corrected from pendulum pitch to gimbal pitch
euler_rph(2) = atan(tan(inclinometer_rph(2)) * cos(inclinometer_rph(1)));
