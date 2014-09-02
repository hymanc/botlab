function lcmsend_dynamixel_command_list_t(position_radians, speeds, max_torques,varargin)
%LCMSEND_DYNAMIXEL_COMMAND_LIST_T  Publishes a dynamixel_command_list_t to LCM.
%   LCMSEND_DYNAMIXEL_COMMAND_LIST_T(POSITION_RADIANS,SPEEDS,MAX_TORQUES)
%   publishes a dynamixel_command_list_t() lcmtype on channel 'ARM_COMMAND'.
%   POSITION_RADIANS is a N-vector of joint position angles in radians [-pi,pi].
%   SPEEDS can either be a N-vector of servo speeds or a scalar speed used for
%   all servos [0,1], similarly for MAX_TORQUES [0,1].
%
%   LCMSEND_DYNAMIXEL_COMMAND_LIST_T(POSITION_RADIANS,SPEEDS,MAX_TORQUES,CHANNEL)
%   publishes on the specified CHANNEL name.

import lcm.lcm.*
import rob550.lcmtypes.*

lc = LCM.getSingleton();

if isscalar(speeds)
    speeds = repmat(speeds, size(position_radians));
end

if isscalar(max_torques)
    max_torques = repmat(max_torques, size(position_radians));
end

if nargin == 3
    channel = 'ARM_COMMAND';
else
    channel = varargin{1};
end

cmds = dynamixel_command_list_t();
cmds.len = length(position_radians);
cmds.commands = javaArray('rob550.lcmtypes.dynamixel_command_t', cmds.len);
utime = utime_now();
for k=1:cmds.len
    cmd = dynamixel_command_t();
    cmd.utime = utime;
    cmd.position_radians = position_radians(k);
    cmd.speed = speeds(k);
    cmd.max_torque = max_torques(k);

    cmds.commands(k) = cmd;
end
lc.publish(channel, cmds);
