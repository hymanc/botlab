function sl = lcmrecv_dynamixel_status_list_t(varargin)
%LCMRECV_DYNAMIXEL_STATUS_LIST_T  Listens for a dynamixel_status_list_t from LCM.
%   STATUS_LIST = LCMRECV_DYNAMIXEL_STATUS_LIST_T() listens on channel 'ARM_STATUS'
%   and returns a dynamixel_command_list_t upon success.
%
%   STATUS_LIST = LCMRECV_DYNAMIXEL_STATUS_LIST_T(CHANNEL, TIMEOUT) listens on
%   channel CHANNEL instead and times out after TIMEOUT as measured in
%   milliseconds.

channel = 'ARM_STATUS';
switch nargin
  case 0
    timeout = -1;
  case 2
    channel = varargin{1};
    timeout = varargin{2};
  otherwise
    error('unrecognized number of arguments');
end

if timeout > 0
    status_list = lcmrecv(channel, 'dynamixel_status_list_t', timeout);
else
    status_list = lcmrecv(channel, 'dynamixel_status_list_t');
end

if ~isempty(status_list)
    sl = lcmconv_dynamixel_status_list_t(status_list);
else
    sl = [];
end

%===============================================================================
function sl = lcmconv_dynamixel_status_list_t(status_list)
%LCMCONV_DYNAMIXEL_STATUS_LIST_T  Converts the java object to a Matlab struct.
%   S = LCMCONV_DYNAMIXEL_STATUS_LIST_T(STATUS_LIST) returns a convenient Matlab
%   struct represenation of the java dynamixel_status_list_t STATUS_LIST object.

% alloc memory
z = zeros(1, status_list.len);
sl.len = status_list.len;
sl.utime = z;
sl.error_flags = z;
sl.position_radians = z;
sl.speed = z;
sl.load = z;
sl.voltage = z;
sl.temperature = z;

% stuff struct
for k=1:status_list.len
    sl.utime(k) = status_list.statuses(k).utime;
    sl.error_flags(k) = status_list.statuses(k).error_flags;
    sl.position_radians(k) = status_list.statuses(k).position_radians;
    sl.speed(k) = status_list.statuses(k).speed;
    sl.load(k) = status_list.statuses(k).load;
    sl.voltage(k) = status_list.statuses(k).voltage;
    sl.temperature(k) = status_list.statuses(k).temperature;
end
