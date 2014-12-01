function lcmtypeobj = lcmrecv(channel, lcmtype, varargin)
%LCMRECV Listens for a returns a lcmtype on the network.
%   LCMOBJ = LCMRECV(CHANNEL, LCMTYPE) listens on channel CHANNEL
%   for an lcmtype of LCMTYPE and returns a LCMOBJ upon success.
%
%   LCMOBJ = LCMRECV(CHANNEL, LCMTYPE, TIMEOUT) same as above but times out
%   after TIMEOUT as measured in milliseconds.

import rob550.lcmtypes.*

lc = lcm.lcm.LCM.getSingleton();
ag = lcm.lcm.MessageAggregator();

if nargin == 3
    timeout = varargin{1};
    lc.subscribe(channel, ag);
    msg = ag.getNextMessage(timeout);
    lc.unsubscribe(channel, ag);
else
    lc.subscribe(channel, ag);
    while true
        millis_to_wait = 1000;
        msg = ag.getNextMessage(millis_to_wait);
        if length(msg) > 0
            break;
        end
        disp('waiting');
    end
    lc.unsubscribe(channel, ag);
end

if length(msg) > 0
    cmd = sprintf('%s(%s)', lcmtype, 'msg.data');
    lcmtypeobj = eval(cmd);
else
    lcmtypeobj = [];
end
