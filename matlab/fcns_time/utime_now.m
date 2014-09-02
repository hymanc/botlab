function utime = utime_now()
% UTIME_NOW() returns unix time in microseonds.
    utime = round(dateNumToUnixTime(now)*1e6);
