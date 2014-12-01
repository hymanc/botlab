function dateNumTime = unixTimeToDateNum( unixTime )
% Convert unixTime to Matlab datenum format
    dateNumTime = unixTime/(24*3600) + datenum('1970-1-1 00:00:00');
