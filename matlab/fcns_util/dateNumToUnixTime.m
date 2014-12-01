function unixTime = dateNumtoUnixTime( dateNumTime )
% Convert Matlab datenum format time to Unix UTC time
    unixTime = (dateNumTime - datenum('1970-1-1 00:00:00'))*24*3600;
