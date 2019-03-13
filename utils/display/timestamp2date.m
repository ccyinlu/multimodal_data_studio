function date = timestamp2date(timestamp)
%TIMESTAMP2DATE Summary of this function goes here
%   Detailed explanation goes here
    date_day = (timestamp + 28800)/86400 + datenum(1970,1,1);
    milliseconds = timestamp - floor(timestamp);
    date_str = datestr(date_day, 'yyyy-mm-dd HH:MM:SS');
    date = sprintf('%s.%03d',date_str, floor(milliseconds*1000));
end

