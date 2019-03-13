function log = myLog(string)
% myLog Summary of this function goes here
% Detailed explanation goes here
  % get current time
  curTime = datestr(clock, 'yyyy-mm-dd HH:MM:SS');
  log = sprintf('[%s]%s', curTime, string);
end

