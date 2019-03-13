function selection = loadDataAlertDialog(parentHandle)
    message = sprintf('Loading a ROS data source removes the current data resource and any data with it from the current session. Would you like to proceed?');
    selection = uiconfirm(parentHandle, message, 'Load warning', 'Icon', 'warning');
end