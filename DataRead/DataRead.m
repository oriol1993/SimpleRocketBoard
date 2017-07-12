%% Data
port = '';
BaudRate = '';
dataDumpCommand = 9;

%% Open comunication
s = serial(port);
fopen(s)
