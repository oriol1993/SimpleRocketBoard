%% Data
% port = '';
% BaudRate = '';
% dataDumpCommand = 9;

%% Read .log file
A = importdata('screen.log');

%% Plot data
plot((A.data(:,2)),A.data(:,1));