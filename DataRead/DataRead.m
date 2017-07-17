%% Data
% port = '';
% BaudRate = '';
% dataDumpCommand = 9;

%% Read .log file
A = importdata('screen.log');

%% Plot data
A.data(:,2) = A.data(:,2)/10000;
plot((A.data(:,2)),A.data(:,1));