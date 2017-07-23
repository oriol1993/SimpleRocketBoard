clear all

% Read .log file
A = importdata('screen.log');

% Plot data
plot((A.data(:,2)),A.data(:,1));

% Data introduction + search for max
prompt = 'Input the time of lift-off ';
t1 = input(prompt);
prompt = 'Input the time of initial touchdown ';
t2 = input(prompt);

[ymax, indexymax] = max(A.data(:,1));
tmax = A.data(indexymax,2);

% Gradient calculations
