clear all

% Read .log file
A = importdata('screen.log');

% Plot data
plot((A.data(:,2)),A.data(:,1));

% Data introduction
prompt = 'Input the time of lift-off ';
t1 = input(prompt);
prompt = 'Input the time of initial touchdown ';
t2 = input(prompt);

% Data correction [y = Ax + B]
index_t1 = find(A.data(:,2)==t1);
index_t2 = find(A.data(:,2)==t2);
A_coef = (A.data(index_t1,1)-A.data(index_t2,1))/(t2-t1);
B_coef = A.data(index_t1,1) - (A_coef * t1);
A.data(:,1) = A.data(:,1) - [(A_coef * A.data(:,2)) + B_coef];

% Max search
[ymax, indexymax] = max(A.data(:,1));
tmax = A.data(indexymax,2);

% Gradient calculations
ascendv = (ymax)/(tmax - t1);
descendv = (-ymax)/(t2 - tmax);

% Impact velocity calculation
reg2_index = find(A.data(:,2)==t2) - 10;
reg1_index = reg2_index - 70;
impact_alt = A.data([reg1_index:reg2_index], 1);
impact_time = A.data([reg1_index:reg2_index], 2);
mdl = fitlm(impact_time,impact_alt,'linear')


