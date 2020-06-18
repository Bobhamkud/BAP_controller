close all
clear
%% Initializations %%
dt = 100e-4;                              % 10 ms sample time
Tfinal = 200;                             % simulation will last 200 s
N = 3;                                    % number of wind turbines
dt_DF = 1;                                % 1 s time step distribution factor DF

t = 0:dt:Tfinal;                          % time vector
u = zeros(length(t), 1);                  % initialize input vector u
P_a = zeros(N,1);                         % initialize vector containing available active power per wind turbine
P = zeros(N,1);                           % initialize vector containing currently produced active power per wind turbine

%% Setpoint times and values %%
sp_1 = 20e6;                               % first setpoint 20 MW
sp_2 = 40e6;                               % second setpoint 40 MW
t_sp2 = 40;                                % second setpoint at 40 seconds
sp_3 = 30e6;                               % third setpoint 20 MW
t_sp3 = 110;                               % third setpoint at 110 seconds
sp_4 = 10e6;                               % fourth setpoint 30 MW
t_sp4 = 160;                               % fourth setpoint at 160 seconds

%% setting up input vector u containing the setpoints %%
u(1:ceil(t_sp2/dt)) = sp_1;                      % setpoint 1, 0<t<t_sp2
u(ceil(t_sp2/dt)+1:ceil(t_sp3/dt)) = sp_2;       % setpoint 2, t_sp2<t<t_sp3
u(ceil(t_sp3/dt)+1:ceil(t_sp4/dt)) = sp_3;       % setpoint 3, t_sp3<t<t_sp4
u(ceil(t_sp4/dt)+1:end) = sp_4;                  % setpoint 4, t_sp4<t<Tfinal

%% Creating Control Loop %%

s = tf('s'); % laplace variable s

% PI Controller %
k_p = 2;
k_i = 0.3;
C = pidstd(k_p,k_p/k_i);
C.Inputname = 'e'; 
C.OutputName = 'v';

% Wind turbines %
H_wt_1 = tf([10.01 15.75],[1 11.64 15.75]);    % transfer function wind turbine 1 (from paper)
H_wt_2 = tf([10.01 15.75],[1 11.64 15.75]);    % transfer function wind turbine 2
H_wt_3 = tf([10.01 15.75],[1 11.64 15.75]);    % transfer function wind turbine 3
L_wt_1 = H_wt_1/(1-H_wt_1);                    % transfer function wind turbine 1 including feedback
L_wt_2 = H_wt_2/(1-H_wt_2);                    % transfer function wind turbine 2 including feedback
L_wt_3 = H_wt_3/(1-H_wt_3);                    % transfer function wind turbine 3 including feedback
L_wt_1.InputName = 'a1';
L_wt_2.InputName = 'a2'; 
L_wt_3.InputName = 'a3'; 
L_wt_1.OutputName = 'tb1';
L_wt_2.OutputName = 'tb2';
L_wt_3.OutputName = 'tb3';

% distribution factors %
blk1 = tunableGain('DF1',1,1);  % tunable gain wind turbine 1
blk2 = tunableGain('DF2',1,1);  % tunable gain wind turbine 2
blk3 = tunableGain('DF3',1,1);  % tunable gain wind turbine 3
blk1.InputName = 'v';
blk2.InputName = 'v';
blk3.InputName = 'v';
blk1.OutputName = 'a1';
blk2.OutputName = 'a2';
blk3.OutputName = 'a3';

% summing elements %
sum_in = sumblk('e = Psp - Pwf');
sum2 = sumblk('Pwf = tb1 + tb2 + tb3');

% Inputs, outputs and analysis points % 
input = 'Psp';
output = 'Pwf';
APs = {'tb1','tb2','tb3'};
%% Simulate the behaviour %%

P_a(1:end) = 1e6*[10;10;10];   % initialize available power
P(1:end) = 1e6*[5;5;5];     % intialize currently produced active power
y1_tot = zeros(1, length(t));   % initialize output vector
ytb1_tot = zeros(1, length(t)); % initialize output vector turbine 1
ytb2_tot = zeros(1, length(t)); % initialize output vector turbine 2
ytb3_tot = zeros(1, length(t)); % initialize output vector turbine 3
a_p = 3*10e12;                  % initialize a_p
x0 = zeros(1,13);               % intialize vector containing states
x1 = zeros(1,13);               % intialize vector containing states
x2 = zeros(1,13);               % intialize vector containing states
x3 = zeros(1,13);               % intialize vector containing states
t_end = 1;                      % initialize end time step
u_end = 1;                      % initialize end input step

for i=0:dt_DF:Tfinal
    
    DF = calc_DF( P, P_a, a_p , u, t_end ); % calculate distribution factors
    blk1.Gain.Value = DF(1);
    blk2.Gain.Value = DF(2);
    blk3.Gain.Value = DF(3);

    T = connect(C,L_wt_1,L_wt_2,L_wt_3,blk1,blk2,blk3,sum_in,sum2,input,output,APs); % connect the blocks to
    Twf = getIOTransfer(T,'Psp','Pwf');                                              % obtain transfer
                                                                                     % function wind farm
    Ttb1 = getIOTransfer(T,'Psp','tb1');                                             % transfer turbine 1
    Ttb2 = getIOTransfer(T,'Psp','tb2');                                             % transfer turbine 2
    Ttb3 = getIOTransfer(T,'Psp','tb3');                                             % transfer turbine 3
    %figure(7)
    %pzmap(Twf)
    %pole(Twf)

    if t_end >= (length(t))
        t_end = length(t);
        u_end = length(t);
        [y0,t0,x0]=lsim(ss(Twf),'g',u(u_begin:u_end),t(t_begin:t_end)-t(t_begin),x0(end,:)); % solve whole farm for time interval
        y1_tot(t_begin:t_end) = y0;                                                          % update output vector
        break
    else
        t_begin = ceil((i*dt_DF)/dt) + 1;                         % index corresponding to beginning of time interval
        u_begin = ceil((i*dt_DF)/dt) + 1;                         % index corresponding to beginning of setpoint interval
        t_end = t_begin + ceil(dt_DF/dt);                         % index corresponding to end of time interval 
        u_end = u_begin + ceil(dt_DF/dt);                         % index corresponding to end of setpoint interval
    end
    i
    [y0,t0,x0]=lsim(ss(Twf),'g',u(u_begin:u_end),t(t_begin:t_end)-t(t_begin),x0(end,:)); % solve whole farm for time interval

    % response per turbine including PI % 
    [wt_1_PI_out,~,x1] = lsim(ss(Ttb1),u(u_begin:u_end),t(t_begin:t_end)-t(t_begin),x1(end,:));
    [wt_2_PI_out,~,x2] = lsim(ss(Ttb2),u(u_begin:u_end),t(t_begin:t_end)-t(t_begin),x2(end,:));
    [wt_3_PI_out,~,x3] = lsim(ss(Ttb3),u(u_begin:u_end),t(t_begin:t_end)-t(t_begin),x3(end,:));
    DF
    
    % update output vectors %
    ytb1_tot(t_begin:t_end) = wt_1_PI_out; 
    ytb2_tot(t_begin:t_end) = wt_2_PI_out; 
    ytb3_tot(t_begin:t_end) = wt_3_PI_out; 
    y1_tot(t_begin:t_end) = y0;                                           
    
    P(1:end) = [wt_1_PI_out(end);wt_2_PI_out(end);wt_3_PI_out(end)];     % DF based on produced active power
                                                                         % at the end of the interval
                                                                            
end

plot(t, u)      % plot setpoint values as function of time
hold on
plot(t, y1_tot) % plot total active power output as function of time

% plot setpoint values and active power outputs of individual turbines %
figure(2)
plot(t,u)
hold on
plot(t, ytb1_tot)
figure(3)
plot(t,u)
hold on
plot(t, ytb2_tot)
figure(4)
plot(t,u)
hold on
plot(t, ytb3_tot)
% plot setpoint values and the sum of the active powers generated by the turbines % 
figure(5)
plot(t,u)
hold on
plot(t, ytb1_tot+ytb2_tot+ytb3_tot)

%% Distribution factor %%
function [ DF ] = calc_DF( P, P_a, a_p, u, t_end )

s = ((P_a - P) .* (P_a.^2) .* (sum(P)- u(t_end))^2 ) ./ (P_a.^2 + a_p);
DF = (s./sum(s))';
end