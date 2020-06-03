close all
%% Initializations %%
dt = 100e-3;                              % 100 ms sample time
Tfinal = 200;                             % simulation will last 200 s
H_wf = tf([6.075 145],[1 29.17 145]);     % create transfer function of wind farm, by default, the time unit of sys is seconds


t = 0:dt:Tfinal;                          % time vector
u = zeros(length(t), 1);                  % initialize input vector u
%% Setpoint times and values %%
sp_1 = 20e6;                               % first setpoint 20 MW
sp_2 = 40e6;                               % second setpoint 40 MW
t_sp2 = 40;                                % second setpoint at 40 seconds
sp_3 = 30e6;                               % third setpoint 20 MW
t_sp3 = 110;                               % third setpoint at 110 seconds
sp_4 = 10e6;                               % fourth setpoint 30 MW
t_sp4 = 160;                               % fourth setpoint at 160 seconds


%% setting up input vector u containing the setpoints %%
u(1:ceil(t_sp2/dt)) = sp_1;                       % setpoint 1, 0<t<t_sp2
u(ceil(t_sp2/dt)+1:ceil(t_sp3/dt)) = sp_2;       % setpoint 2, t_sp2<t<t_sp3
u(ceil(t_sp3/dt)+1:ceil(t_sp4/dt)) = sp_3;       % setpoint 3, t_sp3<t<t_sp4
u(ceil(t_sp4/dt)+1:end) = sp_4;                   % setpoint 4, t_sp4<t<Tfinal

%% PI controller %%
k_p = 2;
k_i = 0.5;
H_pi = tf([k_p k_i],[1 0]);

%% Total transfer %%
H_tot = (H_wf*(1+H_pi)) / (1+H_pi*H_wf);

%% Simulate the behaviour %%
hold on
%lsim(H_wf,u,t)
result = lsim(H_tot,u,t);
ylim([0 40e6])
xlim([0 0.1])