close all
%% Initializations %%
dt = 100e-3;                              % 100 ms sample time
Tfinal = 200;                             % simulation will last 200 s
N = 3;                                   % number of wind turbines
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

%% PI controller %%
k_p = 2;
k_i = 0.3;
H_pi = tf([k_p k_i],[1 0]);

%% Transfer functions, by default the time unit is seconds %%
H_wf = tf([6.075 145],[1 29.17 145]);          % transfer function wind farm

H_wt_1 = tf([10.01 15.75],[1 11.64 15.75]);    % transfer function wind turbine 1 (from paper)
H_wt_2 = tf([8.47 12.21],[2 10.7 11]);         % transfer function wind turbine 2
H_wt_3 = tf([11.11 17.99],[3 14.44 18.89]);    % transfer function wind turbine 3

H_agg = [H_wt_1/(1 - H_wt_1);H_wt_2/(1 - H_wt_2);H_wt_3/(1 - H_wt_3)]; % vector used for calculation total transfer
                                                                       % function using individual 
                                                                       % wind turbine transfer functions
H_tot_wf = (H_wf*(1+H_pi)) / (1+H_pi*H_wf);    % total transfer function using H_wf
%% Simulate the behaviour %%
P_a(1:end) = 10e6*[30;40;50]; % initialize available power
P(1:end) = 10e6*[15;15;15];   % intialize currently produced active power
y1_tot = zeros(1, length(t)); % initialize output vector
a_p = 3;                      % initialize a_p
x_1 = zeros(1,26);            % intialize vector containing states

for i=0:dt_DF:Tfinal
    
    DF = calc_DF( P, P_a, a_p );                       % calculate distribution factors
    H_agg_d = DF * H_agg;                              % calculate transfer fuction of wind turbines combined
    H_tot_agg = (H_pi*H_agg_d) /(1+H_pi*H_agg_d);      % total transfer function farm using H_agg
    
    t_begin = ceil((i*dt_DF)/dt) + 1;                         % index corresponding to beginning of time interval
    u_begin = ceil((i*dt_DF)/dt) + 1;                         % index corresponding to beginning of setpoint interval
    t_end = t_begin + ceil(dt_DF/dt);                         % index corresponding to end of time interval 
    u_end = u_begin + ceil(dt_DF/dt);                         % index corresponding to end of setpoint interval
    
    [y1,t1,x1]=lsim(ss(H_tot_agg),'g',u(u_begin:u_end),t(t_begin:t_end));   % solve for time interval
    y1_tot(t_begin:t_end) = y1;                                             % update output vector
    P = P(end,:);                                                           % DF based on produced active power
                                                                            % at the end of the interval
end
plot(t, u)      % plot setpoint values as function of time
hold on
plot(t, y1_tot) % plot active power output as function of time

%% Distribution factor %%
function [ DF ] = calc_DF( P, P_a, a_p )
s = ((P_a - P) .* (P_a.^2)) ./ (P_a.^2 + a_p);
DF = (s./sum(s))';
end