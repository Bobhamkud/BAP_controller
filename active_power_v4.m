clear
close all
%% Initializations %%

dt = 200e-3;                                    % 10 ms sample time
Tfinal = 200;                                   % simulation will last 200 s
Ntb = 13;                                       % number of wind turbine strings
Npv = 4;                                        % number of PV module strings
t = 0:dt:Tfinal;                                % time vector
P_sp_pcc = zeros(length(t), 1);                 % initialize input vector with P set points
P_a_string = zeros(length(t), Ntb+Npv);         % initialize matrix with available P per string over time
P_sp_string = zeros(length(t),Ntb+Npv);         % initialize active power set point per string over time
P_current_string  = zeros(length(t), Ntb+Npv);  % initialize current active power produced per string over time
a_p = 70;                                       % initialize a_p constant for the distribution factors 
P_pcc = zeros(1,length(t));                     % initialize vector with current PCC value
case_log = zeros(1,length(t));                  % initialize case log 

%% Set point times and values %%

sp_1 = 200;                              % first set point 200 MW
t_sp1 = 0;                               % first set point at 0 seconds
sp_2 = 75;                               % second set point 400 MW
t_sp2 = 40;                              % second set point at 40 seconds
sp_3 = 0;                                % third set point 200 MW
t_sp3 = 110;                             % third set point at 110 seconds
sp_4 = 100;                              % fourth set point 100 MW
t_sp4 = 160;                             % fourth set point at 160 seconds

setpoint_values = [sp_1 sp_2 sp_3 sp_4];    % vector containing set points
setpoint_times = [t_sp1 t_sp2 t_sp3 t_sp4]; % vector containing set point times
%% setting up input vector P_sp_PCC containing the setpoints %%

idx_begin = 1;
for i = 2:length(setpoint_values)
    [val,idx_end]=min(abs(t-setpoint_times(i)));
    P_sp_pcc(idx_begin:idx_end-1)= setpoint_values(i-1);
    idx_begin = idx_end;
end
P_sp_pcc(idx_begin:end) = setpoint_values(end);

%% Initialize wind profile %%

v_profile = zeros(1,length(t));
for i =1:length(v_profile)
v_profile(i) = normrnd(15,1);
end
v_profile(1:end) = 25;
[P_a_string(:,1:Ntb),Qwtg_string_profile] = compute_pq_wtg(v_profile);

%% Initialize irradiance profile  %%

solar_profile = zeros(1,length(t));
for i =1:length(solar_profile)
    solar_profile(i) = normrnd(680,50);
end
[P_a_pv,Qpvg_string_profile] = compute_pq_pvg(solar_profile,Npv);
P_a_string(:,Ntb+1:end) = P_a_pv.';

%% Run simulation %%

% run powerflow % 
system = loadcase('system_17');
current_values = runpf(system);
current_values.gen(6:18,2)
P_current_string(1,1:Ntb) = current_values.gen(6:18,2);
P_current_string(1,Ntb+1:end) = current_values.gen(2:5,2);
P_pcc(1) = -1 * current_values.branch(1,14);
Delta_pcc = P_sp_pcc(1)-P_pcc(1);

% determine which case applies at the start % 
%P_tot_cap = sum(current_values.gen(2:18,2));
P_a_tot = sum(P_a_string(1,:));
case_1 = 0;
case_2 = 0;
case_3 = 0;

if P_sp_pcc(1) < P_a_tot 
    case_1 = 1;
    case_log(1) = 1;
    beta = calc_DF(P_current_string(1,:),P_a_string(1,:),a_p);
    beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
    P_sp_string(1,:) = P_current_string(1,:) + beta*Delta_pcc;
else
    case_2 = 1;
    case_log(1) = 2;
    beta = (P_a_string(1,:) - P_current_string(1,:))/Delta_pcc;
    beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
    P_sp_string(1,:) = P_current_string(1,:) + beta*Delta_pcc;
end

% in case a set point becomes negative, the set point is set to zero %
% and the set points corresponding to other turbines that do produce positive power are %
% reduced with that amount %

% for j = 1:Ntb+Npv
%     if P_sp_string(1,j) < 0
%         for k = 1:Ntb+Npv
%             if k ~= j && P_sp_string(1,k) > abs(P_sp_string(1,j))
%                 P_sp_string(1,k) = P_sp_string(1,k) - abs(P_sp_string(1,j));
%                 P_sp_string(1,j) = 0;
%                 break
%             end
%         end
%     end   
% end


% beta opslaan!!!


% apply set points %
system.gen(6:end,[2,9,10]) = [P_sp_string(1,1:Ntb).' P_sp_string(1,1:Ntb).' P_sp_string(1,1:Ntb).'];
system.gen(2:5,[2,9,10]) = [P_sp_string(1,Ntb+1:end).' P_sp_string(1,Ntb+1:end).' P_sp_string(1,Ntb+1:end).'];
current_values = runpf(system);
P_pcc(2) = -1 * current_values.branch(1,14);

% start the plot % 
%figure(1)
%hold on

% Iterate over all other time steps %

for j = 2:length(t)-1
    j
    
    P_a_tot = sum(P_a_string(j,:));
    
    Delta_pcc = P_sp_pcc(j)-P_pcc(j);
    
    % check for which case applies %
    if P_sp_pcc(j) < P_a_tot && P_sp_pcc(j)<P_sp_pcc(j-1)
        case_1 = 0;
        case_2 = 0;
        case_3 = 1;
        case_log(j) = 3;
    elseif P_sp_pcc(j) < P_a_tot
        case_1 = 0;
        case_2 = 1;
        case_3 = 0;
        case_log(j) = 2;
    else
        case_1 = 1;
        case_2 = 0;
        case_3 = 0;
        case_log(j) = 1;
    end
    
    % determine set points % 
    if case_1 == 1
        
        beta = calc_DF(P_current_string(j,:),P_a_string(j,:),a_p);
        beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
        P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;
        
    elseif case_2 == 1
        
        beta = (P_a_string(j,:) - P_current_string(j,:))/Delta_pcc;
        beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
        P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;
        
    else
        
        % find out which strings are producing at their max
        Ps_op_max = zeros(1,Ntb+Npv);  % initialize vector with info on which strings are producing at their max
        Ns_max = 0;                    % initialize number of strings producing at their max
        for i = 1:Ntb+Npv
            if P_current_string(j,i) >= P_a_string(j,i)
                Ns_max = Ns_max + 1;
                Ps_op_max(i) = 1;
            end
        end
        
        Psub = ( P_sp_pcc(j-1)-P_sp_pcc(j) ) / Ns_max;  % amount of active power that will be deducted
                                                        % from turbines producing at their max
        
        % reduce the output of the wind turbines producing at maximum % 
        for i = 1:Ntb+Npv
            if Ps_op_max(i)== 1
                P_sp_string(j,i) = P_current_string(j,i) - Psub;
            end
        end
        
    end
    
    % in case a set point becomes negative, the set point is set to zero %
    % and the set points corresponding to other turbines that do produce positive power are %
    % reduced with that amount %

%     for l = 1:Ntb+Npv
%         if P_sp_string(1,l) < 0
%             for k = 1:Ntb+Npv
%                 if k ~= l && P_sp_string(1,k) > abs(P_sp_string(1,l))
%                     P_sp_string(1,k) = P_sp_string(1,k) - abs(P_sp_string(1,l));
%                     P_sp_string(1,l) = 0;
%                     break
%                 end
%             end
%         end   
%     end
    
    % apply set points %
    system.gen(6:end,[2,9,10]) = [P_sp_string(j,1:Ntb).' P_sp_string(j,1:Ntb).' P_sp_string(j,1:Ntb).'];
    system.gen(2:5,[2,9,10]) = [P_sp_string(j,Ntb+1:end).' P_sp_string(j,Ntb+1:end).' P_sp_string(j,Ntb+1:end).'];
    current_values = runpf(system);
    P_pcc(j+1) = -1 * current_values.branch(1,14);
    
end

plot(t,P_pcc)
hold on
plot(t,P_sp_pcc)
ylim([-100 400])
figure(2)
plot(t,v_profile)
figure(3)
plot(t,case_log*100)

%% Distribution factor %%
function [ beta ] = calc_DF( P, P_a, a_p)
s = (P_a.^2) ./ (P_a.^2 + a_p);
for k =1:length(s)
    if P(k) >= P_a(k)
        s(k) = 0;
    end
end
if sum(s) == 0
    beta = 0;
else
    beta = (s./sum(s));
end
end