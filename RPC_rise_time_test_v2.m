function [Q_sp_strings, time_done_Q, rise_end_Q, sp_number] = RPC_rise_time_test_v2( Q_sp_strings,Q_sp_pcc,j, Ntb, Npv, N_rise_Q, rise_Q, rise_end_Q, time_done_Q, Q_available_m, sp_number, setpoint_values, Q_pcc)
% This function incorporates the rise time
% for the active power control

if j>1

    if Q_sp_pcc(j) ~= Q_sp_pcc(j-1) % detect set point change
        
        sp_number = sp_number + 1;
        
        
        % Set time_done to 0 to make sure RPC_time_step_setpoint function won't be used %
        time_done_Q = 0;        
        
        % interpolate linearly between %
        % initial set point and final set point %
        
        for i = 1:1:Ntb+Npv
            rise_Q(:,i) = linspace(Q_sp_strings(j-1,i), Q_sp_strings(j,i), N_rise_Q+1);
            Q_sp_strings((j-1):(j+N_rise_Q-1),i) = rise_Q(:,i);
        end
                
%         for i = 1:1:Ntb+Npv
%             if P_sp_pcc(j) > P_sp_pcc(j-1)
%                 rise(:,i) = linspace(P_sp_string(j-1,i), P_sp_string(j,i), N_rise+1).*(1+(P_sp_pcc(j)/(P_sp_pcc(j)+P_sp_pcc(j-1))).*(1./(1+exp(-0.01.*(linspace(0, N_rise, N_rise+1)-N_rise/2)))));
%             elseif P_sp_pcc(j) < P_sp_pcc(j-1)
%                 rise(:,i) = linspace(P_sp_string(j-1,i), P_sp_string(j,i), N_rise+1).*flip(1./(1+exp(-0.01.*(linspace(0, N_rise, N_rise+1)-N_rise/2))));    
%             end
%             P_sp_string((j-1):(j+N_rise-1),i) = rise(:,i);
%         end
        
        rise_end_Q = j+N_rise_Q-1;    
    end
    
    % in case a set point becomes higher than available power, this extra %
    % amount is equally divided among other turbines and the set point is set %
    % equal to the available power %
    
    z = any(Q_sp_strings(j,:) > Q_available_m(j,:));

    if z == 1
         n=0;
         redivide = 0;
         for l = 1:Ntb+Npv
             if Q_sp_strings(j,l) > Q_available_m(j,l)
                n = n+1;
                redivide = redivide + abs( Q_sp_strings(j,l) - Q_available_m(j,l) );
                Q_sp_strings(j,l) = Q_available_m(j,l);
             end   
         end
         Psubsub = redivide/(Ntb+Npv-n);
         d=0;
         k=0;
         for l = 1:Ntb+Npv
             if ( Q_sp_strings(j,l) + Psubsub ) < Q_available_m(j,l) 
                 k = k + 1;
                 Q_sp_strings(j,l) = Q_sp_strings(j,l) + Psubsub;
             else
                 d = d+1;
                 Psubsub = Psubsub + (( abs((Q_sp_strings(j,l) + Psubsub)-Q_available_m(j,l)) ) / (Ntb+Npv-n-k-d));
                 Q_sp_strings(j,l) = Q_available_m(j,l);
             end
         end
    end
   
    % When rising to the set point is finished, time_done set to 1 % 
    if j == rise_end_Q
        time_done_Q = 1;
    end
    if (sp_number>1) && ( setpoint_values(sp_number) > setpoint_values(sp_number-1) ) && ( Q_pcc(j) > setpoint_values(sp_number) )
        time_done_Q = 1;
    end
    if (sp_number>1) && ( setpoint_values(sp_number) < setpoint_values(sp_number-1) ) && ( Q_pcc(j) < setpoint_values(sp_number) )
        time_done_Q = 1;
    end
end

end

