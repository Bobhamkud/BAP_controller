function [P_sp_string, time_done, rise_end] = RPC_rise_time( P_sp_string,P_sp_pcc,j, Ntb, Npv, N_rise, rise, rise_end, time_done, P_a_string)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if j>1
    if j == 9510
        swer=1;
    end
    if P_sp_pcc(j) ~= P_sp_pcc(j-1) % detect set point change
        
        % Set time_done to 0 to make sure RPC_time_step_setpoint function won't be used %
        time_done = 0;
        
        % interpolate linearly between %
        % initial set point and final set point %
        for i = 1:1:Ntb+Npv
            rise(:,i) = linspace(P_sp_string(j-1,i), P_sp_string(j,i), N_rise+1);
            P_sp_string((j-1):(j+N_rise-1),i) = rise(:,i);
        end
        
        % in case a set point becomes lower than available power, this extra %
        % amount is equally divided among other turbines and the set point is set %
        % equal to the available power %   

         n=0;
         redivide = 0;
         for l = 1:Ntb+Npv
             if P_sp_string(j,l) > P_a_string(j,l)
                n = n+1;
                redivide = redivide + abs( P_sp_string(j,l) - P_a_string(j,l) );
                P_sp_string(j,l) = P_a_string(j,l);
             end   
         end
         Psubsub = redivide/(Ntb+Npv-n);
         d=0;
         k=0;
         for l = 1:Ntb+Npv
             if ( P_sp_string(j,l) + Psubsub ) < P_a_string(j,l) 
                 k = k + 1;
                 P_sp_string(j,l) = P_sp_string(j,l) + Psubsub;
             else
                 d = d+1;
                 Psubsub = Psubsub + (( abs((P_sp_string(j,l) + Psubsub)-P_a_string(j,l)) ) / (Ntb+Npv-n-k-d));
                 P_sp_string(j,l) = P_a_string(j,l);
             end
         end
        
        rise_end = j+N_rise-1;    
    end
   
    % When rising to the set point is finished, time_done set to 1 % 
    if j == rise_end
        time_done = 1;
    end

end

end

