Init Q_pcc setpoint van TSO
droop relation constante bepalen
initialize Q_strings op 0. of setpoint/proportional factor als niet 0 (pro rata) zonder losses
initialize dynamic thresholds
initialize disurbance thresholds
initialize deadband

initialize eerste V setpoint:
V_setpoint = V_pcc  -> Let op V_PCC kan van alles zijn!

Over de lineare droop profiel van modelling: lees af welke Q hoort bij V_setpoint:
Verdeel deze Q over strings D.m.v. RPC EN HOUDT DIT AAN.

Als V dan binnen de groene lijnen blijft, TOP, maar dat doet het waarsch niet.

if Q over dynamic thr
    delta Q = Q_setpoint(i) - Q_pcc(i)
    delta V = delta Q * droop
   
V_setpoint_new = V_setpoint(old)+delta V
Lees af welke Q bij de nieuwe V setpoints hoot


