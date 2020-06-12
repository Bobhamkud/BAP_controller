Als er geen optimisation is!!!:
initialize P available per string over tijd (APC)
initialize Q available die bij P available hoort
count = 0
Opti_switch = FALSE

Initialize Q setpoint profile

for t

error =  Q_setpoint - output
u(t) = VSPI(error)
u(t) + TSO_setpoint

Ben ik stabiel?
if error < waarde
    ++count
    if count = 100
        implement opti setpoints
        Opti_switch = TRUE
else
    count = 0

If Opti_switch = FALSE
Dat verdelen over strings adhv DF (Let op Q_a_i):
    - Can setpoint be reached with generators?
    - regels bedenken verdelen en bij Capabillty limits
    - setpoint feasibility optimisation check
    - Rules voor interrupt naar optimisation
    - component status
    - Last resort: OLTC, Reactor control
Power fow runnen voor nieuwe Q output

Elseif Opti_switch = TRUE
    if error > waarde
        Opti_switch = FALSE
        Interrupt ON
    DF = opti setpoint

++t