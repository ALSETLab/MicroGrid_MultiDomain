within MicroGrid.Optimization.MPComponents;
model PQvar
  "Equations come from the mathematical separation in between reals and imaginary of S=P+jQ=UI*"
  extends OpenIPSL.Electrical.Loads.PSAT.BaseClasses.baseLoad;
  parameter OpenIPSL.Types.Time t_start_1=1
    "Start time of first load variation"
    annotation (Dialog(group="Variation 1"));
  parameter OpenIPSL.Types.Time t_end_1=2 "End time of first load variation"
    annotation (Dialog(group="Variation 1"));
  parameter OpenIPSL.Types.ActivePower dP1=0 "First active load variation"
    annotation (Dialog(group="Variation 1"));
  parameter OpenIPSL.Types.ReactivePower dQ1=0 "First reactive load variation"
    annotation (Dialog(group="Variation 1"));
  parameter OpenIPSL.Types.Time t_start_2=2
    "Start time of second Load variation"
    annotation (Dialog(group="Variation 2"));
  parameter OpenIPSL.Types.Time t_end_2=3 "End time of second load variation"
    annotation (Dialog(group="Variation 2"));
  parameter OpenIPSL.Types.ActivePower dP2=0 "Second active load variation"
    annotation (Dialog(group="Variation 2"));
  parameter OpenIPSL.Types.ReactivePower dQ2=0 "Second reactive load variation"
    annotation (Dialog(group="Variation 2"));
protected
  OpenIPSL.Types.PerUnit Pd(start=P_0/S_b) "active
  power demand";
  OpenIPSL.Types.PerUnit Qd(start=Q_0/S_b) "reactive power demand";
equation

    P = Pd;
    Q = Qd;

  if time >= t_start_1 and time < t_end_1 then
    Pd = smooth(2, (P_0 + dP1*(time-t_start_1))/S_b);
    Qd = smooth(2, (Q_0 + dQ1*(time-t_start_1))/S_b);
  elseif time >= t_start_2 and time < t_end_2 then
    Pd = smooth(2, (P_0 + dP1*(time-t_start_1) + 2*dP2*(time-t_start_2))/S_b);
    Qd = smooth(2, (Q_0 + dP1*(time-t_start_1) + 2*dQ2*(time-t_start_2))/S_b);
  else
    Pd = P_0/S_b;
    Qd = Q_0/S_b;
  end if;
 annotation (
    Evaluate=true,
    choices(checkBox=true),
    Dialog(tab="To Be Implemented"));
end PQvar;
