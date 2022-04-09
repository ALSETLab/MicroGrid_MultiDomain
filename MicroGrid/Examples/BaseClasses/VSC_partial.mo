within MicroGrid.Examples.BaseClasses;
partial model VSC_partial
  extends Modelica.Icons.Example;
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{50,70},{90,90}})));
  OpenIPSL.Electrical.Buses.Bus Grid(V_b=400000)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=400000)   annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-70,0})));
  Electrical.InductionMotor.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC_uninitialized
    AC_2_DC_and_DC_2_AC(
    P_0=0,
    Q_0=0,
    Rdc=0.001,
    Cdc=0.02) annotation (Placement(transformation(extent={{6,-20},{46,20}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=0.001,
    X=0.01,
    G=0,
    B=0)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  OpenIPSL.Electrical.Buses.Bus Bus_VSD(V_b=400000)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  OpenIPSL.Electrical.Events.PwFault Fault(
    R=0.1,
    X=0.1,
    t1=2,
    t2=2.1)
    annotation (Placement(transformation(extent={{-34,20},{-14,40}})));
  Modelica.Blocks.Sources.Pulse          pwm_modulation_index(
    amplitude=-0.6,
    period=5,
    nperiod=1,
    offset=1,
    startTime=10)
    annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
equation
  connect(gENCLS.p,Grid. p) annotation (Line(points={{-60,0},{-50,0}},
                                      color={0,0,255}));
  connect(Grid.p,pwLine. p)
    annotation (Line(points={{-50,0},{-39,0}}, color={0,0,255}));
  connect(pwLine.n,Bus_VSD. p)
    annotation (Line(points={{-21,0},{-10,0}},
                                             color={0,0,255}));
  connect(Bus_VSD.p,AC_2_DC_and_DC_2_AC. p)
    annotation (Line(points={{-10,0},{6,0}},color={0,0,255}));
  connect(Fault.p,pwLine. p) annotation (Line(points={{-35.6667,30},{-42,
          30},{-42,0},{-39,0}}, color={0,0,255}));
  connect(pwm_modulation_index.y, AC_2_DC_and_DC_2_AC.m_input)
    annotation (Line(points={{21,-50},{34.3333,-50},{34.3333,-21.6667}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end VSC_partial;
