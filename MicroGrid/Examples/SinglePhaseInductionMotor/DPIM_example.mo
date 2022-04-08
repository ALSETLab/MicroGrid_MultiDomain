within MicroGrid.Examples.SinglePhaseInductionMotor;
model DPIM_example
  extends Modelica.Icons.Example;

  OpenIPSL.Electrical.Buses.Bus bus(
  V_b = 230)
    annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(
  V_b = 230)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  OpenIPSL.Electrical.Buses.Bus bus1(
  V_b = 230)
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000000) annotation (Placement(transformation(extent={{44,26},
            {84,46}})));
  Electrical.MultiDomain.InductionMotor.SinglePhase.DPIM2 DPIM(
    V_b=230,
    init=2,
    switch_open_speed=0.3,
    poles=2,
    Lmainr=0.000588,
    Lmain=0.0806,
    Lauxr=0.000909,
    Laux=0.196,
    Lr=0.0000047,
    Rmain=0.58,
    Rr=0.0000376,
    Raux=3.37,
    Cc(displayUnit="F") = 0.001,
    H=0.00001,
    a=0.000001,
    b=0,
    c=0) annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  connect(gENCLS.p, bus.p)
    annotation (Line(points={{-40,0},{-20,0}}, color={0,0,255}));
  connect(pwLine2.n, bus1.p)
    annotation (Line(points={{19,0},{40,0}}, color={0,0,255}));
  connect(bus.p, pwLine2.p)
    annotation (Line(points={{-20,0},{1,0}}, color={0,0,255}));
  connect(bus1.p, DPIM.p) annotation (Line(points={{40,0},{60,0}}, color={0,0,255}));
  annotation (                                        experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=10000,
      __Dymola_Algorithm="Dassl"));
end DPIM_example;
