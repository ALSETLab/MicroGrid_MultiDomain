within MicroGrid.Examples.SystemExamples.Islanding;
model Islanded_Grid_Following
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Buses.Bus bus(
    V_b=230,
    v_0=1.0019,
    angle_0=0.011976676219988)
    annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=230)
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  OpenIPSL.Electrical.Buses.Bus bus1(
    V_b=230,
    v_0=1.00101,
    angle_0=0.011901400169349)
    annotation (Placement(transformation(extent={{46,-10},{66,10}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  OpenIPSL.Electrical.Buses.Bus GEN1(
    V_b=230,
    v_0=1.0011,
    angle_0=0.011976449327185)
    annotation (Placement(transformation(extent={{54,-60},{74,-40}})));
  OpenIPSL.Electrical.Sensors.SoftPMU softPMU(V_0=1, angle_0=0)
    annotation (Placement(transformation(extent={{8,-10},{28,10}})));
  OpenIPSL.Electrical.Loads.PSSE.Load load(
    V_b=230,
    P_0=50000000,
    Q_0=20000000,
    v_0=1.00101,
    angle_0=0.011901400169349)
    annotation (Placement(transformation(extent={{76,-40},{96,-20}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
    annotation (Placement(transformation(extent={{36,50},{94,90}})));
  Electrical.Renewables.WECC.REECB controller(
    P_0(displayUnit="MW") = 10000000,
    Q_0(displayUnit="Mvar") = 3000000,
    v_0=1,
    angle_0(displayUnit="deg") = 0,
    lvpnt0=0.4,
    pfflag=false,
    vflag=true,
    qflag=false,
    pqflag=false)
    annotation (Placement(transformation(extent={{-40,-62},{12,-38}})));
  Electrical.Renewables.WECC.REGCA REGC(
    P_0(displayUnit="MW") = 10000000,
    Q_0(displayUnit="Mvar") = 3000000,
    v_0=1,
    angle_0=0,
    Lvplsw=true)
    annotation (Placement(transformation(extent={{22,-62},{46,-38}})));
  Modelica.Blocks.Sources.Constant Pref(k=10/100) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-80,-68})));
  Modelica.Blocks.Sources.Constant Qref(k=3/100)
    annotation (Placement(transformation(extent={{-90,-38},{-70,-18}})));
  Optimization.MPComponents.Breaker breaker(t_o=10)
    annotation (Placement(transformation(extent={{-26,-6},{-14,6}})));
equation
  connect(pwLine2.n,bus1. p)
    annotation (Line(points={{49,0},{56,0}}, color={0,0,255}));
  connect(bus.p, softPMU.p)
    annotation (Line(points={{-4,0},{11,0}},   color={0,0,255}));
  connect(softPMU.n, pwLine2.p)
    annotation (Line(points={{25,0},{31,0}},  color={0,0,255}));
  connect(GEN1.p, softPMU.p) annotation (Line(points={{64,-50},{70,-50},
          {70,-20},{2,-20},{2,0},{11,0}},
                                      color={0,0,255}));
  connect(bus1.p, load.p)
    annotation (Line(points={{56,0},{86,0},{86,-20}}, color={0,0,255}));
  connect(REGC.V_t,controller. Vt) annotation (Line(points={{28,
          -37.1429},{28,-32},{-44,-32},{-44,-41.4286},{-41.625,-41.4286}},
        color={0,0,127}));
  connect(REGC.Pgen,controller. Pe) annotation (Line(points={{34,
          -37.1429},{34,-28},{-48,-28},{-48,-45.7143},{-41.625,-45.7143}},
                                                     color={0,0,127}));
  connect(REGC.Qgen,controller. Qgen) annotation (Line(points={{40,
          -37.1429},{40,-24},{-52,-24},{-52,-50},{-41.625,-50}},
                      color={0,0,127}));
  connect(REGC.IP0,controller. IP00) annotation (Line(points={{28.8571,
          -62.8571},{28.8571,-74},{-1,-74},{-1,-63.7143}},   color={0,0,127}));
  connect(REGC.IQ0,controller. IQ00) annotation (Line(points={{39.1429,
          -62.8571},{39.1429,-78},{-27,-78},{-27,-63.7143}},   color={0,0,
          127}));
  connect(controller.Iqcmd,REGC. Iqcmd) annotation (Line(points={{12.8125,
          -44},{20.2857,-44}},                                      color={
          0,0,127}));
  connect(controller.Ipcmd,REGC. Ipcmd) annotation (Line(points={{12.8125,
          -56},{20.2857,-56}},                              color={0,0,127}));
  connect(REGC.p, GEN1.p) annotation (Line(points={{46.8571,-50},{64,
          -50}}, color={0,0,255}));
  connect(Pref.y, controller.Pref) annotation (Line(points={{-69,-68},{
          -56,-68},{-56,-58.5714},{-41.625,-58.5714}}, color={0,0,127}));
  connect(Qref.y, controller.Qext) annotation (Line(points={{-69,-28},{
          -58,-28},{-58,-54.2857},{-41.625,-54.2857}}, color={0,0,127}));
  connect(gENCLS.p, breaker.s)
    annotation (Line(points={{-38,0},{-26,0}}, color={0,0,255}));
  connect(breaker.r, bus.p)
    annotation (Line(points={{-14,0},{-4,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=30, __Dymola_Algorithm="Dassl"));
end Islanded_Grid_Following;
