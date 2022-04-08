within MicroGrid.Examples.SystemExamples.Islanding;
model Islanded_Grid_Forming_MPC_Simulation
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Buses.Bus bus(
    V_b=230,
    v_0=0.976773,
    angle_0=-0.017717535368695)
    annotation (Placement(transformation(extent={{-16,10},{4,30}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(
    V_b=230,
    P_0=89770000,
    Q_0=7254000,
    v_0=0.976773,
    angle_0=-0.017717535368695)
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  OpenIPSL.Electrical.Buses.Bus bus1(
    V_b=230,
    v_0=0.974813,
    angle_0=-0.016734216868122)
    annotation (Placement(transformation(extent={{44,10},{64,30}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{28,10},{48,30}})));
  Electrical.Renewables.PNNL.GridForming.CVS cVS(
    V_b=230,
    P_0=10000000,
    Q_0=45803300,
    v_0=0.978198,
    angle_0=-0.018654079045315,
    M_b=100000000,
    XL=0.05)
    annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
  Electrical.Renewables.PNNL.GridForming.GFMDRP_A_MPC controller(
    V_b=230,
    P_0=10000000,
    Q_0=45803300,
    v_0=0.978198,
    angle_0=-0.018654079045315,
    M_b=100000000,
    Kpv=0.01,
    mp=0.05)
    annotation (Placement(transformation(extent={{-26,-40},{-6,-20}})));
  OpenIPSL.Electrical.Buses.Bus GEN1(
    V_b=230,
    v_0=0.978198,
    angle_0=-0.018654079045315)
    annotation (Placement(transformation(extent={{24,-40},{44,-20}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
    annotation (Placement(transformation(extent={{40,60},{80,80}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine1(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-10})));
  Optimization.CostFunction.OpenIPSL_Optimizer optimizer(
    costIntegrand_y=(SysData.fn - controller.freq)^2,
    costIntegrand_u=(CONT.dp_u)^2,
    exclude_integrators=false,
    T_optimization=30)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ pQ(
    V_b = 230,
    P_0=100000000,
    Q_0=60000000,
    v_0=0.974813,
    angle_0=-0.016734216868122)
    annotation (Placement(transformation(extent={{70,-20},{90,0}})));
  Optimization.Control.MPCcontroller CONT(y_start=controller.Pset)
    annotation (Placement(transformation(extent={{-74,-20},{-54,0}})));
  OpenIPSL.Electrical.Sensors.SoftPMU softPMU(V_0=1, angle_0=0)
    annotation (Placement(transformation(extent={{4,10},{24,30}})));
  Modelica.Blocks.Sources.Constant const(k=controller.Pset) annotation (
     Placement(transformation(extent={{-90,-60},{-70,-40}})));
  Optimization.MPComponents.Breaker breaker(t_o=10)
    annotation (Placement(transformation(extent={{-28,14},{-16,26}})));
equation
  connect(pwLine2.n,bus1. p)
    annotation (Line(points={{47,20},{54,20}},
                                             color={0,0,255}));
  connect(controller.theta_droop, cVS.Theta_droop) annotation (Line(
        points={{-5.28571,-24},{-1,-24}}, color={0,0,127}));
  connect(controller.Edroop, cVS.E_droop) annotation (Line(points={{
          -5.28571,-36},{-1,-36}}, color={0,0,127}));
  connect(cVS.THETADROOP_0, controller.THETADROOP_0) annotation (Line(
        points={{4.2,-41},{4.2,-46},{-9.57143,-46},{-9.57143,-40.7143}},
        color={0,0,127}));
  connect(cVS.EDROOP_0, controller.EDROOP_0) annotation (Line(points={{16,-41},
          {16,-50},{-22.4286,-50},{-22.4286,-40.7143}},         color={
          0,0,127}));
  connect(cVS.P_inv, controller.P_inv) annotation (Line(points={{3,-19},
          {3,-14},{-32,-14},{-32,-22.8571},{-26.7143,-22.8571}}, color=
          {0,0,127}));
  connect(cVS.Q_inv, controller.Q_inv) annotation (Line(points={{10,-19},
          {10,-10},{-38,-10},{-38,-30},{-26.7143,-30}}, color={0,0,127}));
  connect(cVS.V_inv, controller.V_inv) annotation (Line(points={{17,-19},
          {17,-6},{-46,-6},{-46,-37.1429},{-26.7143,-37.1429}}, color={
          0,0,127}));
  connect(cVS.p,GEN1. p)
    annotation (Line(points={{20,-30},{34,-30}},
                                               color={0,0,255}));
  connect(GEN1.p, pwLine1.p)
    annotation (Line(points={{34,-30},{50,-30},{50,-19}}, color={0,0,255}));
  connect(pQ.p, bus1.p) annotation (Line(points={{80,0},{80,20},{54,20}},
        color={0,0,255}));
  connect(CONT.u, controller.u1) annotation (Line(points={{-53,-10},{
          -50,-10},{-50,0},{-16,0},{-16,-18.5714}},
                                      color={0,0,127}));
  connect(pwLine1.n, bus.p) annotation (Line(points={{50,-1},{50,6},{0,6},{0,20},
          {-6,20}}, color={0,0,255}));
  connect(softPMU.p, bus.p)
    annotation (Line(points={{7,20},{-6,20}}, color={0,0,255}));
  connect(softPMU.n, pwLine2.p)
    annotation (Line(points={{21,20},{29,20}}, color={0,0,255}));
  connect(const.y, CONT.dp_u) annotation (Line(points={{-69,-50},{-60,
          -50},{-60,-28},{-88,-28},{-88,-10},{-76,-10}}, color={0,0,127}));
  connect(gENCLS.p, breaker.s)
    annotation (Line(points={{-40,20},{-28,20}}, color={0,0,255}));
  connect(breaker.r, bus.p)
    annotation (Line(points={{-16,20},{-6,20}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end Islanded_Grid_Forming_MPC_Simulation;
