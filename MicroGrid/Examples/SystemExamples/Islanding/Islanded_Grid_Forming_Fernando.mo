within MicroGrid.Examples.SystemExamples.Islanding;
model Islanded_Grid_Forming_Fernando
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
    annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
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
    Q_0=45083300,
    v_0=0.978198,
    angle_0=-0.018654079045315,
    M_b=100000000)
            annotation (Placement(transformation(extent={{-26,-40},{-6,
            -20}})));
  OpenIPSL.Electrical.Buses.Bus GEN1(
    V_b=230,
    v_0=0.978198,
    angle_0=-0.018654079045315)
    annotation (Placement(transformation(extent={{24,-40},{44,-20}})));
  OpenIPSL.Electrical.Sensors.SoftPMU softPMU(V_0=1, angle_0=0)
    annotation (Placement(transformation(extent={{6,10},{26,30}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
    annotation (Placement(transformation(extent={{50,70},{90,90}})));
  Optimization.MPComponents.PQ Load(
    V_b=230,
    P_0=100000000,
    Q_0=60000000,
    v_0=0.974813,
    angle_0=-0.016734216868122)
    annotation (Placement(transformation(extent={{70,-20},{90,0}})));
  Optimization.Control.MPCcontroller CONT(y_start=controller.Pset)
    annotation (Placement(transformation(extent={{-76,-30},{-56,-10}})));
  Modelica.Blocks.Sources.Ramp           ramp(
    height=0.2,
    duration=10,
    startTime=100)
    annotation (Placement(transformation(extent={{-96,-70},{-76,-50}})));
  Optimization.CostFunction.OpenIPSL_Optimizer optimizer(
    costIntegrand_y=(SysData.fn - controller.freq)^2,
    costIntegrand_u=(CONT.dp_u)^2,
    exclude_integrators=true,
    T_optimization=30)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  OpenIPSL.Electrical.Events.Breaker breaker(t_o=10)
    annotation (Placement(transformation(extent={{-38,10},{-18,30}})));
equation
  connect(pwLine2.n,bus1. p)
    annotation (Line(points={{47,20},{54,20}},
                                             color={0,0,255}));
  connect(controller.theta_droop, cVS.Theta_droop)
    annotation (Line(points={{-5.28571,-24},{-1,-24}}, color={0,0,127}));
  connect(controller.Edroop, cVS.E_droop)
    annotation (Line(points={{-5.28571,-36},{-1,-36}}, color={0,0,127}));
  connect(cVS.THETADROOP_0, controller.THETADROOP_0) annotation (Line(points={{4.2,-41},
          {4.2,-46},{-9.57143,-46},{-9.57143,-40.7143}},      color={0,0,127}));
  connect(cVS.EDROOP_0, controller.EDROOP_0) annotation (Line(points={{16,-41},
          {16,-50},{-22.4286,-50},{-22.4286,-40.7143}},color={0,0,127}));
  connect(cVS.P_inv, controller.P_inv) annotation (Line(points={{3,-19},
          {3,-14},{-32,-14},{-32,-22.8571},{-26.7143,-22.8571}},
                                                         color={0,0,127}));
  connect(cVS.Q_inv, controller.Q_inv) annotation (Line(points={{10,-19},
          {10,-10},{-38,-10},{-38,-30},{-26.7143,-30}},
                                               color={0,0,127}));
  connect(cVS.V_inv, controller.V_inv) annotation (Line(points={{17,-19},
          {17,-6},{-46,-6},{-46,-37.1429},{-26.7143,-37.1429}},
                                                        color={0,0,127}));
  connect(cVS.p,GEN1. p)
    annotation (Line(points={{20,-30},{34,-30}},
                                               color={0,0,255}));
  connect(bus.p, softPMU.p)
    annotation (Line(points={{-6,20},{9,20}},  color={0,0,255}));
  connect(softPMU.n, pwLine2.p)
    annotation (Line(points={{23,20},{29,20}},color={0,0,255}));
  connect(bus1.p, Load.p)
    annotation (Line(points={{54,20},{80,20},{80,0}}, color={0,0,255}));
  connect(CONT.u, controller.u1) annotation (Line(points={{-55,-20},{
          -50,-20},{-50,0},{-16,0},{-16,-18.5714}},
                                      color={0,0,127}));
  connect(ramp.y, CONT.dp_u) annotation (Line(points={{-75,-60},{-80,
          -60},{-80,-20},{-78,-20}}, color={0,0,127}));
  connect(gENCLS.p, breaker.s)
    annotation (Line(points={{-50,20},{-38,20}}, color={0,0,255}));
  connect(breaker.r, bus.p)
    annotation (Line(points={{-18,20},{-6,20}}, color={0,0,255}));
  connect(GEN1.p, bus.p) annotation (Line(points={{34,-30},{52,-30},{52,
          0},{-6,0},{-6,20}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end Islanded_Grid_Forming_Fernando;
