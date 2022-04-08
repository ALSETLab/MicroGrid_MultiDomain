within MicroGrid.Examples.SystemExamples.Islanding;
model Load_Following_MPC
  extends Modelica.Icons.Example;
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
    annotation (Placement(transformation(extent={{40,60},{80,80}})));
  Optimization.CostFunction.OpenIPSL_Optimizer optimizer(
    costIntegrand_y=(SysData.fn - controller.freq)^2/100,
    costIntegrand_u=controller.Pset,
    exclude_integrators=false,
    T_optimization=30)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  OpenIPSL.Electrical.Buses.Bus bus(
    V_b=230,
    v_0=1,
    angle_0=0)
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  OpenIPSL.Electrical.Buses.Bus bus1(
    V_b=230,
    v_0=1,
    angle_0=0)
    annotation (Placement(transformation(extent={{40,10},{60,30}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{24,10},{44,30}})));
  Electrical.Renewables.PNNL.GridForming.CVS cVS(
    V_b=230,
    P_0=10000000,
    Q_0=5000000,
    v_0=1,
    angle_0=0,
    M_b=100000000,
    XL=0.05)
    annotation (Placement(transformation(extent={{-4,-40},{16,-20}})));
  Electrical.Renewables.PNNL.GridForming.GFMDRP_A_MPC controller(
    V_b=230,
    P_0=10000000,
    Q_0=5000000,
    v_0=1,
    angle_0=0,
    M_b=100000000,
    mq=0.1,
    Kpv=0,
    Emax=5,
    mp=0.05,
    Pmax=0.5)
    annotation (Placement(transformation(extent={{-30,-40},{-10,-20}})));
  OpenIPSL.Electrical.Buses.Bus GEN1(
    V_b=230,
    v_0=1,
    angle_0=0)
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  OpenIPSL.Electrical.Sensors.SoftPMU softPMU(V_0=1, angle_0=0)
    annotation (Placement(transformation(extent={{2,10},{22,30}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine1(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={46,-10})));
  Optimization.MPComponents.PQvar Load(
    V_b=230,
    P_0=10000000,
    Q_0=5000000,
    t_start_1=10,
    t_end_1=30,
    dP1=5000000,
    dQ1=500000,
    t_start_2=1000,
    t_end_2=10000,
    dP2=-5000000,
    dQ2=-500000)
    annotation (Placement(transformation(extent={{62,-20},{82,0}})));
  Optimization.Control.MPCcontroller CONT(y_start=0.1)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Interfaces.RealInput dp_u_opt
    "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-112,-60},{-72,-20}})));
equation
  connect(pwLine2.n,bus1. p)
    annotation (Line(points={{43,20},{50,20}},
                                             color={0,0,255}));
  connect(controller.theta_droop,cVS. Theta_droop) annotation (Line(
        points={{-9.28571,-24},{-5,-24}}, color={0,0,127}));
  connect(controller.Edroop,cVS. E_droop) annotation (Line(points={{-9.28571,-36},
          {-5,-36}},      color={0,0,127}));
  connect(cVS.THETADROOP_0,controller. THETADROOP_0) annotation (Line(
        points={{0.2,-41},{0.2,-46},{-13.5714,-46},{-13.5714,-40.7143}},
        color={0,0,127}));
  connect(cVS.EDROOP_0,controller. EDROOP_0) annotation (Line(points={{12,-41},
          {12,-50},{-26.4286,-50},{-26.4286,-40.7143}},         color={
          0,0,127}));
  connect(cVS.P_inv,controller. P_inv) annotation (Line(points={{-1,-19},
          {-1,-14},{-36,-14},{-36,-22.8571},{-30.7143,-22.8571}},color=
          {0,0,127}));
  connect(cVS.Q_inv,controller. Q_inv) annotation (Line(points={{6,-19},
          {6,-10},{-42,-10},{-42,-30},{-30.7143,-30}},  color={0,0,127}));
  connect(cVS.V_inv,controller. V_inv) annotation (Line(points={{13,-19},
          {13,-6},{-50,-6},{-50,-37.1429},{-30.7143,-37.1429}}, color={
          0,0,127}));
  connect(cVS.p,GEN1. p)
    annotation (Line(points={{16,-30},{30,-30}}, color={0,0,255}));
  connect(bus.p,softPMU. p)
    annotation (Line(points={{-10,20},{5,20}}, color={0,0,255}));
  connect(softPMU.n,pwLine2. p)
    annotation (Line(points={{19,20},{25,20}},color={0,0,255}));
  connect(GEN1.p,pwLine1. p) annotation (Line(points={{30,-30},{46,-30},{46,-19}},
                     color={0,0,255}));
  connect(pwLine1.n,bus. p) annotation (Line(points={{46,-1},{46,4},{-34,4},{-34,
          20},{-10,20}},color={0,0,255}));
  connect(bus1.p, Load.p)
    annotation (Line(points={{50,20},{72,20},{72,0}}, color={0,0,255}));
  connect(CONT.dp_u,dp_u_opt)  annotation (Line(points={{-82,0},{-86,0},{-86,-20},
          {-66,-20},{-66,-40},{-92,-40}},      color={0,0,127}));
  connect(CONT.u, controller.u1) annotation (Line(points={{-59,0},{-20,
          0},{-20,-18.5714}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end Load_Following_MPC;
