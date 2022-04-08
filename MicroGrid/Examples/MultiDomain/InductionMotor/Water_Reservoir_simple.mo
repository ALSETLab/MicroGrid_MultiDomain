within MicroGrid.Examples.MultiDomain.InductionMotor;
model Water_Reservoir_simple
  extends BaseClasses.Reservoir_partial;
  OpenIPSL.Electrical.Buses.Bus bus(
    v_0=1.01688,
    angle_0=-0.023205548668666,
    V_b=380)                     annotation (Placement(transformation(extent={{-170,
            -10},{-150,10}})));
  OpenIPSL.Electrical.Events.PwFault pwFault(
    R=0.01,
    X=0.01,
    t1=200000,
    t2=200000000)
           annotation (Placement(transformation(extent={{-138,30},{-118,50}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=380)   annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-184,0})));
  Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC_uninitialized
    aC_2_DC_and_DC_2_AC(
    V_b=380,
    P_0=10000000,
    Q_0=2500000,
    Rdc=0.001,
    Cdc=0.002) annotation (Placement(transformation(extent={{-100,-20},
            {-60,20}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=0.01,
    X=0.01,
    G=0,
    B=0.1)
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2(V_b=380)
                                    annotation (Placement(transformation(extent={{-130,
            -10},{-110,10}})));
  Electrical.MultiDomain.InductionMotor.ThreePhase.PSAT.MotorTypel_MultiDomain_Simples
    Type1_Motor(V_b=380, N=1)
    annotation (Placement(transformation(extent={{-22,-10},{-42,10}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=1)
    annotation (Placement(transformation(extent={{-120,-60},{-100,-40}})));
equation
  connect(gENCLS.p,bus. p) annotation (Line(points={{-174,-7.21645e-16},{-174,0},
          {-160,0}},color={0,0,255}));
  connect(bus.p,pwLine. p)
    annotation (Line(points={{-160,0},{-149,0}}, color={0,0,255}));
  connect(pwLine.n,bus2. p)
    annotation (Line(points={{-131,0},{-120,0}}, color={0,0,255}));
  connect(pwFault.p,pwLine. p) annotation (Line(points={{-139.667,40},{
          -154,40},{-154,0},{-149,0}},color={0,0,255}));
  connect(bus2.p,aC_2_DC_and_DC_2_AC. p) annotation (Line(points={{-120,0},{-100,
          0}},                         color={0,0,255}));
  connect(aC_2_DC_and_DC_2_AC.n, Type1_Motor.p)
    annotation (Line(points={{-60,0},{-42,0}}, color={0,0,255}));
  connect(Type1_Motor.flange_b, torqueSensor.flange_a)
    annotation (Line(points={{-22,0},{0,0}}, color={0,0,0}));
  connect(torqueSensor.tau, Type1_Motor.mech_torque) annotation (Line(
        points={{2,-11},{2,-20},{-26,-20},{-26,-12}}, color={0,0,127}));
  connect(realExpression.y, aC_2_DC_and_DC_2_AC.m_input) annotation (
      Line(points={{-99,-50},{-71.6667,-50},{-71.6667,-21.6667}}, color=
         {0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-200,-100},{200,100}})), Icon(
        coordinateSystem(extent={{-200,-100},{200,100}})),
    experiment(StopTime=100, __Dymola_Algorithm="Dassl"));
end Water_Reservoir_simple;
