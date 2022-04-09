within MicroGrid.Examples.MultiDomain.InductionMotor;
model Water_Reservoir
  import MicroGrid;
  extends BaseClasses.Reservoir_partial(SOURCE(p=300000, T=303.15),
      inertia_of_the_pump(w(start=370)));
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
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=0.01,
    X=0.01,
    G=0,
    B=0.1)
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2(V_b=380)
                                    annotation (Placement(transformation(extent={{-130,
            -10},{-110,10}})));
  MicroGrid.MultiDomain.InductionMotor.ThreePhase.PSAT.MotorTypeIII_MultiDomain_Full
    motorTypeIII_MultiDomain(
    N=2,
    M_b(displayUnit="MVA"),
    V_b=380) annotation (Placement(transformation(extent={{-22,-10},{-42,10}})));
  Modelica.Blocks.Sources.RealExpression we(y=Modelica.Constants.pi*
        SysData.fn)
    annotation (Placement(transformation(extent={{-64,-60},{-44,-40}})));
equation
  connect(gENCLS.p,bus. p) annotation (Line(points={{-174,-7.21645e-16},{-174,0},
          {-160,0}},color={0,0,255}));
  connect(bus.p,pwLine. p)
    annotation (Line(points={{-160,0},{-149,0}}, color={0,0,255}));
  connect(pwLine.n,bus2. p)
    annotation (Line(points={{-131,0},{-120,0}}, color={0,0,255}));
  connect(pwFault.p,pwLine. p) annotation (Line(points={{-139.667,40},{
          -154,40},{-154,0},{-149,0}},color={0,0,255}));
  connect(motorTypeIII_MultiDomain.flange, torqueSensor.flange_a)
    annotation (Line(points={{-22,0},{0,0}}, color={0,0,0}));
  connect(torqueSensor.tau, motorTypeIII_MultiDomain.mech_torque) annotation (
      Line(points={{2,-11},{2,-26},{-26,-26},{-26,-12}}, color={0,0,127}));
  connect(we.y, motorTypeIII_MultiDomain.we)
    annotation (Line(points={{-43,-50},{-32,-50},{-32,-12}}, color={0,0,127}));
  connect(motorTypeIII_MultiDomain.p, bus2.p)
    annotation (Line(points={{-42,0},{-120,0}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-200,-100},{200,100}})), Icon(
        coordinateSystem(extent={{-200,-100},{200,100}})),
    experiment(StopTime=100, __Dymola_Algorithm="Dassl"));
end Water_Reservoir;
