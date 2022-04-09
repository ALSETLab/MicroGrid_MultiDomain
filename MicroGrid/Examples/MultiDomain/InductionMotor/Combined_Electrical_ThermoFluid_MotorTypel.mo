within MicroGrid.Examples.MultiDomain.InductionMotor;
model Combined_Electrical_ThermoFluid_MotorTypel
  import MicroGrid;
    extends Modelica.Icons.Example;

  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000,
                  fn=60)
    annotation (Placement(transformation(extent={{126,70},{166,90}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=380)   annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-184,32})));
  Electrical.InductionMotor.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC_uninitialized
    aC_2_DC_and_DC_2_AC(
    V_b=380,
    P_0=50000000,
    Q_0=10000000,
    Cdc=0.02) annotation (Placement(transformation(extent={{-96,12},{-56,52}})));
  OpenIPSL.Electrical.Buses.Bus Grid(
    V_b=380,
    v_0=1.04169,
    angle_0=0.0019198621771938)     annotation (Placement(transformation(extent={{-168,22},
            {-148,42}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=0.001,
    X=0.01,
    G=0,
    B=0.1)
    annotation (Placement(transformation(extent={{-144,22},{-124,42}})));
  OpenIPSL.Electrical.Buses.Bus Bus_VSD(
    V_b=380,
    v_0=1.04273,
    angle_0=0.0018151424220741)
    annotation (Placement(transformation(extent={{-120,22},{-100,42}})));
    OpenIPSL.Electrical.Events.PwFault fault(
    t1=100,
    t2=101,
    R=0.01,
    X=0.01) annotation (Placement(transformation(extent={{-142,46},{-122,66}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia_of_the_pump(J=2.6485,
      w(start=188.275))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={22,32})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{-16,22},{4,42}})));
  Electrical.InductionMotor.VariableSpeedDrive.Controls.VoltsHertz_Controller
    volts_Hertz_Control(
    V_b=380,
    Kf=0.7/188.275,
    Kp=1,
    Ki=0.1,
    we_max=300,
    we_min=150)
    annotation (Placement(transformation(extent={{-96,-40},{-60,0}})));

  Modelica.Blocks.Sources.RealExpression Water_Flow_Ref(y=243.912)
    annotation (Placement(transformation(extent={{-180,-18},{-160,2}})));
  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{174,70},{194,90}})));

  MicroGrid.MultiDomain.InductionMotor.ThreePhase.PSAT.MotorTypel_MultiDomain_Full
    MotorTypel(
    V_b=380,
    P_0(displayUnit="W") = 35836.1,
    Q_0(displayUnit="var") = 103714,
    v_0=0.71,
    Rs=0.013,
    Xs=0.14,
    Rr1=0.009,
    Xr1=0.12,
    Xm=2.4,
    Hm=0.8,
    M_b=500000,
    N=2) annotation (Placement(transformation(extent={{-26,22},{-46,42}})));
  Electrical.InductionMotor.VariableSpeedDrive.Controls.pump_controller
    pump_controller(kp=1, mflow_2_speed=188.275/243.912)
    annotation (Placement(transformation(extent={{-150,-40},{-110,0}})));
  Modelica.Fluid.Machines.Pump pump(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_start=243.912,
    redeclare function flowCharacteristic =
        Modelica.Fluid.Machines.BaseClasses.PumpCharacteristics.linearFlow (
         V_flow_nominal={0,0.1}, head_nominal={10,0}),
    N_nominal(displayUnit="rad/s") = 626.91132083898)
    annotation (Placement(transformation(extent={{20,-22},{40,-2}})));
  Modelica.Fluid.Sources.FixedBoundary SOURCE(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    p=300000,
    T=303.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{-4,-22},{16,-2}})));
  Modelica.Fluid.Sources.FixedBoundary SINK(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    p=300000,
    T=303.15,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={162,-26})));
  Modelica.Fluid.Valves.ValveIncompressible clogged_pipe(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    dp_nominal=100000,
    m_flow_nominal=243.912)
    annotation (Placement(transformation(extent={{76,-18},{96,-38}})));
  Modelica.Blocks.Sources.Ramp valveClosing(
    height=-0.5,
    duration=100,
    startTime=1000,
    offset=1)
    annotation (Placement(transformation(extent={{56,-62},{76,-42}})));
  Modelica.Fluid.Pipes.StaticPipe pipe2(
    allowFlowReversal=true,
    length=1000,
    height_ab=0,
    diameter=0.6,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_start=243.912)
    annotation (Placement(transformation(
        origin={111,-28},
        extent={{-9,-10},{11,10}},
        rotation=0)));
  Modelica.Fluid.Pipes.StaticPipe pipe3(
    allowFlowReversal=true,
    length=1,
    height_ab=0,
    diameter=0.1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_start=0)
    annotation (Placement(transformation(
        origin={111,8},
        extent={{-9,10},{11,-10}},
        rotation=0)));
  Modelica.Fluid.Sources.FixedBoundary LEAKAGE(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    p=100000,
    T=303.15,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={162,8})));
  Modelica.Fluid.Valves.ValveIncompressible cracked_pipe(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    dp_nominal=100000,
    m_flow_nominal=243.912)
    annotation (Placement(transformation(extent={{76,-2},{96,18}})));
  Modelica.Fluid.Pipes.StaticPipe pipe1(
    allowFlowReversal=true,
    length=1000,
    height_ab=0,
    diameter=0.6,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow_start=243.912)
                      annotation (Placement(transformation(
        origin={55,-12},
        extent={{-9,-10},{11,10}},
        rotation=0)));
  Modelica.Fluid.Sensors.MassFlowRate Sensor_flow(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
      m_flow_nominal=500)
    annotation (Placement(transformation(extent={{128,-18},{148,-38}})));
  Modelica.Blocks.Sources.Ramp valveOpening(
    height=0.4,
    duration=100,
    startTime=86401,
    offset=0)
    annotation (Placement(transformation(extent={{56,22},{76,42}})));
equation
  connect(Grid.p, pwLine.p)
    annotation (Line(points={{-158,32},{-143,32}},
                                               color={0,0,255}));
  connect(pwLine.n, Bus_VSD.p)
    annotation (Line(points={{-125,32},{-110,32}},
                                                 color={0,0,255}));
  connect(gENCLS.p,Grid. p)
    annotation (Line(points={{-174,32},{-158,32}},
                                               color={0,0,255}));
  connect(Bus_VSD.p, aC_2_DC_and_DC_2_AC.p)
    annotation (Line(points={{-110,32},{-96,32}},
                                                color={0,0,255}));
  connect(torqueSensor.flange_b, inertia_of_the_pump.flange_a)
    annotation (Line(points={{4,32},{12,32}},    color={0,0,0}));
  connect(volts_Hertz_Control.m, aC_2_DC_and_DC_2_AC.m_input) annotation (
     Line(points={{-68,2},{-68,10.3333},{-67.6667,10.3333}},
                                              color={0,0,127}));
  connect(aC_2_DC_and_DC_2_AC.Vc, volts_Hertz_Control.Vc) annotation (
      Line(points={{-84.3333,10.3333},{-84,10.3333},{-84,2}},
        color={0,0,127}));
  connect(aC_2_DC_and_DC_2_AC.n, MotorTypel.p)
    annotation (Line(points={{-56,32},{-46,32}}, color={0,0,255}));
  connect(MotorTypel.flange, torqueSensor.flange_a)
    annotation (Line(points={{-26,32},{-16,32}}, color={0,0,0}));
  connect(torqueSensor.tau, MotorTypel.mech_torque) annotation (Line(
        points={{-14,21},{-14,10},{-30,10},{-30,20}}, color={0,0,127}));
  connect(MotorTypel.wr, volts_Hertz_Control.motor_speed) annotation (
      Line(points={{-42,20},{-42,-10},{-54,-10}}, color={0,0,127}));
  connect(volts_Hertz_Control.we, MotorTypel.we) annotation (Line(
        points={{-54,-30},{-36,-30},{-36,20}}, color={0,0,127}));
  connect(volts_Hertz_Control.W_ref, pump_controller.Wref)
    annotation (Line(points={{-98,-20},{-108,-20}},
                                                color={0,0,127}));
  connect(Water_Flow_Ref.y, pump_controller.m_flow_ref)
    annotation (Line(points={{-159,-8},{-152,-8}},
                                                 color={0,0,127}));
  connect(SOURCE.ports[1],pump. port_a)
    annotation (Line(points={{16,-12},{20,-12}}, color={0,127,255}));
  connect(inertia_of_the_pump.flange_b,pump. shaft) annotation (Line(
        points={{32,32},{38,32},{38,10},{30,10},{30,-2}},       color={0,
          0,0}));
  connect(valveClosing.y,clogged_pipe. opening)
    annotation (Line(points={{77,-52},{86,-52},{86,-36}}, color={0,0,127}));
  connect(clogged_pipe.port_b,pipe2. port_a)
    annotation (Line(points={{96,-28},{102,-28}},
                                               color={0,127,255}));
  connect(pipe1.port_b, clogged_pipe.port_a) annotation (Line(points={{66,-12},{
          72,-12},{72,-28},{76,-28}},            color={0,127,255}));
  connect(cracked_pipe.port_a, clogged_pipe.port_a) annotation (Line(
        points={{76,8},{72,8},{72,-28},{76,-28}},       color={0,127,255}));
  connect(pump.port_b, pipe1.port_a)
    annotation (Line(points={{40,-12},{46,-12}}, color={0,127,255}));
  connect(pipe2.port_b, Sensor_flow.port_a)
    annotation (Line(points={{122,-28},{128,-28}}, color={0,127,255}));
  connect(Sensor_flow.port_b, SINK.ports[1])
    annotation (Line(points={{148,-28},{150,-28},{150,-26},{152,-26}},
                                                   color={0,127,255}));
  connect(cracked_pipe.port_b, pipe3.port_a)
    annotation (Line(points={{96,8},{102,8}},    color={0,127,255}));
  connect(pipe3.port_b, LEAKAGE.ports[1])
    annotation (Line(points={{122,8},{152,8}},   color={0,127,255}));
  connect(valveOpening.y, cracked_pipe.opening) annotation (Line(points={{77,32},
          {86,32},{86,16}},            color={0,0,127}));
  connect(fault.p, Grid.p) annotation (Line(points={{-143.667,56},{-150,
          56},{-150,32},{-158,32}},
                              color={0,0,255}));
  connect(Sensor_flow.m_flow, pump_controller.m_flow) annotation (Line(
        points={{138,-39},{138,-78},{-160,-78},{-160,-32},{-152,-32}},
        color={0,0,127}));
  annotation (experiment(
      StopTime=20,
      __Dymola_NumberOfIntervals=20000,
      Tolerance=0.01,
      __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-200,-100},{200,100}})),
    Icon(coordinateSystem(extent={{-200,-100},{200,100}})),
    __Dymola_Commands(executeCall=simulateModel(
          "MicroGrid.Induction_Motor.VSD_NEW.Testing_Motors.GM_Example4",
          stopTime=500,
          numberOfIntervals=10000,
          tolerance=1e-05,
          resultFile="GM_Example4") "Electrical Fault Sweep"));
end Combined_Electrical_ThermoFluid_MotorTypel;
