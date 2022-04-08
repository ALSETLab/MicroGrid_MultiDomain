within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model PumpMechExample02
  extends Modelica.Icons.Example;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  replaceable package Medium =
      Modelon.Media.PreDefined.TwoPhase.WaterIF97;
  Modelica.Blocks.Sources.RealExpression realExpression(y=10000) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={82,-6})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
    init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=500000,
    p_return=400000)
    annotation (Placement(transformation(extent={{-30,60},{-10,80}})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer consumer1(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium = Medium,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=50000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{40,-20},{60,0}})));
  inner ThermalPower.System_TPL system_TPL(n_consumers=n_consumer,
      use_T_ambient_in=true)
    annotation (Placement(transformation(extent={{28,60},{48,80}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932,
    offset=30)
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
    Modelica.Blocks.Sources.RealExpression T_boundaryExpr(y=system_TPL.summary.T_ambient)
      annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
      annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe pipe(
    L=10,
    D=0.5,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_supply,
    T_start_out=init.T_supply,
    m_flow_start=1)
    annotation (Placement(transformation(extent={{0,-4},{20,-24}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SOURCE(
    p0=init.p_supply,
    T0=333.15,
    N_ports=1) annotation (Placement(transformation(extent={{-90,-30},{
            -70,-10}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SINK(
    p0=init.p_return,
    T0=303.15,
    N_ports=1) annotation (Placement(transformation(extent={{-90,-90},{
            -70,-70}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe pipe1(
    L=100,
    D=0.5,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_return,
    T_start_out=init.T_return,
    m_flow_start=1)
    annotation (Placement(transformation(extent={{0,-90},{20,-70}})));
  Modelica.Blocks.Sources.Constant rpm(k=1500) annotation (Placement(
        transformation(extent={{80,10},{60,30}}, rotation=0)));
  Modelica.Mechanics.Rotational.Sources.Speed speed
    annotation (Placement(transformation(extent={{-6,10},{-26,30}})));
  Modelica.Blocks.Math.UnitConversions.From_rpm from_rpm
    annotation (Placement(transformation(extent={{40,10},{20,30}})));
  ThermalPower.TwoPhase.TurboMachinery.Pumps.PumpMech pumpMech(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    redeclare package SatMedium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    usePowerCharacteristic=true,
    V=1,
    pin_start=init.p_supply,
    pout_start=init.p_supply,
    steadyState=true) annotation (Placement(transformation(extent={{-42,
            -30},{-22,-10}})));
equation
  connect(consumer1.heatDemand, realExpression.y)
    annotation (Line(points={{61,-6},{71,-6}}, color={0,0,127}));
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{21,70},{27,70}},   color={0,0,127}));
  connect(T_boundaryExpr.y, prescribedTemperature.T)
    annotation (Line(points={{-39,-50},{-22,-50}}, color={0,0,127}));
  connect(pipe.portB, consumer1.portA)
    annotation (Line(points={{20,-14},{40,-14}}, color={0,0,255}));
  connect(pipe.q[1], prescribedTemperature.port) annotation (Line(
      points={{10,-19},{10,-50},{0,-50}},
      color={191,0,0},
      thickness=0.5));
  connect(pipe1.q[1], prescribedTemperature.port) annotation (Line(
      points={{10,-75},{10,-50},{0,-50}},
      color={191,0,0},
      thickness=0.5));
  connect(SINK.port[1], pipe1.portA)
    annotation (Line(points={{-71,-80},{0,-80}}, color={0,0,255}));
  connect(pipe1.portB, consumer1.portB) annotation (Line(points={{20,
          -80},{32,-80},{32,-18},{40,-18}}, color={0,0,255}));
  connect(rpm.y,from_rpm. u)
    annotation (Line(points={{59,20},{42,20}},            color={0,0,127}));
  connect(speed.w_ref,from_rpm. y)
    annotation (Line(points={{-4,20},{19,20}},   color={0,0,127}));
  connect(SOURCE.port[1], pumpMech.feed)
    annotation (Line(points={{-71,-20},{-40.2,-20}}, color={0,0,255}));
  connect(pumpMech.drain, pipe.portA)
    annotation (Line(points={{-24,-14},{0,-14}}, color={0,0,255}));
  connect(pumpMech.flange, speed.flange) annotation (Line(points={{
          -22.8,-18},{-14,-18},{-14,-8},{-40,-8},{-40,20},{-26,20}},
        color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end PumpMechExample02;
