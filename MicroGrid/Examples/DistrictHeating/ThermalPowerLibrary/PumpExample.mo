within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model PumpExample
  extends Modelica.Icons.Example;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  replaceable package Medium =
      Modelon.Media.PreDefined.TwoPhase.WaterIF97;
  ThermalPower.TwoPhase.TurboMachinery.Pumps.Pump pump(
    m_flow_nom=1,
    head_nom={60,30,0},
    P_cons={800,1800,2000},
    q_nom={0.0,0.001,0.0015},
    checkValve=true,
    V=0.1,
    steadyState=true,
    redeclare package Medium = Medium,
    pin_start=300000,
    pout_start=800000) annotation (Placement(transformation(extent={{-30,
            -10},{-10,10}}, rotation=0)));
  Modelica.Blocks.Sources.Ramp     rpm(
    height=200,
    duration=100,
    offset=1500,
    startTime=1000)                            annotation (Placement(
        transformation(extent={{-72,30},{-52,50}}, rotation=0)));
  Modelica.Blocks.Sources.RealExpression realExpression(y=100000)
                                                                 annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={82,14})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
    init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=500000,
    p_return=500000)
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer consumer1(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium = Medium,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=50000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{40,0},{60,20}})));
  inner ThermalPower.System_TPL system_TPL(n_consumers=n_consumer,
      use_T_ambient_in=true)
    annotation (Placement(transformation(extent={{76,60},{96,80}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932,
    offset=30)
    annotation (Placement(transformation(extent={{46,60},{66,80}})));
    Modelica.Blocks.Sources.RealExpression T_boundaryExpr(y=system_TPL.summary.T_ambient)
      annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
      annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
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
    annotation (Placement(transformation(extent={{0,16},{20,-4}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SOURCE(
    p0=init.p_supply,
    T0=373.15,
    N_ports=1)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SINK(
    p0=init.p_return,
    T0=303.15,
    N_ports=1) annotation (Placement(transformation(extent={{-90,-70},{
            -70,-50}})));
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
    annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
equation
  connect(rpm.y,pump. in_n) annotation (Line(points={{-51,40},{-26.3,40},{-26.3,
          8.1}},  color={0,0,127}));
  connect(consumer1.heatDemand, realExpression.y)
    annotation (Line(points={{61,14},{71,14}}, color={0,0,127}));
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{67,70},{75,70}},   color={0,0,127}));
  connect(T_boundaryExpr.y, prescribedTemperature.T)
    annotation (Line(points={{-39,-30},{-22,-30}}, color={0,0,127}));
  connect(pump.drain, pipe.portA)
    annotation (Line(points={{-12,6},{0,6}}, color={0,0,255}));
  connect(pipe.portB, consumer1.portA)
    annotation (Line(points={{20,6},{40,6}}, color={0,0,255}));
  connect(pipe.q[1], prescribedTemperature.port) annotation (Line(
      points={{10,1},{10,-30},{0,-30}},
      color={191,0,0},
      thickness=0.5));
  connect(SOURCE.port[1], pump.feed)
    annotation (Line(points={{-71,0},{-28.2,0}}, color={0,0,255}));
  connect(pipe1.q[1], prescribedTemperature.port) annotation (Line(
      points={{10,-55},{10,-30},{0,-30}},
      color={191,0,0},
      thickness=0.5));
  connect(SINK.port[1], pipe1.portA)
    annotation (Line(points={{-71,-60},{0,-60}}, color={0,0,255}));
  connect(pipe1.portB, consumer1.portB) annotation (Line(points={{20,
          -60},{32,-60},{32,2},{40,2}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end PumpExample;
