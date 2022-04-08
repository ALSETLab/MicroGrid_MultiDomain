within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model ResidentialLoad01
  extends Modelica.Icons.Example;
  import ThermalPower;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay
    dualPipe_dynamicDelay(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    L=3000,
    N_B_supply=2,
    N_B_return=2)
    annotation (Placement(transformation(extent={{0,-6},{20,14}})));
  inner ThermalPower.System_TPL
                   system_TPL(n_consumers=n_consumer, use_T_ambient_in=true)
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932)
    annotation (Placement(transformation(extent={{30,60},{50,80}})));
  ThermalPower.DistrictHeating.Producers.IdealProducer_dp
                                             cogenerationPlant(
    use_T_in=true,
    use_dp_in=true,
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97)
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  ThermalPower.Experiments.SubComponents.Control.DistrictHeatingPressureControl
                                                       control
    annotation (Placement(transformation(extent={{-80,4},{-60,24}})));
  Modelica.Blocks.Sources.Step step_T_supply(
    height=5,
    offset=T_supply,
    startTime=86400)
    annotation (Placement(transformation(extent={{-80,-26},{-60,-6}})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
                                           init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=500000,
    p_return=500000)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer
                                consumer1(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=50000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{40,0},{60,20}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=1000000)
                                                                 annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={82,14})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer
                                consumer2(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=50000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=500000)
                                                                 annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={82,-26})));
equation
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{51,70},{59,70}},   color={0,0,127}));
  connect(cogenerationPlant.portSupply, dualPipe_dynamicDelay.portA_supply)
    annotation (Line(points={{-20,6},{0,6}}, color={0,0,255}));
  connect(cogenerationPlant.portReturn, dualPipe_dynamicDelay.portA_return)
    annotation (Line(points={{-20,2},{0,2}}, color={0,0,255}));
  connect(control.dp, cogenerationPlant.dp_in) annotation (Line(points={{-59,14},
          {-48,14},{-48,6},{-41,6}}, color={0,0,127}));
  connect(step_T_supply.y, cogenerationPlant.T_in) annotation (Line(points={{-59,
          -16},{-48,-16},{-48,2},{-41,2}}, color={0,0,127}));
  connect(dualPipe_dynamicDelay.portB_supply[1], consumer1.portA)
    annotation (Line(points={{20.2,5.5},{30,5.5},{30,6},{40,6}},
                                               color={0,0,255}));
  connect(dualPipe_dynamicDelay.portB_return[1], consumer1.portB)
    annotation (Line(points={{20,1.5},{30,1.5},{30,2},{40,2}},
                                             color={0,0,255}));
  connect(consumer1.heatDemand, realExpression.y)
    annotation (Line(points={{61,14},{71,14}}, color={0,0,127}));
  connect(consumer2.heatDemand, realExpression1.y)
    annotation (Line(points={{61,-26},{71,-26}}, color={0,0,127}));
  connect(consumer2.portA, dualPipe_dynamicDelay.portB_supply[2]) annotation (
      Line(points={{40,-34},{30,-34},{30,6.5},{20.2,6.5}}, color={0,0,255}));
  connect(consumer2.portB, dualPipe_dynamicDelay.portB_return[2]) annotation (
      Line(points={{40,-38},{26,-38},{26,2.5},{20,2.5}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ResidentialLoad01;
