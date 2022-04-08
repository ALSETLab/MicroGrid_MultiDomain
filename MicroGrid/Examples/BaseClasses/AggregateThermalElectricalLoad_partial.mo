within MicroGrid.Examples.BaseClasses;
partial model AggregateThermalElectricalLoad_partial
  extends Modelica.Icons.Example;
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer
    limitedHeatConsumer(linearFriction=false)
    annotation (Placement(transformation(extent={{60,-40},{80,-20}})));
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay
    dualPipe_dynamicDelay(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    N_B_supply=1,
    N_B_return=1)
    annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
  replaceable OpenIPSL.Electrical.Loads.PSSE.Load baseLoad constrainedby
    OpenIPSL.Electrical.Loads.PSSE.BaseClasses.baseLoad
    annotation (Placement(transformation(extent={{60,20},{80,40}})));
  OpenIPSL.Electrical.Buses.Bus bus
    annotation (Placement(transformation(extent={{30,40},{50,60}})));
equation
  connect(dualPipe_dynamicDelay.portB_supply[1], limitedHeatConsumer.portA)
    annotation (Line(points={{40.2,-34},{60,-34}}, color={0,0,255}));
  connect(dualPipe_dynamicDelay.portB_return[1], limitedHeatConsumer.portB)
    annotation (Line(points={{40,-38},{60,-38}}, color={0,0,255}));
  connect(bus.p, baseLoad.p)
    annotation (Line(points={{40,50},{70,50},{70,40}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end AggregateThermalElectricalLoad_partial;
