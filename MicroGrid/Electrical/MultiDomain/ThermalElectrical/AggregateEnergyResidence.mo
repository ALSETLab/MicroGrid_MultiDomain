within MicroGrid.Electrical.MultiDomain.ThermalElectrical;
model AggregateEnergyResidence
  extends ThermalPower.DistrictHeating.Interfaces.System(            n_consumers=1);
  parameter Integer N_B_supply=0 annotation(Dialog(connectorSizing=true));
  parameter Integer N_B_return=0 annotation(Dialog(connectorSizing=true));

  Modelica.Units.NonSI.Temperature_degC T_supply_in_C=
      Medium.temperature(Medium.setState_phX(portA_supply.p, inStream(
      portA_supply.h_outflow))) - 273.15;
  Real Q_demand_MW = sum(loadProfile.heatload)/1e6;

  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer
    limitedHeatConsumer(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    use_heatDemand_in=false,
    Q0=loadProfile.heatload[1],
    linearFriction=false)
    annotation (Placement(transformation(extent={{40,0},{60,20}})));
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay
    dualPipe_dynamicDelay(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    N_B_supply=1,
    N_B_return=1)
    annotation (Placement(transformation(extent={{0,-6},{20,14}})));
  OpenIPSL.Interfaces.PwPin p
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
  inner ThermalPower.System_TPL system_TPL(use_T_ambient_in=false)
    annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
  inner OpenIPSL.Electrical.SystemBase SysData
    annotation (Placement(transformation(extent={{-52,70},{-28,90}})));
  parameter OpenIPSL.Types.ApparentPower S_b=baseLoad.SysData.S_b
    "System base power" annotation (Dialog(tab="Electrical Load Parameters"));
  parameter OpenIPSL.Types.ActivePower P_0=1e6 "Initial active power"
    annotation (Dialog(tab="Electrical Load Parameters"));
  parameter OpenIPSL.Types.ReactivePower Q_0=0 "Initial reactive power"
    annotation (Dialog(tab="Electrical Load Parameters"));
  parameter OpenIPSL.Types.PerUnit v_0=1 "Initial voltage magnitude"
    annotation (Dialog(tab="Electrical Load Parameters"));
  parameter OpenIPSL.Types.Angle angle_0=0 "Initial voltage angle"
    annotation (Dialog(tab="Electrical Load Parameters"));
  OpenIPSL.Electrical.Loads.PSSE.Load load(V_b=380)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={70,50})));
equation
  connect(dualPipe_dynamicDelay.portB_supply[1],limitedHeatConsumer. portA)
    annotation (Line(points={{20.2,6},{40,6}},     color={0,0,255}));
  connect(dualPipe_dynamicDelay.portB_return[1],limitedHeatConsumer. portB)
    annotation (Line(points={{20,2},{40,2}},     color={0,0,255}));
  connect(dualPipe_dynamicDelay.portA_supply, portA_supply) annotation (Line(
        points={{0,6},{-54,6},{-54,40},{-100,40}}, color={0,0,255}));
  connect(dualPipe_dynamicDelay.portA_return, portA_return) annotation (Line(
        points={{0,2},{-54,2},{-54,-40},{-100,-40}}, color={0,0,255}));
  connect(load.p, p)
    annotation (Line(points={{60,50},{0,50},{0,100}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-64,26},{78,-24}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end AggregateEnergyResidence;
