within MicroGrid.Thermal.DistrictHeating.BaseClass;
partial model Consumer_partial
  replaceable package Medium =
      ThermalPower.Media.Liquid.ConstantPropertyWater;
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay pipe1(
    L=60,
    redeclare package Medium = Medium,
    T_start_supply=init.T_supply,
    T_start_return=init.T_return,
    p_start_supply=init.p_supply,
    p_start_return=init.p_return,
    from_dp=false,
    D=0.5,
    wallThickness=0.005)
    annotation (Placement(transformation(extent={{-60,-20},{-20,20}})));
  ThermalPower.TwoPhase.Interfaces.FlowPort portA_heat
    annotation (Placement(transformation(extent={{-120,20},{-100,40}})));
  ThermalPower.TwoPhase.Interfaces.FlowPort portA_cool
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));
  PowerSystems.Electrical.Loads.PSAT.TCL tCL
    annotation (Placement(transformation(extent={{60,-80},{80,-60}})));
equation
  connect(pipe1.portA_supply, portA_heat) annotation (Line(points={{-60,
          4},{-68,4},{-68,30},{-110,30}}, color={0,0,255}));
  connect(pipe1.portA_return, portA_cool) annotation (Line(points={{-60,
          -4},{-68,-4},{-68,-30},{-110,-30}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Consumer_partial;
