within MicroGrid.Thermal.DistrictHeating.ConsumerLoad.Modelon_ThermalPower;
model CombinationElectricalThermalLoad
  replaceable OpenIPSL.Electrical.Loads.PSSE.Load                 baseLoad
    constrainedby OpenIPSL.Electrical.Loads.PSSE.BaseClasses.baseLoad
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  replaceable package Medium =
      ThermalPower.Media.Liquid.ConstantPropertyWater;
  OpenIPSL.Interfaces.PwPin pwpin
    annotation (Placement(transformation(extent={{-120,40},{-100,60}})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer consumer1(
    use_heatDemand_in=false,
    i=i_subsystem + 1,
    Q0=loadProfile.heatload[2],
    redeclare package Medium = Medium,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=50000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{60,-40},{80,-20}})));
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay pipe1(
    N_B_return=1,
    N_B_supply=1,
    L=60,
    redeclare package Medium = Medium,
    T_start_supply=init.T_supply,
    T_start_return=init.T_return,
    p_start_supply=init.p_supply,
    p_start_return=init.p_return,
    from_dp=false,
    D=0.5,
    wallThickness=0.005)
    annotation (Placement(transformation(extent={{0,-46},{20,-26}})));
  ThermalPower.TwoPhase.Interfaces.FlowPort portA_supply1
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));
  ThermalPower.TwoPhase.Interfaces.FlowPort portA_return1
    annotation (Placement(transformation(extent={{-120,-80},{-100,-60}})));
equation
  connect(baseLoad.p, pwpin)
    annotation (Line(points={{-70,40},{-70,50},{-110,50}},
                                                         color={0,0,255}));
  connect(pipe1.portB_supply[1], consumer1.portA)
    annotation (Line(points={{20.2,-34},{60,-34}}, color={0,0,255}));
  connect(pipe1.portB_return[1], consumer1.portB)
    annotation (Line(points={{20,-38},{60,-38}}, color={0,0,255}));
  connect(pipe1.portA_supply, portA_supply1) annotation (Line(points={{0,-34},
          {-54,-34},{-54,-30},{-110,-30}},
                                      color={0,0,255}));
  connect(pipe1.portA_return, portA_return1) annotation (Line(points={{0,-38},
          {-54,-38},{-54,-70},{-110,-70}},
                                      color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CombinationElectricalThermalLoad;
