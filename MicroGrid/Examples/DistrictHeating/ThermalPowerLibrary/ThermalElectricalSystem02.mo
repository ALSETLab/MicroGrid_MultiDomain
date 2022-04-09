within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model ThermalElectricalSystem02
  import MicroGrid;
  extends Modelica.Icons.Example;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  replaceable package Medium =
      Modelon.Media.PreDefined.TwoPhase.WaterIF97;
  Modelica.Blocks.Sources.RealExpression realExpression(y=1000)  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={120,-36})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
    init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=300000,
    p_return=300000)
    annotation (Placement(transformation(extent={{52,110},{72,130}})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer consumer1(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium = Medium,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=10000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{78,-50},{98,-30}})));
  inner ThermalPower.System_TPL system_TPL(n_consumers=n_consumer,
      use_T_ambient_in=true) annotation (Placement(transformation(
          extent={{110,110},{130,130}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932,
    offset=30)
    annotation (Placement(transformation(extent={{82,110},{102,130}})));
    Modelica.Blocks.Sources.RealExpression T_boundaryExpr(y=system_TPL.summary.T_ambient)
      annotation (Placement(transformation(extent={{44,-84},{24,-64}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
      annotation (Placement(transformation(extent={{14,-84},{-6,-64}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe hot_water(
    L=100,
    D=0.6,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_supply,
    T_start_out=init.T_supply,
    m_flow_start=1)
    annotation (Placement(transformation(extent={{-26,-34},{-6,-54}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SOURCE(
    p0=init.p_supply,
    T0=333.15,
    N_ports=1) annotation (Placement(transformation(extent={{-120,-60},
            {-100,-40}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SINK(
    p0=init.p_return,
    T0=303.15,
    N_ports=1) annotation (Placement(transformation(extent={{-120,-120},
            {-100,-100}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe cold_water(
    L=100,
    D=0.6,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_return,
    T_start_out=init.T_return,
    m_flow_start=1) annotation (Placement(transformation(extent={{-26,-120},
            {-6,-100}})));
  ThermalPower.TwoPhase.TurboMachinery.Pumps.PumpMech pumpMech(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    redeclare package SatMedium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    usePowerCharacteristic=true,
    V=1,
    pin_start=init.p_supply,
    pout_start=init.p_supply,
    steadyState=true) annotation (Placement(transformation(extent={{-84,
            -60},{-64,-40}})));
  Electrical.InductionMotor.SinglePhase.DPIM dPIM(
    V_b=380,
    init=2,
    Lmainr=0.000588,
    Lmain=0.0806,
    Lauxr=0.000909,
    Laux=0.196,
    Lr=0.0000047,
    Rmain=0.58,
    Rr=0.0000376,
    Raux=3.37,
    Cc=0.0005,
    H=0.0001,
    a=0.0001,
    b=0,
    c=0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={90,-74})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000000) annotation (Placement(transformation(extent={{0,110},
            {40,130}})));
  OpenIPSL.Electrical.Loads.PSSE.Load load(V_b=380, P_0(displayUnit="W") = 600)
             annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={90,-100})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=380)   annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-114,66})));
  Electrical.InductionMotor.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC_uninitialized
    aC_2_DC_and_DC_2_AC(
    V_b=380,
    P_0=50000000,
    Q_0=10000000,
    Cdc=0.02) annotation (Placement(transformation(extent={{-26,46},{14,86}})));
  OpenIPSL.Electrical.Buses.Bus Grid(
    V_b=380,
    v_0=1.04169,
    angle_0=0.0019198621771938)     annotation (Placement(transformation(extent={{-98,56},
            {-78,76}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=0.001,
    X=0.01,
    G=0,
    B=0.1)
    annotation (Placement(transformation(extent={{-74,56},{-54,76}})));
  OpenIPSL.Electrical.Buses.Bus Bus_VSD(
    V_b=380,
    v_0=1.04273,
    angle_0=0.0018151424220741)
    annotation (Placement(transformation(extent={{-50,56},{-30,76}})));
    OpenIPSL.Electrical.Events.PwFault fault(
    t1=100,
    t2=101,
    R=0.01,
    X=0.01) annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  Electrical.InductionMotor.VariableSpeedDrive.Controls.VoltsHertz_Controller
    volts_Hertz_Control(
    V_b=380,
    Kf=0.7/188.275,
    Kp=1,
    Ki=0.1,
    we_max=300,
    we_min=150)
    annotation (Placement(transformation(extent={{-26,-6},{10,34}})));
  Modelica.Blocks.Sources.RealExpression Water_Flow_Ref(y=1)
    annotation (Placement(transformation(extent={{-110,16},{-90,36}})));
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
    N=2) annotation (Placement(transformation(extent={{44,56},{24,76}})));
  Electrical.InductionMotor.VariableSpeedDrive.Controls.pump_controller
    pump_controller(kp=1, mflow_2_speed=188.275/2)
    annotation (Placement(transformation(extent={{-80,-6},{-40,34}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{54,56},{74,76}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia_of_the_pump(J=2.6485,
      w(start=188.275))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={92,66})));
  ThermalPower.TwoPhase.Sensors.MassFlowRate massFlowRate(redeclare package
              Medium = Modelon.Media.PreDefined.TwoPhase.WaterIF97)
    annotation (Placement(transformation(extent={{-52,-54},{-32,-34}})));
equation
  connect(consumer1.heatDemand, realExpression.y)
    annotation (Line(points={{99,-36},{109,-36}},
                                               color={0,0,127}));
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{103,120},{109,120}},
                                                 color={0,0,127}));
  connect(SINK.port[1], cold_water.portA)
    annotation (Line(points={{-101,-110},{-26,-110}}, color={0,0,255}));
  connect(cold_water.portB, consumer1.portB) annotation (Line(points={{-6,-110},
          {70,-110},{70,-48},{78,-48}}, color={0,0,255}));
  connect(SOURCE.port[1], pumpMech.feed)
    annotation (Line(points={{-101,-50},{-82.2,-50}},color={0,0,255}));
  connect(prescribedTemperature.port, hot_water.q[1])
    annotation (Line(points={{-6,-74},{-16,-74},{-16,-49}}, color={191,0,0}));
  connect(cold_water.q[1], prescribedTemperature.port) annotation (Line(
      points={{-16,-105},{-16,-74},{-6,-74}},
      color={191,0,0},
      thickness=0.5));
  connect(Grid.p,pwLine. p)
    annotation (Line(points={{-88,66},{-73,66}},
                                               color={0,0,255}));
  connect(pwLine.n,Bus_VSD. p)
    annotation (Line(points={{-55,66},{-40,66}}, color={0,0,255}));
  connect(gENCLS.p,Grid. p)
    annotation (Line(points={{-104,66},{-88,66}},
                                               color={0,0,255}));
  connect(Bus_VSD.p,aC_2_DC_and_DC_2_AC. p)
    annotation (Line(points={{-40,66},{-26,66}},color={0,0,255}));
  connect(volts_Hertz_Control.m,aC_2_DC_and_DC_2_AC. m_input) annotation (
     Line(points={{2,36},{2,44.3333},{2.33333,44.3333}},
                                              color={0,0,127}));
  connect(aC_2_DC_and_DC_2_AC.Vc,volts_Hertz_Control. Vc) annotation (
      Line(points={{-14.3333,44.3333},{-14,44.3333},{-14,36}},
        color={0,0,127}));
  connect(aC_2_DC_and_DC_2_AC.n,MotorTypel. p)
    annotation (Line(points={{14,66},{24,66}},   color={0,0,255}));
  connect(MotorTypel.wr,volts_Hertz_Control. motor_speed) annotation (
      Line(points={{28,54},{28,24},{16,24}},      color={0,0,127}));
  connect(volts_Hertz_Control.we,MotorTypel. we) annotation (Line(
        points={{16,4},{34,4},{34,54}},        color={0,0,127}));
  connect(volts_Hertz_Control.W_ref,pump_controller. Wref)
    annotation (Line(points={{-28,14},{-38,14}},color={0,0,127}));
  connect(Water_Flow_Ref.y,pump_controller. m_flow_ref)
    annotation (Line(points={{-89,26},{-82,26}}, color={0,0,127}));
  connect(fault.p,Grid. p) annotation (Line(points={{-73.6667,90},{-80,
          90},{-80,66},{-88,66}},
                              color={0,0,255}));
  connect(MotorTypel.flange, torqueSensor.flange_a)
    annotation (Line(points={{44,66},{54,66}}, color={0,0,0}));
  connect(torqueSensor.flange_b, inertia_of_the_pump.flange_a)
    annotation (Line(points={{74,66},{82,66}}, color={0,0,0}));
  connect(torqueSensor.tau, MotorTypel.mech_torque) annotation (Line(points={{56,
          55},{56,46},{40,46},{40,54}}, color={0,0,127}));
  connect(inertia_of_the_pump.flange_b, pumpMech.flange) annotation (Line(
        points={{102,66},{128,66},{128,-12},{-60,-12},{-60,-48},{-64.8,-48}},
        color={0,0,0}));
  connect(hot_water.portB, consumer1.portA)
    annotation (Line(points={{-6,-44},{78,-44}}, color={0,0,255}));
  connect(massFlowRate.port_b, hot_water.portA)
    annotation (Line(points={{-32,-44},{-26,-44}}, color={0,0,255}));
  connect(pumpMech.drain, massFlowRate.port_a)
    annotation (Line(points={{-66,-44},{-52,-44}}, color={0,0,255}));
  connect(load.p, pwLine.p) annotation (Line(points={{100,-100},{138,-100},{138,
          -132},{-138,-132},{-138,40},{-80,40},{-80,66},{-73,66}}, color={0,0,255}));
  connect(dPIM.p, pwLine.p) annotation (Line(points={{100,-74},{138,-74},{138,-132},
          {-138,-132},{-138,40},{-80,40},{-80,66},{-73,66}}, color={0,0,255}));
  connect(massFlowRate.m_flow, pump_controller.m_flow) annotation (Line(
        points={{-42,-35},{-42,-24},{-100,-24},{-100,2},{-82,2}}, color=
         {0,0,127}));
  connect(prescribedTemperature.T, T_boundaryExpr.y)
    annotation (Line(points={{16,-74},{23,-74}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},
            {140,140}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}}),
        graphics={
        Rectangle(
          extent={{-136,-20},{52,-126}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5),
        Text(
          extent={{-14,-22},{56,-32}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5,
          textString="Exterior Piping"),
        Rectangle(
          extent={{64,-20},{134,-126}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5),
        Text(
          extent={{76,-116},{146,-126}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5,
          textString="Residence")}),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end ThermalElectricalSystem02;
