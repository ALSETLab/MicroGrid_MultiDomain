within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model ThermalElectricalSystem01
  extends Modelica.Icons.Example;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  replaceable package Medium =
      Modelon.Media.PreDefined.TwoPhase.WaterIF97;
  Modelica.Blocks.Sources.RealExpression realExpression(y=10000) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={120,-46})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
    init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=500000,
    p_return=400000)
    annotation (Placement(transformation(extent={{52,110},{72,130}})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer consumer1(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium = Medium,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=50000,
    linearFriction=false)
    annotation (Placement(transformation(extent={{78,-60},{98,-40}})));
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
      annotation (Placement(transformation(extent={{30,-94},{10,-74}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
      annotation (Placement(transformation(extent={{0,-94},{-20,-74}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe hot_water(
    L=10,
    D=0.5,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_supply,
    T_start_out=init.T_supply,
    m_flow_start=1) annotation (Placement(transformation(extent={{-40,-44},
            {-20,-64}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SOURCE(
    p0=init.p_supply,
    T0=333.15,
    N_ports=1) annotation (Placement(transformation(extent={{-120,-54},
            {-100,-34}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T SINK(
    p0=init.p_return,
    T0=303.15,
    N_ports=1) annotation (Placement(transformation(extent={{-120,-130},
            {-100,-110}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe cold_water(
    L=100,
    D=0.5,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_return,
    T_start_out=init.T_return,
    m_flow_start=1) annotation (Placement(transformation(extent={{-40,-130},
            {-20,-110}})));
  Modelica.Blocks.Sources.Constant rpm(k=1797.89)
                                               annotation (Placement(
        transformation(extent={{-134,-94},{-114,-74}},
                                                 rotation=0)));
  Modelica.Mechanics.Rotational.Sources.Speed speed
    annotation (Placement(transformation(extent={{-74,-94},{-54,-74}})));
  Modelica.Blocks.Math.UnitConversions.From_rpm from_rpm
    annotation (Placement(transformation(extent={{-104,-94},{-84,-74}})));
  ThermalPower.TwoPhase.TurboMachinery.Pumps.PumpMech pumpMech(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    redeclare package SatMedium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    usePowerCharacteristic=true,
    V=1,
    pin_start=init.p_supply,
    pout_start=init.p_supply,
    steadyState=true,
    n_nom=1797.89) annotation (Placement(transformation(extent={{-84,-70},
            {-64,-50}})));
  Electrical.InductionMotor.SinglePhase.DPIM dPIM(
    V_b=230,
    init=2,
    Lmainr=0.000588,
    Lmain=0.0806,
    Lauxr=0.000909,
    Laux=0.196,
    Lr=0.0000047,
    Rmain=0.58,
    Rr=0.0000376,
    Raux=3.37,
    Cc=0.00005,
    H=0.0001,
    a=0.0001,
    b=0,
    c=0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={90,-84})));
  OpenIPSL.Electrical.Buses.Bus Street_Bus(V_b=230)
    annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=230)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  OpenIPSL.Electrical.Buses.Bus Residence_Bus(V_b=230)
    annotation (Placement(transformation(extent={{-10,30},{10,50}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000000) annotation (Placement(transformation(extent={{0,110},
            {40,130}})));
  OpenIPSL.Electrical.Loads.PSSE.Load load(V_b=230, P_0(displayUnit="W")=
         600)
             annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={90,-110})));
equation
  connect(consumer1.heatDemand, realExpression.y)
    annotation (Line(points={{99,-46},{109,-46}},
                                               color={0,0,127}));
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{103,120},{109,120}},
                                                 color={0,0,127}));
  connect(hot_water.portB, consumer1.portA)
    annotation (Line(points={{-20,-54},{78,-54}}, color={0,0,255}));
  connect(SINK.port[1], cold_water.portA) annotation (Line(points={{-101,
          -120},{-40,-120}}, color={0,0,255}));
  connect(cold_water.portB, consumer1.portB) annotation (Line(points={{
          -20,-120},{70,-120},{70,-58},{78,-58}}, color={0,0,255}));
  connect(SOURCE.port[1], pumpMech.feed)
    annotation (Line(points={{-101,-44},{-92,-44},{-92,-60},{-82.2,-60}},
                                                     color={0,0,255}));
  connect(pumpMech.drain, hot_water.portA)
    annotation (Line(points={{-66,-54},{-40,-54}}, color={0,0,255}));
  connect(gENCLS.p, Street_Bus.p)
    annotation (Line(points={{-80,40},{-60,40}}, color={0,0,255}));
  connect(pwLine2.n, Residence_Bus.p)
    annotation (Line(points={{-21,40},{0,40}}, color={0,0,255}));
  connect(Street_Bus.p, pwLine2.p)
    annotation (Line(points={{-60,40},{-39,40}}, color={0,0,255}));
  connect(dPIM.p, Residence_Bus.p) annotation (Line(points={{100,-84},{
          136,-84},{136,40},{0,40}}, color={0,0,255}));
  connect(prescribedTemperature.port, hot_water.q[1]) annotation (Line(
        points={{-20,-84},{-30,-84},{-30,-59}}, color={191,0,0}));
  connect(cold_water.q[1], prescribedTemperature.port) annotation (Line(
      points={{-30,-115},{-30,-84},{-20,-84}},
      color={191,0,0},
      thickness=0.5));
  connect(prescribedTemperature.T, T_boundaryExpr.y)
    annotation (Line(points={{2,-84},{9,-84}}, color={0,0,127}));
  connect(speed.flange, pumpMech.flange) annotation (Line(points={{-54,-84},{-50,
          -84},{-50,-58},{-64.8,-58}}, color={0,0,0}));
  connect(from_rpm.y, speed.w_ref)
    annotation (Line(points={{-83,-84},{-76,-84}}, color={0,0,127}));
  connect(rpm.y, from_rpm.u)
    annotation (Line(points={{-113,-84},{-106,-84}}, color={0,0,127}));
  connect(load.p, Residence_Bus.p) annotation (Line(points={{100,-110},
          {136,-110},{136,40},{0,40}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},
            {140,140}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}}),
        graphics={
        Rectangle(
          extent={{-136,-30},{52,-136}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5),
        Text(
          extent={{-14,-32},{56,-42}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5,
          textString="Exterior Piping"),
        Rectangle(
          extent={{64,-30},{134,-136}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5),
        Text(
          extent={{76,-126},{146,-136}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5,
          textString="Residence")}),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end ThermalElectricalSystem01;
