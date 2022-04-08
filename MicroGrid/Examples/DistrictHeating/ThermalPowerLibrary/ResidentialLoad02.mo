within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model ResidentialLoad02
  extends Modelica.Icons.Example;
  import ThermalPower;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay
    dualPipe_dynamicDelay(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    L=30,
    N_B_supply=1,
    N_B_return=1)
    annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
  inner ThermalPower.System_TPL
                   system_TPL(n_consumers=n_consumer, use_T_ambient_in=true)
    annotation (Placement(transformation(extent={{40,70},{60,90}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932)
    annotation (Placement(transformation(extent={{10,70},{30,90}})));
  ThermalPower.DistrictHeating.Producers.IdealProducer_dp
                                             cogenerationPlant(
    use_T_in=true,
    use_dp_in=true,
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97)
    annotation (Placement(transformation(extent={{-40,-54},{-20,-34}})));
  ThermalPower.Experiments.SubComponents.Control.DistrictHeatingPressureControl
                                                       control
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  Modelica.Blocks.Sources.Step step_T_supply(
    height=5,
    offset=T_supply,
    startTime=86400)
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
                                           init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=500000,
    p_return=500000)
    annotation (Placement(transformation(extent={{70,70},{90,90}})));
  Electrical.MultiDomain.ThermalElectrical.AggregateEnergyResidence AggregLoad(S_b=
        SysData.S_b)
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  OpenIPSL.Electrical.Buses.Bus bus(
    v_0=1,
    angle_0=0,
    V_b=380)                     annotation (Placement(transformation(extent={{-70,-10},
            {-50,10}})));
  OpenIPSL.Electrical.Events.PwFault pwFault(
    R=0.01,
    X=0.01,
    t1=200000,
    t2=200000000)
           annotation (Placement(transformation(extent={{-60,20},{-80,
            40}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=380)   annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-84,0})));
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=0.01,
    X=0.01,
    G=0,
    B=0.1)
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2(V_b=380)
                                    annotation (Placement(transformation(extent={{-30,-10},
            {-10,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData
    annotation (Placement(transformation(extent={{40,40},{80,60}})));
  Electrical.Renewables.WECC.PV_Module_for_irradiance
    pV_Module_for_irradiance(use_irradiance_in=true, angle_0=0.02574992)
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Electrical.Renewables.WECC.Irradiance_to_Power irradiance_to_Power(
      use_irradiance_out=true)
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
equation
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{31,80},{39,80}},   color={0,0,127}));
  connect(cogenerationPlant.portSupply, dualPipe_dynamicDelay.portA_supply)
    annotation (Line(points={{-20,-48},{0,-48}},
                                             color={0,0,255}));
  connect(cogenerationPlant.portReturn, dualPipe_dynamicDelay.portA_return)
    annotation (Line(points={{-20,-52},{0,-52}},
                                             color={0,0,255}));
  connect(control.dp, cogenerationPlant.dp_in) annotation (Line(points={{-59,-40},
          {-48,-40},{-48,-48},{-41,-48}},
                                     color={0,0,127}));
  connect(step_T_supply.y, cogenerationPlant.T_in) annotation (Line(points={{-59,-70},
          {-48,-70},{-48,-52},{-41,-52}},  color={0,0,127}));
  connect(dualPipe_dynamicDelay.portB_supply[1], AggregLoad.portA_supply)
    annotation (Line(points={{20.2,-48},{46,-48},{46,-46},{60,-46}}, color={0,0,
          255}));
  connect(dualPipe_dynamicDelay.portB_return[1], AggregLoad.portA_return)
    annotation (Line(points={{20,-52},{46,-52},{46,-54},{60,-54}}, color={0,0,255}));
  connect(gENCLS.p,bus. p) annotation (Line(points={{-74,0},{-60,0}},
                    color={0,0,255}));
  connect(bus.p,pwLine. p)
    annotation (Line(points={{-60,0},{-49,0}},   color={0,0,255}));
  connect(pwLine.n,bus2. p)
    annotation (Line(points={{-31,0},{-20,0}},   color={0,0,255}));
  connect(pwFault.p,pwLine. p) annotation (Line(points={{-58.3333,30},{
          -54,30},{-54,0},{-49,0}},   color={0,0,255}));
  connect(bus2.p, AggregLoad.p)
    annotation (Line(points={{-20,0},{70,0},{70,-40}}, color={0,0,255}));
  connect(pV_Module_for_irradiance.pwPin, bus2.p) annotation (Line(
        points={{-60,60},{-26,60},{-26,0},{-20,0}}, color={0,0,255}));
  connect(irradiance_to_Power.irradiance_out, pV_Module_for_irradiance.bambu)
    annotation (Line(points={{-69,80},{-55.6,80},{-55.6,71}}, color={0,
          0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=86400,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end ResidentialLoad02;
