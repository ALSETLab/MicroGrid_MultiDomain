within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model ResidentialLoad03
  import MicroGrid;
  import MicroGrid;
  extends Modelica.Icons.Example;
  import ThermalPower;
  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  ThermalPower.TwoPhase.FlowChannels.DualPipe_dynamicDelay
    dualPipe_dynamicDelay(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    p_start_supply=2400000,
    p_start_return=1000000,
    L=30,
    N_B_supply=1,
    N_B_return=1)
    annotation (Placement(transformation(extent={{20,-58},{40,-38}})));
  inner ThermalPower.System_TPL
                   system_TPL(n_consumers=n_consumer, use_T_ambient_in=true)
    annotation (Placement(transformation(extent={{40,70},{60,90}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932)
    annotation (Placement(transformation(extent={{10,70},{30,90}})));
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
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2(V_b=380)
                                    annotation (Placement(transformation(extent={{-10,-10},
            {10,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData
    annotation (Placement(transformation(extent={{40,40},{80,60}})));
  Electrical.Renewables.WECC.PV_Module_for_irradiance
    pV_Module_for_irradiance(use_irradiance_in=true, angle_0=0.02574992)
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Electrical.Renewables.WECC.Irradiance_to_Power irradiance_to_Power(
      use_irradiance_out=true)
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
  MicroGrid.Thermal_Power.ThermalFluid_Sources.Models.Gas_Turbines.Gen_GT
    gen_GT(M_b=10000) annotation (Placement(transformation(extent={{-40,
            -60},{-20,-40}})));
  Modelica.Blocks.Sources.Constant const(k=1) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-84,-50})));
  MicroGrid.Thermal_Power.ThermalFluid_Sources.Models.Gas_Turbines.ThermalPower_GasTurbine2
    thermalPower_GasTurbine2_1 annotation (Placement(transformation(
          extent={{-66,-60},{-46,-40}})));
  ThermalPower.FlueGas.HeatExchangers.Plate_gas2ph hex(redeclare package
              TwoPhaseMedium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97, redeclare package
      GasMedium = ThermalPower.Media.Gases.MoistFlueGas) annotation (
      Placement(transformation(extent={{-40,-88},{-20,-68}})));
  ThermalPower.FlueGas.SourcesAndSinks.PressureBoundary_pTX FLUE_OUT(
    redeclare package Medium = ThermalPower.Media.Gases.MoistFlueGas,
    p0=1500000,
    T0=800,
    N_ports=1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={2,-72})));
equation
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{31,80},{39,80}},   color={0,0,127}));
  connect(dualPipe_dynamicDelay.portB_supply[1], AggregLoad.portA_supply)
    annotation (Line(points={{40.2,-46},{60,-46}},                   color={0,0,
          255}));
  connect(dualPipe_dynamicDelay.portB_return[1], AggregLoad.portA_return)
    annotation (Line(points={{40,-50},{46,-50},{46,-54},{60,-54}}, color={0,0,255}));
  connect(gENCLS.p,bus. p) annotation (Line(points={{-74,0},{-60,0}},
                    color={0,0,255}));
  connect(bus.p,pwLine. p)
    annotation (Line(points={{-60,0},{-39,0}},   color={0,0,255}));
  connect(pwLine.n,bus2. p)
    annotation (Line(points={{-21,0},{0,0}},     color={0,0,255}));
  connect(pwFault.p,pwLine. p) annotation (Line(points={{-58.3333,30},{
          -54,30},{-54,0},{-39,0}},   color={0,0,255}));
  connect(bus2.p, AggregLoad.p)
    annotation (Line(points={{0,0},{70,0},{70,-40}},   color={0,0,255}));
  connect(pV_Module_for_irradiance.pwPin, bus2.p) annotation (Line(
        points={{-60,60},{-12,60},{-12,0},{0,0}},   color={0,0,255}));
  connect(irradiance_to_Power.irradiance_out, pV_Module_for_irradiance.bambu)
    annotation (Line(points={{-69,80},{-55.6,80},{-55.6,71}}, color={0,
          0,127}));
  connect(gen_GT.pwPin, bus2.p) annotation (Line(points={{-19,-50},{-12,
          -50},{-12,0},{0,0}}, color={0,0,255}));
  connect(thermalPower_GasTurbine2_1.flange_a, gen_GT.shaft)
    annotation (Line(points={{-46,-50},{-40,-50}}, color={0,0,0}));
  connect(thermalPower_GasTurbine2_1.gas_out1, hex.portB_primary)
    annotation (Line(points={{-56,-60},{-56,-72},{-39,-72}}, color={0,
          191,0}));
  connect(hex.portA_primary, FLUE_OUT.port[1])
    annotation (Line(points={{-21,-72},{-7,-72}}, color={0,191,0}));
  connect(hex.portB_secondary, dualPipe_dynamicDelay.portA_supply)
    annotation (Line(points={{-21,-84},{-20,-84},{-20,-90},{14,-90},{14,
          -46},{20,-46}}, color={0,0,255}));
  connect(dualPipe_dynamicDelay.portA_return, hex.portA_secondary)
    annotation (Line(points={{20,-50},{20,-96},{-39,-96},{-39,-84}},
        color={0,0,255}));
  connect(const.y, thermalPower_GasTurbine2_1.fuel_mass_flow)
    annotation (Line(points={{-73,-50},{-68,-50}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=86400,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end ResidentialLoad03;
