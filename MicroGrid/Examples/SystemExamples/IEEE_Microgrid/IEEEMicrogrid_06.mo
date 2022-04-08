within MicroGrid.Examples.SystemExamples.IEEE_Microgrid;
model IEEEMicrogrid_06 "Model used for regular simulation tasks."
  extends Modelica.Icons.Example;
  import MicroGrid;

  parameter Integer n_consumer = 1;
  parameter Modelica.Units.SI.Temperature T_supply=343.15;
  replaceable package Medium =
  Modelon.Media.PreDefined.TwoPhase.WaterIF97;

  OpenIPSL.Electrical.Buses.Bus CENTRAL_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VCentral,
    angle_0=pf_DATA.voltages.ACentral,
    displayPF=true) annotation (Placement(transformation(
        extent={{-34,-34},{34,34}},
        rotation=0,
        origin={120,-60})));
  OpenIPSL.Electrical.Branches.PwLine load_line(
    R=0.2686,
    X=0.089300,
    G=0,
    B=0)                                        annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={230,-60})));
  OpenIPSL.Electrical.Buses.Bus LOAD_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VLoad,
    angle_0=pf_DATA.voltages.ALoad,
    displayPF=true)               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={280,-60})));
  OpenIPSL.Electrical.Branches.PwLine substation_line_2(
    R=0.0785,
    X=0.0818,
    G=0,
    B=0)                                                annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={70,-40})));
  OpenIPSL.Electrical.Buses.Bus SB_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VSb,
    angle_0=pf_DATA.voltages.ASb,
    displayPF=true) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-20,-60})));
  OpenIPSL.Electrical.Branches.PwLine substation_line_1(
    R=0.04257,
    X=0.0796,
    G=0,
    B=0)                                                annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-70,-40})));
  OpenIPSL.Electrical.Buses.Bus LV_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VGrid,
    angle_0=pf_DATA.voltages.AGrid,
    displayPF=true)               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-160,-60})));
  OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer Transformer(
    CZ=1,
    R=0,
    X=0.057,
    G=0,
    B=0,
    CW=1) annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-200,-60})));
  OpenIPSL.Electrical.Buses.Bus GRID_bus(
    V_b=11000,
    v_0=pf_DATA.voltages.VGrid,
    angle_0=pf_DATA.voltages.AGrid,
    displayPF=true)
               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-240,-60})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS GRID(
    V_b=11000,
    v_0=pf_DATA.voltages.VGrid,
    angle_0=pf_DATA.voltages.AGrid,
    P_0=160150.6,
    Q_0=-82900,
    M_b=9999000000,
    R_a=0.01,
    X_d=0.1)
    annotation (Placement(transformation(extent={{-320,-80},{-280,-40}})));

  inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
    annotation (Placement(transformation(extent={{-320,-260},{-180,-180}})));
  OpenIPSL.Electrical.Buses.Bus BESS_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VBess,
    angle_0=pf_DATA.voltages.ABess,
    displayPF=true)               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={316,356})));
  OpenIPSL.Electrical.Buses.Bus PV_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VPv,
    angle_0=pf_DATA.voltages.APv,
    displayPF=true)               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={216,316})));
  OpenIPSL.Electrical.Buses.Bus DIESEL_bus(
    V_b=400,
    v_0=pf_DATA.voltages.VDiesel,
    angle_0=pf_DATA.voltages.ADiesel,
    displayPF=true)               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={416,356})));
  OpenIPSL.Electrical.Branches.PwLine PV_line(
    R=0.2686,
    X=0.089300,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={180,294})));
  OpenIPSL.Electrical.Branches.PwLine BESS_line(
    R=0.2686,
    X=0.089300,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={300,306})));
    //Q_0=pf_results.machines.Q1_1,
    //P_0=pf_results.machines.P1_1,
  OpenIPSL.Electrical.Buses.Bus Mechanical_Services(
    V_b=400,
    v_0=pf_DATA.voltages.VMulti,
    angle_0=pf_DATA.voltages.AMulti,
    displayPF=true) annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={258,200})));
  replaceable
    MicroGrid.Electrical.MultiDomain.Gas_and_Diesel_Generators.Diesel_Generator
    diesel constrainedby
    MicroGrid.Electrical.MultiDomain.BaseClasses.DieselBase
    annotation (Placement(transformation(extent={{438,338},{474,374}})));
  OpenIPSL.Electrical.Branches.PwLine DG_line(
    R=0.2686,
    X=0.089300,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={400,306})));
  OpenIPSL.Electrical.Branches.PwLine Pump_line(
    R=0.2686,
    X=0.089300,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={200,148})));
  OpenIPSL.Electrical.Events.PwFault LV_SB_Fault(
    R=0.5,
    X=0.5,
    t1=300,
    t2=300.15)
             annotation (Placement(transformation(extent={{-72,-80},{-52,
            -60}})));
  OpenIPSL.Electrical.Events.PwFault SB_CENTRAL_Fault(
    R=0.5,
    X=0.5,
    t1=350,
    t2=350.15)
          annotation (Placement(transformation(extent={{68,-80},{88,-60}})));
  OpenIPSL.Electrical.Branches.PwLine substation_line_3(
    R=2*0.0785,
    X=2*0.0818,
    G=0,
    B=0)                                                annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={50,-100})));
  OpenIPSL.Electrical.Branches.PwLine substation_line_4(
    R=2*0.04257,
    X=2*0.0796,
    G=0,
    B=0)                                                annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-90,-100})));
  OpenIPSL.Electrical.Loads.PSSE.Load load(
    V_b=400,
    P_0=pf_DATA.loads.PLoad,
    Q_0=pf_DATA.loads.QLoad,
    v_0=pf_DATA.voltages.VLoad,
    angle_0=pf_DATA.voltages.ALoad)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={450,-30})));
  MicroGrid.Examples.SystemExamples.Data.Records.pf_DATA pf_DATA
    annotation (Placement(transformation(extent={{-160,-258},{-122,-228}})));
  MicroGrid.Electrical.Renewables.WECC.Irradiance_to_Power
    irradiance_to_Power(derating_factor=1, use_irradiance_out=true)
    annotation (Placement(transformation(extent={{196,356},{216,376}})));
  OpenIPSL.Electrical.Buses.Bus fault_bus_01
    annotation (Placement(transformation(extent={{34,-56},{66,-24}})));
  OpenIPSL.Electrical.Branches.PwLine substation_line_5(
    R=0.0785,
    X=0.0818,
    G=0,
    B=0)                                                annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={32,-40})));
  OpenIPSL.Electrical.Buses.Bus fault_bus_02
    annotation (Placement(transformation(extent={{-106,-56},{-74,-24}})));
  OpenIPSL.Electrical.Branches.PwLine substation_line_6(
    R=0.04257,
    X=0.0796,
    G=0,
    B=0)                                                annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-110,-40})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank(G=0, B=0.02/4)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={450,-246})));
  OpenIPSL.Electrical.Events.Breaker breaker4(enableTrigger=false)
    annotation (Placement(transformation(extent={{400,-256},{420,-236}})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank1(G=0, B=0.02/4)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={450,-216})));
  OpenIPSL.Electrical.Events.Breaker breaker3(enableTrigger=false)
    annotation (Placement(transformation(extent={{400,-226},{420,-206}})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank2(G=0, B=0.02/4)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={450,-186})));
  OpenIPSL.Electrical.Events.Breaker breaker2(enableTrigger=false)
    annotation (Placement(transformation(extent={{400,-196},{420,-176}})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank3(G=0, B=0.02/4)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={450,-156})));
  OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=false)
    annotation (Placement(transformation(extent={{400,-166},{420,-146}})));
  ThermalPower.DistrictHeating.SubComponents.Records.InitHeatingNetwork
    init(
    T_supply=T_supply,
    T_return=303.15,
    p_supply=300000,
    p_return=300000) annotation (Placement(transformation(extent={{-166,
            -214},{-146,-194}})));
  inner ThermalPower.System_TPL system_TPL(n_consumers=
        n_consumer, use_T_ambient_in=true) annotation (Placement(
        transformation(extent={{-108,-214},{-88,-194}})));
  Modelica.Blocks.Sources.Sine groundTemperature(
    amplitude=3,
    f=1/86400,
    phase=-2.0943951023932,
    offset=30) annotation (Placement(transformation(extent={{-136,-214},
            {-116,-194}})));
  MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC_uninitialized
    aC_2_DC_and_DC_2_AC(
    V_b=380,
    P_0=50000000,
    Q_0=10000000,
    Cdc=0.02)
    annotation (Placement(transformation(extent={{296,180},{336,220}})));
  MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Controls.VoltsHertz_Controller
    volts_Hertz_Control(
    V_b=380,
    Kf=0.7/188.275,
    Kp=1,
    Ki=0.1,
    we_max=300,
    we_min=150)
    annotation (Placement(transformation(extent={{294,120},{330,160}})));
  MicroGrid.Electrical.MultiDomain.InductionMotor.ThreePhase.PSAT.MotorTypel_MultiDomain_Full
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
    N=2)
    annotation (Placement(transformation(extent={{366,190},{346,210}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{376,190},{396,210}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia_of_the_pump(J=2.6485,
      w(start=188.275))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={414,200})));
  MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Controls.pump_controller
    pump_controller(kp=1, mflow_2_speed=188.275/2)       annotation (
      Placement(transformation(extent={{242,120},{282,160}})));
  Modelica.Blocks.Sources.RealExpression Water_Flow_Ref(y=1)
    annotation (Placement(transformation(extent={{210,142},{230,162}})));
  Modelica.Blocks.Sources.RealExpression HeatDemand(y=1000) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={450,-90})));
  ThermalPower.DistrictHeating.Consumers.LimitedHeatConsumer
    consumer1(
    use_heatDemand_in=true,
    i=1,
    redeclare package Medium = Medium,
    min_opening=0.00001,
    m_flow_nom=10,
    dp_nom=10000,
    linearFriction=false) annotation (Placement(transformation(extent={
            {490,-104},{470,-84}})));
    Modelica.Blocks.Sources.RealExpression T_boundaryExpr(y=system_TPL.summary.T_ambient)
      annotation (Placement(transformation(extent={{418,36},{398,56}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
      annotation (Placement(transformation(extent={{388,36},{368,56}})));
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
    annotation (Placement(transformation(extent={{348,86},{368,66}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T
    SOURCE(
    p0=init.p_supply,
    T0=333.15,
    N_ports=1)
    annotation (Placement(transformation(extent={{44,90},{64,110}})));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_T
    SINK(
    p0=init.p_return,
    T0=303.15,
    N_ports=1)
    annotation (Placement(transformation(extent={{254,0},{274,20}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe cold_water(
    L=100,
    D=0.6,
    initOpt=Modelon.ThermoFluid.Choices.InitOptions.steadyState,
    p_start_in=init.p_supply,
    p_start_out=init.p_return,
    initFromEnthalpy=false,
    T_start_in=init.T_return,
    T_start_out=init.T_return,
    m_flow_start=1)
    annotation (Placement(transformation(extent={{348,0},{368,20}})));
  ThermalPower.TwoPhase.TurboMachinery.Pumps.PumpMech
    pumpMech(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    redeclare package SatMedium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    usePowerCharacteristic=true,
    V=1,
    pin_start=init.p_supply,
    pout_start=init.p_supply,
    steadyState=true)
    annotation (Placement(transformation(extent={{290,60},{310,80}})));
  MicroGrid.Electrical.MultiDomain.InductionMotor.SinglePhase.DPIM dPIM(
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
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={450,-58})));
  ThermalPower.TwoPhase.Sensors.MassFlowRate massFlowRate(
      redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97)
    annotation (Placement(transformation(extent={{322,66},{342,86}})));
  Modelica.Blocks.Sources.Constant const(k=5) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={30,130})));
  Thermal_Power.ThermalFluid_Sources.Models.Gas_Turbines.Gen_GT
    gen_GT(
    V_b=400,
    M_b=1000000,
    Q_0=585000,
    P_0=5000000,
    v_0=1,
    angle_0=0) annotation (Placement(transformation(extent={{104,120},{
            124,140}})));
  Thermal_Power.ThermalFluid_Sources.Models.Gas_Turbines.GasTurbineExhaust
    gasTurbineExhaust
    annotation (Placement(transformation(extent={{68,120},{88,140}})));
  MicroGrid.Electrical.Renewables.WECC.PV_Module_for_irradiance
    pV_Module_for_irradiance(
    use_irradiance_in=true,
    M_b=5000000,
    P_0=4000000,
    Q_0=1000000,
    v_0=1,
    angle_0=0) annotation (Placement(transformation(extent={{240,296},{
            280,336}})));
  MicroGrid.Electrical.Renewables.WECC.Battery battery annotation (
      Placement(transformation(extent={{340,336},{380,376}})));
equation
  connect(Transformer.p, LV_bus.p)
    annotation (Line(points={{-178,-60},{-160,-60}}, color={0,0,255}));
  connect(Transformer.n, GRID_bus.p)
    annotation (Line(points={{-222,-60},{-240,-60}}, color={0,0,255}));
  connect(GRID.p, GRID_bus.p)
    annotation (Line(points={{-280,-60},{-240,-60}}, color={0,0,255}));
  connect(BESS_line.p, CENTRAL_bus.p) annotation (Line(points={{300,
          295.2},{300,260},{180,260},{180,-60},{120,-60}},
                                                  color={0,0,255}));
  connect(BESS_line.n, BESS_bus.p) annotation (Line(points={{300,316.8},
          {300,356},{316,356}},
                      color={0,0,255}));
  connect(PV_line.n, PV_bus.p) annotation (Line(points={{180,304.8},{
          180,316},{216,316}},
                 color={0,0,255}));
  connect(DIESEL_bus.p, diesel.pwPin)
    annotation (Line(points={{416,356},{438,356}}, color={0,0,255}));
  connect(DG_line.n, DIESEL_bus.p) annotation (Line(points={{400,316.8},
          {400,356},{416,356}},
                      color={0,0,255}));
  connect(DG_line.p, CENTRAL_bus.p) annotation (Line(points={{400,295.2},
          {400,260},{180,260},{180,-60},{120,-60}},
                                                 color={0,0,255}));
  connect(load_line.p, LOAD_bus.p)
    annotation (Line(points={{240.8,-60},{280,-60}}, color={0,0,255}));
  connect(load_line.n, CENTRAL_bus.p)
    annotation (Line(points={{219.2,-60},{120,-60}}, color={0,0,255}));
  connect(Pump_line.n, Mechanical_Services.p) annotation (Line(points={
          {200,158.8},{200,200},{258,200}}, color={0,0,255}));
  connect(PV_line.p, CENTRAL_bus.p) annotation (Line(points={{180,283.2},{180,-60},
          {120,-60}},                            color={0,0,255}));
  connect(Pump_line.p, CENTRAL_bus.p) annotation (Line(points={{200,
          137.2},{200,-60},{120,-60}}, color={0,0,255}));
  connect(substation_line_2.p, CENTRAL_bus.p) annotation (Line(points={{80.8,
          -40},{100,-40},{100,-60},{120,-60}},      color={0,0,255}));
  connect(substation_line_3.p, CENTRAL_bus.p) annotation (Line(points={{60.8,
          -100},{100,-100},{100,-60},{120,-60}},    color={0,0,255}));
  connect(substation_line_3.n, SB_bus.p) annotation (Line(points={{39.2,
          -100},{0,-100},{0,-60},{-20,-60}},
                                   color={0,0,255}));
  connect(substation_line_1.p, SB_bus.p) annotation (Line(points={{-59.2,-40},{-40,
          -40},{-40,-60},{-20,-60}}, color={0,0,255}));
  connect(substation_line_4.p, SB_bus.p) annotation (Line(points={{-79.2,
          -100},{-40,-100},{-40,-60},{-20,-60}},
                                     color={0,0,255}));
  connect(load.p, LOAD_bus.p)
    annotation (Line(points={{440,-30},{360,-30},{360,-60},{280,-60}},
                                                   color={0,0,255}));
  connect(substation_line_5.n, SB_bus.p) annotation (Line(points={{21.2,-40},
          {0,-40},{0,-60},{-20,-60}}, color={0,0,255}));
  connect(fault_bus_01.p, substation_line_2.n)
    annotation (Line(points={{50,-40},{59.2,-40}}, color={0,0,255}));
  connect(fault_bus_01.p, substation_line_5.p)
    annotation (Line(points={{50,-40},{42.8,-40}}, color={0,0,255}));
  connect(SB_CENTRAL_Fault.p, substation_line_2.n) annotation (Line(points={{66.3333,
          -70},{54,-70},{54,-40},{59.2,-40}},           color={0,0,255}));
  connect(fault_bus_02.p, substation_line_1.n)
    annotation (Line(points={{-90,-40},{-80.8,-40}}, color={0,0,255}));
  connect(substation_line_6.p, fault_bus_02.p)
    annotation (Line(points={{-99.2,-40},{-90,-40}}, color={0,0,255}));
  connect(LV_SB_Fault.p, substation_line_1.n) annotation (Line(points={{
          -73.6667,-70},{-86,-70},{-86,-40},{-80.8,-40}}, color={0,0,255}));
  connect(substation_line_6.n, LV_bus.p) annotation (Line(points={{-120.8,
          -40},{-140,-40},{-140,-60},{-160,-60}}, color={0,0,255}));
  connect(substation_line_4.n, LV_bus.p) annotation (Line(points={{-100.8,
          -100},{-140,-100},{-140,-60},{-160,-60}}, color={0,0,255}));
  connect(breaker4.r, capacitor_bank.p)
    annotation (Line(points={{420,-246},{440,-246}}, color={0,0,255}));
  connect(breaker3.r, capacitor_bank1.p)
    annotation (Line(points={{420,-216},{440,-216}}, color={0,0,255}));
  connect(breaker1.r, capacitor_bank3.p)
    annotation (Line(points={{420,-156},{440,-156}}, color={0,0,255}));
  connect(breaker2.r, capacitor_bank2.p)
    annotation (Line(points={{420,-186},{440,-186}}, color={0,0,255}));
  connect(breaker1.s, CENTRAL_bus.p) annotation (Line(points={{400,-156},
          {180,-156},{180,-60},{120,-60}}, color={0,0,255}));
  connect(breaker2.s, CENTRAL_bus.p) annotation (Line(points={{400,-186},
          {180,-186},{180,-60},{120,-60}}, color={0,0,255}));
  connect(breaker3.s, CENTRAL_bus.p) annotation (Line(points={{400,-216},
          {180,-216},{180,-60},{120,-60}}, color={0,0,255}));
  connect(breaker4.s, CENTRAL_bus.p) annotation (Line(points={{400,-246},
          {180,-246},{180,-60},{120,-60}}, color={0,0,255}));
  connect(groundTemperature.y,system_TPL. T_ambient_in)
    annotation (Line(points={{-115,-204},{-109,-204}},
                                                 color={0,0,127}));
  connect(volts_Hertz_Control.m,aC_2_DC_and_DC_2_AC. m_input) annotation (
     Line(points={{322,162},{322,178.333},{324.333,178.333}},
                                              color={0,0,127}));
  connect(aC_2_DC_and_DC_2_AC.n,MotorTypel. p)
    annotation (Line(points={{336,200},{346,200}},
                                                 color={0,0,255}));
  connect(MotorTypel.wr,volts_Hertz_Control. motor_speed) annotation (
      Line(points={{350,188},{350,150},{336,150}},color={0,0,127}));
  connect(volts_Hertz_Control.we,MotorTypel. we) annotation (Line(
        points={{336,130},{356,130},{356,188}},color={0,0,127}));
  connect(MotorTypel.flange,torqueSensor. flange_a)
    annotation (Line(points={{366,200},{376,200}},
                                               color={0,0,0}));
  connect(torqueSensor.flange_b,inertia_of_the_pump. flange_a)
    annotation (Line(points={{396,200},{404,200}},
                                               color={0,0,0}));
  connect(torqueSensor.tau,MotorTypel. mech_torque) annotation (Line(points={{378,189},
          {378,180},{362,180},{362,188}},
                                        color={0,0,127}));
  connect(Mechanical_Services.p, aC_2_DC_and_DC_2_AC.p)
    annotation (Line(points={{258,200},{296,200}}, color={0,0,255}));
  connect(pump_controller.Wref, volts_Hertz_Control.W_ref)
    annotation (Line(points={{284,140},{292,140}}, color={0,0,127}));
  connect(Water_Flow_Ref.y, pump_controller.m_flow_ref)
    annotation (Line(points={{231,152},{240,152}}, color={0,0,127}));
  connect(consumer1.heatDemand, HeatDemand.y)
    annotation (Line(points={{469,-90},{461,-90}}, color={0,0,127}));
  connect(SINK.port[1],cold_water. portA)
    annotation (Line(points={{273,10},{348,10}},      color={0,140,72},
      thickness=0.5));
  connect(prescribedTemperature.port,hot_water. q[1])
    annotation (Line(points={{368,46},{358,46},{358,71}},   color={191,0,0}));
  connect(cold_water.q[1],prescribedTemperature. port) annotation (Line(
      points={{358,15},{358,46},{368,46}},
      color={191,0,0},
      thickness=0.5));
  connect(massFlowRate.port_b,hot_water. portA)
    annotation (Line(points={{342,76},{348,76}},   color={0,0,255}));
  connect(pumpMech.drain,massFlowRate. port_a)
    annotation (Line(points={{308,76},{322,76}},   color={0,140,72},
      thickness=0.5));
  connect(prescribedTemperature.T, T_boundaryExpr.y)
    annotation (Line(points={{390,46},{397,46}}, color={0,0,127}));
  connect(massFlowRate.m_flow, pump_controller.m_flow) annotation (Line(points={
          {332,85},{332,110},{218,110},{218,128},{240,128}}, color={0,0,127}));
  connect(dPIM.p, LOAD_bus.p)
    annotation (Line(points={{440,-58},{360,-58},{360,-60},{280,-60}},
                                                   color={0,0,255}));
  connect(hot_water.portB, consumer1.portA) annotation (Line(points={{368,76},{502,
          76},{502,-98},{490,-98}}, color={0,140,72},
      thickness=0.5));
  connect(cold_water.portB, consumer1.portB) annotation (Line(points={{368,10},{
          496,10},{496,-102},{490,-102}}, color={0,140,72},
      thickness=0.5));
  connect(inertia_of_the_pump.flange_b, pumpMech.flange) annotation (
      Line(points={{424,200},{440,200},{440,104},{309.2,104},{309.2,72}},
        color={0,0,0}));
  connect(volts_Hertz_Control.Vc, aC_2_DC_and_DC_2_AC.Vc) annotation (
      Line(points={{306,162},{307.667,162},{307.667,178.333}}, color={0,
          0,127}));
  connect(const.y, gasTurbineExhaust.fuel_mass_flow)
    annotation (Line(points={{41,130},{66,130}}, color={0,0,127}));
  connect(gasTurbineExhaust.flange_a, gen_GT.shaft)
    annotation (Line(points={{88,130},{104,130}}, color={0,0,0}));
  connect(SOURCE.port[1], gasTurbineExhaust.portA_secondary1)
    annotation (Line(points={{63,100},{75,100},{75,119}}, color={0,140,72},
      thickness=0.5));
  connect(gasTurbineExhaust.portB_secondary1, pumpMech.feed)
    annotation (Line(points={{81,119},{80,119},{80,70},{291.8,70}},
        color={0,140,72},
      thickness=0.5));
  connect(gen_GT.pwPin, diesel.pwPin) annotation (Line(points={{125,130},
          {142,130},{142,242},{428,242},{428,356},{438,356}}, color={0,
          0,255}));
  connect(PV_bus.p, pV_Module_for_irradiance.pwPin)
    annotation (Line(points={{216,316},{240,316}}, color={0,0,255}));
  connect(irradiance_to_Power.irradiance_out, pV_Module_for_irradiance.bambu)
    annotation (Line(points={{217,366},{248.8,366},{248.8,338}}, color=
          {0,0,127}));
  connect(BESS_bus.p, battery.pwPin)
    annotation (Line(points={{316,356},{340,356}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-340,
            -280},{520,420}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-340,-280},{520,420}}), graphics={
        Rectangle(
          extent={{220,100},{426,-6}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5),
        Text(
          extent={{238,58},{334,22}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5,
          textString="Exterior Piping",
          textStyle={TextStyle.Bold}),
        Rectangle(
          extent={{430,-14},{510,-136}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5),
        Text(
          extent={{442,-110},{506,-142}},
          lineColor={28,108,200},
          pattern=LinePattern.Dash,
          lineThickness=0.5,
          textString="Residence"),
        Text(
          extent={{38,68},{164,52}},
          lineColor={217,67,180},
          textStyle={TextStyle.Bold,TextStyle.Italic},
          textString="Multi Domain Source"),
        Rectangle(
          extent={{0,160},{172,50}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          lineThickness=0.5)}),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end IEEEMicrogrid_06;
