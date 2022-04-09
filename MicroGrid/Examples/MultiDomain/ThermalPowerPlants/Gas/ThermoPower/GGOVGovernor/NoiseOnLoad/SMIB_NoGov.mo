within MicroGrid.Examples.MultiDomain.ThermalPowerPlants.Gas.ThermoPower.GGOVGovernor.NoiseOnLoad;
model SMIB_NoGov "Multi-Domain Network model, no generator controls, no governor"
  import MicroGrid;
  import ThermoPower;
  extends MicroGrid.Examples.BaseClasses.SMIB.Partial.SMIB_Partial_Noise(
    transformer(V_b=13.8e3, Vn=13.8e3),
    LOAD(V_b=13.8e3),
    GEN1(V_b=13.8e3),
    BUS1(V_b=13.8e3),
    BUS2(V_b=13.8e3),
    BUS3(V_b=13.8e3),
    GEN2(V_b=13.8e3),
    sineNoiseInjection(
      amplitude=0.05,
      freqHz=0.02,
      active_sigma=0.0005),
    infiniteGen(
      V_b=13.8e3,
      M_b=1000e6,
      P_0=pf_results.machines.P3_1,
      Q_0=pf_results.machines.Q3_1,
      v_0=pf_results.voltages.V3,
      angle_0=pf_results.voltages.A3),
    variableLoad(
      P_0=pf_results.loads.PL23_1,
      Q_0=pf_results.loads.QL23_1,
      v_0=pf_results.voltages.V23,
      angle_0=pf_results.voltages.A23,
      t1=30,
      d_t=20,
      d_P=0.2),
    pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
  import Modelica.Constants.pi;

  Thermal_Power.Gas.GTModels.ThPowerSSGT gtDyns annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
  MicroGrid.MultiDomain.Generation_Groups.SMIB.Gen_GT genGroup(
    V_b=13.8e3,
    M_b=10e6,
    Q_0=pf_results.machines.Q1_1,
    P_0=pf_results.machines.P1_1,
    v_0=pf_results.voltages.V1,
    angle_0=pf_results.voltages.A1) annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  MicroGrid.Examples.BaseClasses.SMIB.Records.PF_075 pf_results
    annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
  inner Modelica.Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=true) annotation (Placement(transformation(extent={{-28,-90},{-8,-70}})));
  Modelica.Blocks.Sources.Constant const(k=0.649458) annotation (Placement(transformation(extent={{-152,2},{-136,18}})));
equation
  connect(genGroup.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-49,0},{-42,0}}, color={0,0,255}));
  connect(gtDyns.shaft_b, genGroup.shaft) annotation (Line(points={{-86,0},{-86,0},{-70,0}}, color={0,0,0}));
  connect(gtDyns.valvePosition, const.y) annotation (Line(points={{-126.3,9.5},{-132,9.5},{-132,10},{-135.2,10}}, color={0,0,127}));
  annotation (
    Diagram(graphics={Text(
extent={{-112,68},{108,48}},
lineColor={0,0,0},
lineThickness=1,
fillPattern=FillPattern.Solid,
fontSize=15,
textStyle={TextStyle.Bold},
textString="(Constant Efd)")}),
    experiment(
      StopTime=100,
      Interval=0.02,
      __Dymola_fixedstepsize=0.02),
    __Dymola_experimentSetupOutput);
end SMIB_NoGov;
