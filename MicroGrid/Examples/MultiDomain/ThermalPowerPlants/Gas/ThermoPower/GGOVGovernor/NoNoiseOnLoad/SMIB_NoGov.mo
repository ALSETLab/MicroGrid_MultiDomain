within MicroGrid.Examples.MultiDomain.ThermalPowerPlants.Gas.ThermoPower.GGOVGovernor.NoNoiseOnLoad;
model SMIB_NoGov "Multi-Domain Network model, no generator controls, no governor (deterministic load)"
  import MicroGrid;
  import ThermoPower;
  extends
    Electrical.Networks.SMIB.Partial.SMIB_Partial_NoNoise(
    transformer(V_b=13.8e3, Vn=13.8e3),
    LOAD(
      V_b=13.8e3,
      v_0=pf_results.voltages.V21,
      angle_0=pf_results.voltages.A21),
    GEN1(
      V_b=13.8e3,
      v_0=pf_results.voltages.V1,
      angle_0=pf_results.voltages.A1),
    BUS1(
      V_b=13.8e3,
      v_0=pf_results.voltages.V2,
      angle_0=pf_results.voltages.A2),
    BUS2(
      V_b=13.8e3,
      v_0=pf_results.voltages.V22,
      angle_0=pf_results.voltages.A22),
    BUS3(
      V_b=13.8e3,
      v_0=pf_results.voltages.V23,
      angle_0=pf_results.voltages.A23),
    GEN2(
      V_b=13.8e3,
      v_0=pf_results.voltages.V3,
      angle_0=pf_results.voltages.A3),
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
      d_P=0.2,
      d_t=20),
    pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
  import Modelica.Constants.pi;

  Thermal_Power.Gas.GTModels.ThPowerSSGT gtDyns annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
  MicroGrid.MultiDomain.Generation_Groups.SMIB.Gen_GT genGroup(
    V_b=13.8e3,
    M_b=10e6,
    Q_0=pf_results.machines.Q1_1,
    P_0=pf_results.machines.P1_1,
    v_0=pf_results.voltages.V1,
    angle_0=pf_results.voltages.A1)
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  replaceable parameter Electrical.Networks.SMIB.Records.PF_075 pf_results constrainedby
    Electrical.Networks.SMIB.Records.PF_050
    annotation (Placement(transformation(extent={{-134,50},{-114,70}})));
  Modelica.Blocks.Sources.Constant const(k=0.649458) annotation (Placement(transformation(extent={{-156,0},{-136,20}})));
equation
  connect(genGroup.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-49,0},{-42,0}}, color={0,0,255}));
  connect(gtDyns.shaft_b, genGroup.shaft) annotation (Line(points={{-86,0},{-86,0},{-70,0}}, color={0,0,0}));
  connect(const.y, gtDyns.valvePosition) annotation (Line(points={{-135,10},{-126.3,10},{-126.3,9.5}}, color={0,0,127}));
  annotation (
    Diagram(graphics={Text(
extent={{-112,68},{108,48}},
lineColor={0,0,0},
lineThickness=1,
fillPattern=FillPattern.Solid,
fontSize=15,
textStyle={TextStyle.Bold},
textString="(Constant Efd)")}),
    experiment(StopTime=100, __Dymola_NumberOfIntervals=5000),
    __Dymola_experimentSetupOutput);
end SMIB_NoGov;
