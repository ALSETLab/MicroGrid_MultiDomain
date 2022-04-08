within MicroGrid.Electrical.Networks.SMIB.NoNoiseOnLoad;
model SMIB "SMIB network without controls (no noise on load)"
  extends
    Partial.SMIB_Partial_NoNoise(
    transformer(V_b=13.8e3, Vn=13.8e3),
    LOAD(V_b=13.8e3),
    GEN1(V_b=13.8e3),
    BUS1(V_b=13.8e3),
    BUS2(V_b=13.8e3),
    BUS3(V_b=13.8e3),
    GEN2(V_b=13.8e3),
    infiniteGen(
      V_b=13800,
      M_b=1000e6,
      P_0=pf_results.machines.P3_1,
      Q_0=pf_results.machines.Q3_1,
      v_0=pf_results.voltages.V3,
      angle_0=pf_results.voltages.A3),
    variableLoad(
      d_P=0,
      t1=0,
      d_t=0,
      P_0=pf_results.loads.PL23_1,
      Q_0=pf_results.loads.QL23_1,
      v_0=pf_results.voltages.V23,
      angle_0=pf_results.voltages.A23),
    pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
  import Modelica.Constants.pi;

  Generation_Groups.SMIB.Generator                                   generator(
    V_b=13.8e3,
    M_b=10e6,
    Q_0=pf_results.machines.Q1_1,
    P_0=pf_results.machines.P1_1,
    v_0=pf_results.voltages.V1,
    angle_0=pf_results.voltages.A1)
    annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
  Networks.SMIB.Records.PF_050 pf_results
    annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
equation
  connect(generator.pwPin, GEN1.p) annotation (Line(points={{-51,0},{-42,0}}, color={0,0,255}));
  connect(generator.Pm0, generator.Pmech) annotation (Line(points={{-57.4,-9.8},
{-57.4,-20},{-80,-20},{-80,0},{-72.6,0}}, color={0,0,127}));
  annotation (
    Diagram(graphics={Text(
extent={{-112,68},{108,48}},
lineColor={0,0,0},
lineThickness=1,
fillPattern=FillPattern.Solid,
fontSize=15,
textStyle={TextStyle.Bold},
textString="(Constant Efd)")}),
    experiment(StopTime=50),
    __Dymola_experimentSetupOutput);
end SMIB;
