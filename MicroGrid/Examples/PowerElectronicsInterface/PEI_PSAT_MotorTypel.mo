within MicroGrid.Examples.PowerElectronicsInterface;
model PEI_PSAT_MotorTypel
  extends BaseClasses.VSC_partial(
    Fault(t1=2, t2=2.1),
    pwm_modulation_index(startTime=10),
    Bus_VSD(v_0=0.993903, angle_0=-0.10384133090713),
    Grid(v_0=0.994419, angle_0=-0.098782494070225),
    AC_2_DC_and_DC_2_AC(v_0=0.993903, angle_0=-0.10384133090713));
  OpenIPSL.Electrical.Machines.PSAT.MotorTypeI motorTypeI(
    P_0=50000000,
    Q_0=10000000,
    v_0=0.821951,
    Sup=0,
    a=0.5)
         annotation (Placement(transformation(extent={{80,-10},{60,10}})));
equation
  connect(AC_2_DC_and_DC_2_AC.n, motorTypeI.p)
    annotation (Line(points={{46,0},{60,0}}, color={0,0,255}));
end PEI_PSAT_MotorTypel;
