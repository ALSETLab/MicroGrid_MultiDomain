within MicroGrid.Examples.PowerElectronicsInterface;
model PEI_PSAT_MotorTypelll
  extends BaseClasses.VSC_partial;
  OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(P_0=
        25000000, Q_0=10000000)
    annotation (Placement(transformation(extent={{80,-10},{60,10}})));
equation
  connect(AC_2_DC_and_DC_2_AC.n, motorTypeIII.p)
    annotation (Line(points={{46,0},{60,0}}, color={0,0,255}));
end PEI_PSAT_MotorTypelll;
