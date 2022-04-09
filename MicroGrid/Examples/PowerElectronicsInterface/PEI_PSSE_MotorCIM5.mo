within MicroGrid.Examples.PowerElectronicsInterface;
model PEI_PSSE_MotorCIM5
  extends BaseClasses.VSC_partial(pwm_modulation_index(startTime=10));
  Electrical.InductionMotor.ThreePhase.PSSE.CIM5 CIM5(
    P_0=553164,
    Q_0=318329,
    v_0=0.82966,
    angle_0=-0.25464353786597,
    H=6,
    Mtype=2,
    D=1,
    S10=0.06,
    S12=0.6,
    T=0.1) annotation (Placement(transformation(extent={{80,-10},{60,10}})));
equation
  connect(AC_2_DC_and_DC_2_AC.n, CIM5.p)
    annotation (Line(points={{46,0},{60,0}}, color={0,0,255}));
end PEI_PSSE_MotorCIM5;
