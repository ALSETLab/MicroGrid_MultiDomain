within MicroGrid.Electrical.MultiDomain.InductionMotor.SinglePhase;
model SPIM
  "This model is the steady-state circuit model of the single phase induction motor model."
  OpenIPSL.Interfaces.PwPin p
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

    extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enableS_b=true);

    parameter OpenIPSL.Types.PerUnit R1 "Stator winding resistor";
    parameter OpenIPSL.Types.PerUnit R2 "Rotor winding resistor";
    parameter OpenIPSL.Types.PerUnit X1 "Stator winding reactance";
    parameter OpenIPSL.Types.PerUnit X2 "Rotor winding reactance";
    parameter OpenIPSL.Types.PerUnit Xm "Magnitization reactance value";
    parameter OpenIPSL.Types.PerUnit H "Inertia coeficient";
    parameter Real a;
    parameter Real b;
    parameter Real c;

    OpenIPSL.Types.PerUnit RF "Equivalent resistor component of the forward circuit";
    OpenIPSL.Types.PerUnit XF "Equivalent resistor component of the forward circuit";
    OpenIPSL.Types.PerUnit RB "Equivalent resistor component of the backward circuit";
    OpenIPSL.Types.PerUnit XB "Equivalent resistor component of the backward circuit";
    OpenIPSL.Types.PerUnit s(start = s0)  "Induction motor slip";
    OpenIPSL.Types.PerUnit I "Magnitude of the consumed current";
    OpenIPSL.Types.PerUnit P_AGF "Air Gap Power for forward magnetic field";
    OpenIPSL.Types.PerUnit P_AGB "Air Gap Power for reverse field";
    OpenIPSL.Types.PerUnit P( start = P_0/S_b) "Net air gap power in single-phase induction motor";
    //OpenIPSL.Types.PerUnit Q "Reactive Power consumed by the single phase induction motor model";
    OpenIPSL.Types.PerUnit Te "Electrical Torque";
    OpenIPSL.Types.PerUnit Tm "Mechanical Torque of the Load";
    OpenIPSL.Types.PerUnit Pc;
    OpenIPSL.Types.PerUnit Qc;

protected
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit s0 = 0.1;
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
  parameter Real A = a + b + c;
  parameter Real B = -b - 2*c;
  parameter Real C = c;

initial equation
  //der(s) = 0;

equation

  Pc = p.vr*p.ir + p.vi*p.ii;
  Qc = (-p.vr*p.ii) + p.vi*p.ir;

  RF = (-X2*Xm*R2/s + R2*Xm*(X2 + Xm)/s)/((R2/s)^2 + (X2 + Xm)^2);
  XF = ((R2/s)^2*Xm + X2*Xm*(X2 + Xm))/((R2/s)^2 + (X2 + Xm)^2);
  RB = (-X2*Xm*R2/(2-s) + R2*Xm*(X2 + Xm)/(2-s))/((R2/(2-s))^2 + (X2 + Xm)^2);
  XB = ((R2/(2-s))^2*Xm + X2*Xm*(X2 + Xm))/((R2/(2-s))^2 + (X2 + Xm)^2);

  p.ir = p.vr*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) + p.vi*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
  p.ii = p.vi*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) - p.vr*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
  I = sqrt(p.ir^2 + p.ii^2);

  P_AGF = I^2*0.5*RF;
  P_AGB = I^2*0.5*RB;
  P     = P_AGF - P_AGB;

  Te = P/(1-s);
  Tm = A + B*s + C*s^2;
  der(s) = (Tm - Te)/(2*H);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0}),
        Text(
          extent={{-100,-52},{100,-92}},
          lineColor={28,108,200},
          textString="Single Phase"),      Text(
          extent={{-50,50},{50,-50}},
          lineColor={0,0,0},
          textString="M"),                Ellipse(
          fillColor={255,255,255},
          extent={{-56,-56},{55.932,56}})}),                     Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SPIM;
