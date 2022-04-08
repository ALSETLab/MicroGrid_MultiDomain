within MicroGrid.Electrical.MultiDomain.InductionMotor.SinglePhase;
model DPIM
  "This model is the steady-state circuit model of the single phase induction motor model initialized by a split-phase auxiliary circuit."
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

    parameter Integer init "Initialization Method: (1) Split-Phase Motor, (2) Capacitor-Start Motor" annotation (choices(choice=1, choice=2));
    parameter Real switch_open_speed = 0.2 "Auxiliary winding cut-off speed";
  parameter Modelica.Units.SI.Inductance Lmainr
    "Mutual-inductance of the main winding";
  parameter Modelica.Units.SI.Inductance Lmain
    "Self-inductance of the magnetizing branch";
  parameter Modelica.Units.SI.Inductance Lauxr
    "Mutual-inductance of the auxiliary winding";
  parameter Modelica.Units.SI.Inductance Laux
    "Self-inductance of the auxiliary winding";
  parameter Modelica.Units.SI.Inductance Lr
    "Self-inductance of the equivalent rotor windings";
  parameter Modelica.Units.SI.Resistance Rmain
    "Resistance of the main winding";
  parameter Modelica.Units.SI.Resistance Rr
    "Resistance of the rotor winding";
  parameter Modelica.Units.SI.Resistance Raux
    "Resistance of the auxiliary winding";
  parameter Modelica.Units.SI.Capacitance Cc
    "Capacitance of the capacitor-start configuration"
    annotation (Dialog(enable=(init == 2)));
    parameter OpenIPSL.Types.PerUnit H "Inertia Constant";
    parameter Real a "Load Torque Coefficient a";
    parameter Real b "Load Torque Coefficient b";
    parameter Real c "Load Torque Coefficient c";

    OpenIPSL.Types.PerUnit Pc;
    OpenIPSL.Types.PerUnit Qc;
    OpenIPSL.Types.PerUnit s(start = s0);
    OpenIPSL.Types.PerUnit Te1 "First Component of the Electrical Torque";
    OpenIPSL.Types.PerUnit Te2 "Second Component of the Electrical Torque";
    OpenIPSL.Types.PerUnit Te "Total Electrical Torque";
    OpenIPSL.Types.PerUnit Tm "Mechanical Torque of the Load";
  Modelica.Units.SI.Power P;

    //Modelica.SIunits.Torque Tele;
  Modelica.Units.SI.Current Iaux_real;
  Modelica.Units.SI.Current Iaux_imag;
  Modelica.Units.SI.Current Imain_real;
  Modelica.Units.SI.Current Imain_imag;
  Modelica.Units.SI.Current Itotal;
  Modelica.Units.SI.Current Itotal_real;
  Modelica.Units.SI.Current Itotal_imag;
  Modelica.Units.SI.Voltage Vmain_real;
  Modelica.Units.SI.Voltage Vmain_imag;
  Modelica.Units.SI.Voltage Vaux_real;
  Modelica.Units.SI.Voltage Vaux_imag;
  Modelica.Units.SI.Voltage Vmain_aux_real;
  Modelica.Units.SI.Voltage Vmain_aux_imag;
    Real K1_real;
    Real K1_imag;
    Real K2_real;
    Real K2_imag;
    Real K3_real;
    Real K3_imag;
    Real KplusK_real;
    Real KplusK_imag;
    Real KminusK_real;
    Real KminusK_imag;
    Real Kden_real;
    Real Kden_imag;
  Modelica.Units.SI.Conductance Cond1_aux_real;
  Modelica.Units.SI.Conductance Cond1_aux_imag;
  Modelica.Units.SI.Conductance Cond2_aux_real;
  Modelica.Units.SI.Conductance Cond2_aux_imag;
  Modelica.Units.SI.Conductance Cond1_main_real;
  Modelica.Units.SI.Conductance Cond1_main_imag;
    Real Constant1;
    Real Constant2;

protected
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
  parameter Modelica.Units.SI.AngularVelocity we=2*Modelica.Constants.pi
      *fn;
  parameter Real A = a + b + c;
  parameter Real B = -b - 2*c;
  parameter Real C = c;
  parameter OpenIPSL.Types.PerUnit s0 = 0.999;
  Modelica.Units.SI.Impedance Zb=V_b^2/S_b;
  Modelica.Units.SI.Current I_b=S_b/V_b;
  Modelica.Units.SI.Torque Tb=S_b/we;

equation

  // Calculation of the coeficients of the two-phase motor

  KplusK_real = Rr*we*(Rr^2 - Lr^2*(s-2)*s*we^2)/(((Lr^2*(s-2)^2*we^2) + Rr^2)*((Lr*s*we)^2 + Rr^2));
  KplusK_imag = -Lr*we^2*((Lr*(s-2)*s*we)^2 + Rr^2*(s^2-2*s+2))/(((Lr*(s-2)*we)^2 + Rr^2)*((Lr*s*we)^2 + Rr^2));
  KminusK_real = Rr*(s-1)*we*(Lr^2*(s-2)*s*we^2 + Rr^2)/(((Lr*(s-2)*we)^2 + Rr^2)*((Lr*s*we)^2 + Rr^2));
  KminusK_imag = -2*Lr*Rr^2*(s-1)*we^2/(((Lr*(s-2)*we)^2 + Rr^2)*((Lr*s*we)^2 + Rr^2));

  K1_real = Rmain + we*Lmainr^2*KplusK_real;
  K1_imag = we*Lmain + we*Lmainr^2*KplusK_imag;

  K2_real = -we*Lmainr*Lauxr*KminusK_imag;
  K2_imag = we*Lmainr*Lauxr*KminusK_real;

  K3_real = Raux + we*Lauxr^2*KplusK_real;
  K3_imag = if init == 1 then we*Laux + we*Lauxr^2*KplusK_imag else we*Laux + we*Lauxr^2*KplusK_imag - 1/(we*Cc);

  Kden_real = (K2_real^2 - K2_imag^2 + K1_real*K3_real - K1_imag*K3_imag);
  Kden_imag = (2*K2_real*K2_imag + K1_real*K3_imag + K3_real*K1_imag);

  Cond2_aux_real = (K2_real*Kden_real + K2_imag*Kden_imag)/(Kden_real^2 + Kden_imag^2);
  Cond2_aux_imag = (K2_imag*Kden_real - K2_real*Kden_imag)/(Kden_real^2 + Kden_imag^2);

  Cond1_aux_real = (K1_real*Kden_real + K1_imag*Kden_imag)/(Kden_real^2 + Kden_imag^2);
  Cond1_aux_imag = (K1_imag*Kden_real - K1_real*Kden_imag)/(Kden_real^2 + Kden_imag^2);

  Cond1_main_real = K1_real/(K1_real^2 + K1_imag^2);
  Cond1_main_imag = K1_imag/(K1_real^2 + K1_imag^2);
  Constant1 = (K2_real*K1_real + K2_imag*K1_imag)/(K1_real^2 + K1_imag^2);
  Constant2 = (K2_imag*K1_real - K2_real*K1_imag)/(K1_real^2 + K1_imag^2);

  Vmain_real = p.vr*V_b;
  Vmain_imag = p.vi*V_b;
  Vmain_aux_real = if s > switch_open_speed then p.vr*V_b else 0;
  Vmain_aux_imag = if s > switch_open_speed then p.vi*V_b else 0;
  Vaux_real = if s > switch_open_speed then p.vr*V_b else 0;
  Vaux_imag = if s > switch_open_speed then p.vi*V_b else 0;

  Iaux_real = Cond1_aux_real*Vaux_real - Cond1_aux_imag*Vaux_imag + Cond2_aux_real*Vaux_real - Cond2_aux_imag*Vaux_imag;
  Iaux_imag = (Cond1_aux_real*Vaux_imag + Cond1_aux_imag*Vaux_real + Cond2_aux_real*Vaux_imag + Cond2_aux_imag*Vaux_real);
  Imain_real = Cond1_main_real*Vmain_real - Cond1_main_imag*Vmain_imag - Constant1*Iaux_real + Constant2*Iaux_imag;
  Imain_imag = -(Cond1_main_imag*Vmain_real + Cond1_main_real*Vmain_imag - Constant2*Iaux_real - Constant1*Iaux_imag);
  Itotal = sqrt((Imain_real + Iaux_real)^2 + (Imain_imag + Iaux_imag)^2);
  Itotal_real = Imain_real + Iaux_real;
  Itotal_imag = (Imain_imag + Iaux_imag);
  p.ir = Itotal_real/I_b;
  p.ii = Itotal_imag/I_b;

  Pc = p.vr*p.ir + p.vi*p.ii;
  Qc = (-p.vr*p.ii) + p.vi*p.ir;
  P = Pc*S_b;

  Te1 = ((Lmainr^2*(Imain_real^2 + Imain_imag^2) + Lauxr^2*(Iaux_real^2 + Iaux_imag^2))*KminusK_real)/Tb;
  Te2 = (2*Lmainr*Lauxr*KplusK_real*(Imain_real*Iaux_imag - Imain_imag*Iaux_real))/Tb;
  Te = Te1 + Te2;
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
end DPIM;
