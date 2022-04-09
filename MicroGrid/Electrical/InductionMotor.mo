within MicroGrid.Electrical;
package InductionMotor
  package SinglePhase

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

    model DPIM2
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
        parameter Real poles = 4 "Number of poles";
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
        //Modelica.SIunits.Voltage Vmain_aux_real;
        //Modelica.SIunits.Voltage Vmain_aux_imag;
        Real K1_real;
        Real K1_imag;
        Real K2_real;
        Real K2_imag;
        Real K3_real;
        Real K3_imag;
        Real K4_real;
        Real K4_imag;
      Modelica.Units.SI.Conductance K5_real;
      Modelica.Units.SI.Conductance K5_imag;
      Modelica.Units.SI.Conductance K6_real;
      Modelica.Units.SI.Conductance K6_imag;
      Modelica.Units.SI.Conductance K7_real;
      Modelica.Units.SI.Conductance K7_imag;
        Real K8_real;
        Real K8_imag;

        Real KplusK_real;
        Real KplusK_imag;
        Real KminusK_real;
        Real KminusK_imag;
        Real Kden_real;
        Real Kden_imag;
        //Modelica.SIunits.Conductance Cond1_aux_real;
        //Modelica.SIunits.Conductance Cond1_aux_imag;
        //Modelica.SIunits.Conductance Cond2_aux_real;
        //Modelica.SIunits.Conductance Cond2_aux_imag;
        //Modelica.SIunits.Conductance Cond1_main_real;
        //Modelica.SIunits.Conductance Cond1_main_imag;
        //Real Constant1;
        //Real Constant2;

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
      parameter OpenIPSL.Types.PerUnit s0 = 1;
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

      K3_real = we*Lmainr*Lauxr*KminusK_imag;
      K3_imag = -we*Lmainr*Lauxr*KminusK_real;

      K4_real = Raux + we*Lauxr^2*KplusK_real;
      K4_imag = if init == 1 then we*Laux + we*Lauxr^2*KplusK_imag else we*Laux + we*Lauxr^2*KplusK_imag - 1/(we*Cc);

      Kden_real = (K3_real*K2_real - K3_imag*K2_imag - K1_real*K4_real + K1_imag*K4_imag);
      Kden_imag = (K2_real*K3_imag + K3_real*K2_imag - K1_real*K4_imag - K1_imag*K4_real);

      K5_real = (K3_real*Kden_real + K3_imag*Kden_imag)/(Kden_real^2 + Kden_imag^2);
      K5_imag = (K3_imag*Kden_real - K3_real*Kden_imag)/(Kden_real^2 + Kden_imag^2);

      K6_real = (K1_real*Kden_real + K1_imag*Kden_imag)/(Kden_real^2 + Kden_imag^2);
      K6_imag = (K1_imag*Kden_real - K1_real*Kden_imag)/(Kden_real^2 + Kden_imag^2);

      K7_real =  K1_real/(K1_real^2 + K1_imag^2);
      K7_imag = -K1_imag/(K1_real^2 + K1_imag^2);

      K8_real = K2_real*K1_real + K2_imag*K1_imag;
      K8_imag = K2_imag*K1_real - K2_real*K1_imag;

      Vmain_real = p.vr*V_b;
      Vmain_imag = p.vi*V_b;
      Vaux_real = if s > switch_open_speed then p.vr*V_b else 0;
      Vaux_imag = if s > switch_open_speed then p.vi*V_b else 0;

      Iaux_real = K5_real*Vaux_real - K5_imag*Vaux_imag - K6_real*Vaux_real + K6_imag*Vaux_imag;
      Iaux_imag = K5_imag*Vaux_real + K5_real*Vaux_imag - K6_imag*Vaux_real - K6_real*Vaux_imag;

      Imain_real = K7_real*Vmain_real - K7_imag*Vmain_imag - K8_real*Iaux_real + K8_imag*Iaux_imag;
      Imain_imag = K7_imag*Vmain_real + K7_real*Vmain_imag - K8_imag*Iaux_real - K8_real*Iaux_imag;

      Itotal = sqrt((Imain_real + Iaux_real)^2 + (Imain_imag + Iaux_imag)^2);
      Itotal_real = Imain_real + Iaux_real;
      Itotal_imag = (Imain_imag + Iaux_imag);
      p.ir = Itotal_real/I_b;
      p.ii = Itotal_imag/I_b;

      Pc = p.vr*p.ir + p.vi*p.ii;
      Qc = (-p.vr*p.ii) + p.vi*p.ir;
      P = Pc*S_b;

      Te1 = ((Lmainr^2*(Imain_real^2 + Imain_imag^2) + Lauxr^2*(Iaux_real^2 + Iaux_imag^2))*KminusK_real)/Tb;
      Te2 = ((2*Lmainr*Lauxr*KplusK_real*(Imain_imag*Iaux_real - Imain_real*Iaux_imag))/Tb);
      Te = abs((poles/2)*(Te1 + Te2));
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
    end DPIM2;
  end SinglePhase;

  package VariableSpeedDrive

    package Power_Electronics

      model AC_2_DC_and_DC_2_AC
        "Phasor based rectifier plus its dc link combined with an inverter."
         extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enabledisplayPF=false,
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=true,
          final enablev_0=true,
          final enableQ_0=true,
          final enableS_b=true);
        OpenIPSL.Interfaces.PwPin p
          annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
        OpenIPSL.Interfaces.PwPin n annotation (Placement(transformation(extent={{110,-10},{130,10}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage Voltage annotation (
            Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-46,0})));
        Modelica.Blocks.Sources.RealExpression Vd0(y=3*sqrt(6)*Vs.y*(V_b)/Modelica.Constants.pi)
                                                   annotation (Placement(transformation(extent={{-92,-10},
                  {-72,10}})));
        Modelica.Electrical.Analog.Basic.Resistor Resistor(R=Rdc)
          annotation (Placement(transformation(extent={{-36,6},{-16,26}})));
        Modelica.Electrical.Analog.Basic.Inductor Inductor(i(start=Ir0),
                                                           L=Ldc)
          annotation (Placement(transformation(extent={{-10,6},{10,26}})));
        Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch(Goff=1e-8)
          annotation (Placement(transformation(extent={{16,6},{36,26}})));
        Modelica.Electrical.Analog.Basic.Capacitor Capacitor(v(start=Vc0), C=Cdc)
                                                                     annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={36,0})));
        Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={56,0})));
        Modelica.Blocks.Sources.RealExpression Ii(y=Smotor.y*S_b/Capacitor.v)
                                                                     annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={86,0})));
        Modelica.Blocks.Sources.RealExpression Vs(y=sqrt(p.vr^2 + p.vi^2))
          annotation (Placement(transformation(extent={{-70,78},{-50,98}})));
        Modelica.Blocks.Sources.BooleanExpression open_circuit_condition(y=if Voltage.v <
              Capacitor.v then true else false)
          annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
        Modelica.Blocks.Sources.RealExpression Pmotor(y=-(n.vr*n.ir + n.vi*n.ii))
          annotation (Placement(transformation(extent={{30,90},{50,110}})));
        Modelica.Blocks.Sources.RealExpression Qmotor(y=n.vr*n.ii - n.vi*n.ir)
          annotation (Placement(transformation(extent={{30,74},{50,94}})));
        OpenIPSL.Types.PerUnit P;
        OpenIPSL.Types.PerUnit Q;
        //OpenIPSL.Types.PerUnit S;

        Modelica.Blocks.Sources.RealExpression Vmotor(y=Capacitor.v*m_input/(2*sqrt(2)
              *V_b))
          annotation (Placement(transformation(extent={{30,58},{50,78}})));
        Modelica.Blocks.Interfaces.RealInput m_input annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,-130}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,-130})));
        Modelica.Blocks.Sources.RealExpression vr_m(y=Vmotor.y*cos(0))
          annotation (Placement(transformation(extent={{68,90},{88,110}})));
        Modelica.Blocks.Sources.RealExpression vi_m(y=Vmotor.y*sin(0))
          annotation (Placement(transformation(extent={{68,74},{88,94}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-10,-46},{10,-26}})));

        Modelica.Blocks.Interfaces.RealOutput Vc "Value of Real output" annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-50,-130})));
        parameter Modelica.Units.SI.Resistance Rdc=0.1
          "Resistance at temperature T_ref"
          annotation (Dialog(group="DC Link Parameters"));
        parameter Modelica.Units.SI.Inductance Ldc=0.001 "Inductance"
          annotation (Dialog(group="DC Link Parameters"));
        parameter Modelica.Units.SI.Capacitance Cdc=0.02 "Capacitance"
          annotation (Dialog(group="DC Link Parameters"));
        Modelica.Blocks.Sources.RealExpression Smotor(y=sqrt(Pmotor.y^2 + Qmotor.y^2))
          annotation (Placement(transformation(extent={{68,58},{88,78}})));
        Modelica.Blocks.Interfaces.RealInput V00 annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={140,-100}),iconTransformation(
              extent={{-7.5,32.5},{12.5,12.5}},
              rotation=180,
              origin={132.5,-77.5})));
        Modelica.Blocks.Interfaces.RealInput S00 annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=180,
              origin={140,-60}), iconTransformation(
              extent={{-7.5,32.5},{12.5,12.5}},
              rotation=180,
              origin={132.5,-37.5})));
      protected
        parameter OpenIPSL.Types.Voltage Vc0 = 2*sqrt(2)*Vmotor0*V_b/m0;
        parameter OpenIPSL.Types.PerUnit Vmotor0(fixed = false);
        parameter OpenIPSL.Types.PerUnit S0(fixed = false);
        parameter Real m0 = 1;
        parameter OpenIPSL.Types.Current Ir0 = S0*S_b/Vc0;

        Modelica.Blocks.Sources.RealExpression Capacitor_Voltage(y=Capacitor.v)
          annotation (Placement(transformation(extent={{-90,-100},{-70,-80}})));
      initial equation
        der(Resistor.i) = 0;
        der(Capacitor.v) = 0;
        Vmotor0 = V00;
        S00 = S0;
      equation
        connect(Vd0.y, Voltage.v)
          annotation (Line(points={{-71,0},{-58,0}},     color={0,0,127}));
        connect(Voltage.p, Resistor.p)
          annotation (Line(points={{-46,10},{-46,16},{-36,16}},    color={0,0,255}));
        connect(Resistor.n, Inductor.p)
          annotation (Line(points={{-16,16},{-10,16}},   color={0,0,255}));
        connect(Inductor.n, switch.p)
          annotation (Line(points={{10,16},{16,16}},  color={0,0,255}));
        connect(switch.n, Capacitor.p)
          annotation (Line(points={{36,16},{36,10}},   color={0,0,255}));
        connect(Voltage.n, Capacitor.n) annotation (Line(points={{-46,-10},{-46,-16},{
                36,-16},{36,-10}}, color={0,0,255}));
        connect(switch.n, signalCurrent.p) annotation (Line(points={{36,16},{56,16},{56,
                10}},                                                                            color={0,0,255}));
        connect(signalCurrent.n, Capacitor.n) annotation (Line(points={{56,-10},{56,-16},
                {36,-16},{36,-10}}, color={0,0,255}));
        connect(signalCurrent.i, Ii.y) annotation (Line(points={{68,0},{75,0}},     color={0,0,127}));
        connect(open_circuit_condition.y, switch.control)
          annotation (Line(points={{-19,40},{26,40},{26,28}},   color={255,0,255}));
          P =  p.vr*p.ir + p.vi*p.ii;
          Q = (-p.vr*p.ii) + p.vi*p.ir;
          Q = 0;
          Resistor.i = smooth(0,(P*S_b)/Vd0.y);

          n.vr = vr_m.y;
          n.vi = vi_m.y;
        connect(ground.p, Capacitor.n) annotation (Line(points={{0,-26},{0,-16},{36,-16},
                {36,-10}}, color={0,0,255}));
        connect(Capacitor_Voltage.y, Vc) annotation (Line(points={{-69,-90},{-50,-90},
                {-50,-130}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}}),
              graphics={Rectangle(
                extent={{-120,120},{120,-120}},
                lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Bitmap(
                extent={{-110,-94},{110,36}},
                imageSource="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",
                fileName=
                    "modelica://MicroGrid/../../../../../Desktop/PhD/Figures/VSD_Motor/Power_Electronics.png")}),
                                                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}}), graphics={Rectangle(
                extent={{20,110},{100,60}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash), Text(
                extent={{30,118},{90,112}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash,
                textString="Motor Variables"),                                                       Rectangle(
                extent={{-80,108},{-40,68}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash), Text(
                extent={{-92,116},{-30,110}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash,
                textString="Grid Variables"),                                                        Rectangle(
                extent={{-100,-70},{-60,-110}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash), Text(
                extent={{-112,-62},{-50,-68}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash,
                textString="DC Link Variables")}));
      end AC_2_DC_and_DC_2_AC;

      model AC2DC_and_DC2AC_uninitialized
        "Phasor based Voltage Source Converter model."
         extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enabledisplayPF=false,
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=true,
          final enableP_0 = false,
          final enableQ_0=false,
          final enablev_0=true,
          final enableS_b=true);
        OpenIPSL.Interfaces.PwPin p
          annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
        OpenIPSL.Interfaces.PwPin n annotation (Placement(transformation(extent={{110,-10},
                  {130,10}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage Voltage annotation (
            Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-46,0})));
        Modelica.Blocks.Sources.RealExpression Vd0(y=3*sqrt(6)*Vs.y*(V_b)/Modelica.Constants.pi)
                                                   annotation (Placement(transformation(extent={{-84,-10},
                  {-64,10}})));
        Modelica.Electrical.Analog.Basic.Resistor Resistor(R=Rdc)
          annotation (Placement(transformation(extent={{-42,4},{-22,24}})));
        Modelica.Electrical.Analog.Basic.Inductor Inductor(L=Ldc)
          annotation (Placement(transformation(extent={{-16,4},{4,24}})));
        Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch(Ron=1e-5, Goff=1e-5)
          annotation (Placement(transformation(extent={{10,4},{30,24}})));
        Modelica.Electrical.Analog.Basic.Capacitor Capacitor(v(start=Vc0), C=Cdc)
                                                                     annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,0})));
        Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={50,0})));
        Modelica.Blocks.Sources.RealExpression Ii(y=Pmotor.y*S_b/Capacitor.v)
                                                                     annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={78,0})));
        Modelica.Blocks.Sources.RealExpression Vs(y=sqrt(p.vr^2 + p.vi^2))
          annotation (Placement(transformation(extent={{-70,78},{-50,98}})));
        Modelica.Blocks.Sources.BooleanExpression open_circuit_condition(y=if
              Resistor.i <= 0 then true else false)
          annotation (Placement(transformation(extent={{52,20},{32,40}})));
        Modelica.Blocks.Sources.RealExpression Pmotor(y=-(n.vr*n.ir + n.vi*n.ii))
          annotation (Placement(transformation(extent={{30,90},{50,110}})));
        Modelica.Blocks.Sources.RealExpression Qmotor(y=n.vr*n.ii - n.vi*n.ir)
          annotation (Placement(transformation(extent={{30,74},{50,94}})));
        OpenIPSL.Types.PerUnit P;
        OpenIPSL.Types.PerUnit Q;
        OpenIPSL.Types.PerUnit S;

        Modelica.Blocks.Sources.RealExpression Vmotor(y=Capacitor.v*m_input/(2*sqrt(2)
              *V_b))
          annotation (Placement(transformation(extent={{30,58},{50,78}})));
        Modelica.Blocks.Interfaces.RealInput m_input annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,-130}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={50,-130})));
        Modelica.Blocks.Sources.RealExpression vr_m(y=Vmotor.y*cos(0))
          annotation (Placement(transformation(extent={{68,90},{88,110}})));
        Modelica.Blocks.Sources.RealExpression vi_m(y=Vmotor.y*sin(0))
          annotation (Placement(transformation(extent={{68,74},{88,94}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-14,-36},{6,-16}})));

        Modelica.Blocks.Interfaces.RealOutput Vc "Value of Real output" annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-50,-130})));
        parameter Modelica.Units.SI.Resistance Rdc=0.1
          "DC Link Resistance"
          annotation (Dialog(group="DC Link Parameters"));
        parameter Modelica.Units.SI.Inductance Ldc=0.001
          "DC Link Inductance"
          annotation (Dialog(group="DC Link Parameters"));
        parameter Modelica.Units.SI.Capacitance Cdc=0.02
          "DC Link Capacitance"
          annotation (Dialog(group="DC Link Parameters"));

        Modelica.Blocks.Sources.RealExpression Smotor(y=sqrt(Pmotor.y^2 + Qmotor.y^2))
          annotation (Placement(transformation(extent={{68,58},{88,78}})));
      protected
        parameter OpenIPSL.Types.Voltage Vc0 = 2*sqrt(2)*Vmotor0*V_b/m0;
        parameter OpenIPSL.Types.PerUnit Vmotor0 = 0.70005;
        parameter Real m0= 0.811776;

        Modelica.Blocks.Sources.RealExpression Capacitor_Voltage(y=Capacitor.v)
          annotation (Placement(transformation(extent={{-90,-100},{-70,-80}})));
      initial equation
        //der(Resistor.i) = 0;
        //der(Capacitor.v) = 0;

      equation
        connect(Vd0.y, Voltage.v)
          annotation (Line(points={{-63,0},{-58,0}},     color={0,0,127}));
        connect(Voltage.p, Resistor.p)
          annotation (Line(points={{-46,10},{-46,14},{-42,14}},    color={0,0,255}));
        connect(Resistor.n, Inductor.p)
          annotation (Line(points={{-22,14},{-16,14}},   color={0,0,255}));
        connect(Inductor.n, switch.p)
          annotation (Line(points={{4,14},{10,14}},   color={0,0,255}));
        connect(switch.n, Capacitor.p)
          annotation (Line(points={{30,14},{30,10}},   color={0,0,255}));
        connect(Voltage.n, Capacitor.n) annotation (Line(points={{-46,-10},{-46,-14},{
                30,-14},{30,-10}}, color={0,0,255}));
        connect(switch.n, signalCurrent.p) annotation (Line(points={{30,14},{50,14},{50,
                10}},                                                                            color={0,0,255}));
        connect(signalCurrent.n, Capacitor.n) annotation (Line(points={{50,-10},{50,-14},
                {30,-14},{30,-10}}, color={0,0,255}));
        connect(signalCurrent.i, Ii.y) annotation (Line(points={{62,0},{67,0}},     color={0,0,127}));
        connect(open_circuit_condition.y, switch.control)
          annotation (Line(points={{31,30},{20,30},{20,26}},    color={255,0,255}));
          P =  p.vr*p.ir + p.vi*p.ii;
          Q = (-p.vr*p.ii) + p.vi*p.ir;
          Q = 0;
          S = sqrt(P^2 + Q^2);
          Resistor.i = smooth(0,(P*S_b)/Vd0.y);

          n.vr = vr_m.y;
          n.vi = vi_m.y;
        connect(ground.p, Capacitor.n) annotation (Line(points={{-4,-16},{-4,-14},{30,
                -14},{30,-10}},
                           color={0,0,255}));
        connect(Capacitor_Voltage.y, Vc) annotation (Line(points={{-69,-90},{-50,-90},
                {-50,-130}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},
                  {120,120}}),
              graphics={Rectangle(
                extent={{-120,120},{120,-120}},
                lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Text(
                extent={{-110,104},{108,-116}},
                lineColor={0,0,0},
                textString="Power Electronics
Interface")}),                                                                                         Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}}), graphics={Rectangle(
                extent={{20,110},{100,60}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash), Text(
                extent={{30,118},{90,112}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash,
                textString="Motor Variables"),                                                       Rectangle(
                extent={{-80,108},{-40,68}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash), Text(
                extent={{-92,116},{-30,110}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash,
                textString="Grid Variables"),                                                        Rectangle(
                extent={{-100,-70},{-60,-110}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash), Text(
                extent={{-112,-62},{-50,-68}},
                lineColor={0,0,255},
                pattern=LinePattern.Dash,
                textString="DC Link Variables")}));
      end AC2DC_and_DC2AC_uninitialized;
    end Power_Electronics;

    package Controls

      model VoltsHertz_Controller
        extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enabledisplayPF=false,
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=false,
          final enablev_0=false,
          final enableQ_0=false,
          final enableS_b=true);
        Modelica.Blocks.Interfaces.RealInput motor_speed annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-110,-20}),
                                iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={110,50})));
        OpenIPSL.NonElectrical.Continuous.SimpleLag Speed_Sensor(K=1, T=Tr,
          y_start=188.275)
          annotation (Placement(transformation(extent={{-88,-30},{-68,-10}})));
        parameter OpenIPSL.Types.Time Tr=0.01 "Lag time constant"
          annotation (Dialog(group="Control Parameters"));
        Modelica.Blocks.Math.Add add(k1=-1)
          annotation (Placement(transformation(extent={{-88,-68},{-68,-48}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=we_max, uMin=we_min)
          annotation (Placement(transformation(extent={{44,-36},{64,-16}})));
        Modelica.Blocks.Math.Gain gain(k=Kp)
          annotation (Placement(transformation(extent={{-48,-52},{-28,-32}})));
        Modelica.Blocks.Continuous.Integrator integrator(k=Ki, y_start=
              188.524 - 188.275)
          annotation (Placement(transformation(extent={{-48,-84},{-28,-64}})));
        Modelica.Blocks.Math.Add add1(k1=+1)
          annotation (Placement(transformation(extent={{-18,-68},{2,-48}})));
        Modelica.Blocks.Math.Add add2(k1=+1)
          annotation (Placement(transformation(extent={{16,-36},{36,-16}})));
        Modelica.Blocks.Interfaces.RealOutput we
                     "Connector of Real output signal"
          annotation (Placement(transformation(extent={{100,-60},{120,-40}}),
              iconTransformation(extent={{100,-60},{120,-40}})));
        Modelica.Blocks.Math.Gain gain1(k=2*sqrt(2)*Kf)
          annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=0,
              origin={30,10})));
        Modelica.Blocks.Interfaces.RealOutput m "Output signal connector"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=90,
              origin={40,110}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={40,110})));
          parameter Real Kf= 0.0001 "Gain value multiplied with input signal"
          annotation (Dialog(group="Control Parameters"));
        parameter Real Kp=5 "Gain value multiplied with input signal"
          annotation (Dialog(group="Control Parameters"));
        parameter Real Ki=0.1 "Integrator gain"
          annotation (Dialog(group="Control Parameters"));
        parameter Real we_max=Modelica.Constants.inf "Upper limits of input signals"
          annotation (Dialog(group="Control Parameters"));
        parameter Real we_min=-limiter.uMax "Lower limits of input signals"
          annotation (Dialog(group="Control Parameters"));
        Modelica.Blocks.Interfaces.RealInput Vc annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,110}), iconTransformation(
              extent={{-10,-10},{10,10}},
              origin={-40,110},
              rotation=270)));
        Modelica.Blocks.Interfaces.RealInput W_ref "Connector of Real input signal 2"
          annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
              iconTransformation(extent={{-120,-10},{-100,10}})));
        Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=1, uMin=0) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={24,52})));
        Modelica.Blocks.Math.Gain gain2(k=1/V_b)
          annotation (Placement(transformation(extent={{-52,48},{-32,68}})));
        Modelica.Blocks.Math.Division division
          annotation (Placement(transformation(extent={{-14,42},{6,62}})));
        Modelica.Blocks.Continuous.FirstOrder firstOrder(T=0.001, y_start=
              0.817078)
          annotation (Placement(transformation(extent={{44,42},{64,62}})));
      equation
        connect(motor_speed, Speed_Sensor.u) annotation (Line(points={{-110,-20},{-90,
                -20}},               color={0,0,127}));
        connect(Speed_Sensor.y, add.u1) annotation (Line(points={{-67,-20},{-62,-20},{
                -62,-38},{-96,-38},{-96,-52},{-90,-52}},
                                 color={0,0,127}));
        connect(add.y, gain.u) annotation (Line(points={{-67,-58},{-56,-58},{-56,-42},
                {-50,-42}},
                     color={0,0,127}));
        connect(integrator.u, gain.u) annotation (Line(points={{-50,-74},{-56,
                -74},{-56,-42},{-50,-42}},
                             color={0,0,127}));
        connect(gain.y, add1.u1) annotation (Line(points={{-27,-42},{-22,-42},{-22,-52},
                {-20,-52}},
                       color={0,0,127}));
        connect(integrator.y, add1.u2) annotation (Line(points={{-27,-74},{
                -22,-74},{-22,-64},{-20,-64}},
                                color={0,0,127}));
        connect(add2.u1, Speed_Sensor.y)
          annotation (Line(points={{14,-20},{-67,-20}},
                                                      color={0,0,127}));
        connect(add.u2, W_ref) annotation (Line(points={{-90,-64},{-100,-64},{-100,0},
                {-110,0}},   color={0,0,127}));
        connect(Vc, gain2.u) annotation (Line(points={{-40,110},{-40,74},{-66,
                74},{-66,58},{-54,58}}, color={0,0,127}));
        connect(gain2.y, division.u2) annotation (Line(points={{-31,58},{-24,
                58},{-24,46},{-16,46}}, color={0,0,127}));
        connect(division.y, limiter1.u)
          annotation (Line(points={{7,52},{12,52}}, color={0,0,127}));
        connect(gain1.u, we) annotation (Line(points={{42,10},{72,10},{72,-50},{110,-50}},
                                                                 color={0,0,
                127}));
        connect(add1.y, add2.u2) annotation (Line(points={{3,-58},{8,-58},{8,-32},{14,
                -32}},             color={0,0,127}));
        connect(add2.y, limiter.u)
          annotation (Line(points={{37,-26},{42,-26}}, color={0,0,127}));
        connect(limiter.y, we) annotation (Line(points={{65,-26},{88,-26},{88,-50},{110,
                -50}},       color={0,0,127}));
        connect(integrator.u, add.y) annotation (Line(points={{-50,-74},{-56,
                -74},{-56,-58},{-67,-58}},
                                 color={0,0,127}));
        connect(gain1.y, division.u1) annotation (Line(points={{19,10},{-22,10},{-22,58},
                {-16,58}}, color={0,0,127}));
        connect(limiter1.y, firstOrder.u)
          annotation (Line(points={{35,52},{42,52}}, color={0,0,127}));
        connect(firstOrder.y, m) annotation (Line(points={{65,52},{70,52},{70,
                80},{40,80},{40,110}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {80,100}}),                                         graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Text(
                extent={{-80,76},{80,-72}},
                lineColor={0,0,0},
                textString="Volts/Hertz
Controller")}),                                                        Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{80,100}})));
      end VoltsHertz_Controller;

      model pump_controller
        Modelica.Blocks.Interfaces.RealInput m_flow_ref annotation (Placement(
              transformation(extent={{-120,50},{-100,70}}), iconTransformation(extent=
                 {{-120,50},{-100,70}})));
        Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(
              transformation(extent={{-54,-10},{-34,10}})));
        Modelica.Blocks.Continuous.Integrator integrator(k=ki, y_start=1)
          annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
        Modelica.Blocks.Math.Gain gain(k=kp)
          annotation (Placement(transformation(extent={{-8,20},{12,40}})));
        Modelica.Blocks.Math.Add add1(k2=+1)
          annotation (Placement(transformation(extent={{26,-10},{46,10}})));
        Modelica.Blocks.Math.Gain PI_convert(k=mflow_2_speed)
          annotation (Placement(transformation(extent={{56,-10},{76,10}})));
        Modelica.Blocks.Interfaces.RealInput m_flow annotation (Placement(
              transformation(extent={{-120,-70},{-100,-50}}), iconTransformation(
                extent={{-120,-70},{-100,-50}})));
        Modelica.Blocks.Interfaces.RealOutput Wref "Output signal connector"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        parameter Real kp=1 "Proportional Gain in the PI Controller."
          annotation (Dialog(group="Pump Control Setting"));
        parameter Real ki=0.1 "Integrator gain in the PI Controller."
          annotation (Dialog(group="Pump Control Setting"));
        parameter Real mflow_2_speed=188.495/585.18
          "Linear gain converter from mass flow rate to motor reference speed."
          annotation (Dialog(group="Pump Control Setting"));
      equation
        connect(gain.u,add. y) annotation (Line(points={{-10,30},{-28,30},{-28,0},{-33,
                0}},        color={0,0,127}));
        connect(integrator.u,add. y) annotation (Line(points={{-12,-30},{-28,-30},{-28,
                0},{-33,0}},      color={0,0,127}));
        connect(integrator.y,add1. u2) annotation (Line(points={{11,-30},{22,-30},{22,
                -6},{24,-6}},    color={0,0,127}));
        connect(gain.y,add1. u1) annotation (Line(points={{13,30},{22,30},{22,6},{24,6}},
                           color={0,0,127}));
        connect(add1.y,PI_convert. u)
          annotation (Line(points={{47,0},{54,0}},       color={0,0,127}));
        connect(m_flow_ref, add.u1) annotation (Line(points={{-110,60},{-62,60},{-62,6},
                {-56,6}}, color={0,0,127}));
        connect(m_flow, add.u2) annotation (Line(points={{-110,-60},{-62,-60},{-62,-6},
                {-56,-6}}, color={0,0,127}));
        connect(PI_convert.y, Wref)
          annotation (Line(points={{77,0},{110,0}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
                Text(
                extent={{-78,30},{80,-20}},
                lineColor={0,0,0},
                textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end pump_controller;
    end Controls;

    package Script_Functions
      function ElectricalFaultSweep

      algorithm
        pathToModel := "MicroGrid.Induction_Motor.VSD_NEW.Testing_Motors.GM_";
        translateModel(pathToModel);
        simulateModel(pathToModel, stopTime=50, numberOfIntervals=10000, tolerance=0.01, resultFile="Banana");
        pwLine.R :=100;
        simulateModel(pathToModel, stopTime=50, numberOfIntervals=10000, tolerance=0.01, resultFile="Banana");
        pwLine.R :=1000;
        simulateModel(pathToModel, stopTime=50, numberOfIntervals=10000, tolerance=0.01, resultFile="Banana");
      end ElectricalFaultSweep;
    end Script_Functions;
  end VariableSpeedDrive;

  package ThreePhase
    package PSSE

      model CIM5 "Induction Machine - Order V"
        extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enabledisplayPF=false,
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=true,
          final enablev_0=true,
          final enableQ_0=true,
          final enableP_0=true,
          final enableS_b=true);
          import OpenIPSL.NonElectrical.Functions.SE_exp;

        parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "Generator base power.";
        parameter OpenIPSL.Types.PerUnit Ra=0.053 "Stator resistance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit Xa=0.076 "Stator reactance"
          annotation (Dialog(group="Machine parameters"));
            parameter OpenIPSL.Types.PerUnit Xm=2.4 "Magnetizing reactance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit R1=0.048 "1st cage rotor resistance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit X1=0.062 "1st cage rotor reactance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit R2=0.048 "2nd cage rotor resistance. To model single cage motor set R2 = inf."
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit X2=0.062 "2nd cage rotor reactance. To model single cage motor set X2 = inf."
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit H = 0.28 "Inertia constant"
          annotation (Dialog(group="Machine parameters"));
        parameter Integer Mtype = 1 "1- Double Cage Type 1; 2- Double Cage Type 2" annotation (Dialog(group=
                "Machine Type"), choices(choice=1, choice=2));
        parameter Real D = 1 "Load Damping Factor"
                                                  annotation (Dialog(group="Machine parameters"));
        parameter Real S10 = 0.06 annotation (Dialog(group="Machine parameters"));
        parameter Real S12 = 0.6 annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit T = 0.5 "Load torque at 1 pu speed" annotation (Dialog(group="Machine parameters"));

        OpenIPSL.Types.PerUnit v(start=v_0) "Bus voltage magnitude";
        OpenIPSL.Types.Angle anglev(start=angle_0) " Bus voltage angle";
        OpenIPSL.Types.Angle delta " Bus voltage angle";
        OpenIPSL.Types.PerUnit s;
        //(start=Rr1*P_0*(Q_0+v_0*v_0/Xm)/(v_0*v_0*v_0*v_0*(Xs + Xr1)));
        OpenIPSL.Types.PerUnit Tm;
        OpenIPSL.Types.PerUnit Te;
        OpenIPSL.Types.PerUnit P(start=P_0/S_b);
        //OpenIPSL.Types.PerUnit P;
        OpenIPSL.Types.PerUnit Q(start=Q_0/S_b);
        //OpenIPSL.Types.PerUnit Q;
        OpenIPSL.Types.PerUnit Epr;
        OpenIPSL.Types.PerUnit Epi;
        OpenIPSL.Types.PerUnit Eppr( start = Eppr0);
        OpenIPSL.Types.PerUnit Eppi( start = Eppi0);
        OpenIPSL.Types.PerUnit Epp;
        OpenIPSL.Types.PerUnit Ekr;
        OpenIPSL.Types.PerUnit Eki;
        OpenIPSL.Types.PerUnit Vr;
        OpenIPSL.Types.PerUnit Vi;
        OpenIPSL.Types.PerUnit Ir;
        OpenIPSL.Types.PerUnit Ii;
        OpenIPSL.Types.PerUnit o1;
        OpenIPSL.Types.PerUnit o2;
        OpenIPSL.Types.PerUnit o3;
        OpenIPSL.Types.PerUnit o4;
        OpenIPSL.Types.PerUnit o5;
        OpenIPSL.Types.PerUnit o6;
        OpenIPSL.Types.PerUnit o7;
        OpenIPSL.Types.PerUnit Omegar(start = wr0) "Rotor angular velocity";
        //OpenIPSL.Types.PerUnit dw "Per unit difference of the angular velocities";

        OpenIPSL.Interfaces.PwPin p(
          vr(start=vr0),
          vi(start=vi0),
          ir(start=ir0_sys),
          ii(start=ii0_sys))
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        import Modelica.Constants.pi;

      protected
        parameter OpenIPSL.Types.PerUnit p0 = P_0/M_b;
        parameter OpenIPSL.Types.PerUnit q0 = Q_0/M_b;
        parameter OpenIPSL.Types.PerUnit Omegab=1 "Base freq in rad/s";
        parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
        parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
        parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
        parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
        parameter OpenIPSL.Types.PerUnit ir0_sys = CoB*ir0 "Initial real current in system base";
        parameter OpenIPSL.Types.PerUnit ii0_sys = CoB*ii0 "Initial imaginary current in system base";
        parameter OpenIPSL.Types.PerUnit Eppr0 = -(Ra*ir0 - Lpp*ii0 - vr0);
        parameter OpenIPSL.Types.PerUnit Eppi0 = -(Lpp*ir0 + Ra*ii0 - vi0);
        parameter OpenIPSL.Types.PerUnit Ls = Xa + Xm;
        parameter OpenIPSL.Types.PerUnit Ll = Xa;
        parameter OpenIPSL.Types.PerUnit Lp = Xa + X1*Xm/(X1 + Xm);
        parameter OpenIPSL.Types.PerUnit Lpp = if Mtype == 1 then Xa + X1*Xm*X2/(X1*X2 + X1*Xm + X2*Xm) else Xa + (X1*Xm+X2*Xm)/(X1 + X2 + Xm);
        parameter OpenIPSL.Types.Time Tp0 = if Mtype == 1 then (X1 + Xm)/(Omegab*R1) else (X1 + X2 + Xm)/(Omegab*R2);
        parameter OpenIPSL.Types.Time Tpp0 = if Mtype == 1 then (X2 + (X1*Xm/(X1 + Xm)))/(Omegab*R2) else 1/((1/(X1+Xm) + 1/X2)/(Omegab*R1));
        parameter OpenIPSL.Types.PerUnit k1 = Ls - Lp;
        parameter OpenIPSL.Types.PerUnit k2 = Lp - Ll;
        parameter OpenIPSL.Types.PerUnit k3 = (Lp - Lpp)/((Lp - Ll)^2);
        parameter OpenIPSL.Types.PerUnit k4 = (Lp - Lpp)/(Lp - Ll);
        parameter OpenIPSL.Types.PerUnit k5 = (Lpp - Ll)/(Lp - Ll);
        parameter OpenIPSL.Types.PerUnit wr0 = -(Eppr0*ir0/T + Eppi0*ii0/T)^(1/D) +2;
        parameter Real CoB=M_b/S_b;
      initial equation
        der(Epr) = 0;
        der(Epi) = 0;
        der(Omegar) = 0;
        der(Ekr) = 0;
        der(Eki) = 0;
      equation

        anglev = atan2(p.vi, p.vr);
        delta = anglev;
        v = sqrt(p.vr^2 + p.vi^2);
        Vr = p.vr;
        Vi = p.vi;
        //Ir = p.ir/CoB;
        //Ii = p.ii/CoB;
        //[Ir; Ii] = (1/CoB)*[cos(delta), sin(delta); -sin(delta), cos(delta)]*[p.ir; p.ii];
        [Ir; Ii] = (1/CoB)*[p.ir; p.ii];
        Ir = ((Vr - Eppr)*Ra + (Vi - Eppi)*Lpp)/(Ra^2 + Lpp^2);
        Ii = ((Vi - Eppi)*Ra - (Vr - Eppr)*Lpp)/(Ra^2 + Lpp^2);
        P = p.vr*p.ir + p.vi*p.ii;
        Q = (-p.vr*p.ii) + p.vi*p.ir;
        //der(s) = (Tm - Te)/(2*H);

        // Mechanical Equation
        s = 1 - Omegar;
        Te = Eppr*Ir + Eppi*Ii;
        der(Omegar) = (Te - Tm)/(2*H);
        Tm = T*(1 + s)^D;

        // Electrical-Magnetic Equations for the CIM5 double cage rotor model
        o1 = Epr - k2*Ii - Ekr;
        o2 = ((o1*k3) + Ii)*k1;
        o3 = Epi*(Tp0*Omegab*s) - o2 - Epr + o4;
        der(Epr) = o3/Tp0;
        der(Ekr) = (o1 + (Tpp0*Omegab*s)*Eki)/Tpp0;
        Eppr = (Ekr*k4) + (Epr*k5);
        Epp = sqrt(Eppr^2 + Eppi^2);
        o4 = (SE_exp(Epp,S10,S12,1,1.2)/Epp)*Eppi;
        o5 = (SE_exp(Epp,S10,S12,1,1.2)/Epp)*Eppr;
        o7 = -Epr*(Tp0*Omegab*s) - o5 + o6 - Epi;
        der(Epi) = o7/Tp0;
        o6 = k1*(-k3*(Epi - Eki + Ir*k2) + Ir);
        der(Eki) = ((Epi - Eki + Ir*k2) - Ekr*(Tpp0*Omegab*s))/Tpp0;
        Eppi = Epi*k5 + k4*Eki;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Rectangle(
                fillColor={255,255,255},
                extent={{-100,-100},{100,100}}),Ellipse(
                fillColor={255,255,255},
                extent={{-56,-58},{55.9318,54}}),Text(
                extent={{-50,48},{50,-52}},
                lineColor={0,0,0},
                textString="M"),Text(
                origin={0,-76.0978},
                fillPattern=FillPattern.Solid,
                extent={{-57.2101,-15.0},{57.2101,15.0}},
                fontName="Arial",
                textString="%name",
                lineColor={0,0,0})}), Documentation(revisions="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>PSAT Manual 2.1.8</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>September 2015</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Joan Russinol, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p>see <a href=\"modelica://OpenIPSL.UsersGuide.Contact\">UsersGuide.Contact</a></p></td>
</tr>
</table>
</html>",       info="<html>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p><img src=\"modelica://MicroGrid/../../Figures/Motor/CIM5/steady_state_circuit.png\"/></p></td>
</tr>
<tr>
<td></td>
</tr>
</table>
<p><br><br><br>The block diagram below represents the dynamics behind the induced voltages in the rotor winding.</p>
<p>The hatched components follow the same structure found in the equations that describes the CIM5 induction motor.</p>
<p><br><br><img src=\"modelica://MicroGrid/../../Figures/Motor/CIM5/CIM5_diagram.png\"/></p>
</html>"));
      end CIM5;

      model CIM6 "Induction Machine - Order V"
        extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enabledisplayPF=false,
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=true,
          final enablev_0=true,
          final enableQ_0=true,
          final enableP_0=true,
          final enableS_b=true);
          import OpenIPSL.NonElectrical.Functions.SE_exp;

        parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "Generator base power.";
        parameter OpenIPSL.Types.PerUnit Ra=0.053 "Stator resistance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit Xa=0.076 "Stator reactance"
          annotation (Dialog(group="Machine parameters"));
            parameter OpenIPSL.Types.PerUnit Xm=2.4 "Magnetizing reactance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit R1=0.048 "1st cage rotor resistance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit X1=0.062 "1st cage rotor reactance"
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit R2=0.048 "2nd cage rotor resistance. To model single cage motor set R2 = inf."
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit X2=0.062 "2nd cage rotor reactance. To model single cage motor set X2 = inf."
          annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit H = 0.28 "Inertia constant"
          annotation (Dialog(group="Machine parameters"));
        parameter Integer Mtype = 1 "1- Double Cage Type 1; 2- Double Cage Type 2" annotation (Dialog(group=
                "Machine Type"), choices(choice=1, choice=2));
        parameter Real D = 1 "Load Damping Factor"
                                                  annotation (Dialog(group="Machine parameters"));
        parameter Real S10 = 0.06 annotation (Dialog(group="Machine parameters"));
        parameter Real S12 = 0.6 annotation (Dialog(group="Machine parameters"));
        parameter OpenIPSL.Types.PerUnit T = 0.5 "Load torque at 1 pu speed" annotation (Dialog(group="Machine parameters"));
        parameter Real A = 1;
        parameter Real B = 1;
        parameter Real C = 1;
        parameter Real E = 1;

        OpenIPSL.Types.PerUnit v(start=v_0) "Bus voltage magnitude";
        OpenIPSL.Types.Angle anglev(start=angle_0) " Bus voltage angle";
        OpenIPSL.Types.Angle delta " Bus voltage angle";
        OpenIPSL.Types.PerUnit s;
        //(start=Rr1*P_0*(Q_0+v_0*v_0/Xm)/(v_0*v_0*v_0*v_0*(Xs + Xr1)));
        OpenIPSL.Types.PerUnit Tm;
        OpenIPSL.Types.PerUnit Te;
        OpenIPSL.Types.PerUnit P(start=P_0/S_b);
        //OpenIPSL.Types.PerUnit P;
        OpenIPSL.Types.PerUnit Q(start=Q_0/S_b);
        //OpenIPSL.Types.PerUnit Q;
        OpenIPSL.Types.PerUnit Epr;
        OpenIPSL.Types.PerUnit Epi;
        OpenIPSL.Types.PerUnit Eppr( start = Eppr0);
        OpenIPSL.Types.PerUnit Eppi( start = Eppi0);
        OpenIPSL.Types.PerUnit Epp;
        OpenIPSL.Types.PerUnit Ekr;
        OpenIPSL.Types.PerUnit Eki;
        OpenIPSL.Types.PerUnit Vr;
        OpenIPSL.Types.PerUnit Vi;
        OpenIPSL.Types.PerUnit Ir;
        OpenIPSL.Types.PerUnit Ii;
        OpenIPSL.Types.PerUnit o1;
        OpenIPSL.Types.PerUnit o2;
        OpenIPSL.Types.PerUnit o3;
        OpenIPSL.Types.PerUnit o4;
        OpenIPSL.Types.PerUnit o5;
        OpenIPSL.Types.PerUnit o6;
        OpenIPSL.Types.PerUnit o7;
        OpenIPSL.Types.PerUnit Omegar(start = wr0) "Rotor angular velocity";
        //OpenIPSL.Types.PerUnit dw "Per unit difference of the angular velocities";

        OpenIPSL.Interfaces.PwPin p(
          vr(start=vr0),
          vi(start=vi0),
          ir(start=ir0_sys),
          ii(start=ii0_sys))
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        import Modelica.Constants.pi;

      protected
        parameter OpenIPSL.Types.PerUnit p0 = P_0/M_b;
        parameter OpenIPSL.Types.PerUnit q0 = Q_0/M_b;
        parameter OpenIPSL.Types.PerUnit Omegab=1 "Base freq in rad/s";
        parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
        parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
        parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
        parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
        parameter OpenIPSL.Types.PerUnit ir0_sys = CoB*ir0 "Initial real current in system base";
        parameter OpenIPSL.Types.PerUnit ii0_sys = CoB*ii0 "Initial imaginary current in system base";
        parameter OpenIPSL.Types.PerUnit Eppr0 = -(Ra*ir0 - Lpp*ii0 - vr0);
        parameter OpenIPSL.Types.PerUnit Eppi0 = -(Lpp*ir0 + Ra*ii0 - vi0);
        parameter OpenIPSL.Types.PerUnit Ls = Xa + Xm;
        parameter OpenIPSL.Types.PerUnit Ll = Xa;
        parameter OpenIPSL.Types.PerUnit Lp = Xa + X1*Xm/(X1 + Xm);
        parameter OpenIPSL.Types.PerUnit Lpp = if Mtype == 1 then Xa + X1*Xm*X2/(X1*X2 + X1*Xm + X2*Xm) else Xa + (X1*Xm+X2*Xm)/(X1 + X2 + Xm);
        parameter OpenIPSL.Types.Time Tp0 = if Mtype == 1 then (X1 + Xm)/(Omegab*R1) else (X1 + X2 + Xm)/(Omegab*R2);
        parameter OpenIPSL.Types.Time Tpp0 = if Mtype == 1 then (X2 + (X1*Xm/(X1 + Xm)))/(Omegab*R2) else 1/((1/(X1+Xm) + 1/X2)/(Omegab*R1));
        parameter OpenIPSL.Types.PerUnit k1 = Ls - Lp;
        parameter OpenIPSL.Types.PerUnit k2 = Lp - Ll;
        parameter OpenIPSL.Types.PerUnit k3 = (Lp - Lpp)/((Lp - Ll)^2);
        parameter OpenIPSL.Types.PerUnit k4 = (Lp - Lpp)/(Lp - Ll);
        parameter OpenIPSL.Types.PerUnit k5 = (Lpp - Ll)/(Lp - Ll);
        parameter OpenIPSL.Types.PerUnit wr0 = -(Eppr0*ir0/T + Eppi0*ii0/T)^(1/D) +2;
        parameter Real CoB=M_b/S_b;
      initial equation
        der(Epr) = 0;
        der(Epi) = 0;
        der(Omegar) = 0;
        der(Ekr) = 0;
        der(Eki) = 0;
      equation

        anglev = atan2(p.vi, p.vr);
        delta = anglev;
        v = sqrt(p.vr^2 + p.vi^2);
        Vr = p.vr;
        Vi = p.vi;
        //Ir = p.ir/CoB;
        //Ii = p.ii/CoB;
        //[Ir; Ii] = (1/CoB)*[cos(delta), sin(delta); -sin(delta), cos(delta)]*[p.ir; p.ii];
        [Ir; Ii] = (1/CoB)*[p.ir; p.ii];
        Ir = ((Vr - Eppr)*Ra + (Vi - Eppi)*Lpp)/(Ra^2 + Lpp^2);
        Ii = ((Vi - Eppi)*Ra - (Vr - Eppr)*Lpp)/(Ra^2 + Lpp^2);
        P = p.vr*p.ir + p.vi*p.ii;
        Q = (-p.vr*p.ii) + p.vi*p.ir;
        //der(s) = (Tm - Te)/(2*H);

        // Mechanical Equation
        s = 1 - Omegar;
        Te = Eppr*Ir + Eppi*Ii;
        der(Omegar) = (Te - Tm)/(2*H);
        Tm = T*(A*Omegar^2 + B*Omegar + C + D*Omegar^E);

        // Electrical-Magnetic Equations for the CIM5 double cage rotor model
        o1 = Epr - k2*Ii - Ekr;
        o2 = ((o1*k3) + Ii)*k1;
        o3 = Epi*(Tp0*Omegab*s) - o2 - Epr + o4;
        der(Epr) = o3/Tp0;
        der(Ekr) = (o1 + (Tpp0*Omegab*s)*Eki)/Tpp0;
        Eppr = (Ekr*k4) + (Epr*k5);
        Epp = sqrt(Eppr^2 + Eppi^2);
        o4 = (SE_exp(Epp,S10,S12,1,1.2)/Epp)*Eppi;
        o5 = (SE_exp(Epp,S10,S12,1,1.2)/Epp)*Eppr;
        o7 = -Epr*(Tp0*Omegab*s) - o5 + o6 - Epi;
        der(Epi) = o7/Tp0;
        o6 = k1*(-k3*(Epi - Eki + Ir*k2) + Ir);
        der(Eki) = ((Epi - Eki + Ir*k2) - Ekr*(Tpp0*Omegab*s))/Tpp0;
        Eppi = Epi*k5 + k4*Eki;

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics={Rectangle(
                fillColor={255,255,255},
                extent={{-100,-100},{100,100}}),Ellipse(
                fillColor={255,255,255},
                extent={{-56,-58},{55.9318,54}}),Text(
                extent={{-50,48},{50,-52}},
                lineColor={0,0,0},
                textString="M"),Text(
                origin={0,-76.0978},
                fillPattern=FillPattern.Solid,
                extent={{-57.2101,-15.0},{57.2101,15.0}},
                fontName="Arial",
                textString="%name",
                lineColor={0,0,0})}), Documentation(revisions="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>PSAT Manual 2.1.8</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>September 2015</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Joan Russinol, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p>see <a href=\"modelica://OpenIPSL.UsersGuide.Contact\">UsersGuide.Contact</a></p></td>
</tr>
</table>
</html>",       info="<html>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p><img src=\"modelica://MicroGrid/../../Figures/Motor/CIM5/steady_state_circuit.png\"/></p></td>
</tr>
<tr>
<td></td>
</tr>
</table>
<p><br><br><br>The block diagram below represents the dynamics behind the induced voltages in the rotor winding.</p>
<p>The hatched components follow the same structure found in the equations that describes the CIM5 induction motor.</p>
<p><br><br><img src=\"modelica://MicroGrid/../../Figures/Motor/CIM5/CIM5_diagram.png\"/></p>
</html>"));
      end CIM6;
    end PSSE;
  end ThreePhase;
end InductionMotor;
