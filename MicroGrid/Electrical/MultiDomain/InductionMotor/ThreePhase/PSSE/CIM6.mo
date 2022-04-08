within MicroGrid.Electrical.MultiDomain.InductionMotor.ThreePhase.PSSE;
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
</html>", info="<html>
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
