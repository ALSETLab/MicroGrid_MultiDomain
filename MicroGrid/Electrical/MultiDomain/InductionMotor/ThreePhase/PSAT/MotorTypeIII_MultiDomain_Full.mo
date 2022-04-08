within MicroGrid.Electrical.MultiDomain.InductionMotor.ThreePhase.PSAT;
model MotorTypeIII_MultiDomain_Full "MultiDomain Induction Machine - Order III"
  extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enableS_b=true);
  parameter Integer Sup=1 "Start up control" annotation (Dialog(group=
          "Machine parameters"), choices(choice=0, choice=1));
  parameter OpenIPSL.Types.PerUnit Rs=0.01 "Stator resistance"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit Xs=0.15 "Stator reactance"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit Rr1=0.05 "1st cage rotor resistance"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit Xr1=0.15 "1st cage rotor reactance"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit Xm=5 "Magnetizing reactance"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.Time Hm=3 "Inertia constant"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit a=0.25 "1st coefficient of tau_m(w)"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit b=0.00 "2nd coefficient of tau_m(w)"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit c=0.00 "3rd coefficient of tau_m(w)"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.Time tup=0 "Start up time"
    annotation (Dialog(group="Machine parameters"));
  OpenIPSL.Types.PerUnit v(start=v_0) "Bus voltage magnitude";
  OpenIPSL.Types.PerUnit anglev(start=angle_0) "Bus voltage angle";
  OpenIPSL.Types.PerUnit s;
  //OpenIPSL.Types.PerUnit Tm;
  OpenIPSL.Types.PerUnit Te;
  Modelica.Units.SI.Torque TORQUE;
  OpenIPSL.Types.PerUnit P(start=P_0/S_b);
  OpenIPSL.Types.PerUnit Q(start=Q_0/S_b);
  OpenIPSL.Types.PerUnit P_motor;
  OpenIPSL.Types.PerUnit Q_motor;
  OpenIPSL.Types.PerUnit Vr;
  OpenIPSL.Types.PerUnit Vm;
  OpenIPSL.Types.PerUnit Ir;
  OpenIPSL.Types.PerUnit Im;
  OpenIPSL.Types.PerUnit epr(start=epr0);
  OpenIPSL.Types.PerUnit epm(start=epm0);
  OpenIPSL.Types.PerUnit I;
  OpenIPSL.Types.Angle anglei;
  OpenIPSL.Interfaces.PwPin p(
    vr(start=vr0),
    vi(start=vi0),
    ir(start=ir0),
    ii(start=ii0))
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
   //Modelica.SIunits.Inertia J_load;
   //Modelica.SIunits.Torque t_base;
  Modelica.Units.SI.AngularVelocity nr;

  parameter Modelica.Units.SI.AngularVelocity w_nom=2*Modelica.Constants.pi
      *fn;
   parameter Real N "Number of Poles";
  parameter Modelica.Units.SI.ApparentPower M_b=10000;

  import Modelica.Constants.pi;
  Modelica.Blocks.Sources.RealExpression Rotor_Speed(y=nr)
    annotation (Placement(transformation(extent={{80,-10},{60,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia Rotor_Inertia(J=J_load, w(
        start=370))
    annotation (Placement(transformation(extent={{-32,-10},{-52,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
      Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  Modelica.Blocks.Interfaces.RealOutput wr
    "Absolute angular velocity of flange as output signal"
    annotation (Placement(transformation(extent={{-100,-50},{-120,-30}}),
        iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={60,-120})));
  Modelica.Blocks.Interfaces.RealInput we(start=188.524) annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={60,-120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-120})));
protected
  parameter OpenIPSL.Types.AngularVelocity Omegab=2*pi*fn "Base freq";
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^
      2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^
      2);
  parameter OpenIPSL.Types.PerUnit i2=ir0*ir0 + ii0*ii0;
  parameter OpenIPSL.Types.PerUnit A=a + b + c;
  parameter OpenIPSL.Types.PerUnit B=(-b) - 2*c;
  parameter OpenIPSL.Types.PerUnit C=c;
  parameter OpenIPSL.Types.PerUnit X0=Xs + Xm;
  parameter OpenIPSL.Types.PerUnit Xp=Xs + Xr1*Xm/(Xr1 + Xm);
  parameter OpenIPSL.Types.Time Tp0=(Xr1 + Xm)/(Omegab*Rr1);
  parameter Real RZs2=1/(Rs*Rs + Xp*Xp);
  parameter Real K=Rr1/((Xr1 + Xm)*A);
  parameter Real K2=1 + Tp0*Tp0*Omegab*Omegab*S0*S0;
  parameter Real K1=Tp0*Omegab*S0;
  parameter Real a03=Rs*Rs + Xp*Xp;
  //r1^2+(xS +xR1xm/(xR1 + xm))^2
  parameter Real a13=Rs/a03;
  parameter Real a23=Xp/a03;
  parameter Real S0=K*((-Q_0/S_b) + X0*i2);
  parameter Real epm0=(K1*(X0 - Xp)*ir0 + (X0 - Xp)*(-1)*ii0)/K2;
  parameter Real epr0=(K1*(X0 - Xp)*(-1)*ii0 - (X0 - Xp)*ir0)/K2;
  Real CoB = M_b/S_b;
  parameter Real J_load = 2*Hm*M_b/((2*Modelica.Constants.pi*fn/N)^2);
  Modelica.Mechanics.Rotational.Sources.Speed speed annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={10,0})));
initial equation
  der(s) = 0;
  der(epr) = 0;
  der(epm) = 0;
equation

  //Active and Reactive Power consumption in the system base
  P =   p.vr*p.ir  + p.vi*p.ii;
  Q = (-p.vr*p.ii) + p.vi*p.ir;

  //Active and Reactive Power consumption in the machine base
  P_motor = P/CoB;
  Q_motor = Q/CoB;

  //Voltage magnitude and voltage angle at the pwpin connection
  v = sqrt(p.vr^2 + p.vi^2);
  anglev = atan2(p.vi, p.vr);

  //Current magnitude and current angle at the pwpin connection
  I = sqrt(p.ii^2 + p.ir^2);
  anglei = atan2(p.ii, p.ir);

  //Rotor speed equation
  nr = (1-s)*we;

  //Conversion of PerUnit torque to SI torque.
  TORQUE = P*S_b/nr;

  //Real and Imaginary components of Voltage and Current
  //Vr = -p.vi;
  //Vm = p.vr;
  //Im = p.ii;
  //Ir = p.ir;
  Vr = p.vi;
  Vm = p.vr;
  Im = p.ir;
  Ir = -p.ii;

  //Mechanical Slip Equation
  der(s) = (mech_torque/(M_b/nr) - Te)/(2*Hm);

  //Electromagnetic differential equations
  der(epr) = Omegab*s*epm - (epr + (X0 - Xp)*Im)/Tp0;
  der(epm) = (-Omegab*s*epr) - (epm - (X0 - Xp)*Ir)/Tp0;

  //The link between voltages, currents and state variables is
  //Vr = epr + Rs*Ir - Xp*Im;
  //Vm = epm  + Rs*Im + Xp*Ir;

  //Electromagnetic torque equation
  Te = epr*Ir + epm*Im;

  //Current consumed by the motor
  Im/CoB = (-a23*((-Vr) - epr)) + a13*(Vm - epm);
  Ir/CoB = a13*((-Vr) - epr) + a23*(Vm - epm);
  connect(Rotor_Inertia.flange_b, flange)
    annotation (Line(points={{-52,0},{-100,0}}, color={0,0,0}));
  connect(speed.w_ref, Rotor_Speed.y)
    annotation (Line(points={{22,-1.9984e-15},{59,0}}, color={0,0,127}));
  connect(speed.flange, Rotor_Inertia.flange_a) annotation (Line(points={{1.77636e-15,
          7.21645e-16},{-32,0}}, color={0,0,0}));
  connect(wr, Rotor_Speed.y) annotation (Line(points={{-110,-40},{40,-40},{40,0},
          {59,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={Rectangle(
          fillColor={255,255,255},
          extent={{-100,-100},{100,100}}),Ellipse(
          fillColor={255,255,255},
          extent={{-56,-58},{55.9318,54}}),Text(
          extent={{-50,48},{50,-52}},
          lineColor={0,0,0},
          textString="M"),
        Text(
          extent={{-142,184},{144,86}},
          lineColor={0,0,0},
          textString="%name")}),Documentation(revisions="<html>
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
</html>"),
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
end MotorTypeIII_MultiDomain_Full;
