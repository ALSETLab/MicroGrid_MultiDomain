within MicroGrid.Electrical.MultiDomain.InductionMotor.ThreePhase.PSAT;
model MotorTypel_MultiDomain_Full "Induction Machine - Order I"
  extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enableS_b=true);
  parameter Integer Sup=1 "Start-up control" annotation (Dialog(group=
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
  parameter OpenIPSL.Types.Time Hm=3 "Inertia constant [Ws/VA]"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit a=0.5 "1st coefficient of tau_m(w)"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit b=0.00 "2nd coefficient of tau_m(w)"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit c=0.00 "3rd coefficient of tau_m(w)"
    annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.Time tup=0 "Start up time"
    annotation (Dialog(group="Machine parameters"));
  OpenIPSL.Types.PerUnit v(start=v_0) "Bus voltage magnitude";
  OpenIPSL.Types.Angle anglev(start=angle_0) "Bus voltage angle";
  OpenIPSL.Types.PerUnit s;
  OpenIPSL.Types.PerUnit Tm;
  OpenIPSL.Types.PerUnit P(start=P_0/S_b);
  OpenIPSL.Types.PerUnit Q(start=Q_0/S_b);
  OpenIPSL.Types.PerUnit P_motor;
  OpenIPSL.Types.PerUnit Q_motor;
  OpenIPSL.Types.PerUnit Re;
  OpenIPSL.Types.PerUnit I;
  OpenIPSL.Interfaces.PwPin p(
    vr(start=vr0),
    vi(start=vi0),
    ir(start=ir0),
    ii(start=ii0))
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
   //Modelica.SIunits.Inertia J_load;
  Modelica.Units.SI.Torque TORQUE;
  Modelica.Units.SI.AngularVelocity nr;
   //Modelica.SIunits.AngularVelocity ns;

  parameter Modelica.Units.SI.AngularVelocity w_nom=2*Modelica.Constants.pi
      *fn;
  parameter Modelica.Units.SI.ApparentPower M_b=10000;
   parameter Real N "Number of pair of Poles";

  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
      Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  Modelica.Blocks.Sources.RealExpression Rotor_Speed(y=nr)
    annotation (Placement(transformation(extent={{80,-10},{60,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia Rotor_Inertia(
    J=J_load,
    w(start=188.275),
    a(fixed=false))
    annotation (Placement(transformation(extent={{-32,-10},{-52,10}})));
  Modelica.Blocks.Interfaces.RealOutput wr
    "Absolute angular velocity of flange as output signal"
    annotation (Placement(transformation(extent={{-100,-50},{-120,
            -30}}),
        iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={60,-120})));
  Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-120}),
                          iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-120})));
  Modelica.Blocks.Interfaces.RealInput we(start = 188.524) annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={60,-120}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
protected
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit A=a + b + c;
  parameter OpenIPSL.Types.PerUnit B=(-b) - 2*c;
  parameter OpenIPSL.Types.PerUnit C=c;
  parameter OpenIPSL.Types.PerUnit Xe=Xs + Xr1;
  parameter OpenIPSL.Types.PerUnit s0 = Rr1*P_0/S_b*(Q_0/S_b + v_0*v_0/Xm)/(v_0*v_0*v_0*v_0*(Xs + Xr1));
  Real CoB = M_b/S_b;
  parameter Real J_load = 2*Hm*M_b/((2*Modelica.Constants.pi*fn/N)^2);
  //parameter OpenIPSL.Types.PerUnit s0 = Rr1*(Xm*(Q_0/S_b)-v_0^2)*Rr1/(P_0*Xm*(Xs + Xr1)/S_b);

  Modelica.Mechanics.Rotational.Sources.Speed speed annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={10,0})));
initial equation
  //der(s) = 0;

equation

  //Active and Reactive Power consumption in the system base
  P =   p.vr*p.ir  + p.vi*p.ii;
  Q = (-p.vr*p.ii) + p.vi*p.ir;

  //Active and Reactive Power consumption in the machine base
  P_motor = P/CoB;
  Q_motor = Q/CoB;

  //Magnitude of the Stator Motor Currents in the machine base
  I = sqrt(p.ir^2 + p.ii^2);

  //Voltage and angle at the pwpin connection
  v = sqrt(p.vr^2 + p.vi^2);
  anglev = atan2(p.vi, p.vr);

  Tm = A + B*s + C*s*s;

  //Rotor speed equation
  nr = (1-s)*we;

  //Conversion of PerUnit torque to SI torque.
  TORQUE = P*S_b/nr;

  Re = Rs + Rr1/s;
  //s=Rr1/(Re-Rs);
  der(s) = (mech_torque*nr/M_b - P_motor*(1-s))/(2*Hm);
  p.ii/CoB = (-p.vr/Xm) + (p.vi*Re - p.vr*Xe)/(Re*Re + Xe*Xe);
  p.ir/CoB =   p.vi/Xm  + (p.vr*Re + p.vi*Xe)/(Re*Re + Xe*Xe);
  connect(Rotor_Inertia.flange_a, speed.flange)
    annotation (Line(points={{-32,0},{1.77636e-15,7.21645e-16}},
                                                        color={0,0,0}));
  connect(speed.w_ref, Rotor_Speed.y) annotation (Line(points={{22,
          -1.9984e-15},{59,0}},
                          color={0,0,127}));
  connect(wr, Rotor_Speed.y) annotation (Line(points={{-110,-40},{
          40,-40},{40,0},{59,0}},
                       color={0,0,127}));
  connect(flange, Rotor_Inertia.flange_b)
    annotation (Line(points={{-100,0},{-52,0}}, color={0,0,0}));
  connect(wr, speed.w_ref) annotation (Line(points={{-110,-40},{40,
          -40},{40,-1.9984e-15},{22,-1.9984e-15}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={Rectangle(
          fillColor={255,255,255},
          extent={{-100,-100},{100,100}},
          lineColor={0,0,0}),             Ellipse(
          fillColor={255,255,255},
          extent={{-56,-58},{55.9318,54}},
          lineColor={0,0,0},
          lineThickness=1),                Text(
          extent={{-36,30},{36,-40}},
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
end MotorTypel_MultiDomain_Full;
