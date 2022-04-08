within MicroGrid.Electrical.MultiDomain.InductionMotor.ThreePhase.PSAT;
model MotorTypeI_MultiDomain_Simple "Induction Machine - Order I"
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
  OpenIPSL.Types.PerUnit s(start=Rr1*P_0/S_b*(Q_0/S_b + v_0*v_0/Xm)/(v_0*v_0*
        v_0*v_0*(Xs + Xr1)));
  OpenIPSL.Types.PerUnit Tm;
  OpenIPSL.Types.PerUnit P(start=P_0/S_b);
  OpenIPSL.Types.PerUnit Q(start=Q_0/S_b);
  OpenIPSL.Types.PerUnit Re;
  OpenIPSL.Interfaces.PwPin p(
    vr(start=vr0),
    vi(start=vi0),
    ir(start=ir0),
    ii(start=ii0))
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));

  // Modelica.SIunits.AngularVelocity w_out;
   //Modelica.SIunits.AngularAcceleration acc;
   //Modelica.SIunits.Angle phi;
   //Modelica.SIunits.Torque Tm;
  Modelica.Units.SI.Inertia J_load;
  Modelica.Units.SI.Torque TORQUE;
  Modelica.Units.SI.AngularVelocity ns;
  Modelica.Units.SI.AngularVelocity nr;

  parameter Modelica.Units.SI.AngularVelocity w_nom=2*Modelica.Constants.pi
      *60;
  parameter Modelica.Units.SI.ApparentPower nom_mech_power=10000;
   parameter Real N "Number of pair of Poles";

  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation (
      Placement(transformation(extent={{-110,-10},{-90,10}}),iconTransformation(
          extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque_rotor(useSupport=
        false)
    annotation (Placement(transformation(extent={{18,-10},{-2,10}})));
  Modelica.Blocks.Sources.RealExpression Torque_Motor(y=TORQUE)
    annotation (Placement(transformation(extent={{72,-10},{52,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia Inertia_of_load(J=2*Hm*S_b/((
        2*Modelica.Constants.pi*fn/N)^2),
    w(start=1),
    a(fixed=false))
             annotation (Placement(transformation(extent={{-32,-10},{-52,10}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-34,-42})));
  Modelica.Blocks.Interfaces.RealOutput wr
    "Absolute angular velocity of flange as output signal"
    annotation (Placement(transformation(extent={{-100,-70},{-120,-50}})));
protected
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^
      2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^
      2);
  parameter OpenIPSL.Types.PerUnit A=a + b + c;
  parameter OpenIPSL.Types.PerUnit B=(-b) - 2*c;
  parameter OpenIPSL.Types.PerUnit C=c;
  parameter OpenIPSL.Types.PerUnit Xe=Xs + Xr1;

initial equation
  der(s) = 0;

equation

  P = p.vr*p.ir + p.vi*p.ii;
  Q = (-p.vr*p.ii) + p.vi*p.ir;
  anglev = atan2(p.vi, p.vr);
  v = sqrt(p.vr^2 + p.vi^2);
  Tm = A + B*s + C*s*s;

  ns = 2*Modelica.Constants.pi*fn/N;
  nr = (1-s)*ns;

  TORQUE = P*S_b/nr;
  J_load = 2*Hm*S_b/((2*Modelica.Constants.pi*fn/N)^2);

  Re = Rs + Rr1/s;
  //s=Rr1/(Re-Rs);
  der(s) = (Tm - P)/(2*Hm);
  p.ii = (-p.vr/Xm) + (p.vi*Re - p.vr*Xe)/(Re*Re + Xe*Xe);
  p.ir = p.vi/Xm + (p.vr*Re + p.vi*Xe)/(Re*Re + Xe*Xe);
  connect(Inertia_of_load.flange_a, torque_rotor.flange)
    annotation (Line(points={{-32,0},{-2,0}}, color={0,0,0}));
  connect(Inertia_of_load.flange_b, flange_b)
    annotation (Line(points={{-52,0},{-100,0}}, color={0,0,0}));
  connect(speedSensor.w, wr) annotation (Line(points={{-45,-42},{-58,
          -42},{-58,-60},{-110,-60}},
                       color={0,0,127}));
  connect(speedSensor.flange, Inertia_of_load.flange_a)
    annotation (Line(points={{-24,-42},{-24,0},{-32,0}}, color={0,0,0}));
  connect(Torque_Motor.y, torque_rotor.tau)
    annotation (Line(points={{51,0},{20,0}},color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),
                         graphics={Rectangle(
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
</html>"),
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
end MotorTypeI_MultiDomain_Simple;
