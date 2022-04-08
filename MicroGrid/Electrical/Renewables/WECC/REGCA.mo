within MicroGrid.Electrical.Renewables.WECC;
model REGCA
  import SIunits =
              Modelica.Units.SI;
  import Modelica.Units.Conversions.*;
  parameter OpenIPSL.Types.ApparentPower S_b = 100e6 "System base power.";
  parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "Generator base power.";
  parameter OpenIPSL.Types.ActivePower P_0 = 100e6 "Generator initial active power.";
  parameter OpenIPSL.Types.ReactivePower Q_0 = 0.5795379e6 "Generator initial reactive power.";
  parameter OpenIPSL.Types.PerUnit v_0 = 1 "Initial Terminal Voltage.";
  parameter OpenIPSL.Types.Angle angle_0(displayUnit = "deg") "Initial Terminal Angle.";

  Modelica.Blocks.Interfaces.RealInput Iqcmd(start=-Iq0)
    annotation (Placement(transformation(extent={{-160,50},{-120,90}}),
        iconTransformation(extent={{-160,50},{-120,90}})));
  Modelica.Blocks.Interfaces.RealInput Ipcmd(start = Ip0)
    annotation (Placement(transformation(extent={{-160,-90},{-120,-50}}),
        iconTransformation(extent={{-160,-90},{-120,-50}})));
  parameter OpenIPSL.Types.Time Tg=0.02
    "Inverter current regulator lag time constant (s)."
    annotation (Dialog(tab="Input Parameters"));
  parameter Real Iqrmax=9999
    "Maximum rate-of-change of reactive current (pu/s)."
    annotation (Dialog(tab="Input Parameters"));
  Modelica.Blocks.Sources.RealExpression Vt(y=sqrt(p.vi^2 + p.vr^2))
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  Modelica.Blocks.Math.Add add(k2=-1)
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
  Modelica.Blocks.Sources.Constant Vo_limit(k=Volim)
    annotation (Placement(transformation(extent={{-100,14},{-80,34}})));

  Modelica.Blocks.Nonlinear.Limiter min_limiter(uMax=Modelica.Constants.inf,
      uMin=0) annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Modelica.Blocks.Math.Add add1(k2=-1)
    annotation (Placement(transformation(extent={{48,26},{68,46}})));
  Modelica.Blocks.Nonlinear.Limiter IOLIM(uMax=1e6, uMin=Iolim)
    annotation (Placement(transformation(extent={{92,26},{112,46}})));
  parameter Real Iolim=-1.3
    "Current limit for high voltage clamp logic (pu on mbase)."
    annotation (Dialog(tab="Input Parameters"));
  Logic_Blocks.REGCA_Logic_Blocks.Low_Voltage_Active_Current_Management
    LVACM(lvpnt0=lvpnt0, lvpnt1=lvpnt1)
    annotation (Placement(transformation(extent={{92,-80},{112,-60}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(
    K=1,                                                     T=Tfltr,
    y_start=v_0)
    annotation (Placement(transformation(extent={{74,-80},{54,-60}})));
  parameter OpenIPSL.Types.Time Tfltr=0.02
    "Terminal voltage filter (for LVPL) time constant (s)."
    annotation (Dialog(tab="Input Parameters"));
  Logic_Blocks.REGCA_Logic_Blocks.Low_Voltage_Power_Logic LVPL(
    Brkpt=Brkpt,
    Lvpl1=Lvpl1,
    Zerox=Zerox)
    annotation (Placement(transformation(extent={{40,-80},{20,-60}})));
  Modelica.Blocks.Math.Product IP
    annotation (Placement(transformation(extent={{60,-26},{80,-6}})));
  parameter Real lvpnt0=0.4
    "Low voltage active current management breakpoint (pu)."
    annotation (Dialog(tab="Input Parameters"));

public
  Modelica.Blocks.Math.Gain KHV(k=Khv)
    annotation (Placement(transformation(extent={{-30,20},{-10,40}})));
  parameter Real Khv=0.7 "High voltage clamp logic acceleration factor."
    annotation (Dialog(tab="Input Parameters"));
  parameter Real Brkpt=0.9 "LVPL breakpoint (pu voltage)."
    annotation (Dialog(tab="Input Parameters"));
  parameter Real Zerox=0.5 "LVPL zero crossing (pu voltage)."
    annotation (Dialog(tab="Input Parameters"));
  parameter Real Lvpl1=1.22
    "LVPL gain breakpoint (pu current on mbase / pu voltage)."
    annotation (Dialog(tab="Input Parameters"));
  Modelica.Blocks.Sources.RealExpression Active_Power(y=-(1/CoB)*(p.vr*p.ir + p.vi
        *p.ii))
    annotation (Placement(transformation(extent={{-10,110},{10,130}})));
  Modelica.Blocks.Sources.RealExpression Reactive_Power(y=-(1/CoB)*(p.vi*p.ir -
        p.vr*p.ii))
    annotation (Placement(transformation(extent={{50,110},{70,130}})));
  Modelica.Blocks.Interfaces.RealOutput Pgen "Value of Real output"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={20,150}),  iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={20,150})));
  Modelica.Blocks.Interfaces.RealOutput Qgen "Value of Real output"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,150}),  iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,150})));
  Modelica.Blocks.Interfaces.RealOutput V_t annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-50,150}), iconTransformation(extent={{-10,-10},{10,10}},
          origin={-50,150},
        rotation=90)));
  parameter Real Volim=1.2 "Voltage limit for high voltage clamp logic (pu)"
    annotation (Dialog(tab="Input Parameters"));
  parameter Real Iqrmin=-9999
    "Minimum rate-of-change of reactive current (pu/s)."
    annotation (Dialog(tab="Input Parameters"));
  parameter Real lvpnt1=0.8
    "Low voltage active current management breakpoint (pu)."
    annotation (Dialog(tab="Input Parameters"));
    //parameter Real B;
  OpenIPSL.Interfaces.PwPin p(
    vr(start=vr0),
    vi(start=vi0),
    ir(start=ir1),
    ii(start=ii1)) annotation (Placement(transformation(extent={{160,-10},{
            180,10}}),    iconTransformation(extent={{160,-10},{180,10}})));
  Modelica.Blocks.Logical.Switch switch1 annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-50,-90})));
  Modelica.Blocks.Sources.BooleanConstant Lvplsw_logic(k=Lvplsw)
    annotation (Placement(transformation(extent={{0,-100},{-20,-80}})));
  parameter Real rrpwr=10
    "Active current up-ramp rate limit on voltage recovery (pu/s)."
    annotation (Dialog(tab="Input Parameters"));
  parameter Boolean Lvplsw=true
    "Enable (True) or disable (False) low voltage power logic."
    annotation (Dialog(tab="Controls"));
  OpenIPSL.NonElectrical.Continuous.IntegratorLimVar integratorLimVar(K=1/Tg, y_start=
        Ip0)
    annotation (Placement(transformation(extent={{-22,-40},{-2,-20}})));
  Modelica.Blocks.Nonlinear.Limiter limiter4(uMax=rrpwr, uMin=-Modelica.Constants.inf)
    annotation (Placement(transformation(extent={{-52,-40},{-32,-20}})));
  Modelica.Blocks.Math.Add add2(k2=-1)
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
     Complex Is "Equivalent internal current source";
  Modelica.Blocks.Interfaces.RealOutput IP0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,-150}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,-150})));
  Modelica.Blocks.Interfaces.RealOutput IQ0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={80,-150}),  iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={80,-150})));
  Modelica.Blocks.Sources.RealExpression initial_IP0(y=Ip0) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-10,-130})));
  Modelica.Blocks.Sources.RealExpression initial_IQ0(y=Iq0) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={110,-130})));
  Modelica.Blocks.Sources.RealExpression Terminal_Voltage(y=Vt.y)
    annotation (Placement(transformation(extent={{-80,110},{-60,130}})));
  Modelica.Blocks.Sources.RealExpression Terminal_Voltage1(y=Vt.y) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={64,-100})));
  Modelica.Blocks.Sources.RealExpression LowerLimit(y=-Modelica.Constants.inf)
    annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
  Modelica.Blocks.Sources.RealExpression Constant(y=Lvpl1)
    annotation (Placement(transformation(extent={{0,-124},{-20,-104}})));
  Modelica.Blocks.Math.Add add3(k2=-1)
    annotation (Placement(transformation(extent={{-56,64},{-36,84}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Iqrmax, uMin=Iqrmin)
    annotation (Placement(transformation(extent={{-20,64},{0,84}})));
  Modelica.Blocks.Continuous.Integrator integrator(k=1/Tg, y_start=Iq0/v_0)
    annotation (Placement(transformation(extent={{20,64},{40,84}})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
protected
  OpenIPSL.Types.Angle delta(start=angle_0);
  OpenIPSL.Types.PerUnit VT(start=v_0) "Bus voltage magnitude";
  OpenIPSL.Types.Angle anglev(start=angle_0) "Bus voltage angle";
  parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
  parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter Real CoB=M_b/S_b;
  parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit Isr0=ir0 "Source current re M_b";
  parameter OpenIPSL.Types.PerUnit Isi0=ii0 "Source current im M_b";
  parameter OpenIPSL.Types.PerUnit ir1=-CoB*(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2) "Real component of initial armature current, S_b";
  parameter OpenIPSL.Types.PerUnit ii1=-CoB*(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2) "Imaginary component of initial armature current, S_b";

  parameter OpenIPSL.Types.PerUnit Ip0=Isr0*cos(-angle_0) - Isi0*sin(-angle_0);
  parameter OpenIPSL.Types.PerUnit Iq0=(Isr0*sin(-angle_0) + cos(-angle_0)*Isi0);
//initial equation
  //B = if v_0<= Volim then 0 else Khv*(v_0-Volim);
protected
  Modelica.Blocks.Interfaces.RealOutput Iq
               "Connector of Real output signal" annotation (Placement(
        transformation(extent={{160,60},{180,80}}), iconTransformation(extent={{
            155,39},{157,41}})));
protected
  Modelica.Blocks.Interfaces.RealOutput Ip
               "Connector of Real output signal" annotation (Placement(
        transformation(extent={{160,-80},{180,-60}}), iconTransformation(extent=
           {{129,-27},{131,-25}})));
equation

  anglev = atan2(p.vi, p.vr);
  VT = sqrt(p.vr*p.vr + p.vi*p.vi);
  delta = anglev;
  Is.re = p.ir/CoB;
  Is.im = p.ii/CoB;
  [Ip; Iq] = -[cos(delta), sin(delta); -sin(delta), cos(delta)]*[Is.re; Is.im];

  connect(add.y, KHV.u)
    annotation (Line(points={{-39,30},{-32,30}},
                                              color={0,0,127}));
  connect(IOLIM.u, add1.y)
    annotation (Line(points={{90,36},{69,36}},   color={0,0,127}));
  connect(LVPL.V, simpleLag.y)
    annotation (Line(points={{41,-70},{53,-70}}, color={0,0,127}));
  connect(LVACM.y, IP.u2) annotation (Line(points={{113,-70},{120,-70},{120,
          -40},{52,-40},{52,-22},{58,-22}},
                             color={0,0,127}));
  connect(Qgen,Qgen)
    annotation (Line(points={{90,150},{90,150}},   color={0,0,127}));
  connect(Active_Power.y, Pgen) annotation (Line(points={{11,120},{20,120},{
          20,150}},                       color={0,0,127}));
  connect(Reactive_Power.y, Qgen) annotation (Line(points={{71,120},{90,120},
          {90,150}},       color={0,0,127}));
  connect(switch1.u1, LVPL.y)
    annotation (Line(points={{-38,-82},{-26,-82},{-26,-70},{19,-70}},
                                                           color={0,0,127}));
  connect(Lvplsw_logic.y, switch1.u2) annotation (Line(points={{-21,-90},{-38,
          -90}},                color={255,0,255}));
  connect(KHV.y, min_limiter.u)
    annotation (Line(points={{-9,30},{-2,30}}, color={0,0,127}));
  connect(Vt.y, add.u1) annotation (Line(points={{-79,50},{-64,50},{-64,36},{
          -62,36}},  color={0,0,127}));
  connect(simpleLag.u,LVACM. Vt) annotation (Line(points={{76,-70},{91,-70}},
                                                           color={0,0,127}));
  connect(Vo_limit.y, add.u2)
    annotation (Line(points={{-79,24},{-62,24}}, color={0,0,127}));
  connect(limiter4.y, integratorLimVar.u)
    annotation (Line(points={{-31,-30},{-24,-30}},
                                                 color={0,0,127}));
  connect(add2.y, limiter4.u)
    annotation (Line(points={{-59,-30},{-54,-30}}, color={0,0,127}));
  connect(integratorLimVar.y, add2.u2) annotation (Line(points={{-1,-30},{4,
          -30},{4,-70},{-88,-70},{-88,-36},{-82,-36}},
                                                     color={0,0,127}));
  connect(IP.u1, add2.u2) annotation (Line(points={{58,-10},{-88,-10},{-88,
          -36},{-82,-36}},                                    color={0,0,127}));
  connect(Terminal_Voltage.y, V_t)
    annotation (Line(points={{-59,120},{-50,120},{-50,150}}, color={0,0,127}));
  connect(Terminal_Voltage1.y, LVACM.Vt) annotation (Line(points={{75,-100},{
          84,-100},{84,-70},{91,-70}},
                                     color={0,0,127}));
  connect(LowerLimit.y, integratorLimVar.outMin)
    annotation (Line(points={{-29,-60},{-20,-60},{-20,-44}}, color={0,0,127}));
  connect(Constant.y, switch1.u3) annotation (Line(points={{-21,-114},{-32,
          -114},{-32,-98},{-38,-98}},
                                color={0,0,127}));
  connect(initial_IP0.y, IP0) annotation (Line(points={{-21,-130},{-40,-130},
          {-40,-150}},        color={0,0,127}));
  connect(IOLIM.y, Iq) annotation (Line(points={{113,36},{140,36},{140,70},{
          170,70}},
                color={0,0,127}));
  connect(IP.y, Ip) annotation (Line(points={{81,-16},{140,-16},{140,-70},{
          170,-70}},
                 color={0,0,127}));
  connect(add3.y, limiter1.u)
    annotation (Line(points={{-35,74},{-22,74}},color={0,0,127}));
  connect(IP.u1, integratorLimVar.y)
    annotation (Line(points={{58,-10},{4,-10},{4,-30},{-1,-30}},
                                                        color={0,0,127}));
  connect(limiter1.y, integrator.u)
    annotation (Line(points={{1,74},{18,74}},  color={0,0,127}));
  connect(integrator.y, add3.u2) annotation (Line(points={{41,74},{46,74},{46,
          56},{-64,56},{-64,68},{-58,68}}, color={0,0,127}));
  connect(min_limiter.y, add1.u2)
    annotation (Line(points={{21,30},{46,30}}, color={0,0,127}));
  connect(gain.y, add3.u1) annotation (Line(points={{-69,80},{-58,80}},
                               color={0,0,127}));
  connect(gain.u, Iqcmd) annotation (Line(points={{-92,80},{-100,80},{-100,70},
          {-140,70}}, color={0,0,127}));
  connect(add1.u1, add3.u2) annotation (Line(points={{46,42},{24,42},{24,56},
          {-64,56},{-64,68},{-58,68}},                 color={0,0,127}));
  connect(initial_IQ0.y, IQ0) annotation (Line(points={{99,-130},{80,-130},{
          80,-150}}, color={0,0,127}));
  connect(Ipcmd, add2.u1) annotation (Line(points={{-140,-70},{-100,-70},{
          -100,-24},{-82,-24}}, color={0,0,127}));
  connect(switch1.y, integratorLimVar.outMax) annotation (Line(points={{-61,
          -90},{-94,-90},{-94,0},{-4,0},{-4,-16}}, color={0,0,127}));
  annotation (Dialog(tab="Input Parameters"),
                Dialog(tab="Input Parameters"),
              Dialog(tab="Input Parameters"),
                Dialog(tab="Input Parameters"),
              Dialog(tab="Input Parameters"),
    Diagram(coordinateSystem(extent={{-120,-140},{160,140}})),
    Icon(coordinateSystem(extent={{-120,-140},{160,140}}), graphics={
          Rectangle(extent={{-120,140},{160,-140}}, lineColor={28,108,200}),
          Text(
          extent={{-76,46},{118,-40}},
          textColor={238,46,47},
          textStyle={TextStyle.Bold},
          textString="REGCA")}));
end REGCA;
