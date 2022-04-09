within MicroGrid.Electrical.Renewables.WECC;
package GridFollowing
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

  model REGCC

    // Imported Libraries
    import SIunits =
                Modelica.Units.SI;
    import Modelica.Units.Conversions.*;

    // Initialization Parameters
    parameter OpenIPSL.Types.ApparentPower S_b = 100e6 "System base power.";
    parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "Generator base power.";
    parameter OpenIPSL.Types.ActivePower P_0 = 100e6 "Generator initial active power.";
    parameter OpenIPSL.Types.ReactivePower Q_0 = 0.5795379e6 "Generator initial reactive power.";
    parameter OpenIPSL.Types.PerUnit v_0 = 1 "Initial Terminal Voltage.";
    parameter OpenIPSL.Types.Angle angle_0(displayUnit = "deg") "Initial Terminal Angle.";

    // Model Parameters
    parameter OpenIPSL.Types.PerUnit re "Voltage Source equivalente resistance"
                                                                               annotation (Dialog(tab="Parameters"));
    parameter OpenIPSL.Types.PerUnit xe "Voltage Source equivalente reactance"
                                                                              annotation (Dialog(tab="Parameters"));

    // Current Conversion Variable
    //Complex Is "Source current coordinate";
    //Complex Ic "Control current coordinate DQ";
    //Complex Vc "Control voltage coordinate DQ";

    // P pin OpenIPSL
    OpenIPSL.Interfaces.PwPin p(
      vr(start=vr0),
      vi(start=vi0),
      ir(start=ir1),
      ii(start=ii1)) annotation (Placement(transformation(extent={{280,-40},{300,-20}}),
                            iconTransformation(extent={{280,-40},{300,-20}})));
    Modelica.Blocks.Math.Add add(k1=-1, k2=-1)
      annotation (Placement(transformation(extent={{-78,36},{-58,56}})));
    Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising=Iqrmax,
        Falling=Iqrmin,
      Td=0.0001,
      y_start=iq0)
      annotation (Placement(transformation(extent={{-120,40},{-100,60}})));
    Modelica.Blocks.Interfaces.RealInput Iqcmd( start = iq0) "Connector of Real input signal"
      annotation (Placement(transformation(extent={{-180,30},{-140,70}})));
    Modelica.Blocks.Math.Gain gain(k=Kip)
      annotation (Placement(transformation(extent={{-48,56},{-28,76}})));
    Modelica.Blocks.Continuous.Integrator integrator(k=Kii, y_start=iq0)
      annotation (Placement(transformation(extent={{-48,16},{-28,36}})));
    Modelica.Blocks.Math.Add add1(k2=+1)
      annotation (Placement(transformation(extent={{-18,36},{2,56}})));
    Modelica.Blocks.Sources.RealExpression Terminal_Voltage(y=sqrt(p.vi^2 + p.vr^2))
      annotation (Placement(transformation(extent={{0,90},{20,110}})));
    Modelica.Blocks.Math.Product product1
      annotation (Placement(transformation(extent={{-124,-100},{-104,-80}})));
    Modelica.Blocks.Interfaces.RealInput Ipcmd( start = id0) "Connector of Real input signal 1"
      annotation (Placement(transformation(extent={{-180,-130},{-140,-90}}),
          iconTransformation(extent={{-180,-130},{-140,-90}})));
    Modelica.Blocks.Logical.Switch RateFlag_switch
      annotation (Placement(transformation(extent={{-50,-170},{-70,-150}})));
    Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter1(Rising=rrpwr,
        Falling=-Modelica.Constants.inf,
      Td=0.0001,
      y_start=if RateFlag == true then id0*v_0 else id0)
      annotation (Placement(transformation(extent={{-94,-100},{-74,-80}})));
    Modelica.Blocks.Math.Division division
      annotation (Placement(transformation(extent={{-56,-100},{-36,-80}})));
    Modelica.Blocks.Math.Add add2(k2=-1)
      annotation (Placement(transformation(extent={{-20,-100},{0,-80}})));
    Modelica.Blocks.Sources.RealExpression Constant(y=1)
      annotation (Placement(transformation(extent={{-10,-190},{-30,-170}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLagLim simpleLagLim(
      K=1,
      T=Tfltr,
      y_start=v_0,                                              outMax=Modelica.Constants.inf,
        outMin=0.01)
      annotation (Placement(transformation(extent={{0,-150},{-20,-130}})));
    Modelica.Blocks.Sources.BooleanConstant RATEFLAG(k=RateFlag)
      annotation (Placement(transformation(extent={{30,-180},{10,-160}})));
    Modelica.Blocks.Sources.RealExpression Vt(y=Terminal_Voltage.y) annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={20,-140})));

    Electrical.Renewables.WECC.Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2
      Vt_conversion annotation (Placement(transformation(extent={{118,-144},
              {138,-124}})));
    Modelica.Blocks.Sources.RealExpression Vt_re(y=p.vr)
      annotation (Placement(transformation(extent={{88,-138},{108,-118}})));
    Modelica.Blocks.Sources.RealExpression Vt_im(y=p.vi)
      annotation (Placement(transformation(extent={{88,-130},{108,-150}})));
    Modelica.Blocks.Math.Gain KPPLL(k=Kppll)
      annotation (Placement(transformation(extent={{160,-130},{180,-110}})));
    Modelica.Blocks.Continuous.Integrator integrator1(k=Kipll, y_start=angle_0)
      annotation (Placement(transformation(extent={{158,-170},{178,-150}})));
    Modelica.Blocks.Math.Add add3
      annotation (Placement(transformation(extent={{188,-150},{208,-130}})));
    Modelica.Blocks.Nonlinear.Limiter wmax_wmin(uMax=wmax, uMin=wmin)
      annotation (Placement(transformation(extent={{218,-150},{238,-130}})));
    Modelica.Blocks.Continuous.Integrator integrator2(k=1, y_start=angle_0)
      annotation (Placement(transformation(extent={{248,-150},{268,-130}})));
    Electrical.Renewables.WECC.Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2
      It_conversion
      annotation (Placement(transformation(extent={{80,-180},{60,-160}})));
    Modelica.Blocks.Sources.RealExpression It_re(y=-p.ir/CoB)
      annotation (Placement(transformation(extent={{108,-174},{88,-154}})));
    Modelica.Blocks.Sources.RealExpression It_im(y=-p.ii/CoB)
      annotation (Placement(transformation(extent={{108,-166},{88,-186}})));
    Modelica.Blocks.Math.Gain KIP(k=Kip)
      annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
    Modelica.Blocks.Continuous.Integrator integrator3(k=Kii, y_start=id0)
      annotation (Placement(transformation(extent={{20,-120},{40,-100}})));
    Modelica.Blocks.Math.Add add4
      annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    Modelica.Blocks.Math.Gain RE_1(k=re)
      annotation (Placement(transformation(extent={{76,36},{96,56}})));
    Modelica.Blocks.Math.Gain RE_2(k=re)
      annotation (Placement(transformation(extent={{90,-100},{110,-80}})));
    Modelica.Blocks.Math.Add3 Eq
      annotation (Placement(transformation(extent={{116,28},{136,48}})));
    Modelica.Blocks.Sources.RealExpression Vtq0(y=vq0)
      annotation (Placement(transformation(extent={{36,20},{56,40}})));
    Modelica.Blocks.Sources.RealExpression Id(y=add4.y)
      annotation (Placement(transformation(extent={{36,0},{56,20}})));
    Modelica.Blocks.Math.Gain XE_1(k=xe)
      annotation (Placement(transformation(extent={{76,0},{96,20}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag Sensor1(K=1, T=Te,
      y_start=Eq0)
      annotation (Placement(transformation(extent={{146,28},{166,48}})));
    Modelica.Blocks.Math.Add3 Ed(k1=-1)
      annotation (Placement(transformation(extent={{116,-54},{136,-34}})));
    Modelica.Blocks.Sources.RealExpression Vtd0(y=vd0)
      annotation (Placement(transformation(extent={{36,-54},{56,-34}})));
    Modelica.Blocks.Sources.RealExpression Iq(y=add1.y)
      annotation (Placement(transformation(extent={{36,-34},{56,-14}})));
    Modelica.Blocks.Math.Gain XE_2(k=xe)
      annotation (Placement(transformation(extent={{76,-14},{96,-34}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag Sensor2(K=1, T=Te,
      y_start=Ep0)
      annotation (Placement(transformation(extent={{148,-54},{168,-34}})));
    Electrical.Renewables.WECC.Logic_Blocks.REGCC_Logic_Blocks.aB_conversion
      aB_conversion
      annotation (Placement(transformation(extent={{190,-10},{210,10}})));
    Modelica.Blocks.Sources.RealExpression Angle_Vt(y=angle_vt)
      annotation (Placement(transformation(extent={{228,-50},{208,-30}})));
    Modelica.Blocks.Interfaces.RealOutput V_t "Value of Real output" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={40,130}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-30,150})));
    Modelica.Blocks.Sources.RealExpression Active_Power(y=-(1/CoB)*(p.vr*p.ir + p.vi
          *p.ii))
      annotation (Placement(transformation(extent={{60,90},{80,110}})));
    Modelica.Blocks.Interfaces.RealOutput Pgen "Value of Real output" annotation (
       Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={100,130}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={80,150})));
    Modelica.Blocks.Sources.RealExpression Reactive_Power(y=-(1/CoB)*(p.vi*p.ir -
          p.vr*p.ii))
      annotation (Placement(transformation(extent={{120,90},{140,110}})));
    Modelica.Blocks.Interfaces.RealOutput Qgen "Value of Real output" annotation (
       Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={160,130}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={190,150})));
    Modelica.Blocks.Sources.RealExpression Iq_prime(y=It_conversion.q_output)
      annotation (Placement(transformation(extent={{-118,10},{-98,30}})));
    Modelica.Blocks.Sources.RealExpression Ip_prime(y=It_conversion.d_output)
      annotation (Placement(transformation(extent={{-60,-126},{-40,-106}})));
    parameter Real rrpwr=10
      "Rate at which active current (power) recovers after a fault (Typical Range 1 to 20)."
      annotation (Dialog(tab="Parameters"));
    parameter Real Kip=1
      "Proportional-gain of the inner-current control loop (Typical Range 1 to 10)."
      annotation (Dialog(tab="Parameters"));
    parameter Real Kii=50
      "Integral-gain of the inner-current control loop (Typical Range 20 to 100)."
      annotation (Dialog(tab="Parameters"));
    parameter Boolean RateFlag=true "rrpwr represents active-current ramp rate (False), rrpwr represents active-power

ramp rate (True)."   annotation (Dialog(tab="Controls"));
    parameter OpenIPSL.Types.Time Tfltr=0.02
      "Filter time constant for voltage measurement (Typical Range 0.02 to 0.05)."
      annotation (Dialog(tab="Parameters"));
    parameter Real Kppll=1 "Proportional-gain of the PLL (Typical Range 1 to 10)."
      annotation (Dialog(tab="Parameters"));
    parameter Real Kipll=500
      "Integral-gain of the PLL (Typical Range 500 to 3000)."
                                                             annotation (Dialog(tab="Parameters"));
    parameter Real wmax
      "Upper limit on the PLL (Typical Range: To be determined)."
      annotation (Dialog(tab="Parameters"));
    parameter Real wmin
      "Lower limit on the PLL (Typical Range: To be determined)."
      annotation (Dialog(tab="Parameters"));
    parameter OpenIPSL.Types.Time Te=0.01
      "Emulated delay in converter controls (Typical Range 0 to 0.02)."
      annotation (Dialog(tab="Parameters"));
    parameter Real Iqrmax=999 "Rate at which reactive current recovers after a fault when the initial reactive
power output (Qgeno) of the unit is greater than zero (Typical Range 1 to 999)."
      annotation (Dialog(tab="Parameters"));
    parameter Real Iqrmin=-999 "Rate at which reactive current recovers after a fault when the initial reactive
power output (Qgeno) of the unit is less than zero (Typical Range -1 to -999)."
      annotation (Dialog(tab="Parameters"));
    //OpenIPSL.Types.Angle anglev(start=angle_0) "Bus voltage angle";
    //OpenIPSL.Types.Angle delta(start=angle_0);
    //OpenIPSL.Types.PerUnit VT(start=v_0) "Bus voltage magnitude";
    Modelica.Blocks.Sources.RealExpression initial_IP0(y=id0) annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={100,-210})));
    Modelica.Blocks.Sources.RealExpression initial_IQ0(y=iq0) annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={100,-230})));
    Modelica.Blocks.Interfaces.RealOutput IP0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={20,-250}),  iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={20,-150})));
    Modelica.Blocks.Interfaces.RealOutput IQ0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={80,-250}),  iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={80,-150})));
  protected
    parameter Real CoB=M_b/S_b;

    parameter OpenIPSL.Types.PerUnit p0 = P_0/M_b "Initial active power (machine base)";
    parameter OpenIPSL.Types.PerUnit q0 = Q_0/M_b "Initial reactive power (machine base)";

    parameter OpenIPSL.Types.PerUnit vr0 = v_0*cos(angle_0) "Initial real voltage";
    parameter OpenIPSL.Types.PerUnit vi0 = v_0*sin(angle_0) "Initial imaginary voltage";

    parameter OpenIPSL.Types.PerUnit vd0 = cos(angle_0)*vr0 + sin(angle_0)*vi0 "Initial d-axis voltage";
    parameter OpenIPSL.Types.PerUnit vq0 = -sin(angle_0)*vr0 + cos(angle_0)*vi0 "Initial q-axis voltage";

    parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2) "Initial real current (machine base)";
    parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2) "Initial imaginary current (machine base)";

    parameter OpenIPSL.Types.PerUnit ir1=-CoB*ir0 "Real component of initial armature current, S_b";
    parameter OpenIPSL.Types.PerUnit ii1=-CoB*ii0 "Imaginary component of initial armature current, S_b";

    parameter OpenIPSL.Types.PerUnit id0 = (cos(angle_0)*ir0 + sin(angle_0)*ii0) "Initial d-axis current (machine base)";
    parameter OpenIPSL.Types.PerUnit iq0 = (-sin(angle_0)*ir0 + cos(angle_0)*ii0) "Initial q-axis current (machine base)";

    parameter OpenIPSL.Types.PerUnit Eq0 = vq0 + iq0*re + id0*xe;
    parameter OpenIPSL.Types.PerUnit Ep0 = vd0 + id0*re - iq0*xe;

    OpenIPSL.Types.Angle angle_vt "Bus voltage angle";
  equation
    connect(slewRateLimiter.y, add.u1)
      annotation (Line(points={{-99,50},{-88,50},{-88,52},{-80,52}},
                                                   color={0,0,127}));
    connect(slewRateLimiter.u, Iqcmd) annotation (Line(points={{-122,50},{-160,50}},
                                  color={0,0,127}));
    connect(add.y, gain.u) annotation (Line(points={{-57,46},{-56,46},{-56,66},{-50,
            66}}, color={0,0,127}));
    connect(integrator.u, add.y) annotation (Line(points={{-50,26},{-56,26},{-56,46},
            {-57,46}}, color={0,0,127}));
    connect(gain.y, add1.u1)
      annotation (Line(points={{-27,66},{-20,66},{-20,52}}, color={0,0,127}));
    connect(integrator.y, add1.u2)
      annotation (Line(points={{-27,26},{-20,26},{-20,40}}, color={0,0,127}));
    connect(product1.u1, Ipcmd) annotation (Line(points={{-126,-84},{-140,-84},{-140,
            -110},{-160,-110}},
                              color={0,0,127}));
    connect(product1.y, slewRateLimiter1.u)
      annotation (Line(points={{-103,-90},{-96,-90}},color={0,0,127}));
    connect(RateFlag_switch.y, product1.u2) annotation (Line(points={{-71,-160},{-134,
            -160},{-134,-96},{-126,-96}}, color={0,0,127}));
    connect(simpleLagLim.u, Vt.y)
      annotation (Line(points={{2,-140},{9,-140}},   color={0,0,127}));

    angle_vt = atan2(p.vi, p.vr);
   // delta = anglev;

    p.ir = -(CoB/(re^2 + xe^2))*((aB_conversion.real_output - p.vr)*re + (aB_conversion.imaginary_output - p.vi)*xe);
    p.ii = -(CoB/(re^2 + xe^2))*((aB_conversion.imaginary_output - p.vi)*re - (aB_conversion.real_output - p.vr)*xe);

    //Is.re = p.ir/CoB;
   // Is.im = p.ii/CoB;
    //[Ic.re; Ic.im] = -[cos(delta), sin(delta); -sin(delta), cos(delta)]*[Is.re; Is.im];
    //[Vc.re; Vc.im] = [cos(delta), sin(delta); -sin(delta), cos(delta)]*[p.vr; p.vi];

    connect(Vt_re.y,Vt_conversion. real_input)
      annotation (Line(points={{109,-128},{116,-128}},
                                                     color={0,0,127}));
    connect(Vt_im.y,Vt_conversion. imaginary_input)
      annotation (Line(points={{109,-140},{116,-140}},
                                                     color={0,0,127}));
    connect(integrator1.u,Vt_conversion. q_output) annotation (Line(points={{156,-160},
            {139,-160},{139,-140}},color={0,0,127}));
    connect(KPPLL.y, add3.u1)
      annotation (Line(points={{181,-120},{186,-120},{186,-134}},
                                                               color={0,0,127}));
    connect(integrator1.y, add3.u2) annotation (Line(points={{179,-160},{186,-160},
            {186,-146}},color={0,0,127}));
    connect(add3.y, wmax_wmin.u)
      annotation (Line(points={{209,-140},{216,-140}}, color={0,0,127}));
    connect(integrator2.u, wmax_wmin.y)
      annotation (Line(points={{246,-140},{239,-140}}, color={0,0,127}));
    connect(integrator2.y,Vt_conversion. angle) annotation (Line(points={{269,-140},
            {272,-140},{272,-190},{128,-190},{128,-146}},color={0,0,127}));
    connect(KPPLL.u,Vt_conversion. q_output) annotation (Line(points={{158,-120},{
            152,-120},{152,-160},{139,-160},{139,-140}},
                                                   color={0,0,127}));
    connect(It_conversion.real_input, It_re.y)
      annotation (Line(points={{82,-164},{87,-164}}, color={0,0,127}));
    connect(It_conversion.imaginary_input, It_im.y)
      annotation (Line(points={{82,-176},{87,-176}}, color={0,0,127}));
    connect(It_conversion.angle, Vt_conversion.angle) annotation (Line(points={{70,-182},
            {70,-190},{128,-190},{128,-146}},       color={0,0,127}));
    connect(slewRateLimiter1.y, division.u1) annotation (Line(points={{-73,-90},{-68,
            -90},{-68,-84},{-58,-84}}, color={0,0,127}));
    connect(add2.y, KIP.u) annotation (Line(points={{1,-90},{12,-90},{12,-70},{18,
            -70}}, color={0,0,127}));
    connect(integrator3.u, add2.y) annotation (Line(points={{18,-110},{12,-110},{12,
            -90},{1,-90}},
                      color={0,0,127}));
    connect(KIP.y, add4.u1)
      annotation (Line(points={{41,-70},{48,-70},{48,-84}}, color={0,0,127}));
    connect(integrator3.y, add4.u2)
      annotation (Line(points={{41,-110},{48,-110},{48,-96}},
                                                            color={0,0,127}));
    connect(add4.y, RE_2.u)
      annotation (Line(points={{71,-90},{88,-90}}, color={0,0,127}));
    connect(Vtq0.y, Eq.u2) annotation (Line(points={{57,30},{102,30},{102,38},{114,
            38}},
          color={0,0,127}));
    connect(Id.y, XE_1.u)
      annotation (Line(points={{57,10},{74,10}}, color={0,0,127}));
    connect(XE_1.y, Eq.u3)
      annotation (Line(points={{97,10},{114,10},{114,30}},  color={0,0,127}));
    connect(Eq.y, Sensor1.u)
      annotation (Line(points={{137,38},{144,38}}, color={0,0,127}));
    connect(RE_2.y, Ed.u3)
      annotation (Line(points={{111,-90},{114,-90},{114,-52}}, color={0,0,127}));
    connect(Vtd0.y, Ed.u2)
      annotation (Line(points={{57,-44},{114,-44}}, color={0,0,127}));
    connect(Iq.y, XE_2.u)
      annotation (Line(points={{57,-24},{74,-24}},
                                                 color={0,0,127}));
    connect(XE_2.y, Ed.u1)
      annotation (Line(points={{97,-24},{114,-24},{114,-36}},color={0,0,127}));
    connect(Sensor1.y, aB_conversion.q_input) annotation (Line(points={{167,38},{182,
            38},{182,6},{188,6}},   color={0,0,127}));
    connect(Sensor2.y, aB_conversion.d_input) annotation (Line(points={{169,-44},{
            182,-44},{182,-6},{188,-6}},
                                       color={0,0,127}));
    connect(Angle_Vt.y, aB_conversion.angle)
      annotation (Line(points={{207,-40},{200,-40},{200,-12}},
                                                             color={0,0,127}));
    connect(Terminal_Voltage.y, V_t)
      annotation (Line(points={{21,100},{40,100},{40,130}}, color={0,0,127}));
    connect(Active_Power.y, Pgen)
      annotation (Line(points={{81,100},{100,100},{100,130}}, color={0,0,127}));
    connect(Reactive_Power.y, Qgen)
      annotation (Line(points={{141,100},{160,100},{160,130}}, color={0,0,127}));
    connect(Iq_prime.y, add.u2) annotation (Line(points={{-97,20},{-88,20},{-88,40},
            {-80,40}}, color={0,0,127}));
    connect(add1.y, RE_1.u)
      annotation (Line(points={{3,46},{74,46}}, color={0,0,127}));
    connect(Ed.y, Sensor2.u)
      annotation (Line(points={{137,-44},{146,-44}}, color={0,0,127}));
    connect(simpleLagLim.y, RateFlag_switch.u1) annotation (Line(points={{-21,-140},
            {-34,-140},{-34,-152},{-48,-152}}, color={0,0,127}));
    connect(RATEFLAG.y, RateFlag_switch.u2) annotation (Line(points={{9,-170},{0,-170},
            {0,-160},{-48,-160}},        color={255,0,255}));
    connect(Constant.y, RateFlag_switch.u3) annotation (Line(points={{-31,-180},{-38,
            -180},{-38,-168},{-48,-168}}, color={0,0,127}));
    connect(division.u2, product1.u2) annotation (Line(points={{-58,-96},{-72,-96},
            {-72,-110},{-82,-110},{-82,-160},{-134,-160},{-134,-96},{-126,-96}},
                                                           color={0,0,127}));
    connect(division.y, add2.u1) annotation (Line(points={{-35,-90},{-32,-90},{-32,
            -84},{-22,-84}}, color={0,0,127}));
    connect(Ip_prime.y, add2.u2) annotation (Line(points={{-39,-116},{-24,-116},{-24,
            -96},{-22,-96}}, color={0,0,127}));
    connect(RE_1.y, Eq.u1)
      annotation (Line(points={{97,46},{114,46}},  color={0,0,127}));
    connect(initial_IQ0.y,IQ0)
      annotation (Line(points={{89,-230},{80,-230},{80,-250}}, color={0,0,127}));
    connect(initial_IP0.y,IP0)  annotation (Line(points={{89,-210},{20,-210},{20,-250}},
                                color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-240},
              {280,120}}), graphics={
          Rectangle(extent={{-140,140},{280,-200}}, lineColor={28,108,200}),
          Text(
            extent={{-10,12},{226,-76}},
            lineColor={238,46,47},
            textString="REGCC",
            textStyle={TextStyle.Bold}),
          Text(
            extent={{-132,100},{-32,22}},
            lineColor={28,108,200},
            textString="Iqcmd"),
          Text(
            extent={{-130,-80},{-30,-158}},
            lineColor={28,108,200},
            textString="Ipcmd"),
          Text(
            extent={{-40,140},{-14,100}},
            lineColor={28,108,200},
            textString="Vt"),
          Text(
            extent={{56,140},{116,100}},
            lineColor={28,108,200},
            textString="Pgen"),
          Text(
            extent={{162,140},{222,100}},
            lineColor={28,108,200},
            textString="Qgen")}),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-240},{280,120}}),
          graphics={
          Text(
            extent={{4,46},{14,28}},
            lineColor={28,108,200},
            textString="S0",
            textStyle={TextStyle.Bold}),
          Text(
            extent={{-92,66},{-82,48}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S3"),
          Text(
            extent={{-40,-126},{-30,-144}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S2"),
          Text(
            extent={{-72,-70},{-62,-88}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S4"),
          Text(
            extent={{74,-76},{84,-94}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S1"),
          Text(
            extent={{170,52},{180,34}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S5"),
          Text(
            extent={{170,-44},{180,-62}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S6"),
          Text(
            extent={{204,-146},{214,-164}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S7"),
          Text(
            extent={{260,-156},{270,-174}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="S8"),
          Text(
            extent={{142,-150},{152,-158}},
            lineColor={28,108,200},
            textString="Vq"),
          Text(
            extent={{132,-172},{156,-188}},
            lineColor={28,108,200},
            textString="angle"),
          Rectangle(
            extent={{-20,114},{180,80}},
            lineColor={238,46,47},
            pattern=LinePattern.Dash,
            lineThickness=0.5),
          Text(
            extent={{40,98},{176,72}},
            lineColor={238,46,47},
            pattern=LinePattern.Dash,
            lineThickness=0.5,
            textStyle={TextStyle.Bold},
            textString="Electrical Control Loop output connectors"),
          Text(
            extent={{218,16},{248,-14}},
            lineColor={28,108,200},
            textString="V_tilda",
            textStyle={TextStyle.Bold}),
          Rectangle(
            extent={{84,-198},{186,-240}},
            lineColor={0,140,72},
            pattern=LinePattern.Dash,
            lineThickness=0.5),
          Text(
            extent={{114,-210},{176,-232}},
            lineColor={0,140,72},
            pattern=LinePattern.Dash,
            lineThickness=0.5,
            textStyle={TextStyle.Bold},
            textString="Initial current
output connectors")}));
  end REGCC;

  model REECB
    parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "PV base power.";
    parameter OpenIPSL.Types.ActivePower P_0 = 100e6 "PV initial active power.";
    parameter OpenIPSL.Types.ReactivePower Q_0 = 0.5795379e6 "PV initial reactive power.";
    parameter OpenIPSL.Types.PerUnit v_0 = 1 "Initial Terminal Voltage.";
    parameter OpenIPSL.Types.Angle angle_0(displayUnit = "deg");
    Modelica.Blocks.Interfaces.RealInput Pe
      annotation (Placement(transformation(extent={{-140,130},{-100,170}}),
          iconTransformation(extent={{-140,130},{-100,170}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(K=1, T=Tp,
      y_start=p0)
      annotation (Placement(transformation(extent={{-62,140},{-42,160}})));
    parameter OpenIPSL.Types.Time Tp=0.05
      "Tp (s), Filter time constant for electrical power."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real pfaref= P_0/sqrt(P_0^2 + Q_0^2) "Power Factor of choice."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Math.Tan tan
      annotation (Placement(transformation(extent={{-62,112},{-42,132}})));
    Modelica.Blocks.Math.Product product
      annotation (Placement(transformation(extent={{-32,134},{-12,154}})));
    Modelica.Blocks.Logical.Switch PfFlag
      "Constant Q (False) or PF (True) local control."
      annotation (Placement(transformation(extent={{14,126},{34,146}})));
    Modelica.Blocks.Interfaces.RealInput Qext
      "Connector of second Real input signal"
      annotation (Placement(transformation(extent={{-140,30},{-100,70}}),
          iconTransformation(extent={{-140,30},{-100,70}})));
    Modelica.Blocks.Sources.BooleanConstant PfFlag_logic(k=pfflag)
      annotation (Placement(transformation(extent={{-20,100},{0,120}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Qmax, uMin=Qmin)
      annotation (Placement(transformation(extent={{54,126},{74,146}})));
    parameter Real Qmax=0.4360
      "Maximum reactive power when Vflag = 1 (pu on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Qmin=-0.4360
      "Minimum reactive power when Vflag = 1 (pu on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real lvpnt0=0.4
      "Low voltage active current management breakpoint (pu)." annotation (Dialog(
          tab="Input Parameters", group="Input Parameter from REGCAU1"));
    parameter Real lvpnt1=0.8
      "Low voltage active current management breakpoint (pu)." annotation (Dialog(
          tab="Input Parameters", group="Input Parameter from REGCAU1"));
    Modelica.Blocks.Math.Add add(k2=-1)
      annotation (Placement(transformation(extent={{84,120},{104,140}})));
    Modelica.Blocks.Interfaces.RealInput Qgen "Connector of Real input signal 2"
      annotation (Placement(transformation(extent={{-140,80},{-100,120}}),
          iconTransformation(extent={{-140,80},{-100,120}})));
    Modelica.Blocks.Continuous.Integrator integrator(k=Kqi,
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=v_0)
      annotation (Placement(transformation(extent={{118,100},{138,120}})));
    Modelica.Blocks.Math.Gain gain(k=Kqp)
      annotation (Placement(transformation(extent={{118,140},{138,160}})));
    Modelica.Blocks.Math.Add add1
      annotation (Placement(transformation(extent={{146,120},{166,140}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Vmax, uMin=Vmin)
      annotation (Placement(transformation(extent={{174,120},{194,140}})));
    Modelica.Blocks.Logical.Switch VFlag
      "Constant Q (False) or PF (True) local control."
      annotation (Placement(transformation(extent={{206,112},{226,132}})));
    Modelica.Blocks.Sources.BooleanConstant Vflag_logic(k=vflag)
      annotation (Placement(transformation(extent={{164,84},{184,104}})));
    parameter Real Kqp=0 "Local Q regulator proportional gain (pu/pu)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Kqi=0.1 "Local Q regulator integral gain (pu/pu-s)."
      annotation (Dialog(tab="Input Parameters"));
      parameter Real Vmax = 1.1 "Maximum voltage at inverter terminal bus (pu)."
      annotation (Dialog(tab="Input Parameters"));
      parameter Real Vmin = 0.9 "Minimum voltage at inverter terminal bus (pu)."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=Vmax, uMin=Vmin)
      annotation (Placement(transformation(extent={{236,112},{256,132}})));
    Modelica.Blocks.Math.Add add2(k2=-1)
      annotation (Placement(transformation(extent={{266,106},{286,126}})));
    Modelica.Blocks.Interfaces.RealInput Vt
      annotation (Placement(transformation(extent={{-140,180},{-100,220}}),
          iconTransformation(extent={{-140,180},{-100,220}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag1(
      K=1,
      T=Trv,
      y_start=v_0)
      annotation (Placement(transformation(extent={{-66,190},{-46,210}})));
    Modelica.Blocks.Math.Division division
      annotation (Placement(transformation(extent={{262,40},{282,60}})));
    Modelica.Blocks.Math.Gain gain1(k=Kvp)
      annotation (Placement(transformation(extent={{302,128},{322,148}})));
    Modelica.Blocks.Continuous.Integrator integrator1(
      k=Kvi,                                          initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=-Iq0 - (-v_0 + Vref0)*Kqv)
      annotation (Placement(transformation(extent={{304,88},{324,108}})));
    parameter Real Kvp=0 "Local voltage regulator proportional gain (pu/pu)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Kvi=40 "Local voltage regulator integral gain (pu/pu-s)."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Math.Add add3
      annotation (Placement(transformation(extent={{338,108},{358,128}})));
    parameter Real Iqmax=1 "Upper limits of input signals"
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Iqmin=-1 "Lower limits of input signals"
      annotation (Dialog(tab="Input Parameters"));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag2(
      K=1,
      T=Tiq,
      y_start=-Iq0 - (-v_0 + Vref0)*Kqv)
      annotation (Placement(transformation(extent={{292,40},{312,60}})));
    parameter OpenIPSL.Types.Time Tiq=0.02
      "Tiq (s), Time constant on delay s4."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Logical.Switch QFlag
      "Constant Q (False) or PF (True) local control."
      annotation (Placement(transformation(extent={{408,100},{428,120}})));
    Modelica.Blocks.Sources.BooleanConstant QFlag_logic(k=qflag)
      annotation (Placement(transformation(extent={{338,60},{358,80}})));
    Modelica.Blocks.Math.Add add4(k1=-1, k2=+1)
      annotation (Placement(transformation(extent={{28,184},{48,204}})));
    parameter Real Vref0=v_0 "Vref0 (pu), User defined reference (if 0, model initializes it to initial
terminal voltage)."
                  annotation (Dialog(tab="Input Parameters"));
    parameter Real Khv=0.7 "High voltage clamp logic acceleration factor."
      annotation (Dialog(tab="Input Parameters", group="Input Parameter from REGCAU1"));
    parameter Real dbd1=-0.05
      "dbd1 (pu), Voltage error dead band lower threshold (<=0)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real dbd2=0.05
      "dbd2 (pu), Voltage error dead band upper threshold (>=0)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Kqv=0 "Kqv (pu), Reactive current injection gain during over and
undervoltage conditions."
                        annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Nonlinear.Limiter limiter4(uMax=Iqh1, uMin=Iql1)
      annotation (Placement(transformation(extent={{284,184},{304,204}})));
    Modelica.Blocks.Math.Add add5(k2=+1)
      annotation (Placement(transformation(extent={{434,150},{454,170}})));
    parameter Real Iqh1=1.05
      "Iqh1 (pu), Upper limit on reactive current injection Iqinj."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Iql1=-1.05
      " Iql1 (pu), Lower limit on reactive current injection Iqinj."
      annotation (Dialog(tab="Input Parameters"));
    parameter OpenIPSL.Types.Time Trv=0
      "Trv (s), Voltage filter time constant"
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Volim=1.2 "Voltage limit for high voltage clamp logic (pu)"
      annotation (Dialog(tab="Input Parameters", group="Input Parameter from REGCAU1"));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter
      annotation (Placement(transformation(extent={{472,160},{492,180}})));
    Logic_Blocks.REECB_Logic_Blocks.Current_Limit_Logic
      current_Limit_Logic(Q_0=-0.398469, angle_0=0.17582045458863)
      annotation (Placement(transformation(extent={{484,84},{524,124}})));
    Modelica.Blocks.Interfaces.RealOutput Iqcmd(start=Iq0)
      "Connector of Real output signal"
      annotation (Placement(transformation(extent={{540,160},{560,180}}),
          iconTransformation(extent={{540,160},{560,180}})));
    Modelica.Blocks.Sources.BooleanConstant Pqflag_logic(k=pqflag)
      annotation (Placement(transformation(extent={{442,78},{462,98}})));
    parameter Real Imax=1.82 "Maximum apparent current (pu on mbase)"
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter1
      annotation (Placement(transformation(extent={{472,20},{492,40}})));
    Modelica.Blocks.Interfaces.RealOutput Ipcmd(start=Ip0)
      "Connector of Real output signal"
      annotation (Placement(transformation(extent={{540,20},{560,40}}),
          iconTransformation(extent={{540,20},{560,40}})));
    Modelica.Blocks.Interfaces.RealInput Pref annotation (Placement(
          transformation(extent={{-140,-20},{-100,20}}),
          iconTransformation(extent={{-140,-20},{-100,20}})));
    parameter OpenIPSL.Types.Time Tpord=0.02
      "Inverter power order lag time constant (s)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Pmax=1 "Maximum active power (pu on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Pmin=0 "Minimum active power (pu on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Math.Division division1
      annotation (Placement(transformation(extent={{80,-2},{100,18}})));
    Modelica.Blocks.Sources.RealExpression PFAREF(y=pfangle)
      annotation (Placement(transformation(extent={{-40,90},{-60,110}})));
    parameter Boolean pfflag=false "Constant Q (False) or PF (True) local control"
      annotation (Dialog(tab="Controls"));
    parameter Boolean vflag=false "Local Q (False) or voltage control (True)" annotation (Dialog(tab="Controls"));
    parameter Boolean qflag=false "Bypass (False) or engage (True) inner voltage regulator loop" annotation (Dialog(tab="Controls"));
    parameter Boolean pqflag=false "Priority to reactive current (False) or active current (True)" annotation (Dialog(tab="Controls"));
    parameter Real dPmax=99 "Active power up-ramp limit (pu/s on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real dPmin=-99 "Active power down-ramp limit (pu/s on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Nonlinear.Limiter limiter5(uMax=Modelica.Constants.inf, uMin=0.01)
      annotation (Placement(transformation(extent={{40,-32},{60,-12}})));
    Modelica.Blocks.Nonlinear.Limiter limiter6(uMax=Modelica.Constants.inf, uMin=0.01)
      annotation (Placement(transformation(extent={{208,20},{228,40}})));
    Modelica.Blocks.Math.Add add6(k2=-1)
      annotation (Placement(transformation(extent={{-64,4},{-44,24}})));
    Modelica.Blocks.Nonlinear.Limiter limiter7(uMax=dPmax, uMin=dPmin)
      annotation (Placement(transformation(extent={{-32,4},{-12,24}})));
    Modelica.Blocks.Continuous.Integrator integrator2(k=1/Tpord,
                                                               y_start=Ip0*v_0)
      annotation (Placement(transformation(extent={{0,4},{20,24}})));
    Modelica.Blocks.Nonlinear.Limiter limiter8(uMax=Pmax, uMin=Pmin)
      annotation (Placement(transformation(extent={{40,4},{60,24}})));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter2
      annotation (Placement(transformation(extent={{376,108},{396,128}})));
    Modelica.Blocks.Sources.RealExpression IQMAX(y=current_Limit_Logic.Iqmax)
      annotation (Placement(transformation(extent={{396,140},{376,160}})));
    Modelica.Blocks.Sources.RealExpression IQMIN(y=current_Limit_Logic.Iqmin)
      annotation (Placement(transformation(extent={{396,80},{376,100}})));
    Modelica.Blocks.Interfaces.RealInput IP00 annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-60}),   iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={380,-60})));
    Modelica.Blocks.Interfaces.RealInput IQ00 annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={380,-60}),  iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-60})));
    Modelica.Blocks.Sources.RealExpression VReF0(y=Vref0) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={220,94})));
    Modelica.Blocks.Sources.RealExpression VREF0(y=Vref0)
      annotation (Placement(transformation(extent={{-40,174},{-20,194}})));
    Modelica.Blocks.Sources.RealExpression Vt_filt(y=simpleLag1.y)
      annotation (Placement(transformation(extent={{168,20},{188,40}})));
    Modelica.Blocks.Sources.RealExpression Vt_filt1(y=simpleLag1.y)
      annotation (Placement(transformation(extent={{-50,-32},{-30,-12}})));
    Modelica.Blocks.Sources.RealExpression Vt_filt2(y=simpleLag1.y)
      annotation (Placement(transformation(extent={{286,100},{266,80}})));
    Modelica.Blocks.Sources.RealExpression IMAX(y=Imax)
      annotation (Placement(transformation(extent={{442,104},{462,124}})));
    Modelica.Blocks.Sources.RealExpression IQMAX_(y=current_Limit_Logic.Iqmax)
      annotation (Placement(transformation(extent={{492,190},{472,210}})));
    Modelica.Blocks.Sources.RealExpression IQMIN_(y=current_Limit_Logic.Iqmin)
      annotation (Placement(transformation(extent={{492,132},{472,152}})));
    Modelica.Blocks.Sources.RealExpression IPMAX(y=current_Limit_Logic.Ipmax)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={482,60})));
    Modelica.Blocks.Sources.RealExpression IPMIN(y=current_Limit_Logic.Ipmin)
      annotation (Placement(transformation(extent={{492,-10},{472,10}})));
    Modelica.Blocks.Nonlinear.DeadZone deadZone(uMax=dbd2, uMin=dbd1)
      annotation (Placement(transformation(extent={{98,184},{118,204}})));
    Modelica.Blocks.Math.Gain gain2(k=Kqv)
      annotation (Placement(transformation(extent={{162,184},{182,204}})));
  protected
    parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
    parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
    parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
    parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
    parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.Angle pfangle = if q0 > 0 then acos(pfaref) else -acos(pfaref);
    parameter OpenIPSL.Types.PerUnit Ip0(fixed=false);
    parameter OpenIPSL.Types.PerUnit Iq0(fixed=false);
  initial equation
    Ip0 = IP00;
    Iq0 = IQ00;
  equation
    connect(simpleLag.u, Pe)
      annotation (Line(points={{-64,150},{-120,150}},
                                                    color={0,0,127}));
    connect(simpleLag.y, product.u1) annotation (Line(points={{-41,150},{-34,
            150}},            color={0,0,127}));
    connect(tan.y, product.u2) annotation (Line(points={{-41,122},{-36,122},{
            -36,138},{-34,138}},
                      color={0,0,127}));
    connect(PfFlag.u1, product.y)
      annotation (Line(points={{12,144},{-11,144}},
                                                 color={0,0,127}));
    connect(PfFlag.u3, Qext) annotation (Line(points={{12,128},{8,128},{8,50},{
            -120,50}},
                   color={0,0,127}));
    connect(PfFlag_logic.y, PfFlag.u2) annotation (Line(points={{1,110},{2,110},
            {2,136},{12,136}},    color={255,0,255}));
    connect(PfFlag.y, limiter.u)
      annotation (Line(points={{35,136},{52,136}},
                                                 color={0,0,127}));
    connect(limiter.y, add.u1)
      annotation (Line(points={{75,136},{82,136}}, color={0,0,127}));
    connect(add.u2, Qgen) annotation (Line(points={{82,124},{82,92},{-96,92},{
            -96,100},{-120,100}},
                   color={0,0,127}));
    connect(gain.y, add1.u1)
      annotation (Line(points={{139,150},{144,150},{144,136}},
                                                            color={0,0,127}));
    connect(integrator.y, add1.u2)
      annotation (Line(points={{139,110},{144,110},{144,124}},
                                                          color={0,0,127}));
    connect(limiter1.u, add1.y)
      annotation (Line(points={{172,130},{167,130}},
                                                   color={0,0,127}));
    connect(Vflag_logic.y, VFlag.u2) annotation (Line(points={{185,94},{198,94},
            {198,122},{204,122}},    color={255,0,255}));
    connect(VFlag.y, limiter2.u)
      annotation (Line(points={{227,122},{234,122}},
                                                   color={0,0,127}));
    connect(simpleLag1.u, Vt) annotation (Line(points={{-68,200},{-120,200}},
                                   color={0,0,127}));
    connect(add2.y, gain1.u) annotation (Line(points={{287,116},{292,116},{292,
            138},{300,138}},
                           color={0,0,127}));
    connect(integrator1.u, gain1.u) annotation (Line(points={{302,98},{292,98},
            {292,138},{300,138}},    color={0,0,127}));
    connect(gain1.y, add3.u1) annotation (Line(points={{323,138},{336,138},{336,
            124}},color={0,0,127}));
    connect(integrator1.y, add3.u2) annotation (Line(points={{325,98},{336,98},
            {336,112}},    color={0,0,127}));
    connect(simpleLag2.u, division.y)
      annotation (Line(points={{290,50},{283,50}},   color={0,0,127}));
    connect(QFlag_logic.y, QFlag.u2) annotation (Line(points={{359,70},{400,70},
            {400,110},{406,110}},  color={255,0,255}));
    connect(QFlag.y, add5.u2)
      annotation (Line(points={{429,110},{432,110},{432,154}},
                                                 color={0,0,127}));
    connect(limiter4.y, add5.u1) annotation (Line(points={{305,194},{432,194},{
            432,166}}, color={0,0,127}));
    connect(variableLimiter.y, Iqcmd)
      annotation (Line(points={{493,170},{550,170}},
                                                 color={0,0,127}));
    connect(Pqflag_logic.y, current_Limit_Logic.Pqflag) annotation (Line(
          points={{463,88},{468,88},{468,94},{480,94}},     color={255,0,255}));
    connect(variableLimiter1.y, Ipcmd) annotation (Line(points={{493,30},{550,
            30}},                             color={0,0,127}));
    connect(current_Limit_Logic.Ipcmd, Ipcmd) annotation (Line(points={{518,80},
            {518,30},{550,30}},                              color={0,0,127}));
    connect(variableLimiter1.u, division1.y)
      annotation (Line(points={{470,30},{398,30},{398,8},{101,8}},
                                                       color={0,0,127}));
    connect(limiter2.y, add2.u1) annotation (Line(points={{257,122},{264,122}},
                       color={0,0,127}));
    connect(add5.y, variableLimiter.u) annotation (Line(points={{455,160},{462,
            160},{462,170},{470,170}},
                                   color={0,0,127}));
    connect(limiter5.y, division1.u2) annotation (Line(points={{61,-22},{76,-22},
            {76,2},{78,2}},         color={0,0,127}));
    connect(limiter6.y, division.u2) annotation (Line(points={{229,30},{256,30},
            {256,44},{260,44}},   color={0,0,127}));
    connect(PFAREF.y, tan.u)
      annotation (Line(points={{-61,100},{-68,100},{-68,122},{-64,122}},
                                                   color={0,0,127}));
    connect(limiter7.y, integrator2.u)
      annotation (Line(points={{-11,14},{-2,14}},      color={0,0,127}));
    connect(add6.y, limiter7.u)
      annotation (Line(points={{-43,14},{-34,14}},     color={0,0,127}));
    connect(integrator2.y, limiter8.u)
      annotation (Line(points={{21,14},{38,14}},       color={0,0,127}));
    connect(add6.u2, limiter8.u) annotation (Line(points={{-66,8},{-84,8},{-84,
            -10},{24,-10},{24,14},{38,14}},          color={0,0,127}));
    connect(add6.u1, Pref) annotation (Line(points={{-66,20},{-90,20},{-90,0},{
            -120,0}},     color={0,0,127}));
    connect(variableLimiter2.u, add3.y)
      annotation (Line(points={{374,118},{359,118}},
                                                   color={0,0,127}));
    connect(IQMAX.y, variableLimiter2.limit1) annotation (Line(points={{375,150},
            {368,150},{368,126},{374,126}}, color={0,0,127}));
    connect(IQMIN.y, variableLimiter2.limit2) annotation (Line(points={{375,90},
            {368,90},{368,110},{374,110}}, color={0,0,127}));
    connect(limiter1.y, VFlag.u1)
      annotation (Line(points={{195,130},{204,130}},
                                                   color={0,0,127}));
    connect(variableLimiter2.y, QFlag.u1)
      annotation (Line(points={{397,118},{406,118}},
                                                   color={0,0,127}));
    connect(simpleLag2.y, QFlag.u3)
      annotation (Line(points={{313,50},{406,50},{406,102}},  color={0,0,127}));
    connect(limiter8.y, division1.u1) annotation (Line(points={{61,14},{78,14}},
                                    color={0,0,127}));
    connect(VREF0.y, add4.u2) annotation (Line(points={{-19,184},{-16,184},{-16,
            188},{26,188}},  color={0,0,127}));
    connect(Vt_filt.y, limiter6.u)
      annotation (Line(points={{189,30},{206,30}}, color={0,0,127}));
    connect(Vt_filt1.y, limiter5.u)
      annotation (Line(points={{-29,-22},{38,-22}}, color={0,0,127}));
    connect(Vt_filt2.y, add2.u2) annotation (Line(points={{265,90},{264,90},{
            264,110}},           color={0,0,127}));
    connect(simpleLag1.y, add4.u1)
      annotation (Line(points={{-45,200},{26,200}},  color={0,0,127}));
    connect(gain.u, add.y) annotation (Line(points={{116,150},{110,150},{110,
            130},{105,130}}, color={0,0,127}));
    connect(integrator.u, add.y) annotation (Line(points={{116,110},{110,110},{
            110,130},{105,130}}, color={0,0,127}));
    connect(division.u1, limiter.u) annotation (Line(points={{260,56},{42,56},{
            42,136},{52,136}}, color={0,0,127}));
    connect(IMAX.y, current_Limit_Logic.Imax)
      annotation (Line(points={{463,114},{480,114}}, color={0,0,127}));
    connect(current_Limit_Logic.Iqcmd, Iqcmd) annotation (Line(points={{518,128},
            {518,170},{550,170}}, color={0,0,127}));
    connect(IQMAX_.y, variableLimiter.limit1) annotation (Line(points={{471,200},
            {464,200},{464,178},{470,178}}, color={0,0,127}));
    connect(IQMIN_.y, variableLimiter.limit2) annotation (Line(points={{471,142},
            {464,142},{464,162},{470,162}}, color={0,0,127}));
    connect(IPMAX.y, variableLimiter1.limit1) annotation (Line(points={{471,60},
            {464,60},{464,38},{470,38}}, color={0,0,127}));
    connect(IPMIN.y, variableLimiter1.limit2) annotation (Line(points={{471,0},
            {464,0},{464,22},{470,22}}, color={0,0,127}));
    connect(VReF0.y, VFlag.u3)
      annotation (Line(points={{209,94},{204,94},{204,114}}, color={0,0,127}));
    connect(deadZone.y, gain2.u)
      annotation (Line(points={{119,194},{160,194}}, color={0,0,127}));
    connect(add4.y, deadZone.u)
      annotation (Line(points={{49,194},{96,194}}, color={0,0,127}));
    connect(gain2.y, limiter4.u)
      annotation (Line(points={{183,194},{282,194}}, color={0,0,127}));
                  annotation (Dialog(tab="Input Parameters"),
      Diagram(coordinateSystem(extent={{-100,-40},{540,240}})),
      Icon(coordinateSystem(extent={{-100,-40},{540,240}}),  graphics={
            Rectangle(extent={{-100,240},{540,-40}}, lineColor={28,108,200}),
            Text(
            extent={{86,140},{346,60}},
            textColor={238,46,47},
            textStyle={TextStyle.Bold},
            textString="REECB")}));
  end REECB;

  model REECC
    parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "PV base power.";
    parameter OpenIPSL.Types.ActivePower P_0 = 100e6 "PV initial active power.";
    parameter OpenIPSL.Types.ReactivePower Q_0 = 0.5795379e6 "PV initial reactive power.";
    parameter OpenIPSL.Types.PerUnit v_0 = 1 "Initial Terminal Voltage.";
    parameter OpenIPSL.Types.Angle angle_0(displayUnit = "deg");
      parameter Real pfaref= P_0/sqrt(P_0^2 + Q_0^2) "Power Factor of choice.";
    Modelica.Blocks.Interfaces.RealInput Vt
      annotation (Placement(transformation(extent={{-240,110},{-200,150}}),
          iconTransformation(extent={{-240,110},{-200,150}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(K=1, T=Trv,
      y_start=v_0)
      annotation (Placement(transformation(extent={{-166,130},{-146,150}})));
    parameter OpenIPSL.Types.Time Trv=0.01 "Lag time constant"
      annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Add add(k1=-1)
      annotation (Placement(transformation(extent={{-96,124},{-76,144}})));
    parameter Real Vref0=1 "Constant output value"
        annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=Iqh1, uMin=Iql1)
      annotation (Placement(transformation(extent={{14,124},{34,144}})));
    parameter Real Iqh1 "Upper limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real Iql1 "Lower limits of input signals"
        annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Interfaces.RealOutput Iqcmd(start=Iq0)
      annotation (Placement(transformation(extent={{280,70},{300,90}}),
          iconTransformation(extent={{280,70},{300,90}})));
    Modelica.Blocks.Interfaces.RealOutput Ipcmd(start=Ip0)
      annotation (Placement(transformation(extent={{280,-90},{300,-70}}),
          iconTransformation(extent={{280,-90},{300,-70}})));
    Modelica.Blocks.Math.Add add1
      annotation (Placement(transformation(extent={{160,118},{180,138}})));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter
      annotation (Placement(transformation(extent={{198,118},{218,138}})));
    Modelica.Blocks.Interfaces.RealInput Pelec
      annotation (Placement(transformation(extent={{-240,60},{-200,100}}),
          iconTransformation(extent={{-240,60},{-200,100}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag1(
     K=1,
     T=Tp,
     y_start=p0)
      annotation (Placement(transformation(extent={{-188,80},{-168,100}})));
    parameter OpenIPSL.Types.Time Tp "Lag time constant"
        annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Product product1
      annotation (Placement(transformation(extent={{-158,74},{-138,94}})));
    Modelica.Blocks.Logical.Switch PfFlag
      annotation (Placement(transformation(extent={{-122,66},{-102,86}})));
    Modelica.Blocks.Interfaces.RealInput Qext
      annotation (Placement(transformation(extent={{-240,-50},{-200,-10}}),
          iconTransformation(extent={{-240,-50},{-200,-10}})));
    Modelica.Blocks.Sources.BooleanConstant PfFlag_logic(k=pfflag)
      annotation (Placement(transformation(extent={{-160,30},{-140,50}})));
    parameter Boolean pfflag=true "Constant output value"
      annotation (Dialog(tab="Control"));
    Modelica.Blocks.Math.Tan tan1
      annotation (Placement(transformation(extent={{-188,52},{-168,72}})));
    Modelica.Blocks.Sources.RealExpression PFAREF(y=pfangle)
      annotation (Placement(transformation(extent={{-168,30},{-188,50}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Qmax, uMin=Qmin)
      annotation (Placement(transformation(extent={{-94,66},{-74,86}})));
    parameter Real Qmax "Upper limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real Qmin "Lower limits of input signals"
      annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Add add2(k1=+1, k2=-1)
      annotation (Placement(transformation(extent={{-64,60},{-44,80}})));
    Modelica.Blocks.Interfaces.RealInput Qelec
      annotation (Placement(transformation(extent={{-240,10},{-200,50}}),
          iconTransformation(extent={{-240,10},{-200,50}})));
    Modelica.Blocks.Math.Gain gain(k=Kqp)
      annotation (Placement(transformation(extent={{-36,74},{-16,94}})));
    Modelica.Blocks.Continuous.Integrator integrator(k=Kqi, y_start=v_0)
      annotation (Placement(transformation(extent={{-36,42},{-16,62}})));
    Modelica.Blocks.Math.Add add3(k1=+1, k2=+1)
      annotation (Placement(transformation(extent={{-8,58},{12,78}})));
    Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=Vmax, uMin=Vmin)
      annotation (Placement(transformation(extent={{20,58},{40,78}})));
    Modelica.Blocks.Logical.Switch Vflag
      annotation (Placement(transformation(extent={{54,50},{74,70}})));
    Modelica.Blocks.Sources.BooleanConstant Vflag_logic(k=vflag)
      annotation (Placement(transformation(extent={{20,26},{40,46}})));
    parameter Boolean vflag=true "Constant output value"
    annotation (Dialog(tab="Control"));
    parameter Real Vmax "Upper limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real Vmin "Lower limits of input signals"
      annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Nonlinear.Limiter limiter3(uMax=Vmax, uMin=Vmin)
      annotation (Placement(transformation(extent={{82,50},{102,70}})));
    Modelica.Blocks.Math.Add add4(k1=+1, k2=-1)
      annotation (Placement(transformation(extent={{116,44},{136,64}})));
    Modelica.Blocks.Sources.RealExpression Vt_filt1(y=simpleLag.y)
      annotation (Placement(transformation(extent={{138,14},{118,34}})));
    Modelica.Blocks.Sources.RealExpression Vref00(y=Vref0)
      annotation (Placement(transformation(extent={{106,20},{86,40}})));
    parameter Real Kqp "Gain value multiplied with input signal"
      annotation (Dialog(tab="Parameter"));
    parameter Real Kqi "Integrator gain" annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Gain gain1(k=Kvp)
      annotation (Placement(transformation(extent={{144,62},{164,82}})));
    Modelica.Blocks.Continuous.Integrator integrator1(k=Kvi, y_start=-Iq0 - (-v_0 +
          Vref0)*Kqv)
      annotation (Placement(transformation(extent={{144,28},{164,48}})));
    parameter Real Kvp "Gain value multiplied with input signal"
      annotation (Dialog(tab="Parameter"));
    parameter Real Kvi "Integrator gain" annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Add add5(k1=+1, k2=+1)
      annotation (Placement(transformation(extent={{174,46},{194,66}})));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter1
      annotation (Placement(transformation(extent={{204,46},{224,66}})));
    Modelica.Blocks.Sources.RealExpression IQMIN_(y=CCL.iqmin) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={214,36})));
    Modelica.Blocks.Sources.RealExpression IQMAX_(y=CCL.iqmax) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={214,86})));
    Modelica.Blocks.Logical.Switch QFlag
      annotation (Placement(transformation(extent={{248,38},{268,58}})));
    Modelica.Blocks.Sources.BooleanConstant QFLAG(k=qflag)
      annotation (Placement(transformation(extent={{202,-4},{222,16}})));
    Modelica.Blocks.Sources.RealExpression Vt_filt2(y=simpleLag.y)
      annotation (Placement(transformation(extent={{-170,-28},{-150,-8}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag2(K=1, T=Tiq,
      y_start=-Iq0 - (-v_0 + Vref0)*Kqv)
      annotation (Placement(transformation(extent={{-58,-22},{-38,-2}})));
    parameter OpenIPSL.Types.Time Tiq "Lag time constant"
      annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Division division
      annotation (Placement(transformation(extent={{-90,-22},{-70,-2}})));
    Modelica.Blocks.Nonlinear.Limiter limiter4(uMax=Modelica.Constants.inf, uMin=0.01)
      annotation (Placement(transformation(extent={{-128,-28},{-108,-8}})));
    parameter Boolean qflag=true "Constant output value"
      annotation (Dialog(tab="Control"));
    Modelica.Blocks.Sources.RealExpression IQMAX(y=CCL.iqmax)
      annotation (Placement(transformation(extent={{260,134},{240,154}})));
    Modelica.Blocks.Sources.RealExpression IQMIN(y=CCL.iqmin)
      annotation (Placement(transformation(extent={{260,104},{240,124}})));
    Modelica.Blocks.Tables.CombiTable1Ds VDL1(table=[0.0,0.75; 0.2,0.75; 0.5,0.75;
          1,0.75])
      annotation (Placement(transformation(extent={{18,-42},{38,-22}})));
    Modelica.Blocks.Tables.CombiTable1Ds VDL2(table=[0.2,1.11; 0.5,1.11; 0.75,
          1.11; 1,1.11])
      annotation (Placement(transformation(extent={{18,-62},{38,-42}})));
    Logic_Blocks.REECC_Logic_Blocks.CCL_REECC CCL(Imax=Imax)
      annotation (Placement(transformation(extent={{56,-52},{76,-32}})));
    Modelica.Blocks.Sources.BooleanConstant PQFLAG(k=pqflag)
      annotation (Placement(transformation(extent={{116,-32},{96,-52}})));
    parameter Boolean pqflag=true "Constant output value"
      annotation (Dialog(tab="Control"));
    Modelica.Blocks.Nonlinear.VariableLimiter variableLimiter2
      annotation (Placement(transformation(extent={{208,-94},{228,-74}})));
    Modelica.Blocks.Sources.RealExpression IPMAX(y=CCL.ipmax) annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=180,
          origin={220,-44})));
    Modelica.Blocks.Sources.RealExpression IPMIN(y=CCL.ipmin) annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=180,
          origin={220,-108})));
    Modelica.Blocks.Interfaces.RealInput Pref
      annotation (Placement(transformation(extent={{-240,-150},{-200,-110}}),
          iconTransformation(extent={{-240,-150},{-200,-110}})));

    Modelica.Blocks.Continuous.Integrator integrator2(k=1/Tpord, y_start=Ip0*v_0)
      annotation (Placement(transformation(extent={{-96,-94},{-76,-74}})));
    Modelica.Blocks.Math.Add add6(k2=-1)
      annotation (Placement(transformation(extent={{-160,-94},{-140,-74}})));
    Modelica.Blocks.Nonlinear.Limiter limiter5(uMax=dPmax, uMin=dPmin)
      annotation (Placement(transformation(extent={{-128,-94},{-108,-74}})));
    Modelica.Blocks.Nonlinear.Limiter limiter6(uMax=Pmax, uMin=Pmin)
      annotation (Placement(transformation(extent={{-56,-94},{-36,-74}})));
    Modelica.Blocks.Math.Division division1
      annotation (Placement(transformation(extent={{-24,-100},{-4,-80}})));
    Modelica.Blocks.Nonlinear.Limiter limiter7(uMax=Modelica.Constants.inf, uMin=0.01)
      annotation (Placement(transformation(extent={{-90,-134},{-70,-114}})));
    Modelica.Blocks.Sources.RealExpression Vt_filt3(y=simpleLag.y)
      annotation (Placement(transformation(extent={{-160,-134},{-140,-114}})));
    Modelica.Blocks.Math.Add add7(k1=+1, k2=+1)
      annotation (Placement(transformation(extent={{20,-94},{40,-74}})));
    Modelica.Blocks.Interfaces.RealInput Paux "Connector of Real input signal 1"
      annotation (Placement(transformation(extent={{-240,-100},{-200,-60}}),
          iconTransformation(extent={{-240,-100},{-200,-60}})));
    Modelica.Blocks.Sources.RealExpression PELEC(y=Pelec)
      annotation (Placement(transformation(extent={{-14,-130},{6,-110}})));
    Modelica.Blocks.Continuous.Integrator integrator3(k=1/T, y_start=p0)
      annotation (Placement(transformation(extent={{16,-130},{36,-110}})));
    Modelica.Blocks.Math.Add add8(k1=-1, k2=+1)
      annotation (Placement(transformation(extent={{46,-148},{66,-128}})));
    Modelica.Blocks.Nonlinear.Limiter limiter8(uMax=SOCmax, uMin=SOCmin)
      annotation (Placement(transformation(extent={{76,-148},{96,-128}})));
    Modelica.Blocks.Interfaces.RealInput SOCini
      "Connector of Real input signal 2" annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-220,-144}), iconTransformation(
          extent={{-32,48},{8,8}},
          rotation=90,
          origin={-52,-168})));
    parameter Real SOCmax "Upper limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real SOCmin "Lower limits of input signals"
      annotation (Dialog(tab="Parameter"));
    Logic_Blocks.REECC_Logic_Blocks.SOC_logic sOC_logic(SOCmin=SOCmin,
        SOCmax=SOCmax) annotation (Placement(transformation(extent={{106,-148},
              {126,-128}})));
    parameter Real dPmax "Upper limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real dPmin "Lower limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real Pmax "Upper limits of input signals"
      annotation (Dialog(tab="Parameter"));
    parameter Real Pmin "Lower limits of input signals"
      annotation (Dialog(tab="Parameter"));
      parameter Real T = 999 "T, battery discharge time (s) (>0)."
          annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Math.Product product2
      annotation (Placement(transformation(extent={{204,-60},{184,-40}})));
    Modelica.Blocks.Math.Product product3
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={194,-114})));
    Modelica.Blocks.Interfaces.RealInput IP00 annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={0,-200}),   iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={160,-180})));
    Modelica.Blocks.Interfaces.RealInput IQ00 annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={180,-200}), iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={40,-180})));
          parameter Real Tpord "Power filter time constant"
              annotation (Dialog(tab="Parameter"));
  parameter Real Kqv=0 "Kqv (pu), Reactive current injection gain during over and
undervoltage conditions."
                        annotation (Dialog(tab="Parameter"));
    parameter Real dbd1 "Voltage error dead band lower threshold" annotation (Dialog(tab="Parameter"));
    parameter Real dbd2 "Voltage error dead band upper threshold" annotation (Dialog(tab="Parameter"));
      parameter Real Imax=1.11 "Maximum apparent current (pu on mbase)"
      annotation (Dialog(tab="Parameter"));
    Modelica.Blocks.Sources.RealExpression VREF0(y=Vref0)
      annotation (Placement(transformation(extent={{-140,114},{-120,134}})));
    Modelica.Blocks.Math.Add add9 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={64,24})));
    Modelica.Blocks.Nonlinear.DeadZone deadZone(uMax=dbd2, uMin=dbd1)
      annotation (Placement(transformation(extent={{-62,124},{-42,144}})));
    Modelica.Blocks.Math.Gain gain2(k=Kqv)
      annotation (Placement(transformation(extent={{-24,124},{-4,144}})));
    Modelica.Blocks.Sources.RealExpression QEXT(y=Qext)
      annotation (Placement(transformation(extent={{-102,42},{-122,62}})));
    Modelica.Blocks.Sources.RealExpression QELEC(y=Qelec)
      annotation (Placement(transformation(extent={{-44,36},{-64,56}})));
    Modelica.Blocks.Sources.RealExpression PfFlag_output(y=PfFlag.y)
      annotation (Placement(transformation(extent={{106,28},{86,8}})));
    Modelica.Blocks.Sources.RealExpression IQCMD(y=Iqcmd)
      annotation (Placement(transformation(extent={{116,-34},{96,-14}})));
    Modelica.Blocks.Sources.RealExpression IPCMD(y=Ipcmd)
      annotation (Placement(transformation(extent={{116,-54},{96,-74}})));
    Modelica.Blocks.Sources.RealExpression SOC_ipmax(y=sOC_logic.ipmax_SOC)
      annotation (Placement(transformation(extent={{230,-46},{210,-66}})));
    Modelica.Blocks.Sources.RealExpression SOC_ipmin(y=sOC_logic.ipmin_SOC)
      annotation (Placement(transformation(extent={{230,-110},{210,-130}})));
  protected
    parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
    parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
    parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
    parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
    parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.Angle pfangle = if q0 > 0 then acos(pfaref) else -acos(pfaref);
    parameter OpenIPSL.Types.PerUnit Ip0(fixed=false);
    parameter OpenIPSL.Types.PerUnit Iq0(fixed=false);
  initial equation
    Ip0 = IP00;
    Iq0 = IQ00;
  equation
    connect(simpleLag.u, Vt)
      annotation (Line(points={{-168,140},{-194,140},{-194,130},{-220,130}},
                                                       color={0,0,127}));
    connect(simpleLag.y, add.u1) annotation (Line(points={{-145,140},{-98,140}},
                              color={0,0,127}));
    connect(limiter.y, add1.u1)
      annotation (Line(points={{35,134},{158,134}},  color={0,0,127}));
    connect(add1.y, variableLimiter.u)
      annotation (Line(points={{181,128},{196,128}}, color={0,0,127}));
    connect(variableLimiter.y, Iqcmd)
      annotation (Line(points={{219,128},{276,128},{276,80},{290,80}},
                                                     color={0,0,127}));
    connect(simpleLag1.u, Pelec) annotation (Line(points={{-190,90},{-206,90},{
            -206,80},{-220,80}},
                              color={0,0,127}));
    connect(simpleLag1.y, product1.u1) annotation (Line(points={{-167,90},{-160,
            90}},                 color={0,0,127}));
    connect(product1.y, PfFlag.u1)
      annotation (Line(points={{-137,84},{-124,84}}, color={0,0,127}));
    connect(PfFlag_logic.y, PfFlag.u2) annotation (Line(points={{-139,40},{-130,
            40},{-130,76},{-124,76}},
                                  color={255,0,255}));
    connect(tan1.y, product1.u2) annotation (Line(points={{-167,62},{-160,62},{
            -160,78}},
                  color={0,0,127}));
    connect(PFAREF.y, tan1.u) annotation (Line(points={{-189,40},{-194,40},{
            -194,62},{-190,62}},
                        color={0,0,127}));
    connect(PfFlag.y, limiter1.u)
      annotation (Line(points={{-101,76},{-96,76}},color={0,0,127}));
    connect(limiter1.y, add2.u1) annotation (Line(points={{-73,76},{-66,76}},
                       color={0,0,127}));
    connect(gain.u, add2.y) annotation (Line(points={{-38,84},{-38,70},{-43,70}},
          color={0,0,127}));
    connect(integrator.u, add2.y) annotation (Line(points={{-38,52},{-38,70},{
            -43,70}},color={0,0,127}));
    connect(gain.y, add3.u1)
      annotation (Line(points={{-15,84},{-10,84},{-10,74}},
                                                         color={0,0,127}));
    connect(integrator.y, add3.u2)
      annotation (Line(points={{-15,52},{-10,52},{-10,62}},
                                                         color={0,0,127}));
    connect(add3.y, limiter2.u)
      annotation (Line(points={{13,68},{18,68}}, color={0,0,127}));
    connect(limiter2.y, Vflag.u1)
      annotation (Line(points={{41,68},{52,68}}, color={0,0,127}));
    connect(Vflag.y, limiter3.u)
      annotation (Line(points={{75,60},{80,60}},   color={0,0,127}));
    connect(limiter3.y, add4.u1) annotation (Line(points={{103,60},{114,60}},
                           color={0,0,127}));
    connect(Vt_filt1.y, add4.u2)
      annotation (Line(points={{117,24},{114,24},{114,48}}, color={0,0,127}));
    connect(gain1.u, add4.y) annotation (Line(points={{142,72},{142,54},{137,54}},
                      color={0,0,127}));
    connect(integrator1.u, add4.y) annotation (Line(points={{142,38},{142,54},{
            137,54}},      color={0,0,127}));
    connect(integrator1.y, add5.u2) annotation (Line(points={{165,38},{172,38},
            {172,50}},     color={0,0,127}));
    connect(gain1.y, add5.u1) annotation (Line(points={{165,72},{172,72},{172,
            62}},      color={0,0,127}));
    connect(add5.y, variableLimiter1.u)
      annotation (Line(points={{195,56},{202,56}}, color={0,0,127}));
    connect(IQMIN_.y, variableLimiter1.limit2) annotation (Line(points={{203,36},
            {202,36},{202,48}},         color={0,0,127}));
    connect(IQMAX_.y, variableLimiter1.limit1) annotation (Line(points={{203,86},
            {202,86},{202,64}},         color={0,0,127}));
    connect(variableLimiter1.y, QFlag.u1)
      annotation (Line(points={{225,56},{246,56}}, color={0,0,127}));
    connect(QFLAG.y, QFlag.u2) annotation (Line(points={{223,6},{236,6},{236,48},
            {246,48}}, color={255,0,255}));
    connect(simpleLag2.y, QFlag.u3)
      annotation (Line(points={{-37,-12},{246,-12},{246,40}}, color={0,0,127}));
    connect(Vt_filt2.y, limiter4.u)
      annotation (Line(points={{-149,-18},{-130,-18}},
                                                     color={0,0,127}));
    connect(limiter4.y, division.u2) annotation (Line(points={{-107,-18},{-92,
            -18}},                color={0,0,127}));
    connect(division.y, simpleLag2.u)
      annotation (Line(points={{-69,-12},{-66,-12},{-66,-10},{-64,-10},{-64,-12},
            {-60,-12}},                              color={0,0,127}));
    connect(variableLimiter2.y, Ipcmd)
      annotation (Line(points={{229,-84},{260,-84},{260,-80},{290,-80}},
                                                       color={0,0,127}));
    connect(add6.y, limiter5.u)
      annotation (Line(points={{-139,-84},{-130,-84}}, color={0,0,127}));
    connect(limiter5.y, integrator2.u)
      annotation (Line(points={{-107,-84},{-98,-84}},color={0,0,127}));
    connect(integrator2.y, add6.u2) annotation (Line(points={{-75,-84},{-66,-84},
            {-66,-106},{-170,-106},{-170,-90},{-162,-90}},color={0,0,127}));
    connect(Pref, add6.u1) annotation (Line(points={{-220,-130},{-180,-130},{
            -180,-78},{-162,-78}},
                         color={0,0,127}));
    connect(limiter6.u, add6.u2) annotation (Line(points={{-58,-84},{-66,-84},{
            -66,-106},{-170,-106},{-170,-90},{-162,-90}},
                                                      color={0,0,127}));
    connect(limiter6.y, division1.u1)
      annotation (Line(points={{-35,-84},{-26,-84}},
                                                   color={0,0,127}));
    connect(limiter7.y, division1.u2) annotation (Line(points={{-69,-124},{-30,
            -124},{-30,-96},{-26,-96}},
                                color={0,0,127}));
    connect(Vt_filt3.y, limiter7.u)
      annotation (Line(points={{-139,-124},{-92,-124}},color={0,0,127}));
    connect(division1.y, add7.u2)
      annotation (Line(points={{-3,-90},{18,-90}}, color={0,0,127}));
    connect(add7.y, variableLimiter2.u) annotation (Line(points={{41,-84},{206,
            -84}},                                        color={0,0,127}));
    connect(PELEC.y, integrator3.u)
      annotation (Line(points={{7,-120},{14,-120}},    color={0,0,127}));
    connect(integrator3.y, add8.u1) annotation (Line(points={{37,-120},{38,-120},
            {38,-132},{44,-132}},     color={0,0,127}));
    connect(add8.y, limiter8.u)
      annotation (Line(points={{67,-138},{74,-138}},   color={0,0,127}));
    connect(limiter8.y, sOC_logic.SOC)
      annotation (Line(points={{97,-138},{104,-138}}, color={0,0,127}));
    connect(product2.y, variableLimiter2.limit1) annotation (Line(points={{183,-50},
            {160,-50},{160,-76},{206,-76}},
                                    color={0,0,127}));
    connect(VREF0.y, add.u2) annotation (Line(points={{-119,124},{-108,124},{-108,
            128},{-98,128}}, color={0,0,127}));
    connect(Vflag_logic.y, Vflag.u2) annotation (Line(points={{41,36},{46,36},{
            46,60},{52,60}},
                          color={255,0,255}));
    connect(VDL2.y[1], CCL.VDL2_out) annotation (Line(points={{39,-52},{54,-52},
            {54,-48},{55,-48}},   color={0,0,127}));
    connect(VDL1.y[1], CCL.VDL1_out) annotation (Line(points={{39,-32},{54,-32},
            {54,-36},{55,-36}},
                           color={0,0,127}));
    connect(VDL2.u, limiter4.u) annotation (Line(points={{16,-52},{-140,-52},{
            -140,-18},{-130,-18}},             color={0,0,127}));
    connect(VDL1.u, limiter4.u) annotation (Line(points={{16,-32},{6,-32},{6,
            -52},{-140,-52},{-140,-18},{-130,-18}},
                                          color={0,0,127}));
    connect(IPMAX.y, product2.u1)
      annotation (Line(points={{209,-44},{206,-44}},           color={0,0,127}));
    connect(product3.u2, IPMIN.y)
      annotation (Line(points={{206,-108},{209,-108}}, color={0,0,127}));
    connect(integrator2.y, limiter6.u)
      annotation (Line(points={{-75,-84},{-58,-84}}, color={0,0,127}));
    connect(SOCini, add8.u2) annotation (Line(points={{-220,-144},{44,-144}},
          color={0,0,127}));
    connect(variableLimiter2.limit2, product3.y) annotation (Line(points={{206,-92},
            {160,-92},{160,-114},{183,-114}},  color={0,0,127}));
    connect(add9.u2, Vref00.y)
      annotation (Line(points={{76,30},{85,30}},   color={0,0,127}));
    connect(add9.y, Vflag.u3)
      annotation (Line(points={{53,24},{52,24},{52,52}}, color={0,0,127}));
    connect(deadZone.y, gain2.u)
      annotation (Line(points={{-41,134},{-26,134}}, color={0,0,127}));
    connect(add.y, deadZone.u)
      annotation (Line(points={{-75,134},{-64,134}}, color={0,0,127}));
    connect(gain2.y, limiter.u)
      annotation (Line(points={{-3,134},{12,134}}, color={0,0,127}));
    connect(QEXT.y, PfFlag.u3) annotation (Line(points={{-123,52},{-126,52},{
            -126,68},{-124,68}}, color={0,0,127}));
    connect(QELEC.y, add2.u2)
      annotation (Line(points={{-65,46},{-66,46},{-66,64}}, color={0,0,127}));
    connect(add9.u1, PfFlag_output.y)
      annotation (Line(points={{76,18},{85,18}}, color={0,0,127}));
    connect(division.u1, PfFlag_output.y) annotation (Line(points={{-92,-6},{
            -100,-6},{-100,8},{80,8},{80,18},{85,18}}, color={0,0,127}));
    connect(QFlag.y, add1.u2) annotation (Line(points={{269,48},{272,48},{272,
            100},{150,100},{150,122},{158,122}}, color={0,0,127}));
    connect(IQMAX.y, variableLimiter.limit1) annotation (Line(points={{239,144},
            {188,144},{188,136},{196,136}}, color={0,0,127}));
    connect(IQMIN.y, variableLimiter.limit2) annotation (Line(points={{239,114},
            {188,114},{188,120},{196,120}}, color={0,0,127}));
    connect(IQCMD.y, CCL.iqcmd) annotation (Line(points={{95,-24},{
            76.9999,-24},{76.9999,-36}},
                            color={0,0,127}));
    connect(IPCMD.y, CCL.ipcmd)
      annotation (Line(points={{95,-64},{77,-64},{77,-48}}, color={0,0,127}));
    connect(PQFLAG.y, CCL.pqflag)
      annotation (Line(points={{95,-42},{77,-42}}, color={255,0,255}));
    connect(product2.u2, SOC_ipmax.y)
      annotation (Line(points={{206,-56},{209,-56}}, color={0,0,127}));
    connect(product3.u1, SOC_ipmin.y)
      annotation (Line(points={{206,-120},{209,-120}}, color={0,0,127}));
    connect(add7.u1, Paux) annotation (Line(points={{18,-78},{0,-78},{0,
            -58},{-188,-58},{-188,-80},{-220,-80}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
              -160},{280,160}}),
                           graphics={
          Text(
            extent={{-120,70},{216,-56}},
            lineColor={238,46,47},
            textString="%name",
            textStyle={TextStyle.Bold}),
          Rectangle(extent={{-200,160},{280,-160}}, lineColor={28,108,200})}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-200,-160},{280,160}})));
  end REECC;

  model REPCA

    import SIunits =
                Modelica.Units.SI;
    import Modelica.Units.Conversions.*;
    parameter OpenIPSL.Types.ApparentPower S_b = 100e6 "System base power.";
    parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "PV base power.";
    parameter OpenIPSL.Types.ActivePower P_0 = 1.5e6 "PV initial active power.";
    parameter OpenIPSL.Types.ReactivePower Q_0 = -5.6658e6 "PV initial reactive power.";
    parameter OpenIPSL.Types.PerUnit v_0 = 0.9999999 "Initial Terminal Voltage.";
    parameter OpenIPSL.Types.Angle angle_0(displayUnit="deg") = 0.02574992 "Initial Terminal Bus Angle";

    Modelica.Blocks.Interfaces.RealInput Freq annotation (Placement(
          transformation(extent={{-140,-100},{-100,-60}}),iconTransformation(
            extent={{-140,-100},{-100,-60}})));
    Modelica.Blocks.Math.Add add(k1=-1)
      annotation (Placement(transformation(extent={{-80,-184},{-60,-164}})));
    Modelica.Blocks.Math.Gain DDN(k=Ddn)
      annotation (Placement(transformation(extent={{-20,-164},{0,-144}})));
    Modelica.Blocks.Math.Gain DUP(k=Dup)
      annotation (Placement(transformation(extent={{-20,-200},{0,-180}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=0, uMin=-Modelica.Constants.inf)
      annotation (Placement(transformation(extent={{10,-164},{30,-144}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Modelica.Constants.inf,
        uMin=0)
      annotation (Placement(transformation(extent={{10,-200},{30,-180}})));
    Modelica.Blocks.Math.Add add1(k1=+1)
      annotation (Placement(transformation(extent={{40,-184},{60,-164}})));
    Modelica.Blocks.Math.Add3 add3_1(k2=-1)
      annotation (Placement(transformation(extent={{72,-176},{92,-156}})));
    Modelica.Blocks.Interfaces.RealInput Plant_pref annotation (Placement(
          transformation(extent={{-140,0},{-100,40}}),  iconTransformation(
            extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-120,20})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(K=1, T=Tp,
      y_start=p0)
      annotation (Placement(transformation(extent={{-20,-136},{0,-116}})));
    Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=femax, uMin=femin)
      annotation (Placement(transformation(extent={{104,-176},{124,-156}})));
    Modelica.Blocks.Math.Gain KPG(k=Kpg)
      annotation (Placement(transformation(extent={{132,-160},{152,-140}})));
    Modelica.Blocks.Continuous.Integrator KIG(k=Kig, y_start=p0)
      annotation (Placement(transformation(extent={{132,-192},{152,-172}})));
    Modelica.Blocks.Math.Add add2(k1=+1)
      annotation (Placement(transformation(extent={{162,-176},{182,-156}})));
    Modelica.Blocks.Nonlinear.Limiter limiter3(uMax=Pmax, uMin=Pmin)
      annotation (Placement(transformation(extent={{190,-176},{210,-156}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag1(K=1, T=Tg,
      y_start=p0)
      annotation (Placement(transformation(extent={{226,-158},{246,-138}})));
    Modelica.Blocks.Logical.Switch FREQ_FLAG
      annotation (Placement(transformation(extent={{262,-166},{282,-146}})));
    Modelica.Blocks.Sources.BooleanConstant FREQ_FLAG_logic(k=fflag)
      annotation (Placement(transformation(extent={{190,-208},{210,-188}})));
    Modelica.Blocks.Sources.Constant PREF(k=p0)
      annotation (Placement(transformation(extent={{230,-208},{250,-188}})));
    Modelica.Blocks.Interfaces.RealOutput Pref(start=p0) "Connector of Real output signal"
      annotation (Placement(transformation(extent={{320,-140},{340,-120}})));
    OpenIPSL.Interfaces.PwPin REGULATE
      annotation (Placement(transformation(extent={{100,180},{120,200}})));
    parameter Real fdbd1=0
      "Deadband for frequency control, lower threshold (<=0)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real fdbd2=0
      "Deadband for frequency control, upper threshold (>=0)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Ddn=20
      "Ddn, reciprocal of droop for over-frequency conditions (pu)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Dup=0 "Up regulation droop (pu power/pu freq on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter OpenIPSL.Types.Time Tp=0.25
      "Active power filter time constant (s)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real femax=999
      "Maximum power error in droop regulator (pu on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real femin=-999
      "Minimum power error in droop regulator (pu on mbase)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Kpg=0.1 "Droop regulator proportional gain."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Kig=0.05 "Droop regulator integral gain."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Pmax=999 "Pmax, upper limit on power reference (pu)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Pmin=-999 "Pmin, lower limit on power reference (pu)."
      annotation (Dialog(tab="Input Parameters"));
    parameter OpenIPSL.Types.Time Tg=0.1
      "Plant controller P output lag time constant (s)."
      annotation (Dialog(tab="Input Parameters"));
    parameter Modelica.Units.SI.Resistance Rc=0
      "Line drop compensation resistance."
      annotation (Dialog(tab="Input Parameters"));
    parameter Modelica.Units.SI.Reactance Xc=0
      "Line drop compensation reactance."
      annotation (Dialog(tab="Input Parameters"));
    OpenIPSL.Interfaces.PwPin BRANCH_n
      annotation (Placement(transformation(extent={{200,180},{220,200}})));
    Modelica.Blocks.Sources.RealExpression Vreg(y=sqrt(REGULATE.vr^2 + REGULATE.vi
          ^2))
      annotation (Placement(transformation(extent={{-60,82},{-40,102}})));
    Modelica.Blocks.Sources.RealExpression Qbranch(y=-(1/CoB)*(BRANCH_p.vi*
          BRANCH_p.ir - BRANCH_p.vr*BRANCH_p.ii))
      annotation (Placement(transformation(extent={{-60,64},{-40,84}})));
    Modelica.Blocks.Sources.RealExpression Pbranch(y=-(1/CoB)*(BRANCH_p.vr*
          BRANCH_p.ir + BRANCH_p.vi*BRANCH_p.ii))
      annotation (Placement(transformation(extent={{-54,-136},{-34,-116}})));
    Modelica.Blocks.Sources.RealExpression Voltage_diff(y=sqrt((REGULATE.vr - Rc*
          BRANCH_n.ir/CoB + Xc*BRANCH_n.ii/CoB)^2 + (REGULATE.vi - Xc*BRANCH_n.ir/
          CoB - Rc*BRANCH_n.ii/CoB)^2))
      annotation (Placement(transformation(extent={{-60,132},{-40,152}})));
    Modelica.Blocks.Logical.Switch VCFLAG
      annotation (Placement(transformation(extent={{38,78},{58,98}})));
    Modelica.Blocks.Math.Add add3(k1=+1)
      annotation (Placement(transformation(extent={{8,70},{28,90}})));
    Modelica.Blocks.Math.Gain KC(k=Kc)
      annotation (Placement(transformation(extent={{-20,64},{0,84}})));
    parameter Real Kc=0.02 "Reactive Current Compensation gain."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Sources.BooleanConstant VCFLAG_logic(k=vcflag)
      annotation (Placement(transformation(extent={{-60,108},{-40,128}})));
    parameter Boolean vcflag=true
      "Reactive dropp (False) or line drop compensation (True)."
      annotation (Dialog(tab="Controls"));
      parameter Boolean refflag=true
      "Plant-level reactive power (False) or voltage control (True)."
      annotation (Dialog(tab="Controls"));
       parameter Boolean fflag=true
      "Governor response disable (False) or enable (True)."
      annotation (Dialog(tab="Controls"));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag2(K=1, T=Tfltr,
      y_start=q0)
      annotation (Placement(transformation(extent={{-22,26},{-2,46}})));
    parameter OpenIPSL.Types.Time Tfltr=0.02
      "Voltage and reactive power filter time constant."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Math.Add add4(k1=-1)
      annotation (Placement(transformation(extent={{12,20},{32,40}})));
    Modelica.Blocks.Logical.Switch REFFLAG
      annotation (Placement(transformation(extent={{124,60},{144,80}})));
    Modelica.Blocks.Sources.BooleanConstant REFFLAG_logic(k=refflag)
      annotation (Placement(transformation(extent={{66,48},{86,68}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag3(K=1, T=Tfltr,
      y_start=Vref)
      annotation (Placement(transformation(extent={{66,78},{86,98}})));
    Modelica.Blocks.Math.Add add5(k1=+1, k2=-1)
      annotation (Placement(transformation(extent={{96,84},{116,104}})));
    parameter Real dbd1 = 0 "Lower threshold for reactive power control deadband."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real dbd2 = 0 "Upper threshold for reactive power control deadband."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Nonlinear.Limiter limiter4(uMax=emax, uMin=emin)
      annotation (Placement(transformation(extent={{180,60},{200,80}})));
    parameter Real emax = 0.1 "Upper limit on deadband output."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real emin = -0.1 "Lower limit on deadband output."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Math.Gain KP(k=Kp)
      annotation (Placement(transformation(extent={{210,80},{230,100}})));
    parameter Real Kp = 18 "Volt/VAR regulator proportional gain"
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Continuous.Integrator KI(k=Ki, y_start=q0)
      annotation (Placement(transformation(extent={{210,40},{230,60}})));
    parameter Real Ki=5 "Reactive Power PI control integral gain."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Math.Add add6(k1=+1)
      annotation (Placement(transformation(extent={{236,60},{256,80}})));
    Modelica.Blocks.Nonlinear.Limiter limiter5(uMax=Qmax, uMin=Qmin)
      annotation (Placement(transformation(extent={{264,60},{284,80}})));
    parameter Real Qmax = 0.436 "Maximum plant reactive power command."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Qmin = -0.436 "Minimum plant reactive power command."
      annotation (Dialog(tab="Input Parameters"));
    parameter Real Tft = 0 "Plant controller Q output lead time constant."
      annotation (Dialog(tab="Input Parameters"));
    parameter OpenIPSL.Types.Time Tfv = 0.075
      "Plant controller Q output lag time constant."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Interfaces.RealOutput Qext(start=q0) "Connector of Real output signal"
      annotation (Placement(transformation(extent={{320,60},{340,80}})));

    OpenIPSL.Interfaces.PwPin BRANCH_p
      annotation (Placement(transformation(extent={{0,180},{20,200}})));
    parameter Modelica.Units.SI.Voltage Vfrz=0
      "Voltage for freezing Volt/VAR regulator integrator."
      annotation (Dialog(tab="Input Parameters"));
    Modelica.Blocks.Sources.Constant VREF(k=Vref)
      annotation (Placement(transformation(extent={{66,114},{86,134}})));
    parameter Real Vref=v_0
   "Regulated bus initial voltage.";
    Modelica.Blocks.Interfaces.RealInput Qref annotation (Placement(
          transformation(extent={{-140,100},{-100,140}}), iconTransformation(
            extent={{-140,100},{-100,140}})));
    OpenIPSL.NonElectrical.Continuous.LeadLag leadLag(
      K=1,
      T1=Tft,
      T2=Tfv,
      y_start=q0,
      x_start=0)
      annotation (Placement(transformation(extent={{292,60},{312,80}})));
    Modelica.Blocks.Nonlinear.DeadZone deadZone(uMax=dbd2, uMin=dbd1)
      annotation (Placement(transformation(extent={{152,60},{172,80}})));
    Modelica.Blocks.Nonlinear.DeadZone deadZone1(uMax=fdbd2, uMin=fdbd1)
      annotation (Placement(transformation(extent={{-50,-184},{-30,-164}})));
    Modelica.Blocks.Interfaces.RealInput Freq_ref
      "Connector of Real input signal 2" annotation (Placement(
          transformation(extent={{-140,-200},{-100,-160}}),
          iconTransformation(extent={{-140,-200},{-100,-160}})));
  protected
      parameter Real CoB=M_b/S_b;
      parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
      parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
  equation
    REGULATE.ir = 0;
    REGULATE.ii = 0;

    connect(Freq, add.u1) annotation (Line(points={{-120,-80},{-92,-80},{
            -92,-168},{-82,-168}},
                             color={0,0,127}));
    connect(DDN.y, limiter.u)
      annotation (Line(points={{1,-154},{8,-154}},
                                                 color={0,0,127}));
    connect(DUP.y, limiter1.u)
      annotation (Line(points={{1,-190},{8,-190}},
                                                 color={0,0,127}));
    connect(limiter.y, add1.u1)
      annotation (Line(points={{31,-154},{38,-154},{38,-168}},
                                                            color={0,0,127}));
    connect(limiter1.y, add1.u2)
      annotation (Line(points={{31,-190},{38,-190},{38,-180}},
                                                            color={0,0,127}));
    connect(add1.y, add3_1.u3) annotation (Line(points={{61,-174},{70,-174}},
                                              color={0,0,127}));
    connect(simpleLag.y, add3_1.u2)
      annotation (Line(points={{1,-126},{64,-126},{64,-166},{70,-166}},
                                                    color={0,0,127}));
    connect(Plant_pref, add3_1.u1) annotation (Line(points={{-120,20},{-96,
            20},{-96,-22},{70,-22},{70,-158}}, color={0,0,127}));
    connect(add3_1.y, limiter2.u)
      annotation (Line(points={{93,-166},{102,-166}},
                                                   color={0,0,127}));
    connect(limiter2.y, KIG.u) annotation (Line(points={{125,-166},{126,
            -166},{126,-182},{130,-182}},
                                 color={0,0,127}));
    connect(KPG.u, limiter2.y) annotation (Line(points={{130,-150},{126,
            -150},{126,-166},{125,-166}},
                             color={0,0,127}));
    connect(KIG.y, add2.u2) annotation (Line(points={{153,-182},{160,-182},
            {160,-172}},
                   color={0,0,127}));
    connect(KPG.y, add2.u1) annotation (Line(points={{153,-150},{160,-150},
            {160,-160}},
                       color={0,0,127}));
    connect(add2.y, limiter3.u)
      annotation (Line(points={{183,-166},{188,-166}},
                                                     color={0,0,127}));
    connect(limiter3.y, simpleLag1.u)
      annotation (Line(points={{211,-166},{216,-166},{216,-148},{224,-148}},
                                                     color={0,0,127}));
    connect(simpleLag1.y, FREQ_FLAG.u1)
      annotation (Line(points={{247,-148},{260,-148}},
                                                     color={0,0,127}));
    connect(FREQ_FLAG_logic.y, FREQ_FLAG.u2) annotation (Line(points={{211,
            -198},{222,-198},{222,-172},{250,-172},{250,-156},{260,-156}},
                                            color={255,0,255}));
    connect(PREF.y, FREQ_FLAG.u3) annotation (Line(points={{251,-198},{256,
            -198},{256,-164},{260,-164}},
                                  color={0,0,127}));
    connect(FREQ_FLAG.y,Pref)  annotation (Line(points={{283,-156},{306,
            -156},{306,-130},{330,-130}},
                        color={0,0,127}));
    connect(Pbranch.y, simpleLag.u)
      annotation (Line(points={{-33,-126},{-22,-126}},
                                                     color={0,0,127}));
    connect(Voltage_diff.y, VCFLAG.u1)
      annotation (Line(points={{-39,142},{36,142},{36,96}},  color={0,0,127}));
    connect(Vreg.y, add3.u1) annotation (Line(points={{-39,92},{6,92},{6,86}},
                   color={0,0,127}));
    connect(Qbranch.y, KC.u)
      annotation (Line(points={{-39,74},{-22,74}},   color={0,0,127}));
    connect(KC.y, add3.u2)
      annotation (Line(points={{1,74},{6,74}},    color={0,0,127}));
    connect(add3.y, VCFLAG.u3)
      annotation (Line(points={{29,80},{36,80}},            color={0,0,127}));
    connect(VCFLAG_logic.y, VCFLAG.u2) annotation (Line(points={{-39,118},{
            36,118},{36,88}},  color={255,0,255}));
    connect(simpleLag2.u, KC.u) annotation (Line(points={{-24,36},{-28,36},
            {-28,74},{-22,74}},
                        color={0,0,127}));
    connect(simpleLag2.y, add4.u1)
      annotation (Line(points={{-1,36},{10,36}},        color={0,0,127}));
    connect(add4.y, REFFLAG.u3)
      annotation (Line(points={{33,30},{122,30},{122,62}}, color={0,0,127}));
    connect(REFFLAG_logic.y, REFFLAG.u2)
      annotation (Line(points={{87,58},{122,58},{122,70}},
                                                  color={255,0,255}));
    connect(VCFLAG.y, simpleLag3.u)
      annotation (Line(points={{59,88},{64,88}},   color={0,0,127}));
    connect(REFFLAG.u1, add5.y)
      annotation (Line(points={{122,78},{122,94},{117,94}},   color={0,0,127}));
    connect(simpleLag3.y, add5.u2)
      annotation (Line(points={{87,88},{94,88}},     color={0,0,127}));
    connect(limiter4.y, KP.u) annotation (Line(points={{201,70},{204,70},{
            204,90},{208,90}},
                        color={0,0,127}));
    connect(KI.u, limiter4.y) annotation (Line(points={{208,50},{204,50},{
            204,70},{201,70}},
                       color={0,0,127}));
    connect(KP.y, add6.u1)
      annotation (Line(points={{231,90},{234,90},{234,76}},   color={0,0,127}));
    connect(KI.y, add6.u2)
      annotation (Line(points={{231,50},{234,50},{234,64}}, color={0,0,127}));
    connect(add6.y, limiter5.u)
      annotation (Line(points={{257,70},{262,70}}, color={0,0,127}));
    connect(VREF.y, add5.u1)
      annotation (Line(points={{87,124},{94,124},{94,100}},    color={0,0,127}));
    connect(add4.u2, Qref) annotation (Line(points={{10,24},{10,14},{-92,14},
            {-92,120},{-120,120}},
                   color={0,0,127}));
    connect(limiter5.y, leadLag.u)
      annotation (Line(points={{285,70},{290,70}}, color={0,0,127}));
    connect(leadLag.y, Qext)
      annotation (Line(points={{313,70},{330,70}}, color={0,0,127}));
    connect(BRANCH_n, BRANCH_p)
      annotation (Line(points={{210,190},{210,174},{10,174},{10,190}},
                                                      color={0,0,255}));
    connect(limiter4.u, deadZone.y)
      annotation (Line(points={{178,70},{173,70}}, color={0,0,127}));
    connect(REFFLAG.y, deadZone.u)
      annotation (Line(points={{145,70},{150,70}}, color={0,0,127}));
    connect(add.y, deadZone1.u)
      annotation (Line(points={{-59,-174},{-52,-174}},
                                                     color={0,0,127}));
    connect(deadZone1.y, DDN.u) annotation (Line(points={{-29,-174},{-22,
            -174},{-22,-154}},
                       color={0,0,127}));
    connect(DUP.u, DDN.u)
      annotation (Line(points={{-22,-190},{-22,-154}},color={0,0,127}));
    connect(add.u2, Freq_ref)
      annotation (Line(points={{-82,-180},{-120,-180}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -240},{320,180}}),
                           graphics={
          Rectangle(extent={{-100,180},{320,-240}}, lineColor={28,108,200}),
            Text(
            extent={{-42,30},{260,-112}},
            textColor={238,46,47},
            textString="REPCA")}),Diagram(coordinateSystem(preserveAspectRatio=
              false, extent={{-100,-240},{320,180}})));
  end REPCA;

  model Irradiance_to_Power

    Modelica.Blocks.Sources.TimeTable IRRADIANCE_DATA(table=[0,0; 60,
          0; 120,0; 180,0; 240,0; 300,0; 360,0; 420,0; 480,0; 540,0;
          600,0; 660,0; 720,0; 780,0; 840,0; 900,0; 960,0; 1020,0;
          1080,0; 1140,0; 1200,0; 1260,0; 1320,0; 1380,0; 1440,0;
          1500,0; 1560,0; 1620,0; 1680,0; 1740,0; 1800,0; 1860,0;
          1920,0; 1980,0; 2040,0; 2100,0; 2160,0; 2220,0; 2280,0;
          2340,0; 2400,0; 2460,0; 2520,0; 2580,0; 2640,0; 2700,0;
          2760,0; 2820,0; 2880,0; 2940,0; 3000,0; 3060,0; 3120,0;
          3180,0; 3240,0; 3300,0; 3360,0; 3420,0; 3480,0; 3540,0;
          3600,0; 3660,0; 3720,0; 3780,0; 3840,0; 3900,0; 3960,0;
          4020,0; 4080,0; 4140,0; 4200,0; 4260,0; 4320,0; 4380,0;
          4440,0; 4500,0; 4560,0; 4620,0; 4680,0; 4740,0; 4800,0;
          4860,0; 4920,0; 4980,0; 5040,0; 5100,0; 5160,0; 5220,0;
          5280,0; 5340,0; 5400,0; 5460,0; 5520,0; 5580,0; 5640,0;
          5700,0; 5760,0; 5820,0; 5880,0; 5940,0; 6000,0; 6060,0;
          6120,0; 6180,0; 6240,0; 6300,0; 6360,0; 6420,0; 6480,0;
          6540,0; 6600,0; 6660,0; 6720,0; 6780,0; 6840,0; 6900,0;
          6960,0; 7020,0; 7080,0; 7140,0; 7200,0; 7260,0; 7320,0;
          7380,0; 7440,0; 7500,0; 7560,0; 7620,0; 7680,0; 7740,0;
          7800,0; 7860,0; 7920,0; 7980,0; 8040,0; 8100,0; 8160,0;
          8220,0; 8280,0; 8340,0; 8400,0; 8460,0; 8520,0; 8580,0;
          8640,0; 8700,0; 8760,0; 8820,0; 8880,0; 8940,0; 9000,0;
          9060,0; 9120,0; 9180,0; 9240,0; 9300,0; 9360,0; 9420,0;
          9480,0; 9540,0; 9600,0; 9660,0; 9720,0; 9780,0; 9840,0;
          9900,0; 9960,0; 10020,0; 10080,0; 10140,0; 10200,0; 10260,0;
          10320,0; 10380,0; 10440,0; 10500,0; 10560,0; 10620,0; 10680,
          0; 10740,0; 10800,0; 10860,0; 10920,0; 10980,0; 11040,0;
          11100,0; 11160,0; 11220,0; 11280,0; 11340,0; 11400,0; 11460,
          0; 11520,0; 11580,0; 11640,0; 11700,0; 11760,0; 11820,0;
          11880,0; 11940,0; 12000,0; 12060,0; 12120,0; 12180,0; 12240,
          0; 12300,0; 12360,0; 12420,0; 12480,0; 12540,0; 12600,0;
          12660,0; 12720,0; 12780,0; 12840,0; 12900,0; 12960,0; 13020,
          0; 13080,0; 13140,0; 13200,0; 13260,0; 13320,0; 13380,0;
          13440,0; 13500,0; 13560,0; 13620,0; 13680,0; 13740,0; 13800,
          0; 13860,0; 13920,0; 13980,0; 14040,0; 14100,0; 14160,0;
          14220,0; 14280,0; 14340,0; 14400,0; 14460,0; 14520,0; 14580,
          0; 14640,0; 14700,0; 14760,0; 14820,0; 14880,0; 14940,0;
          15000,0; 15060,0; 15120,0; 15180,0; 15240,0; 15300,0; 15360,
          0; 15420,0; 15480,0; 15540,0; 15600,0; 15660,0; 15720,0;
          15780,0; 15840,0; 15900,0; 15960,0; 16020,0; 16080,0; 16140,
          0; 16200,0; 16260,0; 16320,0; 16380,0; 16440,0; 16500,0;
          16560,0; 16620,0; 16680,0; 16740,0; 16800,0; 16860,0; 16920,
          0; 16980,0; 17040,0; 17100,0; 17160,0; 17220,0; 17280,0;
          17340,0; 17400,0; 17460,0; 17520,0; 17580,0; 17640,0; 17700,
          0; 17760,0; 17820,0; 17880,0; 17940,0; 18000,0; 18060,0;
          18120,0; 18180,0; 18240,0; 18300,0; 18360,0; 18420,0; 18480,
          0; 18540,0; 18600,0; 18660,0; 18720,0; 18780,0; 18840,0;
          18900,0; 18960,0; 19020,0; 19080,0; 19140,0; 19200,0; 19260,
          0; 19320,0; 19380,0; 19440,0; 19500,0; 19560,0; 19620,0;
          19680,0; 19740,0; 19800,0; 19860,0; 19920,0; 19980,0; 20040,
          0; 20100,0; 20160,0; 20220,0; 20280,0; 20340,0; 20400,0;
          20460,0; 20520,0; 20580,0; 20640,0; 20700,0; 20760,0; 20820,
          0; 20880,0; 20940,0; 21000,0; 21060,0; 21120,0; 21180,0;
          21240,0; 21300,0; 21360,0; 21420,0; 21480,0; 21540,0; 21600,
          0; 21660,0; 21720,0; 21780,0; 21840,0; 21900,0; 21960,0;
          22020,0; 22080,0; 22140,0; 22200,0; 22260,0; 22320,0; 22380,
          0; 22440,0; 22500,0; 22560,0; 22620,0; 22680,0; 22740,0;
          22800,0; 22860,0; 22920,0; 22980,0; 23040,0; 23100,0; 23160,
          0; 23220,0; 23280,0; 23340,0; 23400,0; 23460,0; 23520,0;
          23580,0; 23640,0; 23700,0; 23760,0; 23820,0; 23880,0; 23940,
          0; 24000,0; 24060,0; 24120,0; 24180,0; 24240,0; 24300,0;
          24360,0; 24420,0; 24480,0; 24540,0; 24600,0; 24660,0; 24720,
          0; 24780,0; 24840,0; 24900,0; 24960,0; 25020,0; 25080,0;
          25140,0; 25200,0; 25260,0; 25320,0; 25380,0; 25440,0; 25500,
          0; 25560,0; 25620,0; 25680,0; 25740,0; 25800,0; 25860,0;
          25920,0; 25980,0; 26040,0; 26100,0; 26160,0; 26220,0; 26280,
          0; 26340,0; 26400,0; 26460,0; 26520,0; 26580,0; 26640,0;
          26700,0; 26760,0; 26820,0; 26880,0; 26940,0; 27000,18;
          27060,19; 27120,19; 27180,20; 27240,21; 27300,21; 27360,22;
          27420,23; 27480,24; 27540,24; 27600,25; 27660,26; 27720,26;
          27780,27; 27840,27; 27900,31; 27960,34; 28020,40; 28080,43;
          28140,41; 28200,39; 28260,40; 28320,42; 28380,45; 28440,46;
          28500,47; 28560,50; 28620,56; 28680,62; 28740,62; 28800,61;
          28860,61; 28920,61; 28980,64; 29040,67; 29100,73; 29160,76;
          29220,80; 29280,84; 29340,88; 29400,90; 29460,97; 29520,102;
          29580,107; 29640,101; 29700,96; 29760,96; 29820,101; 29880,
          107; 29940,106; 30000,110; 30060,117; 30120,127; 30180,134;
          30240,137; 30300,140; 30360,140; 30420,140; 30480,129;
          30540,132; 30600,145; 30660,155; 30720,156; 30780,158;
          30840,160; 30900,161; 30960,163; 31020,165; 31080,170;
          31140,172; 31200,172; 31260,170; 31320,173; 31380,176;
          31440,175; 31500,172; 31560,181; 31620,195; 31680,207;
          31740,207; 31800,211; 31860,211; 31920,208; 31980,206;
          32040,208; 32100,211; 32160,220; 32220,225; 32280,230;
          32340,228; 32400,231; 32460,234; 32520,237; 32580,240;
          32640,243; 32700,246; 32760,249; 32820,252; 32880,254;
          32940,257; 33000,260; 33060,263; 33120,266; 33180,268;
          33240,271; 33300,274; 33360,277; 33420,280; 33480,282;
          33540,285; 33600,287; 33660,290; 33720,293; 33780,295;
          33840,298; 33900,300; 33960,302; 34020,305; 34080,307;
          34140,310; 34200,312; 34260,314; 34320,316; 34380,319;
          34440,321; 34500,323; 34560,326; 34620,328; 34680,330;
          34740,333; 34800,335; 34860,337; 34920,340; 34980,342;
          35040,345; 35100,347; 35160,349; 35220,352; 35280,354;
          35340,356; 35400,358; 35460,361; 35520,363; 35580,365;
          35640,367; 35700,369; 35760,371; 35820,373; 35880,375;
          35940,377; 36000,379; 36060,381; 36120,383; 36180,385;
          36240,387; 36300,389; 36360,391; 36420,393; 36480,395;
          36540,397; 36600,399; 36660,401; 36720,403; 36780,405;
          36840,407; 36900,409; 36960,411; 37020,413; 37080,415;
          37140,417; 37200,419; 37260,421; 37320,423; 37380,424;
          37440,426; 37500,428; 37560,429; 37620,431; 37680,433;
          37740,434; 37800,436; 37860,437; 37920,439; 37980,440;
          38040,442; 38100,443; 38160,445; 38220,447; 38280,448;
          38340,450; 38400,451; 38460,453; 38520,455; 38580,456;
          38640,458; 38700,459; 38760,461; 38820,463; 38880,464;
          38940,466; 39000,467; 39060,468; 39120,470; 39180,471;
          39240,472; 39300,474; 39360,475; 39420,476; 39480,477;
          39540,478; 39600,479; 39660,480; 39720,481; 39780,483;
          39840,484; 39900,485; 39960,486; 40020,487; 40080,488;
          40140,489; 40200,490; 40260,491; 40320,493; 40380,494;
          40440,495; 40500,496; 40560,497; 40620,498; 40680,499;
          40740,500; 40800,501; 40860,502; 40920,503; 40980,504;
          41040,505; 41100,505; 41160,506; 41220,507; 41280,507;
          41340,508; 41400,509; 41460,509; 41520,510; 41580,510;
          41640,511; 41700,511; 41760,512; 41820,513; 41880,513;
          41940,514; 42000,515; 42060,515; 42120,516; 42180,517;
          42240,517; 42300,518; 42360,518; 42420,519; 42480,520;
          42540,520; 42600,521; 42660,521; 42720,521; 42780,522;
          42840,522; 42900,522; 42960,522; 43020,523; 43080,523;
          43140,523; 43200,523; 43260,523; 43320,523; 43380,523;
          43440,523; 43500,523; 43560,523; 43620,523; 43680,523;
          43740,523; 43800,524; 43860,524; 43920,524; 43980,524;
          44040,524; 44100,524; 44160,524; 44220,524; 44280,524;
          44340,524; 44400,524; 44460,524; 44520,524; 44580,524;
          44640,524; 44700,523; 44760,523; 44820,523; 44880,522;
          44940,522; 45000,522; 45060,521; 45120,521; 45180,520;
          45240,520; 45300,519; 45360,519; 45420,518; 45480,518;
          45540,517; 45600,517; 45660,517; 45720,516; 45780,516;
          45840,515; 45900,515; 45960,514; 46020,514; 46080,514;
          46140,513; 46200,513; 46260,512; 46320,511; 46380,511;
          46440,510; 46500,510; 46560,509; 46620,508; 46680,507;
          46740,507; 46800,515; 46860,518; 46920,538; 46980,513;
          47040,491; 47100,446; 47160,481; 47220,498; 47280,515;
          47340,501; 47400,500; 47460,495; 47520,489; 47580,486;
          47640,481; 47700,485; 47760,489; 47820,503; 47880,500;
          47940,502; 48000,494; 48060,488; 48120,477; 48180,460;
          48240,470; 48300,474; 48360,491; 48420,481; 48480,480;
          48540,441; 48600,416; 48660,383; 48720,395; 48780,429;
          48840,463; 48900,475; 48960,468; 49020,463; 49080,475;
          49140,478; 49200,478; 49260,466; 49320,448; 49380,436;
          49440,399; 49500,354; 49560,324; 49620,374; 49680,394;
          49740,416; 49800,386; 49860,380; 49920,379; 49980,377;
          50040,458; 50100,534; 50160,585; 50220,474; 50280,382;
          50340,286; 50400,280; 50460,275; 50520,346; 50580,389;
          50640,408; 50700,382; 50760,381; 50820,389; 50880,397;
          50940,416; 51000,418; 51060,368; 51120,311; 51180,263;
          51240,253; 51300,290; 51360,310; 51420,341; 51480,342;
          51540,337; 51600,310; 51660,279; 51720,262; 51780,272;
          51840,282; 51900,287; 51960,289; 52020,307; 52080,329;
          52140,311; 52200,245; 52260,205; 52320,210; 52380,250;
          52440,279; 52500,311; 52560,306; 52620,313; 52680,316;
          52740,323; 52800,309; 52860,297; 52920,299; 52980,310;
          53040,314; 53100,318; 53160,309; 53220,306; 53280,307;
          53340,308; 53400,288; 53460,262; 53520,247; 53580,252;
          53640,267; 53700,274; 53760,257; 53820,262; 53880,265;
          53940,298; 54000,258; 54060,215; 54120,178; 54180,200;
          54240,220; 54300,237; 54360,247; 54420,258; 54480,259;
          54540,257; 54600,263; 54660,264; 54720,263; 54780,246;
          54840,230; 54900,215; 54960,218; 55020,229; 55080,236;
          55140,232; 55200,208; 55260,182; 55320,171; 55380,186;
          55440,202; 55500,203; 55560,224; 55620,194; 55680,170;
          55740,142; 55800,172; 55860,196; 55920,180; 55980,160;
          56040,138; 56100,138; 56160,155; 56220,170; 56280,179;
          56340,183; 56400,196; 56460,203; 56520,202; 56580,195;
          56640,191; 56700,186; 56760,175; 56820,163; 56880,167;
          56940,171; 57000,178; 57060,168; 57120,145; 57180,122;
          57240,114; 57300,135; 57360,146; 57420,143; 57480,131;
          57540,128; 57600,124; 57660,126; 57720,134; 57780,138;
          57840,140; 57900,123; 57960,113; 58020,101; 58080,102;
          58140,97; 58200,99; 58260,103; 58320,104; 58380,99; 58440,
          85; 58500,85; 58560,84; 58620,89; 58680,88; 58740,87; 58800,
          85; 58860,83; 58920,81; 58980,82; 59040,80; 59100,78; 59160,
          74; 59220,69; 59280,63; 59340,60; 59400,52; 59460,53; 59520,
          53; 59580,53; 59640,53; 59700,54; 59760,55; 59820,55; 59880,
          56; 59940,57; 60000,57; 60060,58; 60120,59; 60180,60; 60240,
          61; 60300,23; 60360,24; 60420,26; 60480,27; 60540,29; 60600,
          30; 60660,32; 60720,34; 60780,36; 60840,37; 60900,38; 60960,
          39; 61020,39; 61080,39; 61140,0; 61200,0; 61260,0; 61320,0;
          61380,0; 61440,0; 61500,0; 61560,0; 61620,0; 61680,0; 61740,
          0; 61800,0; 61860,0; 61920,0; 61980,0; 62040,0; 62100,0;
          62160,0; 62220,0; 62280,0; 62340,0; 62400,0; 62460,0; 62520,
          0; 62580,0; 62640,0; 62700,0; 62760,0; 62820,0; 62880,0;
          62940,0; 63000,0; 63060,0; 63120,0; 63180,0; 63240,0; 63300,
          0; 63360,0; 63420,0; 63480,0; 63540,0; 63600,0; 63660,0;
          63720,0; 63780,0; 63840,0; 63900,0; 63960,0; 64020,0; 64080,
          0; 64140,0; 64200,0; 64260,0; 64320,0; 64380,0; 64440,0;
          64500,0; 64560,0; 64620,0; 64680,0; 64740,0; 64800,0; 64860,
          0; 64920,0; 64980,0; 65040,0; 65100,0; 65160,0; 65220,0;
          65280,0; 65340,0; 65400,0; 65460,0; 65520,0; 65580,0; 65640,
          0; 65700,0; 65760,0; 65820,0; 65880,0; 65940,0; 66000,0;
          66060,0; 66120,0; 66180,0; 66240,0; 66300,0; 66360,0; 66420,
          0; 66480,0; 66540,0; 66600,0; 66660,0; 66720,0; 66780,0;
          66840,0; 66900,0; 66960,0; 67020,0; 67080,0; 67140,0; 67200,
          0; 67260,0; 67320,0; 67380,0; 67440,0; 67500,0; 67560,0;
          67620,0; 67680,0; 67740,0; 67800,0; 67860,0; 67920,0; 67980,
          0; 68040,0; 68100,0; 68160,0; 68220,0; 68280,0; 68340,0;
          68400,0; 68460,0; 68520,0; 68580,0; 68640,0; 68700,0; 68760,
          0; 68820,0; 68880,0; 68940,0; 69000,0; 69060,0; 69120,0;
          69180,0; 69240,0; 69300,0; 69360,0; 69420,0; 69480,0; 69540,
          0; 69600,0; 69660,0; 69720,0; 69780,0; 69840,0; 69900,0;
          69960,0; 70020,0; 70080,0; 70140,0; 70200,0; 70260,0; 70320,
          0; 70380,0; 70440,0; 70500,0; 70560,0; 70620,0; 70680,0;
          70740,0; 70800,0; 70860,0; 70920,0; 70980,0; 71040,0; 71100,
          0; 71160,0; 71220,0; 71280,0; 71340,0; 71400,0; 71460,0;
          71520,0; 71580,0; 71640,0; 71700,0; 71760,0; 71820,0; 71880,
          0; 71940,0; 72000,0; 72060,0; 72120,0; 72180,0; 72240,0;
          72300,0; 72360,0; 72420,0; 72480,0; 72540,0; 72600,0; 72660,
          0; 72720,0; 72780,0; 72840,0; 72900,0; 72960,0; 73020,0;
          73080,0; 73140,0; 73200,0; 73260,0; 73320,0; 73380,0; 73440,
          0; 73500,0; 73560,0; 73620,0; 73680,0; 73740,0; 73800,0;
          73860,0; 73920,0; 73980,0; 74040,0; 74100,0; 74160,0; 74220,
          0; 74280,0; 74340,0; 74400,0; 74460,0; 74520,0; 74580,0;
          74640,0; 74700,0; 74760,0; 74820,0; 74880,0; 74940,0; 75000,
          0; 75060,0; 75120,0; 75180,0; 75240,0; 75300,0; 75360,0;
          75420,0; 75480,0; 75540,0; 75600,0; 75660,0; 75720,0; 75780,
          0; 75840,0; 75900,0; 75960,0; 76020,0; 76080,0; 76140,0;
          76200,0; 76260,0; 76320,0; 76380,0; 76440,0; 76500,0; 76560,
          0; 76620,0; 76680,0; 76740,0; 76800,0; 76860,0; 76920,0;
          76980,0; 77040,0; 77100,0; 77160,0; 77220,0; 77280,0; 77340,
          0; 77400,0; 77460,0; 77520,0; 77580,0; 77640,0; 77700,0;
          77760,0; 77820,0; 77880,0; 77940,0; 78000,0; 78060,0; 78120,
          0; 78180,0; 78240,0; 78300,0; 78360,0; 78420,0; 78480,0;
          78540,0; 78600,0; 78660,0; 78720,0; 78780,0; 78840,0; 78900,
          0; 78960,0; 79020,0; 79080,0; 79140,0; 79200,0; 79260,0;
          79320,0; 79380,0; 79440,0; 79500,0; 79560,0; 79620,0; 79680,
          0; 79740,0; 79800,0; 79860,0; 79920,0; 79980,0; 80040,0;
          80100,0; 80160,0; 80220,0; 80280,0; 80340,0; 80400,0; 80460,
          0; 80520,0; 80580,0; 80640,0; 80700,0; 80760,0; 80820,0;
          80880,0; 80940,0; 81000,0; 81060,0; 81120,0; 81180,0; 81240,
          0; 81300,0; 81360,0; 81420,0; 81480,0; 81540,0; 81600,0;
          81660,0; 81720,0; 81780,0; 81840,0; 81900,0; 81960,0; 82020,
          0; 82080,0; 82140,0; 82200,0; 82260,0; 82320,0; 82380,0;
          82440,0; 82500,0; 82560,0; 82620,0; 82680,0; 82740,0; 82800,
          0; 82860,0; 82920,0; 82980,0; 83040,0; 83100,0; 83160,0;
          83220,0; 83280,0; 83340,0; 83400,0; 83460,0; 83520,0; 83580,
          0; 83640,0; 83700,0; 83760,0; 83820,0; 83880,0; 83940,0;
          84000,0; 84060,0; 84120,0; 84180,0; 84240,0; 84300,0; 84360,
          0; 84420,0; 84480,0; 84540,0; 84600,0; 84660,0; 84720,0;
          84780,0; 84840,0; 84900,0; 84960,0; 85020,0; 85080,0; 85140,
          0; 85200,0; 85260,0; 85320,0; 85380,0; 85440,0; 85500,0;
          85560,0; 85620,0; 85680,0; 85740,0; 85800,0; 85860,0; 85920,
          0; 85980,0; 86040,0; 86100,0; 86160,0; 86220,0; 86280,0;
          86340,0; 86400,0], timeScale=1)
      annotation (Placement(transformation(extent={{-66,-8},{-50,8}})));
    Modelica.Blocks.Math.Gain gain(k=1/585)
      annotation (Placement(transformation(extent={{-32,-6},{-20,6}})));
    Modelica.Blocks.Math.Gain gain1(k=derating_factor)
      annotation (Placement(transformation(extent={{20,-6},{32,6}})));
      Modelica.Blocks.Interfaces.RealOutput irradiance_out if use_irradiance_out annotation (
        Placement(transformation(
          origin={120,0},
          extent={{-20,-20},{20,20}},
          rotation=0),   iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={110,0})));
    parameter Real derating_factor=0.7
      "Gain value multiplied with input signal";
       parameter Boolean use_irradiance_out = false
    "If true, supply the power reference to the PV model."
     annotation (
      Evaluate=true,
      HideResult=true,
      choices(checkBox=true),
      Dialog(group="Options"));
  equation
    connect(IRRADIANCE_DATA.y,gain. u) annotation (Line(points={{-49.2,0},
            {-33.2,0}},                     color={0,0,127}));
    connect(gain.y, gain1.u) annotation (Line(points={{-19.4,0},{18.8,
            0}}, color={0,0,127}));
    connect(gain1.y, irradiance_out)
      annotation (Line(points={{32.6,0},{120,0}}, color={0,0,127}));
    annotation (Icon(graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.None),
          Bitmap(
            extent={{-90,-92},{98,92}},
            imageSource=
                "iVBORw0KGgoAAAANSUhEUgAABisAAAYrCAYAAABqOI65AAAgAElEQVR4XuzdebhdV1k/8Hftk7mZmmbonLbpkI7MoKIMyqQICIqgKDLKjK0IqOAPVBwQMQVkEEQQZFDmSRAQBEEURZG2SYekTZN0yNDMadIkZ+/fc+4tQ7HtPffec86ePvd5fPija6/3/X7Wen4/9O3ZO4U/AgQIECBAgAABAgQIECBAgAABAgQIECBAgECJAqnE2koTIECAAAECBAgQIECAAAECBAgQIECAAAECBOJ7w4qiKAoeBAgQIECAAAECBAgQIECAAAECBAgQIECAAIFRCxhWjFpcPQIECBAgQIAAAQIECBAgQIAAAQIECBAgQOAOAoYVLgQBAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBAgAABAoYV7gABAgQIECBAgAABAgQIECBAgAABAgQIECBQqoBhRan8ihMgQIAAAQIECBAgQIAAAQIECBAgQIAAAQKGFe4AAQIECBAgQIAAAQIECBAgQIAAAQIECBAgUKqAYUWp/IoTIECAAAECBAgQIECAAAECBAgQIECAAAEChhXuAAECBAgQIECAAAECBAgQIECAAAECBAgQIFCqgGFFqfyKEyBAgAABAgQIECBAgAABAgQIECBA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dM79r4hZp/ax0hICBAgQIECAAAECBJokYFjRpNOUhQABAgQIECBAgMDtAvlVD4ni0NraeWSnvjXSsT9fu741TIAAAQIECBAgQIDA9AQMK6bn52kCBAgQIECAAAEC1RM4vCm66+5bvb766CgtfUZkJ/1pHystIUCAAAECBAgQIECgSQKGFU06TVkIECBAgAABAgQI9L5XsftTkV//zFpapLkXRnb2P9eyd00TIECAAAECBAgQIDB1AcOKqdt5kgABAgQIECBAgEAlBfKb/jCKbW+qZG/9NNW5cFNENqefpdYQIECAAAECBAgQINAQAcOKhhykGAQIECBAgAABAgS+K5Bv+Pko9v9rbUGyVR+JNP8natu/xgkQIECAAAECBAgQmLyAYcXkzTxBgAABAgQIECBAoLoC+cHorr0woru3uj1O0Fl2/G9HWvGbte1f4wQIECBAgAABAgQITF7AsGLyZp4gQIAAAQIECBAgUFmBYv+/Rb7h5yrbXz+NpQU/FdkZH+hnqTUECBAgQIAAAQIECDREwLCiIQcpBgECBAgQIECAAIGeQLH9LZHf+Op6Y3QWRueC9fXOoHsCBAgQIECAAAECBCYlYFgxKS6LCRAgQIAAAQIECFRbIL/+OVHs/li1m+yju+ycf40055w+VlpCgAABAgQIECBAgEATBAwrmnCKMhAgQIAAAQIECBC4XaB75QMibruu9h7ZyX8R6bhfqX0OAQgQIECAAAECBAgQ6E/AsKI/J6sIECBAgAABAgQIVF/g0FXRveonqt9nHx2mJb8U2Slv6GOlJQQIECBAgAABAgQINEHAsKIJpygDAQIECBAgQIAAgd73Knb9Q+SbXtgMi9lnRmf1vzUjixQECBAgQIAAAQIECEwoYFgxIZEFBAgQIECAAAECBOohkN/wu1Hs+Ot6NNtHl53zr4yYsaSPlZYQIECAAAECBAgQIFB3AcOKup+g/gkQIECAAAECBAjcLpCv/5koDvxXYzyy098baeEjG5NHEAIECBAgQIAAAQIE7lrAsMLtIECAAAECBAgQINAEgaM7o7v2wojiSBPSjGVIy18c2QmvbEweQQgQIECAAAECBAgQMKxwBwgQIECAAAECBAg0WqDY98+RX/tLjcqY5v9oZKs+0ahMwhAgQIAAAQIECBAgcOcCflnhZhAgQIAAAQIECBBogECx9fWR3/zaBiT5gQhpRnQuurFZmaQhQIAAAQIECBAgQOBOBQwrXAwCBAgQIECAAAECDRDIr3tqFHs/14Akd4yQnfVPkebdq3G5BCJAgAABAgQIECBA4I4ChhVuBAECBAgQIECAAIEGCHTXXhRx5OYGJPmhYcVJr4m09Ncbl0sgAgQIECBAgAABAgQMK1ziHKMAACAASURBVNwBAgQIECBAgAABAo0SKG79duTXPKJRmb4bJi1+XGQr39HIbEIRIECAAAECBAgQIPB9Ab+scBsIECBAgAABAgQI1FyguOVvI9/y0pqnuIv2Z54YnfO+3cxsUhEgQIAAAQIECBAg8D0BwwqXgQABAgQIECBAgEDNBfLNl0Sx8301T3HX7XfO+5+ImSc1Np9gBAgQIECAAAECBAhEGFa4BQQIECBAgAABAgRqLpBf/dAoDl5R8xR33X628u2RFv9cY/MJRoAAAQIECBAgQICAYYU7QIAAAQIECBAgQKDeAkduiO7ae9U7wwTdp6XPjuykP2p0RuEIECBAgAABAgQItF3ALyvafgPkJ0CAAAECBAgQqLVAsefTkW98Rq0zTNR8mnfPyM76/ETL/HMCBAgQIECAAAECBGosYFhR48PTOgECBAgQIECAAIH8ptdEse2NjYfoXHRDRJrZ+JwCEiBAgAABAgQIEGirgGFFW09ebgIECBAgQIAAgUYI5Nc+MYp9X2lElrsLka36eKT5P9b4nAISIECAAAECBAgQaKuAYUVbT15uAgQIECBAgACB+gsUR6J7xXkR3T31zzJBguyEV0Ra/huNzykgAQIECBAgQIAAgbYKGFa09eTlJkCAAAECBAgQqL1AceDfI1//2Nrn6CdAWviIyE7/u36WWkOAAAECBAgQIECAQA0FDCtqeGhaJkCAAAECBAgQINATKLa/LfIb/187MDrHRueCq9qRVUoCBAgQIECAAAECLRQwrGjhoYtMgAABAgQIECDQDIF803Oj2PXRZoTpI0Vn9b9FzD6zj5WWECBAgAABAgQIECBQNwHDirqdmH4JECBAgAABAgQI3C7QvfJHIm67tjUe2SlviLTkl1qTV1ACBAgQIECAAAECbRIwrGjTactKgAABAgQIECDQHIHb1kf3yh9rTp4+kqQlT4nslDV9rLSEAAECBAgQIECAAIG6CRhW1O3E9EuAAAECBAgQIECg972KXR+KfNML2mUx5+zonPO1dmWWlgABAgQIECBAgEBLBAwrWnLQYhIgQIAAAQIECDRLIL/hlVHseHuzQvWRpnPBNRGdRX2stIQAAQIECBAgQIAAgToJGFbU6bT0SoAAAQIECBAgQOB2gXz9z0Zx4Jut88hOf3+khQ9rXW6BCRAgQIAAAQIECDRdwLCi6ScsHwECBAgQIECAQPMEunuje8V5EcXh5mWbIFFacUlkx/9O63ILTIAAAQIECBAgQKDpAoYVTT9h+QgQIECAAAECBBonUOz7cuTXPqlxufoJlOb/eGSrPtrPUmsIECBAgAABAgQIEKiRgGFFjQ5LqwQIECBAgAABAgR6AsXWNZHf/CftxEizo3PR5nZml5oAAQIECBAgQIBAgwUMKxp8uKIRIECAAAECBAg0UyDf+GtR7PlsM8P1kSo7+4uR5l7Ux0pLCBAgQIAAAQIECBCoi4BhRV1OSp8ECBAgQIAAAQIEbhforr1nxJEbW+uRnfQnkZY+s7X5BSdAgAABAgQIECDQRAHDiiaeqkwECBAgQIAAAQKNFSgOXhb51T/V2Hz9BEuLnxDZyrf1s9QaAgQIECBAgAABAgRqImBYUZOD0iYBAgQIECBAgACBnkBxy3si3/Jb7caYdUp0zv1Wuw2kJ0CAAAECBAgQINAwAcOKhh2oOAQIECBAgAABAs0WyLe8JIpb3tvskH2k65z3nYiZx/ex0hICBAgQIECAAAECBOogYFhRh1PSIwECBAgQIECAAIHbBXqvgOq9Cqrtf9lp74y06DFtZ5CfAAECBAgQIECAQGMEDCsac5SCECBAgAABAgQINF7gyNborr2w8TH7CZiWPTeyE/+gn6XWECBAgAABAgQIECBQAwHDihockhYJECBAgAABAgQI9ASKPf8Y+canwYiINO8+kZ31WRYECBAgQIAAAQIECDREwLCiIQcpBgECBAgQIECAQPMF8pv/OIqtlzY/aJ8JO/e4OSKyPldbRoAAAQIECBAgQIBAlQUMK6p8OnojQIAAAQIECBAg8AMC+bW/GMW+f2Fyu0B25qciHfMAHgQIECBAgAABAgQINEDAsKIBhygCAQIECBAgQIBAGwSK6F6+OqK7qw1h+8qYnfD/Ii1/YV9rLSJAgAABAgQIECBAoNoChhXVPh/dESBAgAABAgQIEBgTKA58M/L1P0vjBwTSop+O7LS/ZUKAAAECBAgQIECAQAMEDCsacIgiECBAgAABAgQINF+g2P5Xkd/4e80POpmEM46LzvnrJvOEtQQIECBAgAABAgQIVFTAsKKiB6MtAgQIECBAgAABAj8okG96fhS7PgzlhwQ6q/89YvYZXAgQIECAAAECBAgQqLmAYUXND1D7BAgQIECAAAEC7RDoXvljEbetb0fYSaTMTn1TpGOfNIknLCVAgAABAgQIECBAoIoChhVVPBU9ESBAgAABAgQIEPhBgds2RvfK+zO5E4F03FMjO/nP2RAgQIAAAQIECBAgUHMBw4qaH6D2CRAgQIAAAQIEmi9Q7PpI5Jue1/ygU0iY5pwb2TlfmcKTHiFAgAABAgQIECBAoEoChhVVOg29ECBAgAABAgQIELgTgd6HtXsf2PZ35wKdC6+NyObjIUCAAAECBAgQIECgxgKGFTU+PK0TIECAAAECBAi0QyBf/5goDvxHO8JOIWV2xt9HWvDQKTzpEQIECBAgQIAAAQIEqiJgWFGVk9AHAQIECBAgQIAAgTsTyG+N7hWrI/JDfO5CIK34rciOfxkfAgQIECBAgAABAgRqLGBYUePD0zoBAgQIECBAgEDzBYp9X4n82ic2P+g0EqYFD47sjA9NYwePEiBAgAABAgQIECBQtoBhRdknoD4BAgQIECBAgACBuxEotr0h8pv+iNHdCWTzonPhRkYECBAgQIAAAQIECNRYwLCixoendQIECBAgQIAAgeYL5BufHsWezzQ/6DQTZmd/OdLc86e5i8cJECBAgAABAgQIEChLwLCiLHl1CRAgQIAAAQIECPQh0F17r4gjN/Sxst1LspP/LNJxT2s3gvQECBAgQIAAAQIEaixgWFHjw9M6AQIECBAgQIBAswWKQ+siv+rBzQ45oHTp2CdGduqbB7SbbQgQIECAAAECBAgQGLWAYcWoxdUjQIAAAQIECBAg0KdAsfPvIt/8m32ubvmyWadF59xvthxBfAIECBAgQIAAAQL1FTCsqO/Z6ZwAAQIECBAgQKDhAvmW34rilvc0POXg4nXOvyJixrLBbWgnAgQIECBAgAABAgRGJmBYMTJqhQgQIECAAAECBAhMTiC/+uFRHPzfyT3U4tXZae+OtOhnWiwgOgECBAgQIECAAIH6ChhW1PfsdE6AAAECBAgQINBkgaO3RPeKc5uccODZ0rIXRHbiqwa+rw0JECBAgAABAgQIEBi+gGHF8I1VIECAAAECBAgQIDBpgWLP5yLf+NRJP9fmB9Ix94/szE+3mUB2AgQIECBAgAABArUVMKyo7dFpnAABAgQIECBAoMkC+c1/GsXWv2hyxCFkS9G5x9Yh7GtLAgQIECBAgAABAgSGLWBYMWxh+xMgQIAAAQIECBCYgkB+7ZOj2PelKTzZ7keyMz8T6Zj7tRtBegIECBAgQIAAAQI1FDCsqOGhaZkAAQIECBAgQKD5At0rVkcc3dn8oANOmJ346kjLnj/gXW1HgAABAgQIECBAgMCwBQwrhi1sfwIECBAgQIAAAQKTFChu/e/Ir3nUJJ+yvCeQFj06stPeBYMAAQIECBAgQIAAgZoJGFbU7MC0S4AAAQIECBAg0HyBYsc7Ir/hFc0POoyEM5ZH5/zLh7GzPQkQIECAAAECBAgQGKKAYcUQcW1NgAABAgQIECBAYCoC+aYXRLHrQ1N51DMR0Tn3PyNmrWRBgAABAgQIECBAgECNBAwranRYWiVAgAABAgQIEGiHQPeqH484dHU7wg4hZXbqWyId+wtD2NmWBAgQIECAAAECBAgMS8CwYliy9iVAgAABAgQIECAwFYHDW6K77t5TedIztwukpU+P7KTX8iBAgAABAgQIECBAoEYChhU1OiytEiBAgAABAgQINF+g2P2xyK9/TvODDjFhmntBZGd/aYgVbE2AAAECBAgQIECAwKAFDCsGLWo/AgQIECBAgAABAtMQyG98VRTb3zqNHTzaE+hceH1ENhcGAQIECBAgQIAAAQI1ETCsqMlBaZMAAQIECBAgQKAdAvn6x0Vx4BvtCDvElNmqD0ea/6AhVrA1AQIECBAgQIAAAQKDFDCsGKSmvQgQIECAAAECBAhMR6A4HN3Lz4rID05nF89GRHb8yyOteAkLAgQIECBAgAABAgRqImBYUZOD0iYBAgQIECBAgEDzBYr9X498w+ObH3QECdOCn4zsjA+OoJISBAgQIECAAAECBAgMQsCwYhCK9iBAgAABAgQIECAwAIFi2xsjv+k1A9jJFtFZEJ0LNoAgQIAAAQIECBAgQKAmAoYVNTkobRIgQIAAAQIECDRfIN/4jCj2fLr5QUeUMDvnq5HmrB5RNWUIECBAgAABAgQIEJiOgGHFdPQ8S4AAAQIECBAgQGCAAt1194k4vHmAO7Z7q+zk10c67lfbjSA9AQIECBAgQIAAgZoIGFbU5KC0SYAAAQIECBAg0HCBQ9dE96oHNjzkaOOlJU+O7JQ3jraoagQIECBAgAABAgQITEnAsGJKbB4iQIAAAQIECBAgMFiBYuf7I9988WA3bftus1dFZ/U32q4gPwECBAgQIECAAIFaCBhW1OKYNEmAAAECBAgQINB0gXzLy6K45d1NjznyfJ3zr4yYsWTkdRUkQIAAAQIECBAgQGByAoYVk/OymgABAgQIECBAgMBQBPJrHhHFrd8eyt5t3jQ7/b2RFj6yzQSyEyBAgAABAgQIEKiFgGFFLY5JkwQIECBAgAABAo0W6O6J7uVnNTpiWeHS8hdHdsIryyqvLgECBAgQIECAAAECfQoYVvQJZRkBAgQIECBAgACBYQkUe78Y+XW/PKztW71vOuZHIjvzk602EJ4AAQIECBAgQIBAHQQMK+pwSnokQIAAAQIECBBotEB+82uj2Pr6RmcsLVzqROeim0orrzABAgQIECBAgAABAv0JGFb052QVAQIECBAgQIAAgaEJ9H5V0ft1hb/hCGRnfS7SvHsPZ3O7EiBAgAABAgQIECAwEAHDioEw2oQAAQIECBAgQIDA1AW6V5wXcXTH1Dfw5N0KZCf+YaRlz6FEgAABAgQIECBAgECFBQwrKnw4WiNAgAABAgQIEGi+QHHwO5Ff/bDmBy0xYVr82MhW/nWJHShNgAABAgQIECBAgMBEAoYVEwn55wQIECBAgAABAgSGKFDseGfkN/zOECvYOmaeEJ3z/hcEAQIECBAgQIAAAQIVFjCsqPDhaI0AAQIECBAgQKD5AvmmF0Wx6++bH7TkhJ1z/zti1skld6E8AQIECBAgQIAAAQJ3JWBY4W4QIECAAAECBAgQKFEgv+onojh0VYkdtKN0tvKvIi1+fDvCSkmAAAECBAgQIECghgKGFTU8NC0TIECAAAECBAg0RODIzdFde1FDwlQ7Rlr6rMhO+uNqN6k7AgQIECBAgAABAi0WMKxo8eGLToAAAQIECBAgUK5AsftTkV//zHKbaEn1NPcekZ39hZakFZMAAQIECBAgQIBA/QQMK+p3ZjomQIAAAQIECBBoiEB+46uj2P6WhqSpfozORVsi0qzqN6pDAgQIECBAgAABAi0UMKxo4aGLTIAAAQIECBAgUA2BfMPjo9j/9Wo004IuslUfizT/gS1IKiIBAgQIECBAgACB+gkYVtTvzHRMgAABAgQIECDQCIE8upetisgPNCJNHUJkx/9upBUX16FVPRIgQIAAAQIECBBonYBhReuOXGACBAgQIECAAIEqCBQH/iPy9Y+pQiut6SEtfHhkp7+vNXkFJUCAAAECBAgQIFAnAcOKOp2WXgkQIECAAAECBBojUGz7y8hv+oPG5KlFkM7i6FxwdS1a1SQBAgQIECBAgACBtgkYVrTtxOUlQIAAAQIECBCohEB+/bOi2P3JSvTSpiY6q78eMfusNkWWlQABAgQIECBAgEAtBAwranFMmiRAgAABAgQIEGiaQHfdfSMOb2parMrnyU65NNKSX658nxokQIAAAQIECBAg0DYBw4q2nbi8BAgQIECAAAEC5Qvcdl10r3xA+X20sIO05CmRnbKmhclFJkCAAAECBAgQIFBtAcOKap+P7ggQIECAAAECBBooUOz6h8g3vbCByWoQac7Z0TnnazVoVIsECBAgQIAAAQIE2iVgWNGu85aWAAECBAgQIECgAgL5DS+PYse7KtBJO1sY+8h2Z3E7w0tNgAABAgQIECBAoKIChhUVPRhtESBAgAABAgQINFcgv+ZRUdz6380NWPFk2envi7Tw4RXvUnsECBAgQIAAAQIE2iVgWNGu85aWAAECBAgQIECgbIH8QHQvWxURedmdtLZ+WnFxZMf/bmvzC06AAAECBAgQIECgigKGFVU8FT0RIECAAAECBAg0VqDY9y+RX/uLjc1Xh2Bp/gMjW/WxOrSqRwIECBAgQIAAAQKtETCsaM1RC0qAAAECBAgQIFAFgeLm10W+9XVVaKW9PaRZ0bloS3vzS06AAAECBAgQIECgggKGFRU8FC0RIECAAAECBAg0VyC/7ilR7P1CcwPWJFl29hcizb1HTbrVJgECBAgQIECAAIHmCxhWNP+MJSRAgAABAgQIEKiQQPeK8yOObq9QR+1sJTvpjyMtfVY7w0tNgAABAgQIECBAoIIChhUVPBQtESBAgAABAgQINFOgOHhF5Fc/tJnhapYqLX58ZCv/qmZda5cAAQIECBAgQIBAcwUMK5p7tpIRIECAAAECBAhUTKC45d2Rb3lZxbpqaTuzTo7Ouf/d0vBiEyBAgAABAgQIEKiegGFF9c5ERwQIECBAgAABAg0VyDf/RhQ7P9DQdPWL1TnvfyNmnlC/xnVMgAABAgQIECBAoIEChhUNPFSRCBAgQIAAAQIEqimQX/XgKA6tq2ZzLewqW/nXkRY/toXJRSZAgAABAgQIECBQPQHDiuqdiY4IECBAgAABAgSaKHB0e4x9XNtfZQTSsudEduIfVqYfjRAgQIAAAQIECBBos4BhRZtPX3YCBAgQIECAAIGRCRR7/jHyjU8bWT2FJhZI8+4d2Vmfm3ihFQQIECBAgAABAgQIDF3AsGLoxAoQIECAAAECBAgQiMhv+oMotv0liooJdC66KSJ1KtaVdggQIECAAAECBAi0T8Cwon1nLjEBAgQIECBAgEAJAvmGJ0Sx/2slVFby7gSyMz8Z6ZgfgUSAAAECBAgQIECAQMkChhUlH4DyBAgQIECAAAEC7RDoXr4qoruvHWFrlDI74fciLX9RjTrWKgECBAgQIECAAIFmChhWNPNcpSJAgAABAgQIEKiQQHHrf0V+zc9UqCOtfFcgLXxUZKe/BwgBAgQIECBAgAABAiULGFaUfADKEyBAgAABAgQINF+g2P7WyG98VfOD1jHhjOOic/66OnauZwIECBAgQIAAAQKNEjCsaNRxCkOAAAECBAgQIFBFgfz6X49i98er2JqeIqKz+t8jZp/BggABAgQIECBAgACBEgUMK0rEV5oAAQIECBAgQKAdAt119484vLEdYWuYMjvljZGWPLmGnWuZAAECBAgQIECAQHMEDCuac5aSECBAgAABAgQIVFHg8KborrtvFTvT0+0C6bhfjezk1/MgQIAAAQIECBAgQKBEAcOKEvGVJkCAAAECBAgQaL5AsesjkW96XvOD1jhhmrM6snO+WuMEWidAgAABAgQIECBQfwHDivqfoQQECBAgQIAAAQIVFshv+J0odryzwh1qrSfQuWBDRGcBDAIECBAgQIAAAQIEShIwrCgJXlkCBAgQIECAAIF2COTX/HQUt36rHWFrnDI744ORFvxkjRNonQABAgQIECBAgEC9BQwr6n1+uidAgAABAgQIEKiyQH4oupefEVEcrXKXeouItOIlkR3/chYECBAgQIAAAQIECJQkYFhREryyBAgQIECAAAECzRco9v9r5Bt+vvlBG5AwzX9QZKs+3IAkIhAgQIAAAQIECBCop4BhRT3PTdcECBAgQIAAAQI1ECi2/kXkN/9pDTrVYmRzo3Ph9SAIECBAgAABAgQIEChJwLCiJHhlCRAgQIAAAQIEmi+QX/erUez9p+YHbUjC7OwvRZp7QUPSiEGAAAECBAgQIECgXgKGFfU6L90SIECAAAECBAjUSKC79sKII1tr1HG7W81Ofm2k457ebgTpCRAgQIAAAQIECJQkYFhREryyBAgQIECAAAECDRc4dFV0r/qJhodsVrx07C9EdupbmhVKGgIECBAgQIAAAQI1ETCsqMlBaZMAAQIECBAgQKBeAsXOv4t882/Wq+m2dztrZXTO/c+2K8hPgAABAgQIECBAoBQBw4pS2BUlQIDANAQOb47i8OaIzqJIMxaP/Wdk86exoUcJECBAYBgC+eaLo9j5/mFsbc8hCnTOvzxixvIhVrA1AQIECExZ4PCWKA5vijT7tIiZJ055Gw8SIECAQDUFDCuqeS66IkCAwLhAd1cUe78Qxc4PjP2X8ugNKe7sr3NspIU/FWn+AyPm3TfSnHMIEiBAgEDJAvnVD4ni4NqSu1B+sgLZae+KtOjRk33MegIECBAYsECx59NR7P9GxOHrx/6n6P1nfuj7VbI5kWadEdEbXMw6JdIxPxpp0c8MuAvbESBAgMAoBQwrRqmtFgECBPoUKPZ9OYpdH4pi9yciiiN9PvX9ZWnhIyId99To/ac/AgQIEChBoLsrupcbHJcgP+2SadnzIzvx1dPexwYECBAgMAWBo9uj2PXRKHZ/JIpbvz3pDdIxD4h07C9GWvKLEWn2pJ/3AAECBAiUK2BYUa6/6gQIELiDQHHoqii2vy2Kne8biIyhxUAYbUKAAIFJCxR7Px/5db8y6ec8UL5AOuZ+kZ35mfIb0QEBAgRaJFAcvDyKXX8/NqiIo9unn3z2mZH1hhbLXxCRZk5/PzsQIECAwEgEDCtGwqwIAQIEJhbIb35dFDveFtHdN/HiSa5Ixz0tshUXe6/rJN0sJ0CAwFQF8pv+KIptb5jq454rWaBzj20ld6A8AQIE2iNQbHtj5Fsvjcj3Dzz02AB65V9HzDxh4HvbkAABAgQGL2BYMXhTOxIgQGDSAvnGp0Wx5x8n/dykHph1amQrLom05CmTesxiAgQIEJi8QL7hF6LY/9XJP+iJSgj0flnR+z9w+SNAgACB4QkU+74SxbZLo9j/9eEV6e2cHRPZmZ+INPei4daxOwECBAhMW8CwYtqENiBAgMD0BLrr7h9xeOP0NpnE02nxYyItvyTS3Asm8ZSlBAgQIDAZge7lZ0Z0907mEWsrJND7ZkXv2xX+CBAgQGAIAt3dY7+kKLa/ZQib3/WW2dlfjjT3/JHWVIwAAQIEJidgWDE5L6sJECAwUIHu2gsjjmwd6J59bZbNH3stVFr+4r6WW0SAAAEC/Qv0PgiaX/OI/h+wsnICadGjIzvtXZXrS0MECBCou0Cx+2NR9AYVh9aVEiU7518jzTmnlNqKEiBAgMDEAoYVExtZQYAAgaEIjH2jYuvrhrJ3v5um+Q+MtPziSAse3O8j1hEgQIDABALFjndEfsMrONVZYMay6Jx/RZ0T6J0AAQLVErjt2sh7r3za+cFy+5qxLLJVHzWwKPcUVCdAgMBdChhWuBwECBAoQaDY+YHIN/9GCZXvvGRa9ryx71lEZ3FletIIAQIE6iqQX/+c6P2bo/7qLdA595sRs06rdwjdEyBAoAICxY53Rr51TcTRbRXoJsa+XZGd9bmINKMS/WiCAAECBL4vYFjhNhAgQGDUAt290b36IRGHt4y68t3WS3NWR+q9GmrxEyrVl2YIECBQN4HulQ+IuO26urWt3x8SyE59c6Rjn8iFAAECBKYoUNz6X1FsXRPF3i9McYfhPZad+IeRlj1neAXsTIAAAQJTEjCsmBKbhwgQIDB1geKWd0e+5WVT32DIT6ZjnzT2PYuYvWrIlWxPgACBBgocuTG6a+/ZwGDti5SOe1pkJ/9Z+4JLTIAAgekKFEfGfklRbLs0ojg63d2G8/ys06LT+3XFjCXD2d+uBAgQIDAlAcOKKbF5iAABAlMXyNc/JooD/zH1DUbxZO9drisuibT0WaOopgYBAgQaI1Ds/kTk1z+7MXnaHCTNPT+ys7/cZgLZCRAgMGmBYu8/jX9A+9ZvTfrZUT+QVrw0suNfOuqy6hEgQIDA3QgYVrgeBAgQGKFA7yfQ+XVPGWHF6ZVKCx8WafklkY653/Q28jQBAgRaIpDf+Mootr+9JWmbH7Nz4caIbF7zg0pIgACB6Qoc2Rr5tjVR7Pib6e40sufHvl1x9hdHVk8hAgQIEJhYwLBiYiMrCBAgMDCBfNPzotj1kYHtN5KNUifS8ovHXw2VZo+kpCIECBCoq0C+/tFRHPjPurav7x8SyM74UKQFD+ZCgAABAncjUOz8QOS9Vz7V8HtN2VmfjzTP6xtdcAIECFRFwLCiKiehDwIEmi/Q3RPdy8+qbc40797jH+Be+KjaZtA4AQIEhipQHI3uZSsjiiNDLWPz0Qlkx78s0orfGl1BlQgQIFAjgeLQ2vFXPu3+eI26vmOracUlkR3/O7XtX+MECBBomoBhRdNOVB4CBCorUBz8TuRXP6yy/fXb2NgHR1dcEjHzhH4fsY4AAQKtECgOfCPy9Y9rRda2hEwLHhrZGX/flrhyEiBAoG+BYtubx177FN29fT9TxYVp3n0iO+uzVWxNTwQIEGilgGFFK49daAIEyhAo9nw68o3PKKP04GvOWjn2Wqi0pD7f3xg8gh0JECBwR4FiCoi/oAAAIABJREFU2xsjv+k1WJokkM2PzoXXNimRLAQIEJiWQLH/a+O/ptj/1WntU5mHZyyPzvmXV6YdjRAgQKDtAoYVbb8B8hMgMDKBYvubI7/x90dWbxSF0uLHjn3PIs29YBTl1CBAgEClBfKNT41iz+cq3aPmJi+QnfOVSHPOnfyDniBAgECTBLr7xr5LUWx7U5NSjWXpXLTZt/kad6oCESBQVwHDirqenL4JEKidQL7l5VHc8q7a9T1hw9n8sddCpeUvmnCpBQQIEGiyQHftPSKO3NTkiK3Mlp3855GOe2orswtNgACBnkCx+5NRbFsTxcErGgnSWf31iNn1/bZgIw9FKAIEWitgWNHaoxecAIFRC+TXPjmKfV8addmR1UvzHzj+K4sFDx5ZTYUIECBQGYHbNkT3yh+tTDsaGZxAOvZJkZ3avH+TeHBCdiJAoLECh6+PfOuaKHa+v7ERe8GyMz8V6ZgHNDqjcAQIEKiLgGFFXU5KnwQI1F4g3/T8KHZ9uPY5JgqQlj1/7HsW0Vk80VL/nAABAo0RKHZ+MPLNL25MHkF+QGD2GdFZ/e9ICBAg0CqB3i/C862XtuIXg9k5/xppzjmtOl9hCRAgUFUBw4qqnoy+CBBonEB+8+ui2Pq6xuW6s0BpzupIvVdDLX58K/IKSYAAgXzLS6K45b0gGirQOX9dxIzjGppOLAIECHxfoLj1v8c/oL23Pd9g6px3WcTMFa4BAQIECFRAwLCiAoegBQIE2iFQ7PqHyDe9sB1hb0+ZljwpsuWXRMw+o1W5hSVAoH0C+dU/GcXBy9sXvCWJs9PeE2nRo1qSVkwCBNopkEexdc34rymK21pF0LlwU0Q2p1WZhSVAgEBVBQwrqnoy+iJAoHECxf5vRL7hcY3LNWGgGcvHXguVlj5rwqUWECBAoJYC3X3RvXxVLVvXdH8CafmLIjvh9/pbbBUBAgRqJlDs/WIU2y6N4sA3a9b5ANqddUp0zv3WADayBQECBAgMQsCwYhCK9iBAgEA/Aoc3R3fdffpZ2cg1aeHDxz/Afcz9GplPKAIE2itQ7PtS5Nc+ub0ALUiejvmRyM78ZAuSikiAQKsEjm4f+yVFseMdrYr9g2HTkidHdsobW5tfcAIECFRNwLCiaieiHwIEGi2QX/3QKA5e0eiMdxsudcYGFtmKSyLSrPY6SE6AQKME8ptfG8XW1zcqkzA/JJA60bnoJiwECBBojMDYK2q3rom4bUNjMk0lSG9Q0RtY+CNAgACBaggYVlTjHHRBgEBLBIrtb438xle1JO1dx0zz7h2p92qohd7/3frLAIBAAwTya58Yxb6vNCCJCHcnkJ312Ujz2vsLSbeDAIFmCBSHrhp/5dOujzQj0DRTjL0CatYp09zF4wQIECAwKAHDikFJ2ocAAQL9CBy9JbpXPzTiyM39rG78mnTc08e+ZxEzT2h8VgEJEGiuQPfysyO6u5sbULIxgezEP4i07Lk0CBAgUFuBYvvbxn9N0d1V2wyDbDwd99TITv7zQW5pLwIECBCYpoBhxTQBPU6AAIHJCuQ3vSaKbd6L+j23WSvHXguVlvzyZCmtJ0CAQOkCxcHLI7/6J0vvQwPDF0iLHxPZyncOv5AKBAgQGLBAsf8bUWxbE8W+fxnwzvXeLjv7C5Hm3qPeIXRPgACBhgkYVjTsQMUhQKAGAoeuHv91RXGkBs2OrsW0+LHjH+Cee8HoiqpEgACBaQoUt7wr8i0vn+YuHq+FwMzjo3Ped2rRqiYJECAwJpDfOvZLimLbG4D8kIBfVbgSBAgQqKaAYUU1z0VXBAg0XCC/4RVR7HhHw1NOIV5nQWS9gcXyF03hYY8QIEBg9AL5pudHsevDoy+sYikC3m1eCruiBAhMQaDY8+kotl4axUFD1v/Dl82P7MxP+pekpnCvPEKAAIFhCxhWDFvY/gQIELgzgaPbo7v+sRG3beBzJwJp/gMjLb8k0oIH8SFAgEClBbpX/qj/t7zSJzTY5rKVb4u0+AmD3dRuBAgQGKTA4S2R9175dMt7B7lro/bKTnhlpOUvblQmYQgQINAUAcOKppykHAQI1E6g2PX3kW/yC4K7O7i07Plj37OIzqLana+GCRBogcDRbdG9wqvrWnDS34uYlj4zspP+pE2RZSVAoEYCxS3viXzbpRGHt9So69G22vuXorJVHxttUdUIECBAoG8Bw4q+qSwkQIDA4AXy658bxe6PDn7jBu2Y5pwbacXFkRY/vkGpRCFAoAkCxZ7PRL7x6U2IIkOfAmnuRZGd/cU+V1tGgACB0QgUB/93/JVPez4zmoI1rpKd8aFICx5c4wRaJ0CAQLMFDCuafb7SESBQcYHi0JWR914H1d1d8U7Lby8tefLY9yxi9hnlN6MDAgQI9L5beuOro9j+FhYtE+hctDkizW5ZanEJEKiqQG9I0XvtU+QHq9piZfpKy54X2Ym/X5l+NEKAAAEC/1fAsMKtIECAQMkCxfa3Rn7jq0ruoiblZyyPrPcri6XPqknD2iRAoMkC+fqfjeLAN5scUbY7EchWfTTS/B9nQ4AAgVIFin1fHv81xYFvlNpHXYqnOavHPqodncV1aVmfBAgQaKWAYUUrj11oAgSqJpBf+8Qo9n2lam1Vtp+08OGRll8c6Zj7VbZHjREg0HyB7ndOiShua35QCe8gkB3/O5F631PyR4AAgTIEju4c+y5Fsf1tZVSvbc1s5dsiLX5CbfvXOAECBNoiYFjRlpOWkwCBSgsU+/8t8g0/V+keK9dc6kRafsnYLy0izapcexoiQKDZAsWB/4x8/aObHVK6OxVICx8W2envp0OAAIGRCxS7PjL+yqdDV4+8dp0LpmOfFNmpb6pzBL0TIECgNQKGFa05akEJEKi6QH7Ta6LY9saqt1m5/tK8e4/9G65p4SMr15uGCBBorkDvWxW9b1b4a6FAZ1F0LrimhcFFJkCgNIHb1ke+dU0Uuz5UWgu1LTxjWXR6r3+avaq2ETROgACBNgkYVrTptGUlQKDaAvn+yNc/LoqDl1W7z4p2l5Y+ffwD3DNPqGiH2iJAoEkC+canR7HnM02KJMskBDrnfC1iztmTeMJSAgQITE2g2PH2yLdeGnF0x9Q2aPlT2Ul/FGnps1uuID4BAgTqI2BYUZ+z0ikBAi0QKPZ8KvKNz2xB0iFFnHXa+Ae4l/zykArYlgABAuMC3bX3jDhyI46WCmSnrIm05CktTS82AQKjECgO/Mf4B7T3/fMoyjWyhtf2NfJYhSJAoOEChhUNP2DxCBCon0C++ZIodr6vfo1XqOO0+LFj37NIc8+vUFdaIUCgMQKHr4/uuvs1Jo4gkxfoDcWzUy6d/IOeIECAwEQC+W1j36XoDSoi8olW++d3KZBFduYnIx1zf0YECBAgUCMBw4oaHZZWCRBoicDhTdFd/1j/xu50j7uzYOy1UGn5i6a7k+cJECBwB4Fi14cj3/R8Km0WmH1WdFZ/vc0CshMgMASBYs9no9h2aRS3/s8Qdm/XlmnFSyI7/uXtCi0tAQIEGiBgWNGAQxSBAIHmCRS3vDvyLS9rXrASEqX5Px6p92qo+Q8qobqSBAg0USC/4eVR7HhXE6PJNAmBzgVXR3QWT+IJSwkQIHAXAkduHPsuRe9/B/A3fYE0715jv6qINHv6m9mBAAECBEYqYFgxUm7FCBAg0L9AvvHXovdvV/kbjEBa9oKx71lEZ9FgNrQLAQKtFcivflgUB7/T2vyCjwtkp78v0sKH4yBAgMC0BHqvf823rok4vGla+3j4+wLZae+JtOhRSAgQIECghgKGFTU8NC0TINAOgd7Pv/Pe66CK29oReAQp05xzx39lsfjxI6imBAECjRTID0b3spWNjCbU5ATS8osjO+F3J/eQ1QQIELhdoDh4+fgrn3Z/kskABdJxT4vs5D8b4I62IkCAAIFRChhWjFJbLQIECExSoNj6F5Hf/KeTfMryiQTSkiePfc8iZp8x0VL/nAABAncQKPZ/NfINv0CFQKT5PxbZqo+TIECAwKQFim1vGv81Rb5/0s964G4EZq2MzpmfiJh5IiYCBAgQqKmAYUVND07bBAi0RSAf+3VFceCbbQk8upwzV4x/gHvpM0dXUyUCBGovUGx9feQ3v7b2OQQYgECaGZ2LbhjARrYgQKAtAsW+r0axbU0U+7/elsgjzZmdsibSkqeMtKZiBAgQIDBYAcOKwXrajQABAgMXKPZ+MfLrfnng+9pwXKD3vvG0/JJIx9wXCQECBCYUyK99chT7vjThOgvaIZCd9flI8+7ZjrBSEiAwdYHunrFfUhTb3zL1PTx5twJp8WMiW/lOSgQIECBQcwHDipofoPYJEGiHQH7DK6LY8Y52hC0jZZoRY+8eX3FJRJpZRgdqEiBQE4Hu5edEdHfVpFttDlsgO+mPIi199rDL2J8AgRoLFLs/FsXWS6M4tK7GKSreejY/sjM/GWnuBRVvVHsECBAgMJGAYcVEQv45AQIEqiBwdHt0ex/bvm1DFbppbA9p3n3GP8C98JGNzSgYAQJTFygOXRn5VQ+a+gaebJxAWvxzka18e+NyCUSAwAAEbrs28t4HtHd+cACb2eLuBLITXhlp+YshESBAgEADBAwrGnCIIhAg0A6BYtc/RL7phe0IW3LKtPQZ4x/gnnl8yZ0oT4BAlQSKW94b+ZaXVKklvZQtMPOk6Jz3P2V3oT4BAhUTKHa8c2xQEUe2Vqyz5rWT5j8wslUfa14wiQgQINBSAcOKlh682AQI1FMg3/TcKHZ9tJ7N163rWadF1vuVxRLfC6nb0emXwLAE8s0v9m/IDgu3xvt2zvt2xMwTa5xA6wQIDEqguPW/xl/5tPfzg9rSPhMIZGd8KNKCB3MiQIAAgYYIGFY05CDFIECgHQJjryBZ/zjvSx/hcafFjxv7nkWae/4IqypFgEAVBbpXPjDitmuq2JqeShTIVr4jev9/hT8CBFosUByJvDek2LYmojjaYojRRk/Lnh/Zia8ebVHVCBAgQGCoAoYVQ+W1OQECBAYvUGx/a+Q3vmrwG9vxrgU6CyJbfkmk5V7D5ZoQaK3A0Z3RvWJ1a+MLftcCaemvR3bSaxARINBSgd6vKIqta6K49VstFSgndpqzeuyj2tFZXE4DqhIgQIDAUAQMK4bCalMCBAgMVyC/9olR7PvKcIvY/f8IpPk/HmnFJZHm/wQdAgRaJlDs/afIr/vVlqUWtx+BNO9ekZ31T/0stYYAgSYJHNk6/gHtHe9sUqraZMlWvi3S4ifUpl+NEiBAgEB/AoYV/TlZRYAAgUoJFPv/LfINP1epntrUTFr+gvEPcHcWtSm2rARaLZDf9Jootr2x1QbC37VA56IbI9IMRAQItESg2PnByHuvfLrtupYkrlbMdOyTIjv1TdVqSjcECBAgMBABw4qBMNqEAAECoxfwfzgbvfkPVkxzzovU+wD3YkOjck9CdQKjEcg3PC6K/d8YTTFVaieQnfmJSMf8aO361jABApMTKA6tG3/l0+6PT+5BqwcnMGNZdHqvf5q9anB72okAAQIEKiNgWFGZo9AIAQIEJimQH4h8/WOjOHjZJB+0fJACacmTx75nEbNPH+S29iJAoGIC3ctOjcgPVawr7VRFIDvhlZGWv7gq7eiDAIEhCBTb3zz2Ee3o7hnC7rbsVyA76Y8jLX1Wv8utI0CAAIGaCRhW1OzAtEuAAIEfFCj2fCryjc+EUrbAzBVjr4VKS51F2UehPoFhCBS3/k/k1zxyGFvbsyECaeEjIzv9vQ1JIwYBAnf479v7vxbF1kuj2P9VMCULpIUPi+z095fchfIECBAgMEwBw4ph6tqbAAECIxDIN/9mFDv/bgSVlJhIIC18xPiroebdd6Kl/jkBAjUSKHa8PfIbXlmjjrU6coEZS6Jz/pUjL6sgAQJDFOjuG/+A9jbfRhiicv9bp05kq3qv3Lt//89YSYAAAQK1EzCsqN2RaZgAAQI/JHB4U3TXPzbiyI1oqiCQZkRafklkKy6OSDOr0JEeCBCYpkB+/bOj2P2Jae7i8aYLdFZ/wzvUm37I8rVGoNj9ySh6g4qDl7cmc9WDphUviez4l1e9Tf0RIECAwDQFDCumCehxAgQIVEGguOXdkW95WRVa0cPtAmnefcZ/ZbHQq2NcCgJ1F+iuu3fE4S11j6H/IQtkp7wxet8x8keAQI0FDl8/9l2KYuf7ahyiea2nefeKrPdR7TS7eeEkIkCAAIE7CBhWuBAECBBoiEC+8dei2PPZhqRpToy09Blj37OImcc3J5QkBNokcOSG6K69V5sSyzpFgXTcr0Z28uun+LTHCBAoW2DsX/7ZuibiyE1lt6L+Dwlkp70n0qJHcSFAgACBFggYVrTgkEUkQKAdAmMfgO29Dqq4rR2B65Ry9mmRLb8k0pJfqlPXeiVAICKK3R+P/PpfZ0FgQoE0Z3Vk5/gA74RQFhComEDvv0MX29ZEsedzFetMOz2BdNzTIjv5z2AQIECAQEsEDCtactBiEiDQDoFi619EfvOftiNsDVOmxY8b+55FmnteDbvXMoF2CuQ3vCKKHe9oZ3ipJy3QuWB9RGfhpJ/zAAECZQjkUWy9dPzXFP5lnzIOYOKas1ZGp/f6p5knTLzWCgIECBBohIBhRSOOUQgCBAh8VyAf+3VFceCbSKoq0Fk49lqotPyFVe1QXwQI/IBAfs0jorj120wI9CWQnfGBSAt+qq+1FhEgUJ5Asfefx39N4b8zl3cIfVTOTlkTaclT+lhpyf9n707g6yrr/I9/n+dm6ZKmaZom3ds0BVl0HAHHGRGdUVHHcVIYlW0c2Zy/yqKCoM6I4IyyjUqLu4Aby4gCLnVhxoVFRxHHUQEBgbSUNl2SJk3btE2a5D7P/3XuLVhLS+9Nzj33LJ/7evVVkHOe3+/7fu4LS38950EAAQQQSIsAw4q07CQ5EEAAgT0CwX98uSd53VDcvxCm4bjiAdwNx8W9VfpDILsCflT5B+dlNz/JyxYwbRfKzv5A2fdxAwIIRCQw1ld4koIn5iLynkAZ09Qpu+iGCazArQgggAACSRRgWJHEXaNnBBBA4CACvLYkOV8R03pu4TwLXhuSnD2j0+wI+B2/kFt1QnYCk3TCAsEA2nbcMeF1WAABBMIX8APfkOtZIe3uCn9xVgxXwDbILl0pM/n54a7LaggggAACsRdgWBH7LaJBBBBAYBwCY5uV71rGf4yNg64at5hJRxSfsmjiN0Wr4U9NBA4k4Huvldt4OUAIlC5gJyn3grWlX8+VCCBQeYHhx+R6V8gPMEisPHY4FeycS2Ra3xXOYqyCAAIIIJAoAYYVidoumkUAAQRKFyj86bG1nItQulj1rzTNpxbOs1B9e/WboQMEEJB78i3y23+IBAJlCdhDfyIz+QVl3cPFCCBQGQG/+fOFQYXGtlSmAKuGLmAajpXt+Fbo67IgAggggEAyBBhWJGOf6BIBBBAYl4Bb+w75gW+O615uqpJAbVvhtVCm5awqNUBZBBB4WiD/8OHSWD8gCJQlYOddxb/DyxLjYgTCF/A775PvWSE/eHf4i7NiRQXskttkpr2iojVYHAEEEEAgvgIMK+K7N3SGAAIITFjAB4+9d3VK+YEJr8UC0QqYxtcUXw015ZhoC1MNAQSKAru7lP/DS9FAoGwBM+ONsgs/V/Z93IAAAiEIuF2Fcyl88DQFn8QJmFnnyM79cOL6pmEEEEAAgfAEGFaEZ8lKCCCAQCwF/ObPyW24LJa90dRBBExt4bVQwdBCphYuBBCIUMBv+ZrcundHWJFSqRGoW6jc4b9OTRyCIJAUAb/t+/I9y+WHHkxKy/S5l4CZdFjhUG3lmnBBAAEEEMiwAMOKDG8+0RFAIDsCbvVJ8oP3ZCdwypKaKUfLtF2g4GkLPgggEI2A675Qvv/maIpRJXUCuSMekmrbUpeLQAjEUmCkW653uXz/TbFsj6ZKE7CLviDTdGJpF3MVAggggEBqBRhWpHZrCYYAAgj8UcDv+IXcqhMgSbhAcI5FcJ4FvwGW8I2k/UQIuMeOU/AqPT4IjEfALv6SzPQ3jOdW7kEAgTIEggFFMKjQSHcZd3Fp3ATMjJNl"
                 +
                "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"
                 +
                "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"
                 +
                "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"
                 +
                "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AgQIECBAgAABAmkLKCvSfj7SESBAgAABAgQIEBiIwP7vrxgrLIb443sqhojtKgIECBAgQIAAAQINE1BWNOyBiUuAAAECBAgQIECgLoFy5yVRrHlUXeMOOcdvVAyF2SUECBAgQIAAAQIEGiugrGjsoxOcAAECBAgQIECAQA0Cxa4o1r8syts/V8Owuxkxfm7s/46KKfcdzHxTCRAgQIAAAQIECBBohYCyohWP0RIECBAgQIAAAQIEjk2guOUNUW5827EN+ZnT2QnPiHzen0f0jqt1rmEECBAgQIAAAQIECLRPQFnRvmdqIwIECBAgQIAAAQKVBMpdl0a5+UNRbr6o0vmfHMqO+8XIZjw+shOefkxzHCZAgAABAgQIECBAoDsCyoruPGubEiBAgAABAgQIEDgigf2lxdZPRuy4OMpdK4/oTExYENnxjz9QUky595Gd8SkCBAgQIECAAAECBAj8WEBZ4VUgQIAAAQIECBAgQODgAnvXR7njO1HuWRPRvyOif/uB//RmRow/JbIJp0RMWBjZtIdE5FNIEiBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgDKhUE8AAAHiSURBVAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJKAsqISm0MECBAgQIAAAQIECBAgQIAAAQIECBAgQIBAXQLKirokzSFAgAABAgQIECBAgAABAgQIECBAgAABAgQqCSgrKrE5RIAAAQIECBAgQIAAAQIECBAgQIAAAQIECNQloKyoS9IcAgQIECBAgAABAgQIECBAgAABAgQIECBAoJLA/wMOKbj7oal1JQAAAABJRU5ErkJggg==",
            fileName=
                "modelica://MicroGrid/../../../../Downloads/autodraw 4_30_2021 (9).png"),
          Text(
            extent={{-130,140},{134,98}},
            lineColor={28,108,200},
            textString="%name")}));
  end Irradiance_to_Power;
end GridFollowing;
