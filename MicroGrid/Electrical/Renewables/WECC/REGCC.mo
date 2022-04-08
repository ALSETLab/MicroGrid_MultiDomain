within MicroGrid.Electrical.Renewables.WECC;
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

ramp rate (True)." annotation (Dialog(tab="Controls"));
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
