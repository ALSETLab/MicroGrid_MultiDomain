within MicroGrid.Electrical.Renewables.WECC;
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
