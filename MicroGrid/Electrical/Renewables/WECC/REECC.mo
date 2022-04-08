within MicroGrid.Electrical.Renewables.WECC;
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
