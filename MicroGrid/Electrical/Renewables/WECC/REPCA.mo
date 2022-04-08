within MicroGrid.Electrical.Renewables.WECC;
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
