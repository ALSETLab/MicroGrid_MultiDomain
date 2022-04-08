within MicroGrid.Electrical.Renewables.WECC;
model GFM_REEC
  "Generic grid forming renewable electrical control model."
extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enablefn=false,
    final enableV_b=false,
    final enableS_b=false);

parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "IBR rating" annotation (Dialog(group= "Input Parameters"));
parameter Real wdrp = 0.033 "Frequeny droop percent" annotation(Dialog(group= "Input Parameters"));
parameter Real Qdrp = 0.045 "Voltage droop percent" annotation(Dialog(group= "Input Parameters"));
parameter Real KPv = if GFM == 0 then 0.5 else 3.0 "Voltage control proportional gain" annotation(Dialog(group= "Input Parameters"));
parameter Real KIp = 20.0 "Active power integral gain" annotation(Dialog(group= "Input Parameters"));
parameter Real KPp = 0.5  "Active power proportional gain" annotation(Dialog(group= "Input Parameters"));
parameter Real KIv = if GFM == 0 then 150 else 10  "Voltage control integral gain" annotation(Dialog(group= "Input Parameters"));

parameter Integer GFM = 0 "Grid Forming Method: (0) PLL, (1) Droop, (2) VSM, (3) DVOC" annotation (choices(choice=0, choice=1, choice=2, choice=3));
  Modelica.Blocks.Interfaces.RealInput Paux annotation (Placement(
        transformation(extent={{-220,-50},{-200,-30}}), iconTransformation(
          extent={{-220,-50},{-200,-30}})));
  Modelica.Blocks.Interfaces.RealInput vref_inv_d annotation (Placement(
        transformation(extent={{-220,-140},{-200,-120}}), iconTransformation(
          extent={{-220,-140},{-200,-120}})));
  Modelica.Blocks.Interfaces.RealInput p annotation (Placement(transformation(
          extent={{-220,-110},{-200,-90}}),iconTransformation(extent={{-220,
            -110},{-200,-90}})));
  Modelica.Blocks.Interfaces.RealInput dwinv annotation (Placement(
        transformation(extent={{-220,-80},{-200,-60}}),  iconTransformation(
          extent={{-220,-80},{-200,-60}})));
  Modelica.Blocks.Interfaces.RealInput v_inv_d annotation (Placement(
        transformation(extent={{-220,-170},{-200,-150}}), iconTransformation(
          extent={{-220,-170},{-200,-150}})));
  Modelica.Blocks.Interfaces.RealInput v_inv_q annotation (Placement(
        transformation(extent={{-220,40},{-200,60}}), iconTransformation(extent={{-220,30},
            {-200,50}})));
  Modelica.Blocks.Interfaces.RealInput q annotation (Placement(transformation(
          extent={{-220,90},{-200,110}}), iconTransformation(extent={{-220,90},
            {-200,110}})));
  Modelica.Blocks.Interfaces.RealInput Qaux annotation (Placement(
        transformation(extent={{-220,140},{-200,160}}), iconTransformation(
          extent={{-220,150},{-200,170}})));
  Modelica.Blocks.Math.Add add(k2=-1)
    annotation (Placement(transformation(extent={{-140,120},{-120,140}})));
  Modelica.Blocks.Math.Gain QDRP
    annotation (Placement(transformation(extent={{-106,120},{-86,140}})));
  Modelica.Blocks.Math.Add add1(k2=+1)
    annotation (Placement(transformation(extent={{-70,126},{-50,146}})));
  Modelica.Blocks.Math.Add add2(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{-70,96},{-50,76}})));
  Modelica.Blocks.Math.Add add3(k2=+1)
    annotation (Placement(transformation(extent={{-42,100},{-22,120}})));
  Modelica.Blocks.Sources.RealExpression VREF
    annotation (Placement(transformation(extent={{-140,82},{-120,102}})));
  Modelica.Blocks.Sources.RealExpression QREF
    annotation (Placement(transformation(extent={{-180,126},{-160,146}})));
  Modelica.Blocks.Math.Gain gain
    annotation (Placement(transformation(extent={{0,118},{20,138}})));
  Modelica.Blocks.Continuous.Integrator integrator
    annotation (Placement(transformation(extent={{0,84},{20,104}})));
  Modelica.Blocks.Math.Add add4(k2=+1)
    annotation (Placement(transformation(extent={{42,102},{62,122}})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{-140,-116},{-120,-96}})));
  Modelica.Blocks.Sources.RealExpression WDRP
    annotation (Placement(transformation(extent={{-180,-102},{-160,-122}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{84,64},{104,84}})));
  Modelica.Blocks.Sources.BooleanExpression WFLAG
    annotation (Placement(transformation(extent={{46,64},{66,84}})));
  Modelica.Blocks.Nonlinear.Limiter limiter
    annotation (Placement(transformation(extent={{128,64},{148,84}})));
  Modelica.Blocks.Interfaces.RealOutput Iqref "Connector of Real output signal"
    annotation (Placement(transformation(extent={{200,60},{220,80}})));
  Modelica.Blocks.Sources.RealExpression VINV_D(y=v_inv_d)
    annotation (Placement(transformation(extent={{-140,90},{-120,70}})));
  Modelica.Blocks.Math.Gain NEG_ONE(k=-1)
    annotation (Placement(transformation(extent={{-106,30},{-86,50}})));
  Modelica.Blocks.Math.Gain gain1
    annotation (Placement(transformation(extent={{0,46},{20,66}})));
  Modelica.Blocks.Continuous.Integrator integrator1
    annotation (Placement(transformation(extent={{0,14},{20,34}})));
  Modelica.Blocks.Math.Add add5(k2=+1)
    annotation (Placement(transformation(extent={{42,30},{62,50}})));
  Modelica.Blocks.Math.Add add6(k2=-1)
    annotation (Placement(transformation(extent={{-140,-156},{-120,-136}})));
  Modelica.Blocks.Math.Gain gain2
    annotation (Placement(transformation(extent={{-94,-140},{-74,-120}})));
  Modelica.Blocks.Continuous.Integrator integrator2
    annotation (Placement(transformation(extent={{-94,-176},{-74,-156}})));
  Modelica.Blocks.Math.Add add7(k2=+1)
    annotation (Placement(transformation(extent={{-60,-158},{-40,-138}})));
  Modelica.Blocks.Logical.Switch switch2
    annotation (Placement(transformation(extent={{84,-108},{104,-88}})));
  Modelica.Blocks.Sources.BooleanExpression WFLAG1
    annotation (Placement(transformation(extent={{46,-108},{66,-88}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1
    annotation (Placement(transformation(extent={{128,-108},{148,-88}})));
  Modelica.Blocks.Math.Add add8(k2=+1)
    annotation (Placement(transformation(extent={{-100,-44},{-80,-24}})));
  Modelica.Blocks.Math.Add add9(k1=-1, k2=-1)
    annotation (Placement(transformation(extent={{-100,-86},{-80,-66}})));
  Modelica.Blocks.Sources.RealExpression PREF
    annotation (Placement(transformation(extent={{-180,-38},{-160,-18}})));
  Modelica.Blocks.Math.Add add10(k2=+1)
    annotation (Placement(transformation(extent={{-60,-66},{-40,-46}})));
  Modelica.Blocks.Math.Gain gain3
    annotation (Placement(transformation(extent={{0,-48},{20,-28}})));
  Modelica.Blocks.Continuous.Integrator integrator3
    annotation (Placement(transformation(extent={{0,-84},{20,-64}})));
  Modelica.Blocks.Math.Add add11(k2=+1)
    annotation (Placement(transformation(extent={{34,-66},{54,-46}})));
  Modelica.Blocks.Interfaces.RealOutput Idref
    "Connector of Real output signal" annotation (Placement(
        transformation(extent={{200,-80},{220,-60}})));
equation
  connect(add.y, QDRP.u)
    annotation (Line(points={{-119,130},{-108,130}}, color={0,0,127}));
  connect(add1.y, add3.u1) annotation (Line(points={{-49,136},{-46,136},{-46,116},
          {-44,116}}, color={0,0,127}));
  connect(QDRP.y, add1.u2)
    annotation (Line(points={{-85,130},{-72,130}}, color={0,0,127}));
  connect(VREF.y, add2.u2)
    annotation (Line(points={{-119,92},{-72,92}}, color={0,0,127}));
  connect(QREF.y, add.u1)
    annotation (Line(points={{-159,136},{-142,136}}, color={0,0,127}));
  connect(gain.y, add4.u1) annotation (Line(points={{21,128},{38,128},{38,118},{
          40,118}}, color={0,0,127}));
  connect(integrator.y, add4.u2) annotation (Line(points={{21,94},{38,94},{38,106},
          {40,106}}, color={0,0,127}));
  connect(WDRP.y, division.u2)
    annotation (Line(points={{-159,-112},{-142,-112}}, color={0,0,127}));
  connect(integrator.u, add3.y) annotation (Line(points={{-2,94},{-12,94},{-12,110},
          {-21,110}}, color={0,0,127}));
  connect(gain.u, add3.y) annotation (Line(points={{-2,128},{-12,128},{-12,110},
          {-21,110}}, color={0,0,127}));
  connect(add2.y, add3.u2) annotation (Line(points={{-49,86},{-46,86},{-46,104},
          {-44,104}}, color={0,0,127}));
  connect(add4.y, switch1.u1) annotation (Line(points={{63,112},{72,112},{72,82},
          {82,82}}, color={0,0,127}));
  connect(WFLAG.y, switch1.u2)
    annotation (Line(points={{67,74},{82,74}}, color={255,0,255}));
  connect(q, add.u2) annotation (Line(points={{-210,100},{-176,100},{-176,124},{
          -142,124}}, color={0,0,127}));
  connect(Qaux, add1.u1) annotation (Line(points={{-210,150},{-80,150},{-80,142},
          {-72,142}}, color={0,0,127}));
  connect(switch1.y, limiter.u)
    annotation (Line(points={{105,74},{126,74}}, color={0,0,127}));
  connect(limiter.y, Iqref) annotation (Line(points={{149,74},{180,74},{180,70},
          {210,70}}, color={0,0,127}));
  connect(VINV_D.y, add2.u1)
    annotation (Line(points={{-119,80},{-72,80}}, color={0,0,127}));
  connect(v_inv_q, NEG_ONE.u) annotation (Line(points={{-210,50},{-160,50},{-160,
          40},{-108,40}}, color={0,0,127}));
  connect(gain1.y, add5.u1) annotation (Line(points={{21,56},{38,56},{38,46},{40,
          46}}, color={0,0,127}));
  connect(integrator1.y, add5.u2) annotation (Line(points={{21,24},{38,24},{38,34},
          {40,34}}, color={0,0,127}));
  connect(NEG_ONE.y, gain1.u) annotation (Line(points={{-85,40},{-20,40},{-20,56},
          {-2,56}}, color={0,0,127}));
  connect(integrator1.u, gain1.u) annotation (Line(points={{-2,24},{-20,24},{-20,
          56},{-2,56}}, color={0,0,127}));
  connect(add5.y, switch1.u3) annotation (Line(points={{63,40},{72,40},{72,66},{
          82,66}}, color={0,0,127}));
  connect(vref_inv_d, add6.u1) annotation (Line(points={{-210,-130},{-150,-130},
          {-150,-140},{-142,-140}}, color={0,0,127}));
  connect(v_inv_d, add6.u2) annotation (Line(points={{-210,-160},{-150,-160},{-150,
          -152},{-142,-152}}, color={0,0,127}));
  connect(add6.y, gain2.u) annotation (Line(points={{-119,-146},{-108,-146},{-108,
          -130},{-96,-130}}, color={0,0,127}));
  connect(integrator2.u, gain2.u) annotation (Line(points={{-96,-166},{-108,-166},
          {-108,-130},{-96,-130}}, color={0,0,127}));
  connect(gain2.y, add7.u1) annotation (Line(points={{-73,-130},{-62,-130},{-62,
          -142}}, color={0,0,127}));
  connect(integrator2.y, add7.u2) annotation (Line(points={{-73,-166},{-62,-166},
          {-62,-154}}, color={0,0,127}));
  connect(WFLAG1.y, switch2.u2)
    annotation (Line(points={{67,-98},{82,-98}}, color={255,0,255}));
  connect(switch2.y, limiter1.u)
    annotation (Line(points={{105,-98},{126,-98}}, color={0,0,127}));
  connect(add7.y, switch2.u3) annotation (Line(points={{-39,-148},{70,-148},{70,
          -106},{82,-106}}, color={0,0,127}));
  connect(dwinv, division.u1)
    annotation (Line(points={{-210,-70},{-176,-70},{-176,-100},{-142,
          -100}},                                      color={0,0,127}));
  connect(division.y, add9.u2) annotation (Line(points={{-119,-106},{-102,-106},
          {-102,-82}}, color={0,0,127}));
  connect(p, add9.u1)
    annotation (Line(points={{-210,-100},{-156,-100},{-156,-70},{-102,
          -70}},                                     color={0,0,127}));
  connect(Paux, add8.u2)
    annotation (Line(points={{-210,-40},{-156,-40},{-156,-40},{-102,-40}},
                                                     color={0,0,127}));
  connect(PREF.y, add8.u1)
    annotation (Line(points={{-159,-28},{-102,-28}}, color={0,0,127}));
  connect(add8.y, add10.u1)
    annotation (Line(points={{-79,-34},{-62,-34},{-62,-50}}, color={0,0,127}));
  connect(add9.y, add10.u2)
    annotation (Line(points={{-79,-76},{-62,-76},{-62,-62}}, color={0,0,127}));
  connect(gain3.y, add11.u1)
    annotation (Line(points={{21,-38},{32,-38},{32,-50}}, color={0,0,127}));
  connect(integrator3.y, add11.u2)
    annotation (Line(points={{21,-74},{32,-74},{32,-62}}, color={0,0,127}));
  connect(add10.y, gain3.u) annotation (Line(points={{-39,-56},{-20,-56},{-20,-38},
          {-2,-38}}, color={0,0,127}));
  connect(integrator3.u, gain3.u) annotation (Line(points={{-2,-74},{-20,-74},{-20,
          -38},{-2,-38}}, color={0,0,127}));
  connect(add11.y, switch2.u1) annotation (Line(points={{55,-56},{70,-56},{70,-90},
          {82,-90}}, color={0,0,127}));
  connect(limiter1.y, Idref) annotation (Line(points={{149,-98},{192,-98},
          {192,-70},{210,-70}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},
            {200,200}}), graphics={Rectangle(extent={{-200,200},{200,
              -200}}, lineColor={28,108,200}), Text(
          extent={{-180,40},{160,-40}},
          lineColor={28,108,200},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},{200,200}})));
end GFM_REEC;
