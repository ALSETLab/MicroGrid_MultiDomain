within MicroGrid.Electrical.Renewables.PNNL.GridForming;
model GFMDRP_A_MPC
  "This model is a positive-sequence phasor model of a droop-controlled, grid-forming (GFM) inverter-based resources (IBRs)."
parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "Inverter base power" annotation (Dialog(group= "Power flow data"));
extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enablefn=false,
    final enableV_b=false,
    final enableS_b=false);

parameter OpenIPSL.Types.PerUnit mq =  0.05    "Q - V droop gain" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Vset = v_0    "Voltage Set Point" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Kpv = 0       "Proportional gain of the voltage controller" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Kiv = 5.86    "Integral gain of the Voltage Controller" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Emax = 1.2    "Upper Limit of the output of the voltage loop" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Emin = 0      "Lower Limit of the output of the voltage loop" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit mp =  0.05    "P - f droop gain" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Pset = p0     "Power Set Point" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Pmax = 1      "Upper Limit of the inverter output power" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Pmin = 0      "Lower Limit of the inverter output power" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Kppmax = 0.01 "Porportional gain of the overload mitigation controller" annotation (Dialog(tab="Parameters"));
parameter OpenIPSL.Types.PerUnit Kipmax = 0.1  "Integral gain of the overload mitigation controller" annotation (Dialog(tab="Parameters"));
parameter Real Tf = 0.01                       "Time constant of the low-pass filter for P, Q, and V measurements" annotation (Dialog(tab="Parameters"));
//parameter Boolean Pset_free_= false "If true, then Pset is free in the optimization"  annotation (Dialog(group="Design",tab="Optimization"));

  Modelica.Blocks.Interfaces.RealInput P_inv annotation (Placement(
        transformation(extent={{-160,90},{-140,110}}), iconTransformation(
          extent={{-160,90},{-140,110}})));
  Modelica.Blocks.Interfaces.RealInput Q_inv annotation (Placement(
        transformation(extent={{-160,-10},{-140,10}}), iconTransformation(
          extent={{-160,-10},{-140,10}})));
  Modelica.Blocks.Interfaces.RealInput V_inv annotation (Placement(
        transformation(extent={{-160,-110},{-140,-90}}), iconTransformation(
          extent={{-160,-110},{-140,-90}})));
  Modelica.Blocks.Math.Add add(k1=-1)
    annotation (Placement(transformation(extent={{-100,102},{-80,122}})));
  Modelica.Blocks.Math.Add3 add3_1(k2=-1, k3=-1)
    annotation (Placement(transformation(extent={{-32,-88},{-12,-68}})));
  Modelica.Blocks.Continuous.Integrator integrator(y_start=angle_droop_0)
    annotation (Placement(transformation(extent={{104,96},{124,116}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(K=1, T=Tf,
    y_start=q0)
    annotation (Placement(transformation(extent={{-130,-66},{-110,-46}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag1(K=1, T=Tf,
    y_start=v_0)
    annotation (Placement(transformation(extent={{-130,-110},{-110,-90}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag2(K=1, T=Tf,
    y_start=p0)
    annotation (Placement(transformation(extent={{-130,108},{-110,128}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{40,96},{60,116}})));
  Modelica.Blocks.Math.Gain gain1(k=w0)
    annotation (Placement(transformation(extent={{70,96},{90,116}})));
  Modelica.Blocks.Interfaces.RealOutput theta_droop(start = angle_droop_0)
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{140,74},{160,94}}),
        iconTransformation(extent={{140,74},{160,94}})));
  Modelica.Blocks.Sources.RealExpression PMAX(y=Pmax)
    annotation (Placement(transformation(extent={{-130,68},{-110,88}})));
  Modelica.Blocks.Sources.RealExpression PMIN(y=Pmin)
    annotation (Placement(transformation(extent={{-130,22},{-110,42}})));
  Modelica.Blocks.Math.Add add2(k2=-1)
    annotation (Placement(transformation(extent={{-90,62},{-70,82}})));
  Modelica.Blocks.Math.Gain gain2(k=Kppmax)
    annotation (Placement(transformation(extent={{-56,72},{-36,92}})));
  Modelica.Blocks.Math.Add add3
    annotation (Placement(transformation(extent={{8,56},{28,76}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=0, uMin=-Modelica.Constants.eps)
    annotation (Placement(transformation(extent={{42,56},{62,76}})));
  Modelica.Blocks.Math.Add add4(k1=-1)
    annotation (Placement(transformation(extent={{-90,-12},{-70,8}})));
  Modelica.Blocks.Math.Gain gain3(k=mq)
    annotation (Placement(transformation(extent={{-98,-66},{-78,-46}})));
  Modelica.Blocks.Sources.RealExpression VSET(y=Vset + q0*mq)
    annotation (Placement(transformation(extent={{-62,-80},{-42,-60}})));
  Modelica.Blocks.Math.Gain gain4(k=Kpv)
    annotation (Placement(transformation(extent={{6,-78},{26,-58}})));
  Modelica.Blocks.Math.Add add5
    annotation (Placement(transformation(extent={{64,-84},{84,-64}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Emax, uMin=Emin)
    annotation (Placement(transformation(extent={{96,-84},{116,-64}})));
  Modelica.Blocks.Interfaces.RealOutput Edroop(start = e_droop_0)
    "Connector of Real output signal" annotation (Placement(transformation(
          extent={{140,-94},{160,-74}}), iconTransformation(extent={{140,-94},{160,
            -74}})));
  Modelica.Blocks.Math.Gain gain5(k=Kppmax)
    annotation (Placement(transformation(extent={{-52,-44},{-32,-24}})));
  Modelica.Blocks.Math.Add add6
    annotation (Placement(transformation(extent={{6,-18},{26,2}})));
  Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=Modelica.Constants.eps, uMin=0)
    annotation (Placement(transformation(extent={{40,-18},{60,2}})));
  Modelica.Blocks.Math.Add add7
    annotation (Placement(transformation(extent={{74,24},{94,44}})));
  Modelica.Blocks.Interfaces.RealOutput w
    annotation (Placement(transformation(extent={{140,30},{160,50}})));
  Modelica.Blocks.Sources.RealExpression W0(y=w0)
    annotation (Placement(transformation(extent={{78,-16},{98,4}})));
  Modelica.Blocks.Math.Add add8
    annotation (Placement(transformation(extent={{108,-10},{128,10}})));
  Modelica.Blocks.Interfaces.RealInput EDROOP_0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-90,-150}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-90,-150})));
  Modelica.Blocks.Interfaces.RealInput THETADROOP_0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,-150}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,-150})));
  Modelica.Blocks.Math.Gain gain6(k=1/(2*Modelica.Constants.pi))
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
  Modelica.Blocks.Interfaces.RealOutput freq "Output signal connector"
    annotation (Placement(transformation(extent={{140,-50},{160,-30}})));
  Modelica.Blocks.Math.Gain gain7(k=mp)
    annotation (Placement(transformation(extent={{-34,102},{-14,122}})));
  Modelica.Blocks.Interfaces.RealInput u1
               "Connector of Real input signal 2" annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,160})));
protected
parameter OpenIPSL.Types.PerUnit w0 =  2*Modelica.Constants.pi*fn  "Rated angular frequency";
parameter OpenIPSL.Types.Angle angle_droop_0(fixed = false)        "Initialization value for droop angle";
parameter OpenIPSL.Types.PerUnit e_droop_0(fixed = false)          "Initialization value for droop voltage magnitude";
parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0)              "Initial real voltage";
parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0)              "Initial imaginary voltage";
parameter OpenIPSL.Types.PerUnit p0=P_0/M_b                        "Initial active power (machine base)";
parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b                        "Initial reactive power (machine base)";

  Modelica.Blocks.Continuous.Integrator integrator1(k=Kiv, y_start=e_droop_0)
    annotation (Placement(transformation(extent={{8,-120},{28,-100}})));
  Modelica.Blocks.Nonlinear.Limiter limiter3(uMax=Emax, uMin=Emin)
    annotation (Placement(transformation(extent={{40,-120},{60,-100}})));
  Modelica.Blocks.Continuous.Integrator integrator2(k=Kipmax, y_start=0)
    annotation (Placement(transformation(extent={{-58,-4},{-38,16}})));
  Modelica.Blocks.Nonlinear.Limiter limiter4(uMax=Modelica.Constants.eps, uMin=0)
    annotation (Placement(transformation(extent={{-26,-4},{-6,16}})));
  Modelica.Blocks.Continuous.Integrator integrator3(k=Kipmax, y_start=0)
    annotation (Placement(transformation(extent={{-56,32},{-36,52}})));
  Modelica.Blocks.Nonlinear.Limiter limiter5(uMax=0, uMin=-Modelica.Constants.eps)
    annotation (Placement(transformation(extent={{-24,32},{-4,52}})));
initial equation
angle_droop_0 = THETADROOP_0;
e_droop_0 = EDROOP_0;

equation
  connect(P_inv, simpleLag2.u) annotation (Line(points={{-150,100},{-136,100},{-136,
          118},{-132,118}}, color={0,0,127}));
  connect(simpleLag2.y, add.u1)
    annotation (Line(points={{-109,118},{-102,118}}, color={0,0,127}));
  connect(add1.y, gain1.u)
    annotation (Line(points={{61,106},{68,106}},
                                               color={0,0,127}));
  connect(gain1.y, integrator.u)
    annotation (Line(points={{91,106},{102,106}},
                                                color={0,0,127}));
  connect(integrator.y, theta_droop) annotation (Line(points={{125,106},{136,106},
          {136,84},{150,84}}, color={0,0,127}));
  connect(PMAX.y, add2.u1)
    annotation (Line(points={{-109,78},{-92,78}}, color={0,0,127}));
  connect(add2.u2, simpleLag2.u) annotation (Line(points={{-92,66},{-136,66},{-136,
          118},{-132,118}}, color={0,0,127}));
  connect(add2.y, gain2.u) annotation (Line(points={{-69,72},{-62,72},{-62,82},{
          -58,82}}, color={0,0,127}));
  connect(gain2.y, add3.u1)
    annotation (Line(points={{-35,82},{6,82},{6,72}}, color={0,0,127}));
  connect(add3.y, limiter.u)
    annotation (Line(points={{29,66},{40,66}}, color={0,0,127}));
  connect(Q_inv, simpleLag.u) annotation (Line(points={{-150,0},{-136,0},{-136,-56},
          {-132,-56}}, color={0,0,127}));
  connect(V_inv, simpleLag1.u) annotation (Line(points={{-150,-100},{-142,-100},
          {-142,-100},{-132,-100}}, color={0,0,127}));
  connect(PMIN.y, add4.u2) annotation (Line(points={{-109,32},{-102,32},{-102,-8},
          {-92,-8}}, color={0,0,127}));
  connect(add4.u1, simpleLag2.u) annotation (Line(points={{-92,4},{-96,4},{-96,44},
          {-136,44},{-136,118},{-132,118}}, color={0,0,127}));
  connect(simpleLag.y, gain3.u)
    annotation (Line(points={{-109,-56},{-100,-56}}, color={0,0,127}));
  connect(gain3.y, add3_1.u2) annotation (Line(points={{-77,-56},{-68,-56},{-68,
          -78},{-34,-78}}, color={0,0,127}));
  connect(simpleLag1.y, add3_1.u3) annotation (Line(points={{-109,-100},{-68,-100},
          {-68,-86},{-34,-86}}, color={0,0,127}));
  connect(VSET.y, add3_1.u1)
    annotation (Line(points={{-41,-70},{-34,-70}}, color={0,0,127}));
  connect(add3_1.y, gain4.u) annotation (Line(points={{-11,-78},{-4,-78},{-4,-68},
          {4,-68}}, color={0,0,127}));
  connect(gain4.y, add5.u1)
    annotation (Line(points={{27,-68},{62,-68}}, color={0,0,127}));
  connect(add5.y, limiter1.u)
    annotation (Line(points={{85,-74},{94,-74}}, color={0,0,127}));
  connect(limiter1.y, Edroop) annotation (Line(points={{117,-74},{136,-74},{136,
          -84},{150,-84}}, color={0,0,127}));
  connect(gain5.y, add6.u2)
    annotation (Line(points={{-31,-34},{4,-34},{4,-14}}, color={0,0,127}));
  connect(add6.y, limiter2.u)
    annotation (Line(points={{27,-8},{38,-8}}, color={0,0,127}));
  connect(limiter.y, add7.u1)
    annotation (Line(points={{63,66},{72,66},{72,40}}, color={0,0,127}));
  connect(limiter2.y, add7.u2)
    annotation (Line(points={{61,-8},{72,-8},{72,28}}, color={0,0,127}));
  connect(add7.y, add1.u2) annotation (Line(points={{95,34},{100,34},{100,86},{24,
          86},{24,100},{38,100}}, color={0,0,127}));
  connect(add8.y, w)
    annotation (Line(points={{129,0},{136,0},{136,40},{150,40}},
                                               color={0,0,127}));
  connect(W0.y, add8.u2)
    annotation (Line(points={{99,-6},{106,-6}}, color={0,0,127}));
  connect(add8.u1, integrator.u) annotation (Line(points={{106,6},{98,6},{98,24},
          {106,24},{106,90},{96,90},{96,106},{102,106}}, color={0,0,127}));
  connect(gain6.u, w) annotation (Line(points={{98,-40},{86,-40},{86,
          -20},{136,-20},{136,40},{150,40}}, color={0,0,127}));
  connect(gain6.y, freq)
    annotation (Line(points={{121,-40},{150,-40}}, color={0,0,127}));
  connect(gain7.y, add1.u1)
    annotation (Line(points={{-13,112},{38,112}}, color={0,0,127}));
  connect(gain7.u, add.y)
    annotation (Line(points={{-36,112},{-79,112}}, color={0,0,127}));
  connect(add.u2, u1) annotation (Line(points={{-102,106},{-106,106},{-106,98},{
          0,98},{0,160}}, color={0,0,127}));
  connect(integrator1.y, limiter3.u)
    annotation (Line(points={{29,-110},{38,-110}}, color={0,0,127}));
  connect(limiter3.y, add5.u2) annotation (Line(points={{61,-110},{68,-110},{68,
          -92},{56,-92},{56,-80},{62,-80}}, color={0,0,127}));
  connect(integrator1.u, gain4.u) annotation (Line(points={{6,-110},{-4,-110},{-4,
          -68},{4,-68}}, color={0,0,127}));
  connect(integrator2.y, limiter4.u)
    annotation (Line(points={{-37,6},{-28,6}}, color={0,0,127}));
  connect(add4.y, integrator2.u) annotation (Line(points={{-69,-2},{-64,-2},{-64,
          6},{-60,6}}, color={0,0,127}));
  connect(limiter4.y, add6.u1)
    annotation (Line(points={{-5,6},{4,6},{4,-2}}, color={0,0,127}));
  connect(gain5.u, integrator2.u) annotation (Line(points={{-54,-34},{-64,-34},{
          -64,6},{-60,6}}, color={0,0,127}));
  connect(integrator3.y, limiter5.u)
    annotation (Line(points={{-35,42},{-26,42}}, color={0,0,127}));
  connect(limiter5.y, add3.u2)
    annotation (Line(points={{-3,42},{6,42},{6,60}}, color={0,0,127}));
  connect(integrator3.u, gain2.u) annotation (Line(points={{-58,42},{-60,42},{-60,
          44},{-62,44},{-62,82},{-58,82}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},
            {140,140}}), graphics={Text(
          extent={{-80,40},{80,-40}},
          lineColor={28,108,200},
          textString="%name"), Rectangle(extent={{-140,140},{140,-140}},
            lineColor={28,108,200})}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-140,-140},{140,140}})));
end GFMDRP_A_MPC;
