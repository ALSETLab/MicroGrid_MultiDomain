within MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Controls;
model VoltsHertz_Controller
  extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=false,
    final enablev_0=false,
    final enableQ_0=false,
    final enableS_b=true);
  Modelica.Blocks.Interfaces.RealInput motor_speed annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-110,-20}),
                          iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={110,50})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag Speed_Sensor(K=1, T=Tr,
    y_start=188.275)
    annotation (Placement(transformation(extent={{-88,-30},{-68,-10}})));
  parameter OpenIPSL.Types.Time Tr=0.01 "Lag time constant"
    annotation (Dialog(group="Control Parameters"));
  Modelica.Blocks.Math.Add add(k1=-1)
    annotation (Placement(transformation(extent={{-88,-68},{-68,-48}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax=we_max, uMin=we_min)
    annotation (Placement(transformation(extent={{44,-36},{64,-16}})));
  Modelica.Blocks.Math.Gain gain(k=Kp)
    annotation (Placement(transformation(extent={{-48,-52},{-28,-32}})));
  Modelica.Blocks.Continuous.Integrator integrator(k=Ki, y_start=
        188.524 - 188.275)
    annotation (Placement(transformation(extent={{-48,-84},{-28,-64}})));
  Modelica.Blocks.Math.Add add1(k1=+1)
    annotation (Placement(transformation(extent={{-18,-68},{2,-48}})));
  Modelica.Blocks.Math.Add add2(k1=+1)
    annotation (Placement(transformation(extent={{16,-36},{36,-16}})));
  Modelica.Blocks.Interfaces.RealOutput we
               "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-60},{120,-40}}),
        iconTransformation(extent={{100,-60},{120,-40}})));
  Modelica.Blocks.Math.Gain gain1(k=2*sqrt(2)*Kf)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={30,10})));
  Modelica.Blocks.Interfaces.RealOutput m "Output signal connector"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,110})));
    parameter Real Kf= 0.0001 "Gain value multiplied with input signal"
    annotation (Dialog(group="Control Parameters"));
  parameter Real Kp=5 "Gain value multiplied with input signal"
    annotation (Dialog(group="Control Parameters"));
  parameter Real Ki=0.1 "Integrator gain"
    annotation (Dialog(group="Control Parameters"));
  parameter Real we_max=Modelica.Constants.inf "Upper limits of input signals"
    annotation (Dialog(group="Control Parameters"));
  parameter Real we_min=-limiter.uMax "Lower limits of input signals"
    annotation (Dialog(group="Control Parameters"));
  Modelica.Blocks.Interfaces.RealInput Vc annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        origin={-40,110},
        rotation=270)));
  Modelica.Blocks.Interfaces.RealInput W_ref "Connector of Real input signal 2"
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
        iconTransformation(extent={{-120,-10},{-100,10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=1, uMin=0) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={24,52})));
  Modelica.Blocks.Math.Gain gain2(k=1/V_b)
    annotation (Placement(transformation(extent={{-52,48},{-32,68}})));
  Modelica.Blocks.Math.Division division
    annotation (Placement(transformation(extent={{-14,42},{6,62}})));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T=0.001, y_start=
        0.817078)
    annotation (Placement(transformation(extent={{44,42},{64,62}})));
equation
  connect(motor_speed, Speed_Sensor.u) annotation (Line(points={{-110,-20},{-90,
          -20}},               color={0,0,127}));
  connect(Speed_Sensor.y, add.u1) annotation (Line(points={{-67,-20},{-62,-20},{
          -62,-38},{-96,-38},{-96,-52},{-90,-52}},
                           color={0,0,127}));
  connect(add.y, gain.u) annotation (Line(points={{-67,-58},{-56,-58},{-56,-42},
          {-50,-42}},
               color={0,0,127}));
  connect(integrator.u, gain.u) annotation (Line(points={{-50,-74},{-56,
          -74},{-56,-42},{-50,-42}},
                       color={0,0,127}));
  connect(gain.y, add1.u1) annotation (Line(points={{-27,-42},{-22,-42},{-22,-52},
          {-20,-52}},
                 color={0,0,127}));
  connect(integrator.y, add1.u2) annotation (Line(points={{-27,-74},{
          -22,-74},{-22,-64},{-20,-64}},
                          color={0,0,127}));
  connect(add2.u1, Speed_Sensor.y)
    annotation (Line(points={{14,-20},{-67,-20}},
                                                color={0,0,127}));
  connect(add.u2, W_ref) annotation (Line(points={{-90,-64},{-100,-64},{-100,0},
          {-110,0}},   color={0,0,127}));
  connect(Vc, gain2.u) annotation (Line(points={{-40,110},{-40,74},{-66,
          74},{-66,58},{-54,58}}, color={0,0,127}));
  connect(gain2.y, division.u2) annotation (Line(points={{-31,58},{-24,
          58},{-24,46},{-16,46}}, color={0,0,127}));
  connect(division.y, limiter1.u)
    annotation (Line(points={{7,52},{12,52}}, color={0,0,127}));
  connect(gain1.u, we) annotation (Line(points={{42,10},{72,10},{72,-50},{110,-50}},
                                                           color={0,0,
          127}));
  connect(add1.y, add2.u2) annotation (Line(points={{3,-58},{8,-58},{8,-32},{14,
          -32}},             color={0,0,127}));
  connect(add2.y, limiter.u)
    annotation (Line(points={{37,-26},{42,-26}}, color={0,0,127}));
  connect(limiter.y, we) annotation (Line(points={{65,-26},{88,-26},{88,-50},{110,
          -50}},       color={0,0,127}));
  connect(integrator.u, add.y) annotation (Line(points={{-50,-74},{-56,
          -74},{-56,-58},{-67,-58}},
                           color={0,0,127}));
  connect(gain1.y, division.u1) annotation (Line(points={{19,10},{-22,10},{-22,58},
          {-16,58}}, color={0,0,127}));
  connect(limiter1.y, firstOrder.u)
    annotation (Line(points={{35,52},{42,52}}, color={0,0,127}));
  connect(firstOrder.y, m) annotation (Line(points={{65,52},{70,52},{70,
          80},{40,80},{40,110}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {80,100}}),                                         graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-80,76},{80,-72}},
          lineColor={0,0,0},
          textString="Volts/Hertz
Controller")}),                                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{80,100}})));
end VoltsHertz_Controller;
