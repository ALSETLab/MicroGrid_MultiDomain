within MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Controls;
model pump_controller
  Modelica.Blocks.Interfaces.RealInput m_flow_ref annotation (Placement(
        transformation(extent={{-120,50},{-100,70}}), iconTransformation(extent=
           {{-120,50},{-100,70}})));
  Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(
        transformation(extent={{-54,-10},{-34,10}})));
  Modelica.Blocks.Continuous.Integrator integrator(k=ki, y_start=1)
    annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
  Modelica.Blocks.Math.Gain gain(k=kp)
    annotation (Placement(transformation(extent={{-8,20},{12,40}})));
  Modelica.Blocks.Math.Add add1(k2=+1)
    annotation (Placement(transformation(extent={{26,-10},{46,10}})));
  Modelica.Blocks.Math.Gain PI_convert(k=mflow_2_speed)
    annotation (Placement(transformation(extent={{56,-10},{76,10}})));
  Modelica.Blocks.Interfaces.RealInput m_flow annotation (Placement(
        transformation(extent={{-120,-70},{-100,-50}}), iconTransformation(
          extent={{-120,-70},{-100,-50}})));
  Modelica.Blocks.Interfaces.RealOutput Wref "Output signal connector"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  parameter Real kp=1 "Proportional Gain in the PI Controller."
    annotation (Dialog(group="Pump Control Setting"));
  parameter Real ki=0.1 "Integrator gain in the PI Controller."
    annotation (Dialog(group="Pump Control Setting"));
  parameter Real mflow_2_speed=188.495/585.18
    "Linear gain converter from mass flow rate to motor reference speed."
    annotation (Dialog(group="Pump Control Setting"));
equation
  connect(gain.u,add. y) annotation (Line(points={{-10,30},{-28,30},{-28,0},{-33,
          0}},        color={0,0,127}));
  connect(integrator.u,add. y) annotation (Line(points={{-12,-30},{-28,-30},{-28,
          0},{-33,0}},      color={0,0,127}));
  connect(integrator.y,add1. u2) annotation (Line(points={{11,-30},{22,-30},{22,
          -6},{24,-6}},    color={0,0,127}));
  connect(gain.y,add1. u1) annotation (Line(points={{13,30},{22,30},{22,6},{24,6}},
                     color={0,0,127}));
  connect(add1.y,PI_convert. u)
    annotation (Line(points={{47,0},{54,0}},       color={0,0,127}));
  connect(m_flow_ref, add.u1) annotation (Line(points={{-110,60},{-62,60},{-62,6},
          {-56,6}}, color={0,0,127}));
  connect(m_flow, add.u2) annotation (Line(points={{-110,-60},{-62,-60},{-62,-6},
          {-56,-6}}, color={0,0,127}));
  connect(PI_convert.y, Wref)
    annotation (Line(points={{77,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
          Text(
          extent={{-78,30},{80,-20}},
          lineColor={0,0,0},
          textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end pump_controller;
