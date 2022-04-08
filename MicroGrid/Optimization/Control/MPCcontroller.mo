within MicroGrid.Optimization.Control;
model MPCcontroller
  Modelica.Blocks.Continuous.Integrator integrator(k=1, y_start=y_start)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Interfaces.RealOutput u
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput dp_u
    "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  parameter Real y_start=0 "Initial or guess value of output (= state)";
equation
  connect(integrator.y, u)
    annotation (Line(points={{11,0},{110,0}}, color={0,0,127}));
  connect(integrator.u, dp_u)
    annotation (Line(points={{-12,0},{-120,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-82,20},{86,-26}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textStyle={TextStyle.Bold},
          textString="du/dt")}),         Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end MPCcontroller;
