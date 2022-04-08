within MicroGrid.Optimization.CostFunction;
model OpenIPSL_Optimizer
  Modelica.Blocks.Sources.RealExpression OperationalCost(y=costIntegrand_y)
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Modelica.Blocks.Sources.RealExpression ControlChangePenalty(y=costIntegrand_u)
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  Modelica.Blocks.Math.Add add if not exclude_integrators
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  input Real costIntegrand_y "Objective integrand to be minimized" annotation(Dialog(group = "Cost integrand"));
  input Real costIntegrand_u "Control signal penalties" annotation(Dialog(group = "Cost integrand"));

  parameter Boolean exclude_integrators = true "If true, integrators only in optimization routines (more efficient), in Modelica otherwise" annotation(Dialog(group = "Cost integrand"));
  parameter Real T_optimization = 2 * 3600 * 24 "Optimization horizon in seconds";

  Modelica.Blocks.Continuous.Integrator integrator1(k=1, y_start=0) if not exclude_integrators
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Modelica.Blocks.Continuous.Integrator integrator2(k=1, y_start=0) if not exclude_integrators
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Modelica.Blocks.Sources.Constant SimulationRequirement(k=0) if exclude_integrators
    annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));
  Modelica.Blocks.Interfaces.RealOutput costintegrand
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,60},{120,80}})));
  Modelica.Blocks.Interfaces.RealOutput cost "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
  Modelica.Blocks.Interfaces.RealOutput operationalcost
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,20},{120,40}})));
  Modelica.Blocks.Interfaces.RealOutput penaltycost
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-40},{120,-20}})));
  Modelica.Blocks.Math.Gain gain(k=1)
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
equation
  connect(OperationalCost.y, integrator1.u)
    annotation (Line(points={{-59,30},{-42,30}}, color={0,0,127}));
  connect(ControlChangePenalty.y, integrator2.u)
    annotation (Line(points={{-59,-30},{-42,-30}}, color={0,0,127}));
  connect(integrator1.y, add.u1) annotation (Line(points={{-19,30},{-10,30},{-10,
          6},{-2,6}}, color={0,0,127}));
  connect(integrator2.y, add.u2) annotation (Line(points={{-19,-30},{-10,-30},{-10,
          -6},{-2,-6}}, color={0,0,127}));
  connect(integrator1.y, operationalcost)
    annotation (Line(points={{-19,30},{110,30}}, color={0,0,127}));
  connect(integrator2.y, penaltycost)
    annotation (Line(points={{-19,-30},{110,-30}}, color={0,0,127}));
  connect(SimulationRequirement.y, cost)
    annotation (Line(points={{1,-70},{110,-70}}, color={0,0,127}));
  connect(add1.y, gain.u)
    annotation (Line(points={{-19,70},{-2,70}}, color={0,0,127}));
  connect(gain.y, costintegrand)
    annotation (Line(points={{21,70},{110,70}}, color={0,0,127}));
  connect(cost, add.y) annotation (Line(points={{110,-70},{100,-70},{100,
          -66},{66,-66},{66,0},{21,0}}, color={0,0,127}));
  connect(penaltycost, SimulationRequirement.y) annotation (Line(points={
          {110,-30},{20,-30},{20,-70},{1,-70}}, color={0,0,127}));
  connect(operationalcost, SimulationRequirement.y) annotation (Line(
        points={{110,30},{56,30},{56,-70},{1,-70}}, color={0,0,127}));
  connect(add1.u1, OperationalCost.y) annotation (Line(points={{-42,76},{
          -59,76},{-59,30}}, color={0,0,127}));
  connect(ControlChangePenalty.y, add1.u2) annotation (Line(points={{-59,
          -30},{-50,-30},{-50,64},{-42,64}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200},
          fillColor={102,44,145},
          fillPattern=FillPattern.Solid),
          Text(
          extent={{-80,40},{80,-40}},
          lineColor={255,255,255},
          textString="OPTIMIZER",
          textStyle={TextStyle.Bold})}),                         Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end OpenIPSL_Optimizer;
