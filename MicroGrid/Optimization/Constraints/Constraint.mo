within MicroGrid.Optimization.Constraints;
model Constraint
  "Block for time-invariant inequality constraints in the optimization"
  input Real exp(min=min_val, max=max_val) "Expression to be constrained" annotation(Dialog(group="Expression"));
  parameter Real min_val= -Modelica.Constants.inf "Minimum value of exp" annotation(Dialog(group="Constraining values"));
  parameter Real max_val = Modelica.Constants.inf "Maximum value of exp" annotation(Dialog(group="Constraining values"));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -100,-40},{100,40}}), graphics={Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-82,40},{80,-38}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Constraint")}),                            Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{
            100,40}})));
end Constraint;
