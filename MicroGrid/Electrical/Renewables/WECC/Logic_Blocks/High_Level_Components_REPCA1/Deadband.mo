within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.High_Level_Components_REPCA1;
model Deadband

  parameter Real dbd1 = -0.1;
  parameter Real dbd2 = 0.1;
  Modelica.Blocks.Interfaces.RealInput dead_input
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput dead_output
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  dead_output = if dbd1 < dead_input or dead_input < dbd2 then 0 else dead_input;
  annotation (Icon(graphics={Rectangle(extent={{-100,100},{100,-100}},
            lineColor={28,108,200}), Text(
          extent={{-68,24},{64,-28}},
          lineColor={28,108,200},
          textString="%name")}));
end Deadband;
