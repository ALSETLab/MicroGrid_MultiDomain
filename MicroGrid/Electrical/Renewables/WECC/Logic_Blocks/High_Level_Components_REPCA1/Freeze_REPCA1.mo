within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.High_Level_Components_REPCA1;
model Freeze_REPCA1
  parameter Real Vfz = 0 "Voltage for freezing Volt/VAR regulator integrator (pu). Typical Values 0 to 0.9.";
  Modelica.Blocks.Interfaces.RealInput Vreg
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.BooleanOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  y = if Vreg < Vfz then true else false;
  annotation (Icon(graphics={Rectangle(extent={{-100,100},{100,-100}},
            lineColor={28,108,200}), Text(
          extent={{-74,24},{58,-22}},
          lineColor={28,108,200},
          textString="%name")}));
end Freeze_REPCA1;
