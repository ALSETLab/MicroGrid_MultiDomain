within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REGCC_Logic_Blocks;
model dq_conversion "DQ conversion block for voltage and current."
  Modelica.Blocks.Interfaces.RealInput real_input
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput imaginary_input
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealOutput d_output
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput q_output
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Blocks.Interfaces.RealInput angle annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));

equation

  d_output = cos(angle)*real_input + sin(angle)*imaginary_input;
  q_output = -sin(angle)*real_input + cos(angle)*imaginary_input;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
        Text(
          extent={{-114,74},{86,-90}},
          lineColor={28,108,200},
          textString="T"),
        Text(
          extent={{18,76},{82,24}},
          lineColor={28,108,200},
          textString="-1",
          textStyle={TextStyle.Bold})}),
                              Diagram(coordinateSystem(preserveAspectRatio=false)));
end dq_conversion;
