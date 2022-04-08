within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REGCC_Logic_Blocks;
model dq_conversion2 "DQ conversion block for voltage and current."
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
        Line(
          points={{1.04404e-14,-78},{0,60}},
          color={28,108,200},
          arrow={Arrow.Open,Arrow.None},
          origin={-34,-10},
          rotation=180,
          thickness=0.5),
        Line(
          points={{1.8184e-16,-90},{-1.37308e-16,24}},
          color={28,108,200},
          arrow={Arrow.Open,Arrow.None},
          origin={-10,-70},
          rotation=90,
          thickness=0.5),
        Line(
          points={{-34,-70},{-78,56}},
          color={238,46,47},
          arrow={Arrow.None,Arrow.Open}),
        Line(
          points={{42,-78},{-1.37308e-16,24}},
          color={238,46,47},
          arrow={Arrow.Open,Arrow.None},
          origin={-10,-70},
          rotation=90),
        Text(
          extent={{-92,86},{-48,50}},
          lineColor={238,46,47},
          lineThickness=0.5,
          textString="q"),
        Text(
          extent={{44,6},{78,-32}},
          lineColor={238,46,47},
          lineThickness=0.5,
          textString="d"),
        Text(
          extent={{50,-66},{82,-104}},
          lineColor={28,108,200},
          lineThickness=0.5,
          textString="X"),
        Text(
          extent={{-30,90},{8,52}},
          lineColor={28,108,200},
          lineThickness=0.5,
          textString="Y"),
        Text(
          extent={{-132,170},{122,112}},
          lineColor={28,108,200},
          textString="αβ to DQ")}),
                              Diagram(coordinateSystem(preserveAspectRatio=false)));
end dq_conversion2;
