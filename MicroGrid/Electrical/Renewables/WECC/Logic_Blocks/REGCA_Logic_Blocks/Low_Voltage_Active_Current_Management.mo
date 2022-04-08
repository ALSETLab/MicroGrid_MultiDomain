within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REGCA_Logic_Blocks;
model Low_Voltage_Active_Current_Management
  //Modelica.Blocks.Interfaces.RealInput Ivpnt0
  //Modelica.Blocks.Interfaces.RealInput Ivpnt1
        parameter Real lvpnt0;
        parameter Real lvpnt1;
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-20},{140,20}}),
        iconTransformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput Vt
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
        iconTransformation(extent={{-120,-10},{-100,10}})));

equation
  y = noEvent(if Vt <= lvpnt0 then 0 elseif Vt >= lvpnt1 then 1 else (1/(lvpnt1-lvpnt0))*(Vt-lvpnt0));
    annotation (Placement(transformation(extent={{-100,-20},{-60,20}}),
        iconTransformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-28,-102})),
                Placement(transformation(extent={{-100,-80},{-60,-40}}),
        iconTransformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={66,-102})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                         graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0}),
        Text(
          extent={{-80,40},{80,-40}},
          lineColor={28,108,200},
          textStyle={TextStyle.Bold},
          textString="%name"),
        Line(points={{-32,66}}, color={28,108,200})}),           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}})));
end Low_Voltage_Active_Current_Management;
