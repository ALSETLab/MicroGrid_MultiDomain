within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REGCA_Logic_Blocks;
model Low_Voltage_Power_Logic
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-20},{140,20}}),
        iconTransformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput V
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
        iconTransformation(extent={{-120,-10},{-100,10}})));
  //Modelica.Blocks.Interfaces.RealInput Brkpt
 // Modelica.Blocks.Interfaces.RealInput Zerox
  //Modelica.Blocks.Interfaces.RealInput Lvpl1
        parameter Real Brkpt;
        parameter Real Lvpl1;
        parameter Real Zerox;
equation
 // y = if V < Zerox then 0 else if V > Brkpt then Lvpl1 else (V-Zerox)*(Lvpl1/(Brkpt-Zerox));

  y = noEvent(if V < Zerox then 0 else if V > Brkpt then 10e6 else (V-Zerox)*(Lvpl1/(Brkpt-Zerox)));

  //y = if ((ipcmd/(V-Zerox)) <= (Lvpl1/(Brkpt-Zerox)) and ipcmd > Lvpl1) then Lvpl1;

    annotation (Placement(transformation(extent={{-140,0},{-100,40}})),
                Placement(transformation(extent={{-140,-40},{-100,0}})),
                Placement(transformation(extent={{-140,-80},{-100,-40}})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),                                  graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0}),
        Text(
          extent={{-80,40},{80,-40}},
          lineColor={28,108,200},
          textStyle={TextStyle.Bold},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}})));
end Low_Voltage_Power_Logic;
