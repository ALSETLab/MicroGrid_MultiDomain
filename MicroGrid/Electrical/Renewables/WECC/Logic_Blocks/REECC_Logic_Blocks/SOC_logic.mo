within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REECC_Logic_Blocks;
model SOC_logic
  Modelica.Blocks.Interfaces.RealInput SOC
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput ipmax_SOC
    annotation (Placement(transformation(extent={{100,-60},{120,-40}}),
        iconTransformation(extent={{100,-60},{120,-40}})));
  Modelica.Blocks.Interfaces.RealOutput ipmin_SOC
    annotation (Placement(transformation(extent={{100,40},{120,60}}),
        iconTransformation(extent={{100,40},{120,60}})));
        parameter Real SOCmin = 0.2;
        parameter Real SOCmax = 0.8;
equation
  ipmax_SOC = if SOC <= SOCmin then 0 else 1;
  ipmin_SOC = if SOC >= SOCmax then 0 else 1;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SOC_logic;
