within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.High_Level_Components_REPCA1;
model VdRX_squared
  parameter Real Rc = 0;
  parameter Real Xc = 0;
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput ir
    annotation (Placement(transformation(extent={{-140,50},{-100,90}})));
  Modelica.Blocks.Interfaces.RealInput ii
    annotation (Placement(transformation(extent={{-140,10},{-100,50}})));
  Modelica.Blocks.Interfaces.RealInput vr
    annotation (Placement(transformation(extent={{-140,-50},{-100,-10}})));
  Modelica.Blocks.Interfaces.RealInput vi
    annotation (Placement(transformation(extent={{-140,-90},{-100,-50}})));
equation
 y = (vr - Rc*ir + ii*Xc)^2 + (vi - Rc*ii - ir*Xc)^2;
end VdRX_squared;
