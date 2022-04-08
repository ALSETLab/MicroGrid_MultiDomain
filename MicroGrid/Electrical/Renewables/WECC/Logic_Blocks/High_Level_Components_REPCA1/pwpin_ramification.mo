within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.High_Level_Components_REPCA1;
model pwpin_ramification
  Modelica.Blocks.Interfaces.RealOutput Vreg "Regulated Voltage Magnitude"
    annotation (Placement(transformation(extent={{100,60},{120,80}})));
  //Modelica.Blocks.Interfaces.RealOutput Ibranch "Regulated Current Magnitude"
  Modelica.Blocks.Interfaces.RealOutput Qbranch "Injected Reactive Power Magnitude"
    annotation (Placement(transformation(extent={{100,-40},{120,-20}})));
  //Modelica.Blocks.Interfaces.RealOutput Pbranch "Injected Active Power Magnitude"
  Modelica.Blocks.Interfaces.RealInput ir
    annotation (Placement(transformation(extent={{-140,50},{-100,90}})));
  Modelica.Blocks.Interfaces.RealInput ii
    annotation (Placement(transformation(extent={{-140,10},{-100,50}})));
  Modelica.Blocks.Interfaces.RealInput vr
    annotation (Placement(transformation(extent={{-140,-50},{-100,-10}})));
  Modelica.Blocks.Interfaces.RealInput vi
    annotation (Placement(transformation(extent={{-140,-90},{-100,-50}})));
equation
  Vreg = sqrt(vr^2 + vi^2);
  //Ibranch = sqrt(pwPin.ir^2 + pwPin.ii^2);
  Qbranch = vi*ir - vr*ii;
  //Pbranch = pwPin.vr*pwPin.ir + pwPin.vi*pwPin.ii;
    annotation (Placement(transformation(extent={{100,20},{120,40}})),
                Placement(transformation(extent={{100,-80},{120,-60}})));
end pwpin_ramification;
