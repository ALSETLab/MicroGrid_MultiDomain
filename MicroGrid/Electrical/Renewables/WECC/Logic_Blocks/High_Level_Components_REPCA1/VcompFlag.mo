within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.High_Level_Components_REPCA1;
model VcompFlag
  Modelica.Blocks.Math.Gain gain(k=1)
    annotation (Placement(transformation(extent={{-34,2},{-26,10}})));
  Modelica.Blocks.Math.Gain gain1(k=1)
    annotation (Placement(transformation(extent={{-34,-10},{-26,-2}})));
  Modelica.Blocks.Math.Product product
    annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{14,14},{34,34}})));
  Modelica.Blocks.Math.Sqrt sqrt1
    annotation (Placement(transformation(extent={{42,14},{62,34}})));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant
    annotation (Placement(transformation(extent={{42,44},{62,64}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{72,6},{92,26}})));
  Modelica.Blocks.Math.Gain Kc(k=0.02)
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  Modelica.Blocks.Math.Add add1
    annotation (Placement(transformation(extent={{20,-54},{40,-34}})));
  VdRX_squared vdRX_squared
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Modelica.Blocks.Interfaces.RealOutput VcompFlag_out
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{120,-10},{140,10}})));
  Modelica.Blocks.Interfaces.RealInput Vreg
    annotation (Placement(transformation(extent={{-120,40},{-80,80}})));
  Modelica.Blocks.Interfaces.RealInput Qbranch
    annotation (Placement(transformation(extent={{-120,-80},{-80,-40}})));
  Modelica.Blocks.Interfaces.RealInput ir1 annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={90,120})));
  Modelica.Blocks.Interfaces.RealInput ii1 annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={50,120})));
  Modelica.Blocks.Interfaces.RealInput vr1 annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-10,120})));
  Modelica.Blocks.Interfaces.RealInput vi1 annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-50,120})));
equation
  connect(gain.y, product.u1)
    annotation (Line(points={{-25.6,6},{-18,6}}, color={0,0,127}));
  connect(gain1.y, product.u2)
    annotation (Line(points={{-25.6,-6},{-18,-6}}, color={0,0,127}));
  connect(product.y, add.u2) annotation (Line(points={{5,0},{10,0},{10,18},
          {12,18}}, color={0,0,127}));
  connect(add.y, sqrt1.u)
    annotation (Line(points={{35,24},{40,24}}, color={0,0,127}));
  connect(booleanConstant.y, switch1.u2) annotation (Line(points={{63,54},
          {66,54},{66,16},{70,16}}, color={255,0,255}));
  connect(sqrt1.y, switch1.u1)
    annotation (Line(points={{63,24},{70,24}}, color={0,0,127}));
  connect(gain1.u, gain.u) annotation (Line(points={{-34.8,-6},{-44,-6},{
          -44,6},{-34.8,6}}, color={0,0,127}));
  connect(Kc.y, add1.u2)
    annotation (Line(points={{-19,-50},{18,-50}}, color={0,0,127}));
  connect(add1.u1, gain.u) annotation (Line(points={{18,-38},{0,-38},{0,
          -20},{-44,-20},{-44,6},{-34.8,6}}, color={0,0,127}));
  connect(vdRX_squared.y, add.u1)
    annotation (Line(points={{-19,30},{12,30}}, color={0,0,127}));
  connect(add1.y, switch1.u3) annotation (Line(points={{41,-44},{64,-44},
          {64,8},{70,8}}, color={0,0,127}));
  connect(gain.u, Vreg) annotation (Line(points={{-34.8,6},{-70,6},{-70,
          60},{-100,60}}, color={0,0,127}));
  connect(Kc.u, Qbranch) annotation (Line(points={{-42,-50},{-72,-50},{
          -72,-60},{-100,-60}}, color={0,0,127}));
  connect(switch1.y, VcompFlag_out) annotation (Line(points={{93,16},{100,
          16},{100,0},{130,0}}, color={0,0,127}));
  connect(vdRX_squared.ir, ir1) annotation (Line(points={{-42,37},{-46,37},
          {-46,86},{90,86},{90,120}}, color={0,0,127}));
  connect(vdRX_squared.ii, ii1) annotation (Line(points={{-42,33},{-54,33},
          {-54,94},{50,94},{50,120}}, color={0,0,127}));
  connect(vdRX_squared.vr, vr1) annotation (Line(points={{-42,27},{-50,27},
          {-50,120},{-10,120}}, color={0,0,127}));
  connect(vdRX_squared.vi, vi1) annotation (Line(points={{-42,23},{-50,23},
          {-50,120}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-80,-100},{120,100}})),
      Icon(coordinateSystem(extent={{-80,-100},{120,100}}), graphics={
          Rectangle(extent={{-80,100},{120,-100}}, lineColor={28,108,200}),
          Text(
          extent={{-52,16},{90,-30}},
          lineColor={28,108,200},
          textString="%name")}));
end VcompFlag;
