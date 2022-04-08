within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.High_Level_Components_REPCA1;
model RefFlag

  Modelica.Blocks.Interfaces.RealInput RefFlag_input
    annotation (Placement(transformation(extent={{-140,30},{-100,70}})));
  Modelica.Blocks.Interfaces.RealInput Qbranch annotation (Placement(
        transformation(extent={{-140,-70},{-100,-30}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(
    K=1,
    T=Tfltr,
    y_start=0)
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag1(
    K=1,
    T=Tfltr,
    y_start=0)
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Blocks.Interfaces.RealInput Vref
    "Regulated bus initial voltage (pu, from power flow solution)"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));
  Modelica.Blocks.Math.Add add(k2=-1)
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  Modelica.Blocks.Math.Add add1(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
  Modelica.Blocks.Interfaces.RealInput Qref "Regulated branch initial reactive power flow (pu, from power flow

solution)"
     annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=RefFlag)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Blocks.Interfaces.RealOutput RefFlag_output
    "Connector of Real output signal"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  parameter Boolean RefFlag=true
    "Plant level reactive power (0) or voltage control (1)";
  parameter OpenIPSL.Types.Time Tfltr=0.02
    "Voltage and reactive power filter time constant (s).";
equation
  connect(simpleLag.u, RefFlag_input)
    annotation (Line(points={{-82,50},{-120,50}}, color={0,0,127}));
  connect(add.u1, Vref)
    annotation (Line(points={{18,56},{0,56},{0,120}}, color={0,0,127}));
  connect(simpleLag.y, add.u2) annotation (Line(points={{-59,50},{-20,50},
          {-20,44},{18,44}}, color={0,0,127}));
  connect(simpleLag1.u, Qbranch)
    annotation (Line(points={{-82,-50},{-120,-50}}, color={0,0,127}));
  connect(simpleLag1.y, add1.u1) annotation (Line(points={{-59,-50},{-20,
          -50},{-20,-44},{18,-44}}, color={0,0,127}));
  connect(add1.u2, Qref) annotation (Line(points={{18,-56},{0,-56},{0,
          -120}}, color={0,0,127}));
  connect(booleanConstant.y, switch1.u2)
    annotation (Line(points={{1,0},{58,0}}, color={255,0,255}));
  connect(switch1.u1, add.y) annotation (Line(points={{58,8},{50,8},{50,
          50},{41,50}}, color={0,0,127}));
  connect(switch1.u3, add1.y) annotation (Line(points={{58,-8},{50,-8},{
          50,-50},{41,-50}}, color={0,0,127}));
  connect(switch1.y, RefFlag_output)
    annotation (Line(points={{81,0},{110,0}}, color={0,0,127}));
  annotation (Icon(graphics={Rectangle(extent={{-100,100},{100,-100}},
            lineColor={28,108,200}), Text(
          extent={{-52,20},{60,-24}},
          lineColor={28,108,200},
          textString="%name")}));
end RefFlag;
