within MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Power_Electronics;
model AC2DC_and_DC2AC_uninitialized
  "Phasor based Voltage Source Converter model."
   extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=true,
    final enableP_0 = false,
    final enableQ_0=false,
    final enablev_0=true,
    final enableS_b=true);
  OpenIPSL.Interfaces.PwPin p
    annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
  OpenIPSL.Interfaces.PwPin n annotation (Placement(transformation(extent={{110,-10},
            {130,10}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage Voltage annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-46,0})));
  Modelica.Blocks.Sources.RealExpression Vd0(y=3*sqrt(6)*Vs.y*(V_b)/Modelica.Constants.pi)
                                             annotation (Placement(transformation(extent={{-84,-10},
            {-64,10}})));
  Modelica.Electrical.Analog.Basic.Resistor Resistor(R=Rdc)
    annotation (Placement(transformation(extent={{-42,4},{-22,24}})));
  Modelica.Electrical.Analog.Basic.Inductor Inductor(L=Ldc)
    annotation (Placement(transformation(extent={{-16,4},{4,24}})));
  Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch(Ron=1e-5, Goff=1e-5)
    annotation (Placement(transformation(extent={{10,4},{30,24}})));
  Modelica.Electrical.Analog.Basic.Capacitor Capacitor(v(start=Vc0), C=Cdc)
                                                               annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={30,0})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={50,0})));
  Modelica.Blocks.Sources.RealExpression Ii(y=Pmotor.y*S_b/Capacitor.v)
                                                               annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={78,0})));
  Modelica.Blocks.Sources.RealExpression Vs(y=sqrt(p.vr^2 + p.vi^2))
    annotation (Placement(transformation(extent={{-70,78},{-50,98}})));
  Modelica.Blocks.Sources.BooleanExpression open_circuit_condition(y=if
        Resistor.i <= 0 then true else false)
    annotation (Placement(transformation(extent={{52,20},{32,40}})));
  Modelica.Blocks.Sources.RealExpression Pmotor(y=-(n.vr*n.ir + n.vi*n.ii))
    annotation (Placement(transformation(extent={{30,90},{50,110}})));
  Modelica.Blocks.Sources.RealExpression Qmotor(y=n.vr*n.ii - n.vi*n.ir)
    annotation (Placement(transformation(extent={{30,74},{50,94}})));
  OpenIPSL.Types.PerUnit P;
  OpenIPSL.Types.PerUnit Q;
  OpenIPSL.Types.PerUnit S;

  Modelica.Blocks.Sources.RealExpression Vmotor(y=Capacitor.v*m_input/(2*sqrt(2)
        *V_b))
    annotation (Placement(transformation(extent={{30,58},{50,78}})));
  Modelica.Blocks.Interfaces.RealInput m_input annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-130}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-130})));
  Modelica.Blocks.Sources.RealExpression vr_m(y=Vmotor.y*cos(0))
    annotation (Placement(transformation(extent={{68,90},{88,110}})));
  Modelica.Blocks.Sources.RealExpression vi_m(y=Vmotor.y*sin(0))
    annotation (Placement(transformation(extent={{68,74},{88,94}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-14,-36},{6,-16}})));

  Modelica.Blocks.Interfaces.RealOutput Vc "Value of Real output" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-50,-130})));
  parameter Modelica.Units.SI.Resistance Rdc=0.1
    "DC Link Resistance"
    annotation (Dialog(group="DC Link Parameters"));
  parameter Modelica.Units.SI.Inductance Ldc=0.001
    "DC Link Inductance"
    annotation (Dialog(group="DC Link Parameters"));
  parameter Modelica.Units.SI.Capacitance Cdc=0.02
    "DC Link Capacitance"
    annotation (Dialog(group="DC Link Parameters"));

  Modelica.Blocks.Sources.RealExpression Smotor(y=sqrt(Pmotor.y^2 + Qmotor.y^2))
    annotation (Placement(transformation(extent={{68,58},{88,78}})));
protected
  parameter OpenIPSL.Types.Voltage Vc0 = 2*sqrt(2)*Vmotor0*V_b/m0;
  parameter OpenIPSL.Types.PerUnit Vmotor0 = 0.70005;
  parameter Real m0= 0.811776;

  Modelica.Blocks.Sources.RealExpression Capacitor_Voltage(y=Capacitor.v)
    annotation (Placement(transformation(extent={{-90,-100},{-70,-80}})));
initial equation
  //der(Resistor.i) = 0;
  //der(Capacitor.v) = 0;

equation
  connect(Vd0.y, Voltage.v)
    annotation (Line(points={{-63,0},{-58,0}},     color={0,0,127}));
  connect(Voltage.p, Resistor.p)
    annotation (Line(points={{-46,10},{-46,14},{-42,14}},    color={0,0,255}));
  connect(Resistor.n, Inductor.p)
    annotation (Line(points={{-22,14},{-16,14}},   color={0,0,255}));
  connect(Inductor.n, switch.p)
    annotation (Line(points={{4,14},{10,14}},   color={0,0,255}));
  connect(switch.n, Capacitor.p)
    annotation (Line(points={{30,14},{30,10}},   color={0,0,255}));
  connect(Voltage.n, Capacitor.n) annotation (Line(points={{-46,-10},{-46,-14},{
          30,-14},{30,-10}}, color={0,0,255}));
  connect(switch.n, signalCurrent.p) annotation (Line(points={{30,14},{50,14},{50,
          10}},                                                                            color={0,0,255}));
  connect(signalCurrent.n, Capacitor.n) annotation (Line(points={{50,-10},{50,-14},
          {30,-14},{30,-10}}, color={0,0,255}));
  connect(signalCurrent.i, Ii.y) annotation (Line(points={{62,0},{67,0}},     color={0,0,127}));
  connect(open_circuit_condition.y, switch.control)
    annotation (Line(points={{31,30},{20,30},{20,26}},    color={255,0,255}));
    P =  p.vr*p.ir + p.vi*p.ii;
    Q = (-p.vr*p.ii) + p.vi*p.ir;
    Q = 0;
    S = sqrt(P^2 + Q^2);
    Resistor.i = smooth(0,(P*S_b)/Vd0.y);

    n.vr = vr_m.y;
    n.vi = vi_m.y;
  connect(ground.p, Capacitor.n) annotation (Line(points={{-4,-16},{-4,-14},{30,
          -14},{30,-10}},
                     color={0,0,255}));
  connect(Capacitor_Voltage.y, Vc) annotation (Line(points={{-69,-90},{-50,-90},
          {-50,-130}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},
            {120,120}}),
        graphics={Rectangle(
          extent={{-120,120},{120,-120}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-110,104},{108,-116}},
          lineColor={0,0,0},
          textString="Power Electronics
Interface")}),                                                                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}}), graphics={Rectangle(
          extent={{20,110},{100,60}},
          lineColor={0,0,255},
          pattern=LinePattern.Dash), Text(
          extent={{30,118},{90,112}},
          lineColor={0,0,255},
          pattern=LinePattern.Dash,
          textString="Motor Variables"),                                                       Rectangle(
          extent={{-80,108},{-40,68}},
          lineColor={0,0,255},
          pattern=LinePattern.Dash), Text(
          extent={{-92,116},{-30,110}},
          lineColor={0,0,255},
          pattern=LinePattern.Dash,
          textString="Grid Variables"),                                                        Rectangle(
          extent={{-100,-70},{-60,-110}},
          lineColor={0,0,255},
          pattern=LinePattern.Dash), Text(
          extent={{-112,-62},{-50,-68}},
          lineColor={0,0,255},
          pattern=LinePattern.Dash,
          textString="DC Link Variables")}));
end AC2DC_and_DC2AC_uninitialized;
