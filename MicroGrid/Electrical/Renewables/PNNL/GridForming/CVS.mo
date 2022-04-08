within MicroGrid.Electrical.Renewables.PNNL.GridForming;
model CVS
  parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "Inverter base power" annotation (Dialog(group= "Power flow data"));
  extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enablefn=false,
    final enableV_b=false,
    final enableS_b=false);

  OpenIPSL.Types.PerUnit Vinv(start=v_0) "Inverter Terminal Voltage";
  OpenIPSL.Types.Angle anglev(start=angle_0) "Inverter Terminal Angle";

  Modelica.Blocks.Interfaces.RealOutput P_inv annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-70,110})));
  Modelica.Blocks.Interfaces.RealOutput Q_inv annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,110})));
  Modelica.Blocks.Interfaces.RealOutput V_inv annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={70,110})));
  Modelica.Blocks.Sources.RealExpression Active_Power(y=-(1/CoB)*(p.vr*
        p.ir + p.vi*p.ii))                                                                  annotation (Placement(transformation(extent={{-46,70},
            {-66,90}})));
  Modelica.Blocks.Sources.RealExpression Reactive_Power(y=-(1/CoB)*(p.vi
        *p.ir - p.vr*p.ii))                                                                   annotation (Placement(transformation(extent={{26,70},
            {6,90}})));
  Modelica.Blocks.Sources.RealExpression Inverter_Terminal_Voltage(y=Vinv)                  annotation (Placement(transformation(extent={{44,70},{64,90}})));
  OpenIPSL.Interfaces.PwPin p(
    vr(start=vr0),
    vi(start=vi0),
    ir(start=ir0_Sb),
    ii(start=ii0_Sb))
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));

  Modelica.Blocks.Interfaces.RealInput Theta_droop(start = angledroop0) annotation (Placement(
        transformation(extent={{-120,50},{-100,70}}), iconTransformation(extent=
           {{-120,50},{-100,70}})));
  Modelica.Blocks.Interfaces.RealInput E_droop(start = Edroop0) annotation (Placement(
        transformation(extent={{-120,-70},{-100,-50}}), iconTransformation(
          extent={{-120,-70},{-100,-50}})));

  parameter OpenIPSL.Types.PerUnit XL =  0.15 "Inverter Coupling Reactance" annotation (Dialog(group= "Controllable Voltage Source Parameter"));

  Modelica.Blocks.Interfaces.RealOutput THETADROOP_0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-58,-110})));
  Modelica.Blocks.Interfaces.RealOutput EDROOP_0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-110})));
  Modelica.Blocks.Sources.RealExpression initial_theta(y=angledroop0)
    annotation (Placement(transformation(extent={{-38,-90},{-58,-70}})));
  Modelica.Blocks.Sources.RealExpression initial_droop_voltage(y=Edroop0)
    annotation (Placement(transformation(extent={{80,-90},{60,-70}})));
protected
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0) "Initial real voltage";
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0) "Initial imaginary voltage";
  parameter OpenIPSL.Types.PerUnit eidroop0 =  ir0_Mb*XL + vi0 "Initial imaginary droop voltage";
  parameter OpenIPSL.Types.PerUnit erdroop0 = -ii0_Mb*XL + vr0 "Initial real droop voltage";
  parameter OpenIPSL.Types.PerUnit Edroop0 = sqrt(erdroop0^2 + eidroop0^2) "Initial Droop Voltage Magnitude";
  parameter OpenIPSL.Types.Angle   angledroop0 = atan2(eidroop0,erdroop0) "Initial droop angle";
  parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
  parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
  parameter OpenIPSL.Types.PerUnit ir0_Sb = -CoB*(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2) "Initial real current";
  parameter OpenIPSL.Types.PerUnit ii0_Sb = -CoB*(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2) "Initial imaginary current";
  parameter OpenIPSL.Types.PerUnit ir0_Mb = (p0*vr0 + q0*vi0)/(vr0^2 + vi0^2) "Initial real current";
  parameter OpenIPSL.Types.PerUnit ii0_Mb = (p0*vi0 - q0*vr0)/(vr0^2 + vi0^2) "Initial imaginary current";
  parameter Real CoB=M_b/S_b;
equation

// Inverter Terminal Voltage Calculation
Vinv = sqrt(p.vr^2 + p.vi^2);

// Inverter Terminal Angle Calculation
anglev = atan2(p.vi, p.vr);

// Inveter Terminal Current Calculations
p.ir = -CoB*( E_droop*sin(Theta_droop)/XL - p.vi/XL);
p.ii = -CoB*(-E_droop*cos(Theta_droop)/XL + p.vr/XL);

  connect(Active_Power.y, P_inv)
    annotation (Line(points={{-67,80},{-70,80},{-70,110}}, color={0,0,127}));
  connect(Reactive_Power.y, Q_inv)
    annotation (Line(points={{5,80},{0,80},{0,110}},  color={0,0,127}));
  connect(Inverter_Terminal_Voltage.y, V_inv)
    annotation (Line(points={{65,80},{70,80},{70,110}}, color={0,0,127}));
  connect(initial_theta.y, THETADROOP_0) annotation (Line(points={{-59,-80},{-58,
          -80},{-58,-110}}, color={0,0,127}));
  connect(initial_droop_voltage.y, EDROOP_0)
    annotation (Line(points={{59,-80},{60,-80},{60,-110}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
          Text(
          extent={{-60,40},{60,-40}},
          textColor={28,108,200},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CVS;
