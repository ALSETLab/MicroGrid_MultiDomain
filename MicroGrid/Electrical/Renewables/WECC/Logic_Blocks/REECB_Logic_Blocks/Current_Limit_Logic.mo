within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REECB_Logic_Blocks;
model Current_Limit_Logic
import SIunits =
            Modelica.Units.SI;
  import Modelica.Units.Conversions.*;
  parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "PV base power.";
  parameter OpenIPSL.Types.ActivePower P_0 = 100e6 "PV initial active power.";
  parameter OpenIPSL.Types.ReactivePower Q_0 = 0.5795379e6 "PV initial reactive power.";
  parameter OpenIPSL.Types.PerUnit v_0 = 1 "Initial Terminal Voltage.";
  parameter OpenIPSL.Types.Angle angle_0(displayUnit = "deg");
  parameter Real Volim = 1.2;
  parameter Real Khv = 0.7;
  Modelica.Blocks.Interfaces.RealInput Imax
    annotation (Placement(transformation(extent={{-140,30},{-100,70}})));
  Modelica.Blocks.Interfaces.RealInput Iqcmd( start = -ii0) annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={70,120})));
  Modelica.Blocks.Interfaces.RealInput Ipcmd(start = ir0) annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={70,-120})));
  Modelica.Blocks.Interfaces.RealOutput Iqmin annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-70,110})));
  Modelica.Blocks.Interfaces.RealOutput Iqmax annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-30,110})));
  Modelica.Blocks.Interfaces.RealOutput Ipmin annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-70,-110})));
  Modelica.Blocks.Interfaces.RealOutput Ipmax annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-30,-110})));
  Modelica.Blocks.Interfaces.BooleanInput Pqflag
    "Priority to reactive current (false) or active current (true)."
    annotation (Placement(transformation(extent={{-140,-70},{-100,-30}})));
protected
  parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
  parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
  parameter OpenIPSL.Types.Angle pfaref = atan(q0/p0) "Inverter active power reference (pu on mbase, from power flow solution or from plant controller model)";
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);

equation

  Ipmax = if Pqflag == true then Imax else sqrt(Imax^2 - Iqcmd^2);
  Ipmin = 0;
  Iqmax = if Pqflag == true then sqrt(Imax^2 - Ipcmd^2) else Imax;
  Iqmin = -Iqmax;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid), Text(
          extent={{-92,36},{90,-36}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Current Limit Logic",
          textStyle={TextStyle.Bold,TextStyle.Italic})}),        Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Current_Limit_Logic;
