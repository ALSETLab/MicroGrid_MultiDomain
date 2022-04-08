within MicroGrid.Examples.DistrictHeating.ThermalPowerLibrary;
model PumpMechExample01 "Test of normal pump"
  extends Modelica.Icons.Example;
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_h source(
    h0=2e5,
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    p0=100000,
    N_ports=1) annotation (Placement(transformation(extent={{-100,14},{
            -80,34}}, rotation=0)));
  Modelica.Blocks.Sources.Sine Pressure(
    amplitude=3.5e5,
    f=0.2,
    phase=3.14159,
    startTime=0,
    offset=4e5) annotation (Placement(transformation(extent={{80,60},{
            60,80}}, rotation=0)));
  ThermalPower.TwoPhase.SourcesAndSinks.PressureBoundary_h sink(
    use_p_in=true,
    use_h_in=true,
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    N_ports=1) annotation (Placement(transformation(extent={{60,20},{40,
            40}}, rotation=0)));
  Modelica.Blocks.Sources.Constant Enthalpy1(k=1e3) annotation (Placement(
        transformation(extent={{20,60},{40,80}}, rotation=0)));
  replaceable package Medium = Medium;
  Modelica.Blocks.Sources.Constant rpm(k=1500) annotation (Placement(
        transformation(extent={{96,-20},{76,0}}, rotation=0)));
  ThermalPower.TwoPhase.TurboMachinery.Pumps.PumpMech pumpMech(
    redeclare package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    V=1,
    pin_start=100000,
    pout_start=100000)
    annotation (Placement(transformation(extent={{-50,14},{-30,34}})));
  Modelica.Mechanics.Rotational.Sources.Speed speed
    annotation (Placement(transformation(extent={{10,-20},{-10,0}})));
  Modelica.Blocks.Math.UnitConversions.From_rpm from_rpm
    annotation (Placement(transformation(extent={{56,-20},{36,0}})));
  inner ThermalPower.System_TPL system_TPL
    annotation (Placement(transformation(extent={{-88,58},{-68,78}})));
  ThermalPower.TwoPhase.FlowChannels.Pipe_lumpedP pipe_lumpedP(
    L=1,
    D=0.1,
    redeclare replaceable package Medium =
        Modelon.Media.PreDefined.TwoPhase.WaterIF97,
    useMeanTempDrivenQ=false)
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
equation
  connect(Pressure.y, sink.p_in)
    annotation (Line(points={{59,70},{53.9,70},{53.9,38.3}},
                                                       color={0,0,127}));
  connect(Enthalpy1.y, sink.h_in)
    annotation (Line(points={{41,70},{46.3,70},{46.3,38.3}},
                                                    color={0,0,127}));
  connect(source.port[1], pumpMech.feed)
    annotation (Line(points={{-81,24},{-64,24},{-48.2,24}}, color={0,0,255}));
  connect(pumpMech.flange, speed.flange) annotation (Line(points={{-30.8,26},{
          -20,26},{-20,-10},{-10,-10}}, color={0,0,0}));
  connect(rpm.y, from_rpm.u)
    annotation (Line(points={{75,-10},{66,-10},{58,-10}}, color={0,0,127}));
  connect(speed.w_ref, from_rpm.y)
    annotation (Line(points={{12,-10},{35,-10}}, color={0,0,127}));
  connect(pumpMech.drain, pipe_lumpedP.portA)
    annotation (Line(points={{-32,30},{-10,30}},color={0,0,255}));
  connect(pipe_lumpedP.portB, sink.port[1])
    annotation (Line(points={{10,30},{41,30}}, color={0,0,255}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
            -100},{100,100}})),           Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2020, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>Description</h4>
<p>This example presents the pressure behavior at pump discharge at constant pump speed and with a varying discharge pressure. The rotational speed is controlled by the mechanical shafts rotation speed. </p>
<p>The simulation time is 20 sec. </p>
</html>"),
    experiment(StopTime=20, Tolerance=1e-006));
end PumpMechExample01;
