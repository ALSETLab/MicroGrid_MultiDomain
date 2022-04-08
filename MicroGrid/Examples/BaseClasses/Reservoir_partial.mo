within MicroGrid.Examples.BaseClasses;
partial model Reservoir_partial
  extends Modelica.Icons.Example;
  Modelica.Fluid.Machines.Pump pump(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare function flowCharacteristic =
        Modelica.Fluid.Machines.BaseClasses.PumpCharacteristics.linearFlow (
          V_flow_nominal={0,0.1}, head_nominal={10,0}),
    N_nominal(displayUnit="rad/s") = 3533.2397366401)
    annotation (Placement(transformation(extent={{80,-70},{100,-50}})));

  Modelica.Fluid.Sources.FixedBoundary SOURCE(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    p=100000,
    T=323.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{38,-70},{58,-50}})));

  Modelica.Fluid.Pipes.StaticPipe pipe(
    allowFlowReversal=true,
    length=10,
    height_ab=0,
    diameter=0.2,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(
        origin={129,-60},
        extent={{-9,-10},{11,10}},
        rotation=0)));
  Modelica.Fluid.Vessels.OpenTank reservoir(
    T_start=Modelica.Units.Conversions.from_degC(20),
    use_portsData=true,
    crossArea=10,
    level_start=0,
    height=50,
    portsData={Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(
        diameter=0.1)},
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    nPorts=1)
    annotation (Placement(transformation(extent={{130,-34},{192,28}})));

  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{130,70},{170,90}})));
  inner Modelica.Fluid.System system(T_ambient=323.15, energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
                                   annotation (Placement(transformation(extent={{176,70},
            {196,90}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia_of_the_pump(J=1, w(
        start=1))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,0})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
equation
  connect(pump.port_b,pipe. port_a)
    annotation (Line(points={{100,-60},{120,-60}}, color={0,127,255}));
  connect(SOURCE.ports[1],pump. port_a)
    annotation (Line(points={{58,-60},{80,-60}},   color={0,127,255}));
  connect(pipe.port_b, reservoir.ports[1]) annotation (Line(points={{140,-60},
          {160,-60},{160,-38},{161,-38},{161,-34}},
                                            color={0,127,255}));
  connect(inertia_of_the_pump.flange_b, pump.shaft)
    annotation (Line(points={{60,0},{90,0},{90,-50}}, color={0,0,0}));
  connect(torqueSensor.flange_b, inertia_of_the_pump.flange_a)
    annotation (Line(points={{20,0},{40,0}},   color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -200,-100},{200,100}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
            200,100}})));
end Reservoir_partial;
