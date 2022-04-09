within MicroGrid.Thermal_Power.Gas.Tests;
model OriginalGasTurbine_Test

  ThermalPower.FlueGas.TurboMachinery.OpenCycleGasTurbine openBraytonCycle
    annotation (Placement(transformation(extent={{38,6},{-46,62}})));
  ThermalPower.FlueGas.SourcesAndSinks.MassFlowBoundary feedFuel(
    redeclare package Medium = ThermalPower.Media.Gases.NaturalGasWithH2,
    p0=811000,
    m_flow0=4.9,
    use_mdot_in=false) annotation (Placement(transformation(extent={{30,68},{10,
            88}}, rotation=0)));
  ThermalPower.FlueGas.SourcesAndSinks.PressureBoundary_pTX airSource(
    redeclare package Medium = ThermalPower.Media.Gases.MoistFlueGas,
    p0=34300,
    T0=244.4,
    N_ports=1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={46,-14})));
  ThermalPower.FlueGas.SourcesAndSinks.PressureBoundary_pTX airSource1(
    redeclare package Medium = ThermalPower.Media.Gases.MoistFlueGas,
    p0=152000,
    T0=800,
    N_ports=1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-46,-18})));
equation
  connect(airSource.port[1],openBraytonCycle. gas_in) annotation (Line(points={{37,-14},
          {30,-14},{30,5.72},{29.32,5.72}},                  color={0,191,0}));
  connect(feedFuel.port,openBraytonCycle. fuel_in) annotation (Line(points={{11,78},
          {6.92,78},{6.92,61.72}},            color={0,191,0}));
  connect(airSource1.port[1], openBraytonCycle.gas_out) annotation (Line(points={{-37,-18},
          {-16.04,-18},{-16.04,6.28}},           color={0,191,0}));
  annotation (experiment(__Dymola_NumberOfIntervals=5000,
        __Dymola_Algorithm="Dassl"));
end OriginalGasTurbine_Test;
