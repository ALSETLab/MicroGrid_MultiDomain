within MicroGrid.Thermal_Power.ThermalPower_testing_package;
model Test_model

  ThermalFluid_Sources.Models.Gas_Turbines.OpenCycleGasTurbine
    openCycleGasTurbine
    annotation (Placement(transformation(extent={{-12,-10},{18,10}})));
  TM2EPConverter tM2EPConverter
    annotation (Placement(transformation(extent={{32,-10},{52,10}})));
  ThermalPower.FlueGas.FlowChannels.Pipe pipe
    annotation (Placement(transformation(extent={{32,-90},{52,-70}})));
  ThermalPower.FlueGas.SourcesAndSinks.MassFlowBoundary massFlowBoundary
    annotation (Placement(transformation(extent={{-52,14},{-32,34}})));
equation
  connect(openCycleGasTurbine.shaft_b1, tM2EPConverter.shaft)
    annotation (Line(points={{17.6,0},{32,0}}, color={0,0,0}));
end Test_model;
