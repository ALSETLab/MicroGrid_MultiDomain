within MicroGrid.Thermal_Power.GasTurbine_GENROU_ThermalPower;
model Turbine_Comparison_Test
  MODIFIED_GAS_TURBINE mODIFIED_GAS_TURBINE
    annotation (Placement(transformation(extent={{-38,-6},{-8,14}})));
  ThermalPower.FlueGas.SourcesAndSinks.PressureBoundary_pTX AIR_IN(
    redeclare package Medium = ThermalPower.Media.Gases.MoistFlueGas,
    p0=34300,
    T0=244.4,
    N_ports=1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-72,-14})));
  ThermalPower.FlueGas.SourcesAndSinks.PressureBoundary_pTX airSource1(
    redeclare package Medium = ThermalPower.Media.Gases.MoistFlueGas,
    p0=152000,
    T0=800,
    N_ports=1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-20,-30})));
  ThermalPower.FlueGas.SourcesAndSinks.MassFlowBoundary feedFuel(
    redeclare package Medium = ThermalPower.Media.Gases.NaturalGasWithH2,
    p0=811000,
    m_flow0=4.9,
    use_mdot_in=false) annotation (Placement(transformation(extent={{16,14},{-4,
            34}}, rotation=0)));
  inner OpenIPSL.Electrical.SystemBase SysData
    annotation (Placement(transformation(extent={{48,68},{96,94}})));
equation
  connect(AIR_IN.port[1], mODIFIED_GAS_TURBINE.gas_in) annotation (Line(
        points={{-63,-14},{-34.9,-14},{-34.9,-6.1}}, color={0,191,0}));
  connect(airSource1.port[1], mODIFIED_GAS_TURBINE.gas_out) annotation (
      Line(points={{-20,-21},{-20,-12},{-18.7,-12},{-18.7,-5.9}}, color={0,
          191,0}));
  connect(feedFuel.port, mODIFIED_GAS_TURBINE.fuel_in) annotation (Line(
        points={{-3,24},{-26.9,24},{-26.9,13.9}}, color={0,191,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Turbine_Comparison_Test;
