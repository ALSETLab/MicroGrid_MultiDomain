within MicroGrid.Thermal_Power.ThermalFluid_Sources.Models.Gas_Turbines;
model OpenCycleGasTurbine
  "Model of gas turbine, compressor and combustor"
  package Medium_Fuel =ThermalPower.Media.Gases.NaturalGasWithH2;
  package Medium =ThermalPower.Media.Gases.MoistFlueGas;

  Modelon.Visualizers.RealValue realValue1(
      precision=0, number=turbine.summary.shaftSpeed*60/(2*3.14))
    annotation (Placement(transformation(extent={{-45,-71},{-25,-51}})));
  Modelon.Visualizers.RealValue realValue2(
                precision=2, number=combustor.Q_combust*1e-6)
    annotation (Placement(transformation(extent={{-17,-71},{3,-51}})));
  Modelon.Visualizers.RealValue realValue3(
               precision=2, number=combustor.gas_out.p*1e-5)
    annotation (Placement(transformation(extent={{11,-71},{31,-51}})));
  Modelon.Visualizers.RealValue realValue4(
      precision=2, number=combustor.lambda)
    annotation (Placement(transformation(extent={{39,-71},{59,-51}})));
  ThermalPower.FlueGas.Sensors.MultiData multiData(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-86,-40},{-66,-20}})));
  ThermalPower.Visualizers.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot(
      displayUnits=true)
    annotation (Placement(transformation(extent={{-91,-35},{-61,-5}})));
  ThermalPower.FlueGas.Sensors.MultiData multiData1(redeclare package Medium =
        Medium_Fuel)
    annotation (Placement(transformation(extent={{-36,70},{-16,90}})));
  ThermalPower.Visualizers.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot1(
      displayUnits=true)
    annotation (Placement(transformation(extent={{-41,75},{-11,105}})));
  ThermalPower.FlueGas.Sensors.MultiData multiData2(redeclare package Medium =
        Medium) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={82,-20})));
  ThermalPower.Visualizers.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot2(
      displayUnits=true)
    annotation (Placement(transformation(extent={{87,-25},{117,5}})));
  Modelon.ThermoFluid.Compressors.DynamicCompressor compressor(
    redeclare package Medium = Medium,
    eta_mech=eta_mech_compressor,
    positiveFlow=false,
    internalLeakage=false,
    redeclare model CharMap = CharMap_compressor,
    T_start=293.15)  annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-48,20})));
  ThermalPower.FlueGas.TurboMachinery.Turbines.Turbine turbine(
    redeclare package Medium = Medium,
    w_start=w_start,
    redeclare model CharMap = CharMap,
    T_start(displayUnit="K") = Tstart_turbine,
    eta_mech=eta_mech) annotation (Placement(transformation(
        extent={{16,16},{-16,-16}},
        rotation=180,
        origin={32,12})));
  ThermalPower.FlueGas.Combustors.Combustor combustor(
    A_inner=A_inner,
    kc_combust=kc_combust,
    Cm=Cm,
    V=V_combustor,
    pstart=pstart_combustor,
    Tstart=Tstart_combustor,
    initOpt=initOpt_combustor) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-12,42})));

  ThermalPower.FlueGas.Sensors.MultiData multiData3(redeclare package Medium =
        Medium) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={12,30})));
  ThermalPower.Visualizers.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot3(
      displayUnits=true)
    annotation (Placement(transformation(extent={{23,25},{53,55}})));
  ThermalPower.Visualizers.MultiDisplayVis_phTmdot multiDisplayVis_phTmdot4(
      displayUnits=true)
    annotation (Placement(transformation(extent={{-81,29},{-51,59}})));
  ThermalPower.FlueGas.Sensors.MultiData multiData4(redeclare package Medium =
        Medium) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-38,34})));
  ThermalPower.FlueGas.Interfaces.VolumePort fuel_in(redeclare package Medium =
        Medium_Fuel) annotation (Placement(transformation(extent={{-82,70},{-62,
            90}}), iconTransformation(extent={{-46,92},{-32,106}})));
  ThermalPower.FlueGas.Interfaces.FlowPort gas_out(redeclare package Medium =
        Medium) annotation (Placement(transformation(extent={{72,-108},{92,-88}}),
        iconTransformation(extent={{36,-106},{50,-92}})));
  ThermalPower.FlueGas.Interfaces.VolumePort gas_in(redeclare package Medium =
        Medium) annotation (Placement(transformation(extent={{-132,-114},{-112,-94}}),
        iconTransformation(extent={{-126,-108},{-112,-94}})));
  replaceable model CharMap =
      Modelon.ThermoFluid.Turbines.Characteristics.TableBasedSAE (normalizedCorrection=true, effMap=[0, 1, 2, 3, 4, 6, 9, 12, 16, 20, 25, 30; 20000, 0.85, 0.853, 0.856, 0.86, 0.865, 0.87, 0.8725, 0.875, 0.8752, 0.8754, 0.876; 40000, 0.85, 0.853, 0.856, 0.86, 0.865, 0.87, 0.8725, 0.875, 0.8752, 0.8754, 0.876; 60000, 0.85, 0.853, 0.856, 0.86, 0.865, 0.87, 0.8725, 0.875, 0.8752, 0.8754, 0.876], flowMap=[0, 20, 25, 30; 0, 0.0002, 0.0002, 0.0002; 10000, 180, 210, 225; 40000, 220, 235, 242; 60000, 250, 267, 281], p_ref=450000, T_ref=973.15)
    constrainedby Modelon.ThermoFluid.Turbines.Characteristics.TableBasedSAE annotation (__Dymola_choicesAllMatching=true, Dialog(group="Turbine"));
  parameter Real eta_mech=0.99 "Mechanical efficiency" annotation(Dialog(group="Turbine"));
  parameter Modelica.Units.SI.Temperature T_start=573.15
    "Initial temperature" annotation (Dialog(group="Turbine"));
  parameter Modelica.Units.SI.AngularVelocity w_start=1000
    "Initial angular velocity" annotation (Dialog(group="Turbine"));
  replaceable model CharMap_compressor =
      Modelon.ThermoFluid.Compressors.Characteristics.Dynamic.TableBasedFlowFromPR
      (                                                                             normalizedCorrection=true, effMap=[0, 1, 2, 4, 6, 9, 12, 16, 20, 25, 30; 10000, 0.85, 0.84, 0.82, 0.81, 0.795, 0.789, 0.785, 0.783, 0.781, 0.78; 40000, 0.85, 0.84, 0.82, 0.81, 0.795, 0.789, 0.785, 0.783, 0.781, 0.78; 60000, 0.85, 0.84, 0.82, 0.81, 0.795, 0.789, 0.785, 0.783, 0.781, 0.78], flowMap=[0, 20, 25, 30; 0, 0.00002, 0.00002, 0.00002; 10000, 900, 870, 820; 40000, 950, 920, 900; 60000, 980, 940, 920], p_ref=200000, T_ref=973.15)
    constrainedby
    Modelon.ThermoFluid.Compressors.Characteristics.Dynamic.TableBasedSAE               annotation (__Dymola_choicesAllMatching=true, Dialog(group="Compressor"));
  parameter Real eta_mech_compressor=1.0 "Mechanical efficiency" annotation(Dialog(group="Compressor"));
  parameter Boolean internalLeakage_compressor=false "Include internal mass flow leakage" annotation(Dialog(group="Compressor"));

  parameter Modelica.Units.SI.HeatCapacity Cm=0 "Metal Heat Capacity"
    annotation (Dialog(group="Combustor"));
  parameter Modelica.Units.SI.Volume V_combustor=10 "Inner volume"
    annotation (Dialog(group="Combustor"));
  parameter Modelica.Units.SI.Area A_inner=0 "Inner surface"
    annotation (Dialog(group="Combustor"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer kc_combust=0
    "Heat Transfer Coefficient" annotation (Dialog(group="Combustor"));
  parameter Modelica.Units.SI.Pressure pstart_combustor=2400000
    "Combustor pressure start value"
    annotation (Dialog(group="Combustor"));
  parameter Modelica.Units.SI.Temperature Tstart_combustor=1648.15
    "Combustor temperature start value"
    annotation (Dialog(group="Combustor"));
  parameter Modelica.Units.SI.Temperature Tstart_turbine=573.15
    "Turbine temperature start value"
    annotation (Dialog(group="Combustor"));
  parameter Modelon.ThermoFluid.Choices.InitOptions initOpt_combustor=Modelon.ThermoFluid.Choices.InitOptions.steadyState "Initialization option" annotation(Dialog(group="Combustor"));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_b1 annotation (
      Placement(transformation(extent={{136,-10},{156,10}}), iconTransformation(
          extent={{136,-10},{156,10}})));
equation

  connect(multiData.u, multiDisplayVis_phTmdot.y)
    annotation (Line(points={{-76,-30},{-76,-30},{-76,-20}}, color={0,0,0}));
  connect(multiData1.u, multiDisplayVis_phTmdot1.y) annotation (Line(points={
          {-26,80},{-26,84},{-26,86},{-26,90}}, color={0,0,0}));
  connect(multiDisplayVis_phTmdot2.y, multiData2.u) annotation (Line(points={{102,-10},
          {102,-20},{82,-20}},                  color={0,0,0}));
  connect(compressor.portA, multiData.port_b) annotation (Line(points={{-58,20},
          {-58,-30},{-70,-30}},     color={0,191,0}));
  connect(combustor.gas_out, multiData3.port_a)
    annotation (Line(points={{-3,42},{12,42},{12,36}}, color={0,191,0}));
  connect(multiDisplayVis_phTmdot3.y, multiData3.u) annotation (Line(points={
          {38,40},{38,40},{38,30},{12,30}}, color={0,0,0}));
  connect(compressor.flange, turbine.shaft_a) annotation (Line(points={{-48,13},
          {18,13},{18,12.32},{17.28,12.32}},    color={0,0,0}));
  connect(multiData3.port_b, turbine.port_a)
    annotation (Line(points={{12,24},{12,21.92},{17.6,21.92}},
                                                         color={0,191,0}));
  connect(turbine.port_b, multiData2.port_a) annotation (Line(points={{45.76,
          1.12},{82,1.12},{82,-14}}, color={0,191,0}));
  connect(multiData4.u, multiDisplayVis_phTmdot4.y)
    annotation (Line(points={{-38,34},{-66,34},{-66,44}}, color={0,0,0}));
  connect(compressor.portB, multiData4.port_a) annotation (Line(points={{-38,20},
          {-38,28}},              color={255,128,0}));
  connect(multiData4.port_b, combustor.gas_in) annotation (Line(points={{-38,
          40},{-38,40},{-38,42},{-21,42}}, color={0,191,0}));
  connect(multiData1.port_b, combustor.fuel_in)
    annotation (Line(points={{-20,80},{-12,80},{-12,51}}, color={0,191,0}));
  connect(gas_out, multiData2.port_b) annotation (Line(points={{82,-98},{82,-26}},
                                       color={0,191,0}));
  connect(fuel_in, multiData1.port_a)
    annotation (Line(points={{-72,80},{-32,80}}, color={0,191,0}));
  connect(gas_in, multiData.port_a) annotation (Line(points={{-122,-104},{-122,
          -30},{-82,-30}},      color={0,191,0}));
  connect(turbine.shaft_b, shaft_b1) annotation (Line(points={{46.08,12.32},{128,
          12.32},{128,0},{146,0}}, color={0,0,0}));
  annotation (Diagram(coordinateSystem(extent={{-150,-100},{150,100}}),
                               graphics={
        Rectangle(
          extent={{-80,-48},{68,-74}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          radius=2),
        Text(
          extent={{-84,-66},{-34,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Generator [MW]"),
        Text(
          extent={{-56,-66},{-6,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Speed [rpm]"),
        Text(
          extent={{-28,-66},{22,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Q combust [MW]"),
        Text(
          extent={{0,-66},{50,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="Combustor [bar]"),
        Text(
          extent={{28,-66},{78,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="lambda")}),
    experiment(StopTime=1000),
    __Dymola_experimentSetupOutput,
    Documentation(revisions="<html>
<hr><p><font color=\"#E72614\"><b>Copyright &copy; 2004-2020, MODELON AB</b></font> <font color=\"#AFAFAF\"><br /><br /> The use of this software component is regulated by the licensing conditions for Modelon Libraries. <br /> This copyright notice must, unaltered, accompany all components that are derived from, copied from, <br /> or by other means have their origin from any Modelon Library. </font></p>
</html>", info="<html>
<h4>Open brayton cycle</h4>
<p>The open Brayton cycle is a system model component consisting of a combustor, compressor and turbine. Ambient air supplied to the compressor is 
compressed with a compression ratio in the range 20 to 25. The compressed air is then introduced in the combustor where fuel is added as well. The default fuel used in the model is natural gas.
After combustion, which is assumed to be complete, the exhaust gas is fed to the turbine. Work produced by the turbine is supplied both to a generator and the compressor.</p>
</html>"),
    Icon(coordinateSystem(extent={{-150,-100},{150,100}}), graphics={
        Rectangle(
          extent={{-53,1.5},{52,-1.5}},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={-38,60.25},
          rotation=0),
        Rectangle(
          extent={{-135,5},{135,-5}},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={3,1},
          rotation=360),
        Line(
          points={{-142,80}},
          color={0,0,0},
          pattern=LinePattern.None),
        Rectangle(
          extent={{-136,28},{-78,-30}},
          fillColor={119,216,119},
          fillPattern=FillPattern.Solid,
          radius=3,
          pattern=LinePattern.None),
        Rectangle(
          extent={{2,28},{60,-30}},
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          radius=3,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-58,80},{-18,40}},
          pattern=LinePattern.None,
          fillColor={119,216,119},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-38.5,1.5},{38.5,-1.5}},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={42.5,-60.5},
          rotation=-90),
        Polygon(
          points={{-15,15},{-15,-7},{11,-21},{11,29},{-15,15}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          origin={31,-5},
          rotation=360),
        Ellipse(
          extent={{-54,76},{-22,44}},
          pattern=LinePattern.None,
          fillColor={255,168,112},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{-15,-11},{11,15},{15,13},{-11,-13},{-15,-11}},
          lineColor={28,108,200},
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          origin={-37,59},
          rotation=90),
        Polygon(
          points={{-52,50},{-26,76},{-24,72},{-50,46},{-52,50}},
          lineColor={28,108,200},
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{-82,-46},{10,-68}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          textString="%name"),
        Rectangle(
          extent={{-38.5,1.5},{38.5,-1.5}},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={-119.5,-56.5},
          rotation=-90),
        Rectangle(
          extent={{-38.5,1.5},{38.5,-1.5}},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={-89.5,21.5},
          rotation=90),
        Rectangle(
          extent={{-38.5,1.5},{38.5,-1.5}},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={12.5,21.5},
          rotation=90),
        Polygon(
          points={{15,15},{15,-7},{-11,-21},{-11,29},{15,15}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          origin={-107,-5},
          rotation=360),           Rectangle(
          extent={{51,-33},{150,-59}},
          lineColor={175,175,175},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
                          Text(
          extent={{27,-76},{177,-66}},
          lineColor={0,0,0},
          textString="Power [MW]"),      Text(
          extent={{32,-58},{172,-37}},
          lineColor={0,0,0},
          textString=DynamicSelect("0",
              String(
              generator.power/1e6,
              format="1.f"))),
        Rectangle(
          extent={{-10,1},{10,-1}},
          pattern=LinePattern.None,
          fillColor={119,217,119},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          origin={-39,88},
          rotation=-90),                 Text(
          extent={{-184,72},{-44,93}},
          lineColor={0,0,0},
          textString=DynamicSelect("0",
              String(
              generator.power/1e6,
              format="1.f"))),
                          Text(
          extent={{-193,100},{-43,110}},
          lineColor={0,0,0},
          textString="Firing Power [MW]"),
                                   Rectangle(
          extent={{-165,97},{-66,71}},
          lineColor={255,168,112},
          fillColor={255,168,112},
          fillPattern=FillPattern.Solid),Text(
          extent={{-186,72},{-46,93}},
          lineColor={0,0,0},
          textString=DynamicSelect("0",
              String(
              combustor.Q_combust/1e6,
              format="1.f")))}));
end OpenCycleGasTurbine;
