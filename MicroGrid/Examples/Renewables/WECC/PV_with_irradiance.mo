within MicroGrid.Examples.Renewables.WECC;
model PV_with_irradiance
  extends BaseClasses.SMIB_renewable_partial;
  Electrical.Renewables.WECC.GridFollowing.Irradiance_to_Power
    irradiance_to_Power(use_irradiance_out=true)
    annotation (Placement(transformation(extent={{-64,30},{-44,50}})));
  Electrical.Renewables.WECC.PV_Module_for_irradiance
    pV_Module_for_irradiance(use_irradiance_in=true, angle_0=0.02574992)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
equation
  connect(irradiance_to_Power.irradiance_out, pV_Module_for_irradiance.bambu)
    annotation (Line(points={{-43,40},{-35.6,40},{-35.6,11}}, color={0,
          0,127}));
  connect(pV_Module_for_irradiance.pwPin, GEN1.p)
    annotation (Line(points={{-40,0},{40,0}}, color={0,0,255}));
  annotation (experiment(
      StopTime=86400,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end PV_with_irradiance;
