within MicroGrid.Examples.ThermostaticallyControlledLoads;
model TCL_1
  extends OpenIPSL.Examples.BaseClasses.SMIB;
  PowerSystems.Electrical.Loads.PSAT.TCL TCL
    annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
equation
  connect(TCL.p, GEN1.p)
    annotation (Line(points={{-59,0},{-30,0}}, color={0,0,255}));
end TCL_1;
