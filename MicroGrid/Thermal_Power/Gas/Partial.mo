within MicroGrid.Thermal_Power.Gas;
package Partial
  partial model Turbine
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Interfaces.RealInput fuel_mass_flow annotation (
        Placement(transformation(extent={{-140,-20},{-100,20}})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Turbine;
end Partial;
