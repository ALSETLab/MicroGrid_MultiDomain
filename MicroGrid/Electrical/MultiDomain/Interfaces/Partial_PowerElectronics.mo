within MicroGrid.Electrical.MultiDomain.Interfaces;
partial model Partial_PowerElectronics
  Modelica.Blocks.Interfaces.RealInput m_in if use_pwmIndex annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-130}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-130})));
        parameter Boolean use_pwmIndex=false
    "= true, supply an external pwm modulation index value (m)."
    annotation (
    Evaluate=true,
    HideResult=true,
    choices(checkBox=true),
    Dialog(group="Options"));
    parameter Real m = 1 annotation(Dialog(group = "Options"));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},
            {120,120}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}})));
end Partial_PowerElectronics;
