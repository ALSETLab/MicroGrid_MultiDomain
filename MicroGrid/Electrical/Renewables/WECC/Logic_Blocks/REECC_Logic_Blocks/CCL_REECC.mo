within MicroGrid.Electrical.Renewables.WECC.Logic_Blocks.REECC_Logic_Blocks;
model CCL_REECC
  parameter Real Imax;
  Modelica.Blocks.Interfaces.RealInput VDL1_out annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,60}), iconTransformation(extent={{-120,70},{-100,50}})));
  Modelica.Blocks.Interfaces.BooleanInput pqflag annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={120,0}), iconTransformation(
        extent={{0,0},{20,20}},
        rotation=180,
        origin={120,10})));
  Modelica.Blocks.Interfaces.RealInput VDL2_out annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-60}), iconTransformation(extent={{-120,-50},{-100,-70}})));
  Modelica.Blocks.Interfaces.RealInput iqcmd annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={120,60}), iconTransformation(
        extent={{-6.66681,-6.66658},{13.3339,13.3334}},
        rotation=180,
        origin={113.333,63.3334})));
  Modelica.Blocks.Interfaces.RealInput ipcmd annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={120,-60}), iconTransformation(
        extent={{-4.89859e-15,40},{20,20}},
        rotation=180,
        origin={120,-30})));
  Modelica.Blocks.Interfaces.RealOutput iqmax annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-50,110})));
  Modelica.Blocks.Interfaces.RealOutput iqmin annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-10,110})));
  Modelica.Blocks.Interfaces.RealOutput ipmax annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-50,-110})));
  Modelica.Blocks.Interfaces.RealOutput ipmin annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-10,-110})));
equation

    iqmax = if pqflag == false then min(VDL1_out, Imax) else min(VDL1_out, sqrt(Imax^2 - ipcmd^2));
    iqmin = -iqmax;
    ipmax = if pqflag == false then min(VDL2_out, sqrt(Imax^2-iqcmd^2)) else min(VDL2_out, Imax);
    ipmin = -ipmax;

end CCL_REECC;
