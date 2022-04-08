within MicroGrid.Electrical.Networks.SMIB.Records;
record Machines "P,Q power flow results of the machines"
  extends Modelica.Icons.Record;
  // Machine 3_1
  parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
  parameter OpenIPSL.Types.ReactivePower Q3_1=0.932000e6;
  // Machine 1_1
  parameter OpenIPSL.Types.ActivePower P1_1=5.000000e6;
  parameter OpenIPSL.Types.ReactivePower Q1_1=0.585000e6;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Machines;
