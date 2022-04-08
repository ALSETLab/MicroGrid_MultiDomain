within MicroGrid.Electrical.Networks.SMIB.Records;
record Loads "P,Q power flow results of the loads"
  extends Modelica.Icons.Record;
  // Load 23_1
  parameter OpenIPSL.Types.ActivePower PL23_1=5.000000e6;
  parameter OpenIPSL.Types.ReactivePower QL23_1=1.000000e6;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Loads;
