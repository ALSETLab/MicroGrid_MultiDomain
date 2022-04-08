within MicroGrid.Electrical.Networks.SMIB.Records;
record Voltages "P,Q power flow results of every bus"
  extends Modelica.Icons.Record;
  // Bus number 1
  parameter OpenIPSL.Types.PerUnit V1=1.000000;
  parameter OpenIPSL.Types.Angle A1=0.0907344318234292;
  // Bus number 2
  parameter OpenIPSL.Types.PerUnit V2=0.994050;
  parameter OpenIPSL.Types.Angle A2=0.01521403508963457;
  // Bus number 3
  parameter OpenIPSL.Types.PerUnit V3=1.000000;
  parameter OpenIPSL.Types.Angle A3=0.000000;
  // Bus number 21
  parameter OpenIPSL.Types.PerUnit V21=0.991250;
  parameter OpenIPSL.Types.Angle A21=-0.010623819156889484;
  // Bus number 22
  parameter OpenIPSL.Types.PerUnit V22=0.995610;
  parameter OpenIPSL.Types.Angle A22=-0.005288347633542819;
  // Bus number 23
  parameter OpenIPSL.Types.PerUnit V23=0.991250;
  parameter OpenIPSL.Types.Angle A23=-0.010623819156889484;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Voltages;
