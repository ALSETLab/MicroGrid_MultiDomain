within MicroGrid.Electrical.Networks.SMIB.Records;
record PF_071
  //Power flow results for the snapshot h71.0_after_PF

  extends Modelica.Icons.Record;

  record Voltages
    // Bus number 1
    parameter OpenIPSL.Types.PerUnit V1=1.000000;
    parameter OpenIPSL.Types.Angle A1=7.419900*Modelica.Constants.pi/180;
    // Bus number 2
    parameter OpenIPSL.Types.PerUnit V2=0.990450;
    parameter OpenIPSL.Types.Angle A2=1.247100*Modelica.Constants.pi/180;
    // Bus number 3
    parameter OpenIPSL.Types.PerUnit V3=1.000000;
    parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
    // Bus number 21
    parameter OpenIPSL.Types.PerUnit V21=0.986610;
    parameter OpenIPSL.Types.Angle A21=-0.871600*Modelica.Constants.pi/180;
    // Bus number 22
    parameter OpenIPSL.Types.PerUnit V22=0.993280;
    parameter OpenIPSL.Types.Angle A22=-0.432800*Modelica.Constants.pi/180;
    // Bus number 23
    parameter OpenIPSL.Types.PerUnit V23=0.986610;
    parameter OpenIPSL.Types.Angle A23=-0.871600*Modelica.Constants.pi/180;
  end Voltages;

  record Machines
    // Machine 3_1
    parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
    parameter OpenIPSL.Types.ReactivePower Q3_1=1.454000e6;
    // Machine 1_1
    parameter OpenIPSL.Types.ActivePower P1_1=7.100000e6;
    parameter OpenIPSL.Types.ReactivePower Q1_1=1.020000e6;
  end Machines;

  record Loads
    // Load 23_1
    parameter OpenIPSL.Types.ActivePower PL23_1=7.100000e6;
    parameter OpenIPSL.Types.ReactivePower QL23_1=1.420000e6;
  end Loads;

  record Trafos
    // 2WindingTrafo 1_2
    parameter Real t1_1_2=1.000000;
    parameter Real t2_1_2=1.000000;
  end Trafos;
  Voltages voltages;
  Machines machines;
  Loads loads;
  Trafos trafos;
end PF_071;
