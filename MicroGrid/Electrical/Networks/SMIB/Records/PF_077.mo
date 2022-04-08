within MicroGrid.Electrical.Networks.SMIB.Records;
record PF_077
  //Power flow results for the snapshot h77.0_after_PF

  extends Modelica.Icons.Record;

  record Voltages
    // Bus number 1
    parameter OpenIPSL.Types.PerUnit V1=1.000000;
    parameter OpenIPSL.Types.Angle A1=8.060300*Modelica.Constants.pi/180;
    // Bus number 2
    parameter OpenIPSL.Types.PerUnit V2=0.989290;
    parameter OpenIPSL.Types.Angle A2=1.355700*Modelica.Constants.pi/180;
    // Bus number 3
    parameter OpenIPSL.Types.PerUnit V3=1.000000;
    parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
    // Bus number 21
    parameter OpenIPSL.Types.PerUnit V21=0.985170;
    parameter OpenIPSL.Types.Angle A21=-0.947700*Modelica.Constants.pi/180;
    // Bus number 22
    parameter OpenIPSL.Types.PerUnit V22=0.992550;
    parameter OpenIPSL.Types.Angle A22=-0.470300*Modelica.Constants.pi/180;
    // Bus number 23
    parameter OpenIPSL.Types.PerUnit V23=0.985170;
    parameter OpenIPSL.Types.Angle A23=-0.947700*Modelica.Constants.pi/180;
  end Voltages;

  record Machines
    // Machine 3_1
    parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
    parameter OpenIPSL.Types.ReactivePower Q3_1=1.618000e6;
    // Machine 1_1
    parameter OpenIPSL.Types.ActivePower P1_1=7.700000e6;
    parameter OpenIPSL.Types.ReactivePower Q1_1=1.165000e6;
  end Machines;

  record Loads
    // Load 23_1
    parameter OpenIPSL.Types.ActivePower PL23_1=7.700000e6;
    parameter OpenIPSL.Types.ReactivePower QL23_1=1.540000e6;
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
end PF_077;
