within MicroGrid.Examples.SystemExamples.Data.Records;
record pf_DATA
    //Power flow results for the snapshot h30.0_after_PF

  extends Modelica.Icons.Record;

  record Voltages
    // Bus number 1
    parameter Real VGrid=1.00003;
    parameter Real AGrid=0;
    // Bus number 2
    parameter Real VLv=1.00243;
    parameter Real ALv=0;
    // Bus number 3
    parameter Real VSb=1.00171;
    parameter Real ASb=0.000000;
    // Bus number 21
    parameter Real VCentral=0.999712;
    parameter Real ACentral=0;
    // Bus number 22
    parameter Real VLoad=0.958507;
    parameter Real ALoad=0;
    // Bus number 23
    parameter Real VPv=0.999712;
    parameter Real APv=0;
    // Bus number 23
    parameter Real VBess=0.999707;
    parameter Real ABess=0;
    // Bus number 23
    parameter Real VDiesel=1.00079;
    parameter Real ADiesel=0;
    // Bus number 23
    parameter Real VMulti=1.00251;
    parameter Real AMulti=0;

  end Voltages;

  record Machines
    // Machine 3_1
    parameter Real P3_1=0.000000;
    parameter Real Q3_1=0.475000;
    // Machine 1_1
    parameter Real P1_1=3.000000;
    parameter Real Q1_1=0.256000;
  end Machines;

  record Loads
    // Load 23_1
    parameter Real PLoad=15000000;
    parameter Real QLoad=9296200;

    //parameter Real PLoad=150000000;
    //parameter Real QLoad=92962000;
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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end pf_DATA;
