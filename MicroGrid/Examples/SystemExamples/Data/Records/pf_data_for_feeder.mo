within MicroGrid.Examples.SystemExamples.Data.Records;
record pf_data_for_feeder
    //Power flow results for the snapshot h30.0_after_PF

  extends Modelica.Icons.Record;

  record Voltages
    // Bus number 1
    parameter Real VGrid=1;
    parameter Real AGrid=0;
    // Bus number 4
    parameter Real V4=0.991;
    parameter Real A4=-0.370;
    // Bus number 5
    parameter Real V5=0.988;
    parameter Real A5=-0.544;
    // Bus number 6
    parameter Real V6=0.986;
    parameter Real A6=-0.697;
    // Bus number 7
    parameter Real V7=0.985;
    parameter Real A7=-0.704;
    // Bus number 8
    parameter Real V8=0.979;
    parameter Real A8=-0.763;
    // Bus number 9
    parameter Real V9=0.971;
    parameter Real A9=-1.451;
    // Bus number 10
    parameter Real V10=0.977;
    parameter Real A10=-0.770;
    // Bus number 11
    parameter Real V11=0.971;
    parameter Real A11=-1.525;
    // Bus number 12
    parameter Real V12=0.969;
    parameter Real A12=-1.836;
    // Bus number 13
    parameter Real V13=0.994;
    parameter Real A13=-0.332;
    // Bus number 14
    parameter Real V14=0.995;
    parameter Real A14=-0.459;
    // Bus number 15
    parameter Real V15=0.992;
    parameter Real A15=-0.527;
    // Bus number 16
    parameter Real V16=0.991;
    parameter Real A16=-0.596;
  end Voltages;

  record Machines

  end Machines;

  record Loads
    // Load 1_4
    parameter Real P4=2000000;
    parameter Real Q4=1600000;

    // Load 4_5
    parameter Real P5=3000000;
    parameter Real Q5=1500000;

    // Load 4_6
    parameter Real P6=2000000;
    parameter Real Q6=800000;

    // Load 6_7
    parameter Real P7=1500000;
    parameter Real Q7=1200000;

    // Load 2_8
    parameter Real P8=4000000;
    parameter Real Q8=2700000;

    // Load 8_9
    parameter Real P9=5000000;
    parameter Real Q9=3000000;

    // Load 8_10
    parameter Real P10=1000000;
    parameter Real Q10=900000;

    // Load 9_11
    parameter Real P11=600000;
    parameter Real Q11=100000;

    // Load 9_12
    parameter Real P12=4500000;
    parameter Real Q12=2000000;

    // Load 3_13
    parameter Real P13=1000000;
    parameter Real Q13=900000;

    // Load 13_14
    parameter Real P14=1000000;
    parameter Real Q14=700000;

    // Load 13_15
    parameter Real P15=1000000;
    parameter Real Q15=900000;

    // Load 15_16
    parameter Real P16=2100000;
    parameter Real Q16=1000000;

  end Loads;

  record Trafos
  end Trafos;
  Voltages voltages;
  Machines machines;
  Loads loads;
  Trafos trafos;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end pf_data_for_feeder;
