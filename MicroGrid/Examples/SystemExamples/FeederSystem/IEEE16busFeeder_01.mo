within MicroGrid.Examples.SystemExamples.FeederSystem;
model IEEE16busFeeder_01
  extends Modelica.Icons.Example;
  import MicroGrid;
  OpenIPSL.Electrical.Branches.PwLine line_1_4(
    R=0.075,
    X=0.1,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-110,250})));
  OpenIPSL.Electrical.Buses.Bus Bus1(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-160,220})));
  OpenIPSL.Electrical.Buses.Bus GRID_bus(
    V_b=11000,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true)
               annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-258,160})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS GRID(
    V_b=11000,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    P_0=160150.6,
    Q_0=-82900,
    M_b=9999000000,
    R_a=0.01,
    X_d=0.1)
    annotation (Placement(transformation(extent={{-340,140},{-300,180}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
    annotation (Placement(transformation(extent={{-346,-452},{-206,-372}})));
  OpenIPSL.Electrical.Branches.PwLine line_3_13(
    R=2*0.11,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-90,-340})));
  OpenIPSL.Electrical.Buses.Bus Bus2(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder2" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-160,10})));
  OpenIPSL.Electrical.Buses.Bus Bus3(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder3" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-160,-308})));
  OpenIPSL.Electrical.Branches.PwLine line_4_5(
    R=2*0.08,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={70,140})));
  OpenIPSL.Electrical.Buses.Bus Bus4(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V4,
    angle_0=pf_data_for_feeder.voltages.A4,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={0,220})));
  OpenIPSL.Electrical.Loads.PSSE.Load load4(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P4,
    Q_0=pf_data_for_feeder.loads.Q4,
    v_0=pf_data_for_feeder.voltages.V4,
    angle_0=pf_data_for_feeder.voltages.A4)    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-20,260})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank_6(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={150,320})));
  OpenIPSL.Electrical.Branches.PwLine line_4_6(
    R=0.09,
    X=0.18,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={90,300})));
  OpenIPSL.Electrical.Buses.Bus Bus6(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V6,
    angle_0=pf_data_for_feeder.voltages.A6,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={140,270})));
  OpenIPSL.Electrical.Loads.PSSE.Load load6(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P6,
    Q_0=pf_data_for_feeder.loads.Q6,
    v_0=pf_data_for_feeder.voltages.V6,
    angle_0=pf_data_for_feeder.voltages.A6)    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={170,320})));
  OpenIPSL.Electrical.Branches.PwLine line_6_7(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={250,300})));
  OpenIPSL.Electrical.Buses.Bus Bus7(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V7,
    angle_0=pf_data_for_feeder.voltages.A7,
    displayPF=true) "Three feeder example system" annotation (Placement(
        transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={300,270})));
  OpenIPSL.Electrical.Buses.Bus Bus5(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V5,
    angle_0=pf_data_for_feeder.voltages.A5,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={140,170})));
  OpenIPSL.Electrical.Branches.PwLine line_4_1(
    R=2*0.08,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={230,140})));
  OpenIPSL.Electrical.Buses.Bus Bus11(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V11,
    angle_0=pf_data_for_feeder.voltages.A11,
    displayPF=true) "Three feeder example system" annotation (Placement(
        transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={340,170})));
  OpenIPSL.Electrical.Buses.Bus Bus8(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V8,
    angle_0=pf_data_for_feeder.voltages.A8,
    displayPF=true) "Feeder2" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={0,10})));
  OpenIPSL.Electrical.Branches.PwLine line_2_8(
    R=2*0.11,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-90,-20})));
  OpenIPSL.Electrical.Buses.Bus Bus9(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V9,
    angle_0=pf_data_for_feeder.voltages.A9,
    displayPF=true) "Feeder2" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={140,10})));
  OpenIPSL.Electrical.Branches.PwLine line_8_9(
    R=2*0.08,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={70,-20})));
  OpenIPSL.Electrical.Buses.Bus Bus12(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V12,
    angle_0=pf_data_for_feeder.voltages.A12,
    displayPF=true) "Feeder2" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={340,-30})));
  OpenIPSL.Electrical.Branches.PwLine line_8_1(
    R=2*0.08,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={250,-60})));
  OpenIPSL.Electrical.Branches.PwLine line_9_11(
    R=2*0.11,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={250,40})));
  OpenIPSL.Electrical.Loads.PSSE.Load load12(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P12,
    Q_0=pf_data_for_feeder.loads.Q12,
    v_0=pf_data_for_feeder.voltages.V12,
    angle_0=pf_data_for_feeder.voltages.A12)   annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={370,20})));
  OpenIPSL.Electrical.Buses.Bus Bus13(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V13,
    angle_0=pf_data_for_feeder.voltages.A13,
    displayPF=true) "Feeder3" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={0,-310})));
  OpenIPSL.Electrical.Buses.Bus Bus15(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V15,
    angle_0=pf_data_for_feeder.voltages.A15,
    displayPF=true) "Feeder3" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={140,-350})));
  OpenIPSL.Electrical.Branches.PwLine line_13_15(
    R=2*0.08,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={70,-380})));
  OpenIPSL.Electrical.Buses.Bus Bus16(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V16,
    angle_0=pf_data_for_feeder.voltages.A16,
    displayPF=true) "Feeder3" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={300,-350})));
  OpenIPSL.Electrical.Branches.PwLine line_15_16(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={230,-380})));
  OpenIPSL.Electrical.Branches.PwLine line_7_16(
    R=2*0.04,
    X=2*0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={460,-22})));
  OpenIPSL.Electrical.Branches.PwLine line_8_10(
    R=2*0.11,
    X=2*0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={110,-160})));
  OpenIPSL.Electrical.Buses.Bus Bus10(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V10,
    angle_0=pf_data_for_feeder.voltages.A10,
    displayPF=true) "Feeder2" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={180,-130})));
  OpenIPSL.Electrical.Branches.PwLine line_10_14(
    R=2*0.04,
    X=2*0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={250,-160})));
  OpenIPSL.Electrical.Buses.Bus Bus14(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V10,
    angle_0=pf_data_for_feeder.voltages.A10,
    displayPF=true) "Feeder2" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={340,-130})));
  OpenIPSL.Electrical.Branches.PwLine line_13_14(
    R=2*0.09,
    X=2*0.12,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={210,-260})));
  OpenIPSL.Electrical.Loads.PSSE.Load load5(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P5,
    Q_0=pf_data_for_feeder.loads.Q5,
    v_0=pf_data_for_feeder.voltages.V5,
    angle_0=pf_data_for_feeder.voltages.A5)    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={170,220})));
  OpenIPSL.Electrical.Loads.PSSE.Load load7(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P7,
    Q_0=pf_data_for_feeder.loads.Q7,
    v_0=pf_data_for_feeder.voltages.V7,
    angle_0=pf_data_for_feeder.voltages.A7)    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={320,310})));
  OpenIPSL.Electrical.Loads.PSSE.Load load8(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P8,
    Q_0=pf_data_for_feeder.loads.Q8,
    v_0=pf_data_for_feeder.voltages.V8,
    angle_0=pf_data_for_feeder.voltages.A8)    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-20,50})));
  OpenIPSL.Electrical.Loads.PSSE.Load load10(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P10,
    Q_0=pf_data_for_feeder.loads.Q10,
    v_0=pf_data_for_feeder.voltages.V10,
    angle_0=pf_data_for_feeder.voltages.A10)   annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={40,-90})));
  OpenIPSL.Electrical.Loads.PSSE.Load load11(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P11,
    Q_0=pf_data_for_feeder.loads.Q11,
    v_0=pf_data_for_feeder.voltages.V11,
    angle_0=pf_data_for_feeder.voltages.A11)   annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={370,220})));
  OpenIPSL.Electrical.Loads.PSSE.Load load13(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P13,
    Q_0=pf_data_for_feeder.loads.Q13,
    v_0=pf_data_for_feeder.voltages.V13,
    angle_0=pf_data_for_feeder.voltages.A13)   annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-20,-270})));
  OpenIPSL.Electrical.Loads.PSSE.Load load14(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P14,
    Q_0=pf_data_for_feeder.loads.Q14,
    v_0=pf_data_for_feeder.voltages.V14,
    angle_0=pf_data_for_feeder.voltages.A14)   annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={370,-80})));
  OpenIPSL.Electrical.Loads.PSSE.Load load15(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P15,
    Q_0=pf_data_for_feeder.loads.Q15,
    v_0=pf_data_for_feeder.voltages.V15,
    angle_0=pf_data_for_feeder.voltages.A15)   annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={160,-310})));
  OpenIPSL.Electrical.Loads.PSSE.Load load16(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P16,
    Q_0=pf_data_for_feeder.loads.Q16,
    v_0=pf_data_for_feeder.voltages.V16,
    angle_0=pf_data_for_feeder.voltages.A16)   annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={330,-300})));
  OpenIPSL.Electrical.Loads.PSSE.Load load9(
    V_b=400,
    P_0=pf_data_for_feeder.loads.P9,
    Q_0=pf_data_for_feeder.loads.Q9,
    v_0=pf_data_for_feeder.voltages.V9,
    angle_0=pf_data_for_feeder.voltages.A9)    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={180,60})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank_1(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={150,220})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank9(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={160,60})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank12(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={350,20})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank11(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={350,220})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank14(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={350,-80})));
  OpenIPSL.Electrical.Banks.PSSE.Shunt capacitor_bank16(G=0, B=0.02/4)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={310,-300})));
  OpenIPSL.Electrical.Branches.PwLine line_1_2(
    R=2*0.075/2,
    X=2*0.1/2,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-90,190})));
  OpenIPSL.Electrical.Branches.PwLine line_1_3(
    R=0.075,
    X=0.1,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-70,250})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_1to4(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-90,250})));
  OpenIPSL.Electrical.Branches.PwLine line_4_2(
    R=2*0.09,
    X=2*0.18,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={70,240})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_4to6(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V4,
    angle_0=pf_data_for_feeder.voltages.A4,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={70,300})));
  OpenIPSL.Electrical.Branches.PwLine line_4_3(
    R=0.09,
    X=0.18,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={50,300})));
  OpenIPSL.Electrical.Branches.PwLine line_6_1(
    R=2*0.04,
    X=2*0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={230,240})));
  OpenIPSL.Electrical.Branches.PwLine line_6_2(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={210,300})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_6to7(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V6,
    angle_0=pf_data_for_feeder.voltages.A6,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={230,300})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_2to8(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-90,40})));
  OpenIPSL.Electrical.Branches.PwLine line_2_1(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-70,40})));
  OpenIPSL.Electrical.Branches.PwLine line_2_2(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-110,40})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_3to13(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={-90,-280})));
  OpenIPSL.Electrical.Branches.PwLine line_3_1(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-70,-280})));
  OpenIPSL.Electrical.Branches.PwLine line_3_2(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={-110,-280})));
  OpenIPSL.Electrical.Branches.PwLine line_4_4(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={90,200})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_4to5(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V4,
    angle_0=pf_data_for_feeder.voltages.A4,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={70,200})));
  OpenIPSL.Electrical.Branches.PwLine line_4_7(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={50,200})));
  OpenIPSL.Electrical.Branches.PwLine line_4_8(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={250,200})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_5to11(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V6,
    angle_0=pf_data_for_feeder.voltages.A6,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={230,200})));
  OpenIPSL.Electrical.Branches.PwLine line_4_9(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={210,200})));
  OpenIPSL.Electrical.Branches.PwLine line_8_2(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={90,40})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_8to9(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={70,40})));
  OpenIPSL.Electrical.Branches.PwLine line_8_3(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={50,40})));
  OpenIPSL.Electrical.Branches.PwLine line_8_4(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={270,0})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_9to12(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={250,0})));
  OpenIPSL.Electrical.Branches.PwLine line_8_5(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={230,0})));
  OpenIPSL.Electrical.Branches.PwLine line_9_1(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={270,100})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_9to11(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={250,100})));
  OpenIPSL.Electrical.Branches.PwLine line_9_2(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={230,100})));
  OpenIPSL.Electrical.Branches.PwLine line_8_6(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={130,-100})));
  OpenIPSL.Electrical.Branches.PwLine line_8_7(
    R=0.11,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={90,-100})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_8to10(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={110,-100})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_10to14(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={250,-100})));
  OpenIPSL.Electrical.Branches.PwLine line_10_1(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={270,-100})));
  OpenIPSL.Electrical.Branches.PwLine line_10_2(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={230,-100})));
  OpenIPSL.Electrical.Branches.PwLine line_13_1(
    R=0.09,
    X=0.12,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={230,-200})));
  OpenIPSL.Electrical.Branches.PwLine line_13_2(
    R=0.09,
    X=0.12,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={190,-200})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_13to14(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={210,-200})));
  OpenIPSL.Electrical.Branches.PwLine line_13_3(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={90,-320})));
  OpenIPSL.Electrical.Branches.PwLine line_13_4(
    R=0.08,
    X=0.11,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={50,-320})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_13to15(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={70,-320})));
  OpenIPSL.Electrical.Branches.PwLine line_15_1(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={250,-320})));
  OpenIPSL.Electrical.Branches.PwLine line_15_2(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={210,-320})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_15to16(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.VGrid,
    angle_0=pf_data_for_feeder.voltages.AGrid,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=0,
        origin={230,-320})));
  OpenIPSL.Electrical.Branches.PwLine line_7_1(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={520,-50})));
  OpenIPSL.Electrical.Branches.PwLine line_7_2(
    R=0.04,
    X=0.04,
    G=0,
    B=0) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={520,-10})));
  OpenIPSL.Electrical.Buses.Bus FaultBus_7to16(
    V_b=400,
    v_0=pf_data_for_feeder.voltages.V6,
    angle_0=pf_data_for_feeder.voltages.A6,
    displayPF=true) "Feeder1" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=270,
        origin={520,-30})));
  MicroGrid.Examples.SystemExamples.Data.Records.pf_data_for_feeder
    pf_data_for_feeder
    annotation (Placement(transformation(extent={{-182,-442},{-90,-376}})));
equation
  connect(GRID.p,GRID_bus. p)
    annotation (Line(points={{-300,160},{-258,160}}, color={0,0,255}));
  connect(line_1_4.n, Bus1.p)
    annotation (Line(points={{-120.8,250},{-140,250},{-140,220},{-160,220}},
                                                      color={0,0,255}));
  connect(Bus4.p, line_4_5.n) annotation (Line(points={{0,220},{20,220},{20,
          140},{59.2,140}},
                      color={0,0,255}));
  connect(load4.p, Bus4.p)
    annotation (Line(points={{-20,250},{-20,220},{0,220}},  color={0,0,255}));
  connect(line_4_6.p, Bus6.p)
    annotation (Line(points={{100.8,300},{120,300},{120,270},{140,270}},
                                                    color={0,0,255}));
  connect(load6.p, Bus6.p) annotation (Line(points={{170,310},{170,300},{
          160,300},{160,270},{140,270}},
                      color={0,0,255}));
  connect(line_6_7.p, Bus7.p)
    annotation (Line(points={{260.8,300},{280,300},{280,270},{300,270}},
                                                     color={0,0,255}));
  connect(line_4_5.p, Bus5.p)
    annotation (Line(points={{80.8,140},{120,140},{120,170},{140,170}},
                                                 color={0,0,255}));
  connect(Bus5.p, line_4_1.n)
    annotation (Line(points={{140,170},{180,170},{180,140},{219.2,140}},
                                                  color={0,0,255}));
  connect(line_4_1.p, Bus11.p)
    annotation (Line(points={{240.8,140},{280,140},{280,170},{340,170}},
                                                   color={0,0,255}));
  connect(Bus2.p, line_2_8.n)
    annotation (Line(points={{-160,10},{-140,10},{-140,-20},{-100.8,-20}},
                                                    color={0,0,255}));
  connect(line_2_8.p, Bus8.p)
    annotation (Line(points={{-79.2,-20},{-40,-20},{-40,10},{0,10}},
                                                   color={0,0,255}));
  connect(Bus8.p, line_8_9.n)
    annotation (Line(points={{0,10},{20,10},{20,-20},{59.2,-20}},
                                                  color={0,0,255}));
  connect(line_8_9.p, Bus9.p)
    annotation (Line(points={{80.8,-20},{120,-20},{120,10},{140,10}},
                                                  color={0,0,255}));
  connect(Bus9.p, line_8_1.n)
    annotation (Line(points={{140,10},{200,10},{200,-60},{239.2,-60}},
                                                   color={0,0,255}));
  connect(line_8_1.p, Bus12.p)
    annotation (Line(points={{260.8,-60},{300,-60},{300,-30},{340,-30}},
                                                   color={0,0,255}));
  connect(Bus11.p, line_9_11.n)
    annotation (Line(points={{340,170},{320,170},{320,70},{300,70},{300,40},
          {260.8,40}},                                      color={0,0,255}));
  connect(line_9_11.p, Bus9.p)
    annotation (Line(points={{239.2,40},{200,40},{200,10},{140,10}},
                                                            color={0,0,255}));
  connect(load12.p, Bus12.p)
    annotation (Line(points={{370,10},{370,0},{360,0},{360,-30},{340,-30}},
                                                          color={0,0,255}));
  connect(Bus3.p, line_3_13.n)
    annotation (Line(points={{-160,-308},{-140,-308},{-140,-340},{-100.8,
          -340}},                                     color={0,0,255}));
  connect(line_3_13.p, Bus13.p)
    annotation (Line(points={{-79.2,-340},{-40,-340},{-40,-310},{0,-310}},
                                                     color={0,0,255}));
  connect(Bus13.p, line_13_15.n)
    annotation (Line(points={{0,-310},{20,-310},{20,-380},{59.2,-380}},
                                                     color={0,0,255}));
  connect(line_13_15.p, Bus15.p)
    annotation (Line(points={{80.8,-380},{120,-380},{120,-350},{140,-350}},
                                                     color={0,0,255}));
  connect(Bus15.p, line_15_16.n)
    annotation (Line(points={{140,-350},{180,-350},{180,-380},{219.2,-380}},
                                                     color={0,0,255}));
  connect(line_15_16.p, Bus16.p)
    annotation (Line(points={{240.8,-380},{280,-380},{280,-350},{300,-350}},
                                                     color={0,0,255}));
  connect(Bus7.p, line_7_16.n) annotation (Line(points={{300,270},{460,270},
          {460,-11.2}},
                  color={0,0,255}));
  connect(line_7_16.p, Bus16.p) annotation (Line(points={{460,-32.8},{460,
          -350},{300,-350}},
                 color={0,0,255}));
  connect(Bus8.p, line_8_10.n) annotation (Line(points={{0,10},{10,10},{10,
          -130},{60,-130},{60,-160},{99.2,-160}},
                        color={0,0,255}));
  connect(line_8_10.p, Bus10.p)
    annotation (Line(points={{120.8,-160},{160,-160},{160,-130},{180,-130}},
                                                   color={0,0,255}));
  connect(Bus10.p, line_10_14.n)
    annotation (Line(points={{180,-130},{200,-130},{200,-160},{239.2,-160}},
                                                   color={0,0,255}));
  connect(line_10_14.p, Bus14.p)
    annotation (Line(points={{260.8,-160},{300,-160},{300,-130},{340,-130}},
                                                    color={0,0,255}));
  connect(Bus14.p, line_13_14.n) annotation (Line(points={{340,-130},{320,
          -130},{320,-230},{260,-230},{260,-260},{220.8,-260}},
                   color={0,0,255}));
  connect(line_13_14.p, line_13_15.n) annotation (Line(points={{199.2,-260},
          {160,-260},{160,-230},{20,-230},{20,-380},{59.2,-380}},
                             color={0,0,255}));
  connect(load5.p, Bus5.p)
    annotation (Line(points={{170,210},{170,200},{160,200},{160,170},{140,
          170}},                                       color={0,0,255}));
  connect(load7.p, Bus7.p)
    annotation (Line(points={{320,300},{320,270},{300,270}}, color={0,0,255}));
  connect(load8.p, Bus8.p) annotation (Line(points={{-20,40},{-20,10},{0,10}},
                     color={0,0,255}));
  connect(load11.p, Bus11.p)
    annotation (Line(points={{370,210},{370,200},{360,200},{360,170},{340,
          170}},                                          color={0,0,255}));
  connect(load13.p, Bus13.p) annotation (Line(points={{-20,-280},{-20,-310},
          {0,-310}},
                 color={0,0,255}));
  connect(load14.p, Bus14.p)
    annotation (Line(points={{370,-90},{370,-100},{360,-100},{360,-130},{
          340,-130}},                                        color={0,0,255}));
  connect(load15.p, Bus15.p) annotation (Line(points={{160,-320},{160,-350},
          {140,-350}},
                 color={0,0,255}));
  connect(load16.p, Bus16.p) annotation (Line(points={{330,-310},{330,-320},
          {320,-320},{320,-350},{300,-350}},
                 color={0,0,255}));
  connect(load9.p, Bus9.p)
    annotation (Line(points={{180,50},{180,40},{170,40},{170,10},{140,10}},
                                                           color={0,0,255}));
  connect(capacitor_bank_1.p, Bus5.p) annotation (Line(points={{150,210},{
          150,200},{160,200},{160,170},{140,170}},
                                      color={0,0,255}));
  connect(capacitor_bank9.p, Bus9.p) annotation (Line(points={{160,50},{160,
          40},{170,40},{170,10},{140,10}},
                              color={0,0,255}));
  connect(capacitor_bank12.p, Bus12.p) annotation (Line(points={{350,10},{
          350,0},{360,0},{360,-30},{340,-30}},
                              color={0,0,255}));
  connect(capacitor_bank11.p, Bus11.p)
    annotation (Line(points={{350,210},{350,200},{360,200},{360,170},{340,
          170}},                                          color={0,0,255}));
  connect(capacitor_bank14.p, line_13_14.n) annotation (Line(points={{350,-90},
          {350,-100},{360,-100},{360,-130},{320,-130},{320,-230},{260,-230},
          {260,-260},{220.8,-260}},                  color={0,0,255}));
  connect(capacitor_bank16.p, Bus16.p) annotation (Line(points={{310,-310},
          {310,-320},{320,-320},{320,-350},{300,-350}},
                                      color={0,0,255}));
  connect(Bus1.p, GRID_bus.p) annotation (Line(points={{-160,220},{-210,220},
          {-210,160},{-258,160}},
                            color={0,0,255}));
  connect(Bus2.p, GRID_bus.p) annotation (Line(points={{-160,10},{-210,10},
          {-210,160},{-258,160}},
                            color={0,0,255}));
  connect(Bus3.p, GRID_bus.p) annotation (Line(points={{-160,-308},{-210,
          -308},{-210,160},{-258,160}},
                            color={0,0,255}));
  connect(line_1_2.p, Bus4.p) annotation (Line(points={{-79.2,190},{-40,190},
          {-40,220},{0,220}}, color={0,0,255}));
  connect(line_1_2.n, Bus1.p) annotation (Line(points={{-100.8,190},{-140,
          190},{-140,220},{-160,220}}, color={0,0,255}));
  connect(line_1_3.p, Bus4.p) annotation (Line(points={{-59.2,250},{-40,250},
          {-40,220},{0,220}}, color={0,0,255}));
  connect(FaultBus_1to4.p, line_1_3.n)
    annotation (Line(points={{-90,250},{-80.8,250}}, color={0,0,255}));
  connect(line_1_4.p, FaultBus_1to4.p)
    annotation (Line(points={{-99.2,250},{-90,250}}, color={0,0,255}));
  connect(FaultBus_4to6.p, line_4_6.n)
    annotation (Line(points={{70,300},{79.2,300}}, color={0,0,255}));
  connect(line_4_3.p, FaultBus_4to6.p)
    annotation (Line(points={{60.8,300},{70,300}}, color={0,0,255}));
  connect(line_4_3.n, line_4_5.n) annotation (Line(points={{39.2,300},{20,
          300},{20,140},{59.2,140}},
                                  color={0,0,255}));
  connect(line_4_2.p, Bus6.p) annotation (Line(points={{80.8,240},{120,240},
          {120,270},{140,270}},color={0,0,255}));
  connect(capacitor_bank_6.p, Bus6.p) annotation (Line(points={{150,310},{
          150,300},{160,300},{160,270},{140,270}},           color={0,0,255}));
  connect(line_4_2.n, line_4_5.n) annotation (Line(points={{59.2,240},{20,
          240},{20,140},{59.2,140}},
                                  color={0,0,255}));
  connect(FaultBus_6to7.p, line_6_7.n)
    annotation (Line(points={{230,300},{239.2,300}}, color={0,0,255}));
  connect(line_6_2.p, FaultBus_6to7.p)
    annotation (Line(points={{220.8,300},{230,300}}, color={0,0,255}));
  connect(line_6_2.n, Bus6.p) annotation (Line(points={{199.2,300},{180,300},
          {180,270},{140,270}}, color={0,0,255}));
  connect(line_6_1.p, Bus7.p) annotation (Line(points={{240.8,240},{280,240},
          {280,270},{300,270}}, color={0,0,255}));
  connect(line_6_1.n, Bus6.p) annotation (Line(points={{219.2,240},{180,240},
          {180,270},{140,270}}, color={0,0,255}));
  connect(FaultBus_2to8.p, line_2_1.n)
    annotation (Line(points={{-90,40},{-80.8,40}}, color={0,0,255}));
  connect(line_2_1.p, Bus8.p) annotation (Line(points={{-59.2,40},{-40,40},
          {-40,10},{0,10}}, color={0,0,255}));
  connect(line_2_2.p, FaultBus_2to8.p)
    annotation (Line(points={{-99.2,40},{-90,40}}, color={0,0,255}));
  connect(line_2_2.n, line_2_8.n) annotation (Line(points={{-120.8,40},{
          -140,40},{-140,-20},{-100.8,-20}}, color={0,0,255}));
  connect(FaultBus_3to13.p, line_3_1.n)
    annotation (Line(points={{-90,-280},{-80.8,-280}}, color={0,0,255}));
  connect(line_3_1.p, Bus13.p) annotation (Line(points={{-59.2,-280},{-40,
          -280},{-40,-310},{0,-310}}, color={0,0,255}));
  connect(line_3_2.p, FaultBus_3to13.p)
    annotation (Line(points={{-99.2,-280},{-90,-280}}, color={0,0,255}));
  connect(line_3_2.n, line_3_13.n) annotation (Line(points={{-120.8,-280},{
          -140,-280},{-140,-340},{-100.8,-340}}, color={0,0,255}));
  connect(line_4_7.p, FaultBus_4to5.p)
    annotation (Line(points={{60.8,200},{70,200}}, color={0,0,255}));
  connect(FaultBus_4to5.p, line_4_4.n)
    annotation (Line(points={{70,200},{79.2,200}}, color={0,0,255}));
  connect(line_4_4.p, Bus5.p) annotation (Line(points={{100.8,200},{120,200},
          {120,170},{140,170}}, color={0,0,255}));
  connect(line_4_7.n, line_4_5.n) annotation (Line(points={{39.2,200},{20,
          200},{20,140},{59.2,140}}, color={0,0,255}));
  connect(line_4_9.p, FaultBus_5to11.p)
    annotation (Line(points={{220.8,200},{230,200}}, color={0,0,255}));
  connect(line_4_8.n, FaultBus_5to11.p)
    annotation (Line(points={{239.2,200},{230,200}}, color={0,0,255}));
  connect(line_4_9.n, line_4_1.n) annotation (Line(points={{199.2,200},{180,
          200},{180,140},{219.2,140}}, color={0,0,255}));
  connect(line_4_8.p, Bus11.p) annotation (Line(points={{260.8,200},{280,
          200},{280,170},{340,170}}, color={0,0,255}));
  connect(FaultBus_8to9.p, line_8_2.n)
    annotation (Line(points={{70,40},{79.2,40}}, color={0,0,255}));
  connect(line_8_3.p, FaultBus_8to9.p)
    annotation (Line(points={{60.8,40},{70,40}}, color={0,0,255}));
  connect(line_8_3.n, line_8_9.n) annotation (Line(points={{39.2,40},{20,40},
          {20,-20},{59.2,-20}}, color={0,0,255}));
  connect(line_8_2.p, Bus9.p) annotation (Line(points={{100.8,40},{120,40},
          {120,10},{140,10}}, color={0,0,255}));
  connect(line_8_5.p, FaultBus_9to12.p)
    annotation (Line(points={{240.8,0},{250,0}}, color={0,0,255}));
  connect(FaultBus_9to12.p, line_8_4.n)
    annotation (Line(points={{250,0},{259.2,0}}, color={0,0,255}));
  connect(line_8_4.p, Bus12.p) annotation (Line(points={{280.8,-1.33227e-15},
          {300,-1.33227e-15},{300,-30},{340,-30}}, color={0,0,255}));
  connect(line_8_5.n, line_8_1.n) annotation (Line(points={{219.2,0},{200,0},
          {200,-60},{239.2,-60}}, color={0,0,255}));
  connect(FaultBus_9to11.p, line_9_1.p)
    annotation (Line(points={{250,100},{259.2,100}}, color={0,0,255}));
  connect(line_9_2.n, FaultBus_9to11.p)
    annotation (Line(points={{240.8,100},{250,100}}, color={0,0,255}));
  connect(line_9_2.p, Bus9.p) annotation (Line(points={{219.2,100},{200,100},
          {200,10},{140,10}}, color={0,0,255}));
  connect(line_9_1.n, line_9_11.n) annotation (Line(points={{280.8,100},{
          300,100},{300,40},{260.8,40}}, color={0,0,255}));
  connect(load14.p, line_13_14.n) annotation (Line(points={{370,-90},{370,
          -100},{360,-100},{360,-130},{320,-130},{320,-230},{260,-230},{260,
          -260},{220.8,-260}}, color={0,0,255}));
  connect(load10.p, line_8_10.n) annotation (Line(points={{40,-100},{40,
          -130},{60,-130},{60,-160},{99.2,-160}}, color={0,0,255}));
  connect(line_8_10.p, line_10_14.n) annotation (Line(points={{120.8,-160},
          {160,-160},{160,-130},{200,-130},{200,-160},{239.2,-160}}, color=
          {0,0,255}));
  connect(FaultBus_8to10.p, line_8_6.n)
    annotation (Line(points={{110,-100},{119.2,-100}}, color={0,0,255}));
  connect(FaultBus_8to10.p, line_8_7.p)
    annotation (Line(points={{110,-100},{100.8,-100}}, color={0,0,255}));
  connect(line_8_6.p, Bus10.p) annotation (Line(points={{140.8,-100},{160,
          -100},{160,-130},{180,-130}}, color={0,0,255}));
  connect(line_8_7.n, line_8_10.n) annotation (Line(points={{79.2,-100},{60,
          -100},{60,-160},{99.2,-160}}, color={0,0,255}));
  connect(FaultBus_10to14.p, line_10_1.n)
    annotation (Line(points={{250,-100},{259.2,-100}}, color={0,0,255}));
  connect(line_10_2.p, FaultBus_10to14.p)
    annotation (Line(points={{240.8,-100},{250,-100}}, color={0,0,255}));
  connect(line_10_2.n, line_10_14.n) annotation (Line(points={{219.2,-100},
          {200,-100},{200,-160},{239.2,-160}}, color={0,0,255}));
  connect(line_10_1.p, Bus14.p) annotation (Line(points={{280.8,-100},{300,
          -100},{300,-130},{340,-130}}, color={0,0,255}));
  connect(FaultBus_13to14.p, line_13_1.p)
    annotation (Line(points={{210,-200},{219.2,-200}}, color={0,0,255}));
  connect(line_13_2.n, FaultBus_13to14.p)
    annotation (Line(points={{200.8,-200},{210,-200}}, color={0,0,255}));
  connect(line_13_2.p, line_13_15.n) annotation (Line(points={{179.2,-200},
          {160,-200},{160,-230},{20,-230},{20,-380},{59.2,-380}}, color={0,
          0,255}));
  connect(line_13_1.n, line_13_14.n) annotation (Line(points={{240.8,-200},
          {260,-200},{260,-260},{220.8,-260}}, color={0,0,255}));
  connect(FaultBus_13to15.p, line_13_3.n)
    annotation (Line(points={{70,-320},{79.2,-320}}, color={0,0,255}));
  connect(FaultBus_13to15.p, line_13_4.p)
    annotation (Line(points={{70,-320},{60.8,-320}}, color={0,0,255}));
  connect(line_13_4.n, line_13_15.n) annotation (Line(points={{39.2,-320},{
          20,-320},{20,-380},{59.2,-380}}, color={0,0,255}));
  connect(line_13_3.p, Bus15.p) annotation (Line(points={{100.8,-320},{120,
          -320},{120,-350},{140,-350}}, color={0,0,255}));
  connect(line_15_2.p, FaultBus_15to16.p)
    annotation (Line(points={{220.8,-320},{230,-320}}, color={0,0,255}));
  connect(FaultBus_15to16.p, line_15_1.n)
    annotation (Line(points={{230,-320},{239.2,-320}}, color={0,0,255}));
  connect(line_15_2.n, line_15_16.n) annotation (Line(points={{199.2,-320},
          {180,-320},{180,-380},{219.2,-380}}, color={0,0,255}));
  connect(line_15_1.p, Bus16.p) annotation (Line(points={{260.8,-320},{280,
          -320},{280,-350},{300,-350}}, color={0,0,255}));
  connect(line_7_2.p, FaultBus_7to16.p)
    annotation (Line(points={{520,-20.8},{520,-30}}, color={0,0,255}));
  connect(line_7_1.n, FaultBus_7to16.p)
    annotation (Line(points={{520,-39.2},{520,-30}}, color={0,0,255}));
  connect(line_7_1.p, Bus16.p) annotation (Line(points={{520,-60.8},{520,
          -100},{460,-100},{460,-350},{300,-350}}, color={0,0,255}));
  connect(line_7_2.n, line_7_16.n) annotation (Line(points={{520,0.8},{520,
          40},{460,40},{460,-11.2}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-360,
            -460},{560,360}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-360,-460},{560,360}})),
    experiment(StopTime=10));
end IEEE16busFeeder_01;
