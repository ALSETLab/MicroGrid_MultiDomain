within MicroGrid.Examples.SinglePhaseInductionMotor;
model SPIM_example
  "Initial conditions (power-flow) calculation of IEEE4 bus system, in hybrid positive-sequence/three-phase formulation"
  extends Modelica.Icons.Example;
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60) annotation (Placement(
        transformation(
        origin={52,50},
        extent={{-20,-10},{20,10}})));
  OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus annotation (Placement(
        transformation(
        origin={-90,0},
        extent={{-10,-10},{10,10}})));
  OpenIPSL.Electrical.Buses.Bus Bus1 annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  OpenIPSL.Electrical.Branches.PwLine Line1(
    B=0,
    G=0,
    R=0.074563536332,
    X=0.152732235479) annotation (Placement(transformation(extent=
           {{-56,-4},{-44,4}})));
  OpenIPSL.Electrical.Buses.Bus Bus2(angle_0=0, v_0=1) annotation (Placement(
        transformation(extent={{-42,-10},{-22,10}})));
  OpenIPSL.Electrical.ThreePhase.Branches.MonoTri.Transformer_MT Transformer(
    B_0=-1.82768503568,
    B_1=-5.2872570055,
    B_2=-5.2872570055,
    Connection=3,
    G_0=0.729734359723,
    G_1=2.58122706441,
    G_2=2.58122706441,
    ModelType=0,
    R=0.166666666667,
    X=1,
    tap=1) annotation (Placement(transformation(origin={-10,0}, extent={{-10,
            -10},{10,10}})));
  OpenIPSL.Electrical.ThreePhase.Buses.Bus_3Ph Bus3 annotation (Placement(
        transformation(origin={24,0}, extent={{-10,-10},{10,10}})));
  OpenIPSL.Electrical.ThreePhase.Branches.Lines.Line_3Ph Line2(
    Bseraa=-0.3687,
    Bserab=0.1265,
    Bserac=0.0822,
    Bserbb=-0.3821,
    Bserbc=0.1004,
    Bsercc=-0.3559,
    Gseraa=0.1779,
    Gserab=-0.0864,
    Gserac=-0.0312,
    Gserbb=0.1989,
    Gserbc=-0.0529,
    Gsercc=0.1598) annotation (Placement(transformation(origin={46,0},
          extent={{-10,-10},{10,10}})));
  OpenIPSL.Electrical.ThreePhase.Buses.Bus_3Ph Bus4 annotation (Placement(
        transformation(origin={68,0}, extent={{-10,-10},{10,10}})));
  Electrical.InductionMotor.SinglePhase.SPIM AC01(
    P_0=1275000,
    Q_0=1000000,
    v_0=1,
    angle_0=0,
    R1=0.001,
    R2=0.005,
    X1=0.01,
    X2=0.01,
    Xm=0.1,
    H=0.1,
    a=0.001,
    b=0,
    c=0) annotation (Placement(transformation(extent={{80,16},{100,36}})));
  Electrical.InductionMotor.SinglePhase.SPIM AC02(
    P_0=1800000,
    Q_0=1000000,
    angle_0=-2.0943951023932,
    R1=0.001,
    R2=0.005,
    X1=0.01,
    X2=0.01,
    Xm=0.1,
    H=0.1,
    a=0.001,
    b=0,
    c=0) annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  Electrical.InductionMotor.SinglePhase.SPIM AC03(
    P_0=2375000,
    Q_0=1000000,
    angle_0=2.0943951023932,
    R1=0.001,
    R2=0.005,
    X1=0.01,
    X2=0.01,
    Xm=0.1,
    H=0.1,
    a=0.001,
    b=0,
    c=0) annotation (Placement(transformation(extent={{80,-36},{100,-16}})));
  OpenIPSL.Electrical.Events.PwFault pwFault(
    R=0.01,
    X=0.01,
    t1=10000,
    t2=10000)
    annotation (Placement(transformation(extent={{106,-32},{118,-20}})));
  OpenIPSL.Electrical.Events.PwFault pwFault1(
    R=0.01,
    X=0.01,
    t1=10000,
    t2=10000)
    annotation (Placement(transformation(extent={{106,-6},{118,6}})));
  OpenIPSL.Electrical.Events.PwFault pwFault2(
    R=0.01,
    X=0.01,
    t1=1000,
    t2=1001)
    annotation (Placement(transformation(extent={{106,20},{118,32}})));
equation
  connect(Transformer.C, Bus3.p3) annotation (Line(points={{1,-9},{24,-9},{24,-9},
          {24,-9}}, color={0,0,255}));
  connect(Transformer.B, Bus3.p2)
    annotation (Line(points={{1,0},{24,0},{24,0},{24,0}}, color={0,0,255}));
  connect(Transformer.A, Bus3.p1)
    annotation (Line(points={{1,9},{24,9},{24,9},{24,9}}, color={0,0,255}));
  connect(infiniteBus.p, Bus1.p)
    annotation (Line(points={{-80,0},{-68,0}}, color={0,0,255}));
  connect(Line2.Cout, Bus4.p3) annotation (Line(points={{55.2857,-9},{68,
          -9},{68,-9},{68,-9}},
                           color={0,0,255}));
  connect(Line2.Bout, Bus4.p2) annotation (Line(points={{55.2857,0},{68,0},
          {68,0},{68,0}},
                      color={0,0,255}));
  connect(Line2.Aout, Bus4.p1) annotation (Line(points={{55.2857,9},{68,9},
          {68,9},{68,9}},
                      color={0,0,255}));
  connect(Bus3.p3, Line2.Cin) annotation (Line(points={{24,-9},{38,-9},{
          38,-9},{36.7143,-9}},
                         color={0,0,255}));
  connect(Bus3.p2, Line2.Bin) annotation (Line(points={{24,0},{38,0},{38,
          0},{36.7143,0}},
                       color={0,0,255}));
  connect(Bus3.p1, Line2.Ain) annotation (Line(points={{24,9},{36,9},{36,
          9},{36.7143,9}},
                       color={0,0,255}));
  connect(Line1.n, Bus2.p)
    annotation (Line(points={{-44.6,0},{-32,0}}, color={0,0,255}));
  connect(Transformer.p, Bus2.p)
    annotation (Line(points={{-21,0},{-32,0}}, color={0,0,255}));
  connect(Line1.p, Bus1.p)
    annotation (Line(points={{-55.4,0},{-68,0}}, color={0,0,255}));
  connect(Bus4.p2, AC02.p)
    annotation (Line(points={{68,0},{80,0}}, color={0,0,255}));
  connect(Bus4.p1, AC01.p) annotation (Line(points={{68,9},{76,9},{76,26},{
          80,26}}, color={0,0,255}));
  connect(Bus4.p3, AC03.p) annotation (Line(points={{68,-9},{76,-9},{76,-26},
          {80,-26}}, color={0,0,255}));
  connect(pwFault.p, AC03.p) annotation (Line(points={{105,-26},{104,-26},
          {104,-40},{76,-40},{76,-26},{80,-26}}, color={0,0,255}));
  connect(pwFault1.p, AC02.p) annotation (Line(points={{105,0},{102,0},{
          102,-12},{78,-12},{78,0},{80,0}}, color={0,0,255}));
  connect(pwFault2.p, AC01.p) annotation (Line(points={{105,26},{102,26},
          {102,40},{76,40},{76,26},{80,26}}, color={0,0,255}));
  annotation (experiment(
      StopTime=300,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>This test system aims to show how the hybrid positive-sequence/three-phase transformer should be used for simulation.
Its results should be compared to the example IEEE4, in order to show what to expect from the hybrid transformer.
The MonoTri device interconnects to parts of the system which are modeled with different formulations.
The first two buses are positive-sequence only while buses three and four are three-phase buses.</p>
<p>This example, however, is not a dynamic system, and therefore the voltages and angles from all buses are static during the simulation.
The simulation just calculates the initialization variables, which are the solution for the hybrid positive-sequence/three-phase power flow.
The results can be verified against any tool that calculates a hybrid positive-sequence/three-phase power-flow.</p>
<p> </p>
</html>"),
    Diagram(coordinateSystem(extent={{-120,-100},{140,100}})),
    Icon(coordinateSystem(extent={{-120,-100},{140,100}})));
end SPIM_example;
