within MicroGrid.Examples.Renewables.PNNL;
model GridFormingVCS
  extends Modelica.Icons.Example;
  Electrical.Renewables.PNNL.GridForming.CVS cVS(
    P_0=10000000,
    Q_0=5000000,
    v_0=1,
    angle_0=0,
    M_b=100000000,
    XL=0.05)
    annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
  Electrical.Renewables.PNNL.GridForming.GFMDRP_A gFMDRP_A(
    P_0=10000000,
    Q_0=5000000,
    v_0=1,
    angle_0=0,
    M_b=100000000)
    annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=2.50000E-2,
    X=2.50000E-2,
    G=0,
    B=0.05000/2) annotation (Placement(transformation(extent={{32,10},{
            52,30}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine1(
    R=2.50000E-2,
    X=2.50000E-2,
    G=0,
    B=0.05000/2) annotation (Placement(transformation(extent={{32,-30},
            {52,-10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS2_1(
    R_a=0,
    X_d=2.00000E-1,
    M_b=100000000,
    V_b=100000,
    P_0=-15000000,
    Q_0=-8000000)
                 annotation (Placement(transformation(extent={{96,-12},
            {84,12}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-2,-10},{18,10}})));
  OpenIPSL.Electrical.Events.PwFault pwFault(
    R=0.01,
    X=0.01,
    t1=2,
    t2=2.1) annotation (Placement(transformation(extent={{26,-60},{46,
            -40}})));
  OpenIPSL.Electrical.Buses.Bus GEN1
    annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  OpenIPSL.Electrical.Buses.Bus FAULT
    annotation (Placement(transformation(extent={{10,-10},{30,10}})));
  OpenIPSL.Electrical.Buses.Bus GEN2
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=50, S_b=100000000) annotation (Placement(transformation(extent={{52,68},
            {92,88}})));
  OpenIPSL.Electrical.Buses.Bus GEN3
    annotation (Placement(transformation(extent={{-32,-60},{-12,-40}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine3(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-2,-60},{18,-40}})));
  Electrical.Renewables.PNNL.GridForming.CVS cVS1(
    P_0=5000000,
    Q_0=3000000,
    v_0=1,
    angle_0=0,
    M_b=100000000,
    XL=0.05) annotation (Placement(transformation(extent={{-46,-60},{-26,
            -40}})));
  Electrical.Renewables.PNNL.GridForming.GFMDRP_A gFMDRP_A1(
    P_0=5000000,
    Q_0=3000000,
    v_0=1,
    angle_0=0,
    M_b=100000000) annotation (Placement(transformation(extent={{-74,-60},
            {-54,-40}})));
  OpenIPSL.Electrical.Sensors.SoftPMU softPMU(V_0=1, angle_0=0)
    annotation (Placement(transformation(extent={{-22,-10},{-2,10}})));
  OpenIPSL.Electrical.Sensors.SoftPMU softPMU1(V_0=1, angle_0=0)
    annotation (Placement(transformation(extent={{-22,-60},{-2,-40}})));
equation
  connect(gFMDRP_A.theta_droop, cVS.Theta_droop)
    annotation (Line(points={{-51.2857,6},{-47,6}}, color={0,0,127}));
  connect(gFMDRP_A.Edroop, cVS.E_droop) annotation (Line(points={{
          -51.2857,-6},{-47,-6}}, color={0,0,127}));
  connect(cVS.THETADROOP_0, gFMDRP_A.THETADROOP_0) annotation (Line(
        points={{-41.8,-11},{-41.8,-16},{-55.5714,-16},{-55.5714,
          -10.7143}}, color={0,0,127}));
  connect(cVS.EDROOP_0, gFMDRP_A.EDROOP_0) annotation (Line(points={{-30,-11},
          {-30,-20},{-68.4286,-20},{-68.4286,-10.7143}},          color=
         {0,0,127}));
  connect(cVS.P_inv, gFMDRP_A.P_inv) annotation (Line(points={{-43,11},
          {-43,16},{-78,16},{-78,7.14286},{-72.7143,7.14286}}, color={0,
          0,127}));
  connect(cVS.Q_inv, gFMDRP_A.Q_inv) annotation (Line(points={{-36,11},
          {-36,20},{-84,20},{-84,0},{-72.7143,0}}, color={0,0,127}));
  connect(cVS.V_inv, gFMDRP_A.V_inv) annotation (Line(points={{-29,11},
          {-29,24},{-92,24},{-92,-7.14286},{-72.7143,-7.14286}}, color=
          {0,0,127}));
  connect(pwLine2.n,FAULT. p)
    annotation (Line(points={{17,0},{20,0}},        color={0,0,255}));
  connect(FAULT.p,pwLine. p)
    annotation (Line(points={{20,0},{22,0},{22,20},{33,20}}, color={0,0,255}));
  connect(pwLine1.p,pwLine. p) annotation (Line(points={{33,-20},{22,
          -20},{22,20},{33,20}}, color={0,0,255}));
  connect(pwFault.p,FAULT. p) annotation (Line(points={{24.3333,-50},{
          20,-50},{20,0}},
                         color={0,0,255}));
  connect(pwLine.n,GEN2. p)
    annotation (Line(points={{51,20},{62,20},{62,0},{70,0}}, color={0,0,255}));
  connect(pwLine1.n,GEN2. p) annotation (Line(points={{51,-20},{62,-20},
          {62,0},{70,0}},
                   color={0,0,255}));
  connect(GEN2.p,gENCLS2_1. p)
    annotation (Line(points={{70,0},{84,0}},        color={0,0,255}));
  connect(cVS.p, GEN1.p)
    annotation (Line(points={{-26,0},{-22,0}}, color={0,0,255}));
  connect(pwLine3.n, FAULT.p) annotation (Line(points={{17,-50},{20,-50},
          {20,0}},        color={0,0,255}));
  connect(gFMDRP_A1.theta_droop, cVS1.Theta_droop) annotation (Line(
        points={{-53.2857,-44},{-47,-44}}, color={0,0,127}));
  connect(gFMDRP_A1.Edroop, cVS1.E_droop) annotation (Line(points={{
          -53.2857,-56},{-47,-56}}, color={0,0,127}));
  connect(cVS1.THETADROOP_0, gFMDRP_A1.THETADROOP_0) annotation (Line(
        points={{-41.8,-61},{-41.8,-66},{-57.5714,-66},{-57.5714,
          -60.7143}}, color={0,0,127}));
  connect(cVS1.EDROOP_0, gFMDRP_A1.EDROOP_0) annotation (Line(points={{-30,-61},
          {-30,-70},{-70.4286,-70},{-70.4286,-60.7143}},          color=
         {0,0,127}));
  connect(cVS1.P_inv, gFMDRP_A1.P_inv) annotation (Line(points={{-43,-39},
          {-43,-34},{-78,-34},{-78,-42.8571},{-74.7143,-42.8571}},
        color={0,0,127}));
  connect(cVS1.Q_inv, gFMDRP_A1.Q_inv) annotation (Line(points={{-36,-39},
          {-36,-30},{-84,-30},{-84,-50},{-74.7143,-50}},      color={0,
          0,127}));
  connect(cVS1.V_inv, gFMDRP_A1.V_inv) annotation (Line(points={{-29,-39},
          {-29,-26},{-92,-26},{-92,-57.1429},{-74.7143,-57.1429}},
        color={0,0,127}));
  connect(cVS1.p, GEN3.p)
    annotation (Line(points={{-26,-50},{-22,-50}}, color={0,0,255}));
  connect(softPMU.n, pwLine2.p)
    annotation (Line(points={{-5,0},{-1,0}}, color={0,0,255}));
  connect(GEN1.p, softPMU.p)
    annotation (Line(points={{-22,0},{-19,0}}, color={0,0,255}));
  connect(softPMU1.n, pwLine3.p)
    annotation (Line(points={{-5,-50},{-1,-50}}, color={0,0,255}));
  connect(GEN3.p, softPMU1.p)
    annotation (Line(points={{-22,-50},{-19,-50}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=5000,
      Tolerance=0.001,
      __Dymola_Algorithm="Dassl"));
end GridFormingVCS;
