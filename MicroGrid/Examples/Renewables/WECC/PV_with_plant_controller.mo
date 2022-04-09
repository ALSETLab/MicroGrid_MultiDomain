within MicroGrid.Examples.Renewables.WECC;
model PV_with_plant_controller
  extends BaseClasses.SMIB_renewable_partial;
  Electrical.Renewables.WECC.GridFollowing.REGCA REGCA(
    P_0=1500000,
    Q_0=-5665800,
    v_0=0.9999999,
    angle_0=0.02574992)
    annotation (Placement(transformation(extent={{4,-12},{28,12}})));
  Electrical.Renewables.WECC.GridFollowing.REECB controller(
    P_0(displayUnit="MW") = 1500000,
    Q_0(displayUnit="Mvar") = -5665800,
    v_0=0.9999999,
    angle_0(displayUnit="deg") = 0.02574992,
    lvpnt0=0.4,
    pfflag=false,
    vflag=true,
    qflag=false,
    pqflag=false)
    annotation (Placement(transformation(extent={{-56,-12},{-4,12}})));
  Electrical.Renewables.WECC.GridFollowing.REPCA REPCA(
    fdbd1=-0.00083,
    fdbd2=0.00083,
    Ddn=126,
    Dup=126,
    femax=99,
    femin=-99,
    Kpg=1,
    Kig=0,
    Pmax=1,
    Pmin=-0.667,
    Kc=0,
    fflag=false,
    vcflag=true,
    Kp=0,
    Ki=0.0001,
    Qmax=0.75,
    Qmin=-0.75,
    Tft=0,
    refflag=false,
    Tfv=0.05,
    Vfrz=0,
    Vref=0.9999999)
    annotation (Placement(transformation(extent={{-106,-14},{-76,16}})));
equation
  connect(controller.Ipcmd, REGCA.Ipcmd) annotation (Line(points={{-3.1875,
          -6},{2.28571,-6}}, color={0,0,127}));
  connect(controller.Iqcmd, REGCA.Iqcmd)
    annotation (Line(points={{-3.1875,6},{2.28571,6}}, color={0,0,127}));
  connect(REPCA.Pref, controller.Pref) annotation (Line(points={{-75.2857,
          -6.14286},{-74,-6.14286},{-74,-6},{-72,-6},{-72,-8.57143},{-57.625,
          -8.57143}},
        color={0,0,127}));
  connect(REPCA.Qext, controller.Qext) annotation (Line(points={{-75.2857,
          8.14286},{-74,8.14286},{-74,8},{-72,8},{-72,-4.28571},{-57.625,
          -4.28571}},
        color={0,0,127}));
  connect(REGCA.IP0, controller.IP00) annotation (Line(points={{10.8571,
          -12.8571},{10.8571,-24},{-17,-24},{-17,-13.7143}}, color={0,0,
          127}));
  connect(REGCA.IQ0, controller.IQ00) annotation (Line(points={{21.1429,
          -12.8571},{21.1429,-30},{-43,-30},{-43,-13.7143}}, color={0,0,
          127}));
  connect(REGCA.V_t, controller.Vt) annotation (Line(points={{10,12.8571},{10,
          18},{-62,18},{-62,8.57143},{-57.625,8.57143}},     color={0,0,
          127}));
  connect(REGCA.Pgen, controller.Pe) annotation (Line(points={{16,12.8571},{16,
          22},{-66,22},{-66,4.28571},{-57.625,4.28571}},     color={0,0,
          127}));
  connect(REGCA.Qgen, controller.Qgen) annotation (Line(points={{22,12.8571},{
          22,26},{-70,26},{-70,0},{-57.625,0}},           color={0,0,127}));
  connect(REGCA.p, REPCA.BRANCH_n) annotation (Line(points={{28.8571,0},{32,0},
          {32,32},{-83.8571,32},{-83.8571,16.7143}},       color={0,0,255}));
  connect(REPCA.REGULATE, pwLine2.p) annotation (Line(points={{-91,16.7143},{
          -91,38},{42,38},{42,0},{45,0}},
                                      color={0,0,255}));
  connect(REPCA.BRANCH_p, FAULT.p) annotation (Line(points={{-98.1429,16.7143},
          {-98.1429,44},{66,44},{66,0},{68,0}}, color={0,0,255}));
  connect(REPCA.Qref, Qref.y) annotation (Line(points={{-107.429,11.7143},{-110,
          11.7143},{-110,12},{-114,12},{-114,80},{-129,80}}, color={0,0,127}));
  connect(REPCA.Plant_pref, Pref.y) annotation (Line(points={{-107.429,4.57143},
          {-120,4.57143},{-120,40},{-129,40}}, color={0,0,127}));
  connect(freq.y, REPCA.Freq) annotation (Line(points={{-129,0},{-116,0},{-116,
          -2.57143},{-107.429,-2.57143}}, color={0,0,127}));
  connect(REPCA.Freq_ref, REPCA.Freq) annotation (Line(points={{-107.429,
          -9.71429},{-116,-9.71429},{-116,-2.57143},{-107.429,-2.57143}},
                                                                color={0,0,127}));
end PV_with_plant_controller;
