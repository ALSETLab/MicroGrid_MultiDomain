within MicroGrid.Examples.Renewables.WECC;
model BESS_without_plant_controller
  "Validation example for the WECC BESS model with the REGCA and REECB components."
  extends BaseClasses.SMIB_renewable_partial;
  Electrical.Renewables.WECC.GridFollowing.REGCA REGC(
    S_b=SysData.S_b,
    M_b(displayUnit="MVA") = 100000000,
    P_0(displayUnit="MW") = 1500000,
    Q_0(displayUnit="Mvar") = -5665800,
    v_0=0.9999999,
    angle_0=0.02574992,
    Tg=0.017,
    Iqrmax=99,
    lvpnt0=0.05,
    Brkpt=0.1,
    Zerox=0.05,
    Iqrmin=-99,
    lvpnt1=0.2,
    Lvplsw=true)
    annotation (Placement(transformation(extent={{-12,-12},{12,12}})));
  Electrical.Renewables.WECC.GridFollowing.REECC REECC(
    P_0=1500000,
    Q_0=-5665800,
    v_0=0.9999999,
    angle_0=0.02574992,
    Iqh1=0.75,
    Iql1=-0.75,
    Tp=0.05,
    pfflag=false,
    Qmax=0.75,
    Qmin=-0.75,
    vflag=true,
    Vmax=1.1,
    Vmin=0.9,
    Kqp=0,
    Kqi=1,
    Kvp=0,
    Kvi=1,
    Tiq=0.017,
    qflag=true,
    pqflag=false,
    SOCmax=0.8,
    SOCmin=0.2,
    dPmax=99,
    dPmin=-99,
    Pmax=1,
    Pmin=-0.667,
    Tpord=0.017,
    Kqv=0,
    dbd1=-0.05,
    dbd2=0.05)
    annotation (Placement(transformation(extent={{-60,-12},{-22,12}})));
equation
  connect(REECC.Iqcmd, REGC.Iqcmd) annotation (Line(points={{-21.2083,6},{
          -13.7143,6}},  color={0,0,127}));
  connect(REECC.Ipcmd, REGC.Ipcmd) annotation (Line(points={{-21.2083,-6},{
          -13.7143,-6}},  color={0,0,127}));
  connect(REGC.IP0, REECC.IP00) annotation (Line(points={{-5.14286,-12.8571},{
          -5.14286,-22},{-31.5,-22},{-31.5,-13.5}},            color={0,0,
          127}));
  connect(REGC.IQ0, REECC.IQ00) annotation (Line(points={{5.14286,-12.8571},{
          5.14286,-26},{-41,-26},{-41,-13.5}},            color={0,0,127}));
  connect(REGC.V_t, REECC.Vt) annotation (Line(points={{-6,12.8571},{-6,18},{
          -66,18},{-66,9.75},{-61.5833,9.75}},      color={0,0,127}));
  connect(REGC.Pgen, REECC.Pelec) annotation (Line(points={{0,12.8571},{0,22},{
          -70,22},{-70,6},{-61.5833,6}},      color={0,0,127}));
  connect(REGC.Qgen, REECC.Qelec) annotation (Line(points={{6,12.8571},{6,26},{
          -76,26},{-76,2.25},{-61.5833,2.25}},      color={0,0,127}));
  connect(REGC.p, GEN1.p)
    annotation (Line(points={{12.8571,0},{40,0}}, color={0,0,255}));
  connect(soc_ini.y, REECC.SOCini) annotation (Line(points={{-129,-80},{
          -50.5,-80},{-50.5,-13.5}}, color={0,0,127}));
  connect(Qref.y, REECC.Qext) annotation (Line(points={{-129,80},{-82,80},{-82,
          -2.25},{-61.5833,-2.25}},      color={0,0,127}));
  connect(paux.y, REECC.Paux) annotation (Line(points={{-129,-40},{-82,-40},{
          -82,-6},{-61.5833,-6}},       color={0,0,127}));
  connect(Pref.y, REECC.Pref) annotation (Line(points={{-129,40},{-88,40},{-88,
          -9.75},{-61.5833,-9.75}},      color={0,0,127}));
end BESS_without_plant_controller;
