within MicroGrid.Examples.Renewables.WECC;
model BESS_with_plant_controller
  extends BaseClasses.SMIB_renewable_partial;
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
    dbd1=-0.05,
    dbd2=0.05) annotation (Placement(transformation(extent={{-38,-8},{-4,12}})));
  Electrical.Renewables.WECC.GridFollowing.REGCA rEGCA_WECC_3_1(
    P_0=1500000,
    Q_0=-5665800,
    v_0=0.9999999,
    angle_0=0.02574992)
    annotation (Placement(transformation(extent={{6,-10},{26,10}})));
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
    annotation (Placement(transformation(extent={{-98,-14},{-68,16}})));
equation
  connect(REECC.Iqcmd, rEGCA_WECC_3_1.Iqcmd)
    annotation (Line(points={{-3.29167,7},{0,7},{0,5},{4.57143,5}},
                                                        color={0,0,127}));
  connect(REECC.Ipcmd, rEGCA_WECC_3_1.Ipcmd)
    annotation (Line(points={{-3.29167,-3},{0,-3},{0,-5},{4.57143,-5}},
                                                          color={0,0,127}));
  connect(rEGCA_WECC_3_1.IP0, REECC.IP00) annotation (Line(points={{11.7143,
          -10.7143},{11.7143,-18},{-12.5,-18},{-12.5,-9.25}},
                                                     color={0,0,127}));
  connect(rEGCA_WECC_3_1.IQ0, REECC.IQ00) annotation (Line(points={{20.2857,
          -10.7143},{20.2857,-22},{-21,-22},{-21,-9.25}},
                                                 color={0,0,127}));
  connect(rEGCA_WECC_3_1.V_t, REECC.Vt) annotation (Line(points={{11,10.7143},{
          11,16},{-44,16},{-44,10.125},{-39.4167,10.125}},color={0,0,127}));
  connect(rEGCA_WECC_3_1.Pgen, REECC.Pelec) annotation (Line(points={{16,
          10.7143},{16,20},{-48,20},{-48,7},{-39.4167,7}},
                                                  color={0,0,127}));
  connect(rEGCA_WECC_3_1.Qgen, REECC.Qelec) annotation (Line(points={{21,
          10.7143},{21,24},{-52,24},{-52,3.875},{-39.4167,3.875}},
                                                          color={0,0,127}));
  connect(REPCA.Pref, REECC.Pref) annotation (Line(points={{-67.2857,-6.14286},
          {-56,-6.14286},{-56,-6.125},{-39.4167,-6.125}}, color={0,0,127}));
  connect(REPCA.Qext, REECC.Qext) annotation (Line(points={{-67.2857,8.14286},{
          -56,8.14286},{-56,0.125},{-39.4167,0.125}},    color={0,0,127}));
  connect(soc_ini.y, REECC.SOCini) annotation (Line(points={{-129,-80},
          {-29.5,-80},{-29.5,-9.25}},
                                color={0,0,127}));
  connect(freq.y, REPCA.Freq_ref) annotation (Line(points={{-129,0},{-122,0},{
          -122,-2},{-116,-2},{-116,-9.71429},{-99.4286,-9.71429}}, color={0,0,
          127}));
  connect(REPCA.Freq, REPCA.Freq_ref) annotation (Line(points={{-99.4286,
          -2.57143},{-116,-2.57143},{-116,-9.71429},{-99.4286,-9.71429}},
                                                                color={0,0,127}));
  connect(REPCA.Plant_pref, Pref.y) annotation (Line(points={{-99.4286,4.57143},
          {-114,4.57143},{-114,40},{-129,40}}, color={0,0,127}));
  connect(REPCA.Qref, Qref.y) annotation (Line(points={{-99.4286,11.7143},{-108,
          11.7143},{-108,80},{-129,80}}, color={0,0,127}));
  connect(REPCA.BRANCH_n, rEGCA_WECC_3_1.p) annotation (Line(points={{-75.8571,
          16.7143},{-75.8571,28},{30,28},{30,0},{26.7143,0}}, color={0,0,255}));
  connect(REPCA.REGULATE, pwLine2.p) annotation (Line(points={{-83,16.7143},{
          -83,32},{42,32},{42,0},{45,0}},
                                      color={0,0,255}));
  connect(REPCA.BRANCH_p, FAULT.p) annotation (Line(points={{-90.1429,16.7143},
          {-90.1429,36},{66,36},{66,0},{68,0}}, color={0,0,255}));
  connect(REECC.Paux, paux.y) annotation (Line(points={{-39.4167,-3},{-52,-3},{
          -52,-40},{-129,-40}},          color={0,0,127}));
end BESS_with_plant_controller;
