within MicroGrid.Examples.Renewables.WECC;
model PV_without_plant_controller
  "Validation example for the WECC PV model with the REGCA and REECB components."
  extends BaseClasses.SMIB_renewable_partial;
  Electrical.Renewables.WECC.REECB controller(
    P_0(displayUnit="MW") = 1500000,
    Q_0(displayUnit="Mvar") = -5665800,
    v_0=0.9999999,
    angle_0(displayUnit="deg") = 0.02574992,
    lvpnt0=0.4,
    pfflag=false,
    vflag=true,
    qflag=false,
    pqflag=false)
    annotation (Placement(transformation(extent={{-74,-12},{-22,12}})));
  Electrical.Renewables.WECC.REGCA REGC(
    P_0(displayUnit="MW") = 1500000,
    Q_0(displayUnit="Mvar") = -5665800,
    v_0=0.9999999,
    angle_0=0.02574992,
    Lvplsw=true)
    annotation (Placement(transformation(extent={{-12,-12},{12,12}})));
equation
  connect(REGC.V_t,controller. Vt) annotation (Line(points={{-6,12.8571},
          {-6,18},{-78,18},{-78,8.57143},{-75.625,8.57143}},
        color={0,0,127}));
  connect(REGC.Pgen,controller. Pe) annotation (Line(points={{0,12.8571},
          {0,22},{-82,22},{-82,4.28571},{-75.625,4.28571}},
                                                     color={0,0,127}));
  connect(REGC.Qgen,controller. Qgen) annotation (Line(points={{6,
          12.8571},{6,26},{-86,26},{-86,0},{-75.625,0}},
                      color={0,0,127}));
  connect(REGC.IP0,controller. IP00) annotation (Line(points={{-5.14286,
          -12.8571},{-5.14286,-16},{-35,-16},{-35,-13.7143}},color={0,0,127}));
  connect(REGC.IQ0,controller. IQ00) annotation (Line(points={{5.14286,
          -12.8571},{5.14286,-20},{-61,-20},{-61,-13.7143}},   color={0,0,
          127}));
  connect(controller.Iqcmd,REGC. Iqcmd) annotation (Line(points={{
          -21.1875,6},{-13.7143,6}},                                color={
          0,0,127}));
  connect(controller.Ipcmd,REGC. Ipcmd) annotation (Line(points={{
          -21.1875,-6},{-13.7143,-6}},                      color={0,0,127}));
  connect(REGC.p, GEN1.p)
    annotation (Line(points={{12.8571,0},{40,0}}, color={0,0,255}));
  connect(Pref.y, controller.Pref) annotation (Line(points={{-129,40},{
          -110,40},{-110,-8.57143},{-75.625,-8.57143}}, color={0,0,127}));
  connect(Qref.y, controller.Qext) annotation (Line(points={{-129,80},{
          -100,80},{-100,-4.28571},{-75.625,-4.28571}}, color={0,0,127}));
end PV_without_plant_controller;
