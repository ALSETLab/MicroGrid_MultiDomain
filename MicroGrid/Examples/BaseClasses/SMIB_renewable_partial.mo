within MicroGrid.Examples.BaseClasses;
partial model SMIB_renewable_partial
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=2.50000E-2,
    X=2.50000E-2,
    G=0,
    B=0.05000/2) annotation (Placement(transformation(extent={{88,10},{108,
            30}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine1(
    R=2.50000E-2,
    X=2.50000E-2,
    G=0,
    B=0.05000/2) annotation (Placement(transformation(extent={{88,-30},{108,
            -10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS2_1(
    angle_0=-1.570655e-05,
    R_a=0,
    X_d=2.00000E-1,
    M_b=100000000,
    V_b=100000,
    P_0=-1498800,
    Q_0=-4334000,
    v_0=1.00000) annotation (Placement(transformation(extent={{152,-12},{
            140,12}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{44,-10},{64,10}})));
  OpenIPSL.Electrical.Events.PwFault pwFault(
    R=0.5,
    X=0.5,
    t1=2.00,
    t2=2.15)
            annotation (Placement(transformation(extent={{82,-60},{102,-40}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=50, S_b=100000000) annotation (Placement(transformation(extent={{112,92},
            {152,112}})));
  OpenIPSL.Electrical.Buses.Bus GEN1
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  OpenIPSL.Electrical.Buses.Bus FAULT
    annotation (Placement(transformation(extent={{58,-10},{78,10}})));
  OpenIPSL.Electrical.Buses.Bus GEN2
    annotation (Placement(transformation(extent={{118,-10},{138,10}})));
  Modelica.Blocks.Sources.Constant Pref(k=1.5/100)
                                                  annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-140,40})));
  Modelica.Blocks.Sources.Constant Qref(k=-5.6658/100)
    annotation (Placement(transformation(extent={{-150,70},{-130,90}})));
  Modelica.Blocks.Sources.Constant freq(k=0)
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  Modelica.Blocks.Sources.Constant soc_ini(k=0.5)
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-140,-80})));
  Modelica.Blocks.Sources.Constant paux(k=0) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-140,-40})));
equation
  connect(pwLine2.n,FAULT. p)
    annotation (Line(points={{63,0},{68,0}},        color={0,0,255}));
  connect(FAULT.p,pwLine. p)
    annotation (Line(points={{68,0},{78,0},{78,20},{89,20}}, color={0,0,255}));
  connect(pwLine1.p,pwLine. p) annotation (Line(points={{89,-20},{78,-20},{
          78,20},{89,20}},       color={0,0,255}));
  connect(pwFault.p,FAULT. p) annotation (Line(points={{80.3333,-50},{72,
          -50},{72,0},{68,0}},
                         color={0,0,255}));
  connect(pwLine.n,GEN2. p)
    annotation (Line(points={{107,20},{118,20},{118,0},{128,0}},
                                                             color={0,0,255}));
  connect(pwLine1.n,GEN2. p) annotation (Line(points={{107,-20},{118,-20},{
          118,0},{128,0}},
                   color={0,0,255}));
  connect(GEN2.p,gENCLS2_1. p)
    annotation (Line(points={{128,0},{140,0}},      color={0,0,255}));
  connect(GEN1.p,pwLine2. p)
    annotation (Line(points={{40,0},{45,0}},  color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -160,-120},{160,120}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},{
            160,120}}), graphics={Rectangle(
          extent={{-156,100},{-124,-102}},
          lineColor={238,46,47},
          pattern=LinePattern.Dash,
          lineThickness=1), Text(
          extent={{-164,114},{-116,104}},
          lineColor={238,46,47},
          textString="Input Ref")}));
end SMIB_renewable_partial;
