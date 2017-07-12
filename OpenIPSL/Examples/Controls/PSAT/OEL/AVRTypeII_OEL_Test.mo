model AVRTypeII_OEL_Test
  extends OpenIPSL.Examples.BaseTest;
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSAT.Order4 order4(
    V_b=200,
    V_0=1,
    angle_0=0,
    Sn=370,
    Vn=200,
    ra=0.001,
    xd1=0.302,
    M=10,
    D=0,
    P_0=16.0352698692006,
    Q_0=11.859436505981)
    annotation (Placement(transformation(extent={{-45,-10},{-25,10}})));
  Electrical.Controls.PSAT.AVR.AVRTypeII exciter_Type_II(
    vrmin=-5,
    vrmax=5,
    Ta=0.1,
    Te=1,
    Tr=0.001,
    Ae=0.0006,
    Be=0.9,
    Kf=0.45,
    Tf=1,
    Ka=400,
    Ke=0.01)
    annotation (Placement(transformation(extent={{-70,18},{-90,38}})));
  Electrical.Controls.PSAT.OEL.OEL oXL(
    vOEL_max=0.05,
    T0=5,
    xd=order4.xd,
    xq=order4.xq,
    if_lim=2.15)
    annotation (Placement(transformation(extent={{-20,24},{-40,44}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-52,30},{-62,40}})));
  Modelica.Blocks.Sources.Ramp ramp(
    duration=20,
    startTime=1,
    height=-0.1) annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=0,
        origin={-37,59})));
equation
  connect(order4.pm0, order4.pm) annotation (Line(points={{-43,-11},{-43,-16},
          {-60,-16},{-60,-5},{-47,-5}},color={0,0,127}));
  connect(order4.p, bus.p)
    annotation (Line(points={{-25,0},{-25,0},{0,0}}, color={0,0,255}));
  connect(order4.vf, exciter_Type_II.vf) annotation (Line(points={{-47,5},{-96.75,
          5},{-96.75,28},{-92,28}}, color={0,0,127}));
  connect(exciter_Type_II.vf0, order4.vf0) annotation (Line(points={{-80,16},
          {-80,16},{-80,11},{-43,11}},color={0,0,127}));
  connect(exciter_Type_II.vref0, oXL.v_ref0) annotation (Line(points={{-80,40},
          {-80,40},{-80,48},{-30,48},{-30,45.2}},color={0,0,127}));
  connect(oXL.v, order4.v) annotation (Line(points={{-20.8,40},{-12,40},{-12,
          3},{-24,3}}, color={0,0,127}));
  connect(order4.P, oXL.p) annotation (Line(points={{-24,-3},{-14,-3},{-14,36},
          {-20.8,36}}, color={0,0,127}));
  connect(order4.Q, oXL.q) annotation (Line(points={{-24,-7},{-16,-7},{-16,32},
          {-20.8,32}}, color={0,0,127}));
  connect(exciter_Type_II.v, order4.v) annotation (Line(points={{-68,22},{-20,
          22},{-20,3},{-24,3}}, color={0,0,127}));
  connect(exciter_Type_II.vref, add.y) annotation (Line(points={{-68,34},{-66,
          34},{-66,35},{-62.5,35}}, color={0,0,127}));
  connect(add.u2, oXL.v_ref) annotation (Line(points={{-51,32},{-46,32},{-46,
          34},{-40.4,34}}, color={0,0,127}));
  connect(ramp.y, add.u1) annotation (Line(points={{-42.5,59},{-42.5,48.5},{-51,
          48.5},{-51,38}}, color={0,0,127}));
  annotation (Documentation, Diagram(coordinateSystem(preserveAspectRatio=
            false, extent={{-100,-100},{100,100}})));
end AVRTypeII_OEL_Test;
