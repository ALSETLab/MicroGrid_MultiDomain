within MicroGrid.Electrical.Controls.TG.GGOV1.Original.Tests;
model GGOV1_Test "Three phase to ground fault test of GGOV (modified from OpenIPSL test)"
  extends OpenIPSL.Examples.BaseClasses.SMIB(
    constantLoad(angle_0=-0.0100577809552),
    FAULT(displayPF=true),
    GEN2(displayPF=true),
    SHUNT(displayPF=true),
    LOAD(displayPF=true),
    GEN1(displayPF=true));
  OpenIPSL.Electrical.Machines.PSSE.GENROU generator(
    Xppd=0.2,
    Xppq=0.2,
    Xpp=0.2,
    Xl=0.12,
    M_b=100000000,
    Tpd0=5,
    Tppd0=0.50000E-01,
    Tppq0=0.1,
    H=4.0000,
    D=0,
    Xd=1.41,
    Xq=1.3500,
    Xpd=0.3,
    S10=0.1,
    S12=0.5,
    Xpq=0.6,
    Tpq0=0.7,
    angle_0=0.07068583470577,
    P_0=39999950,
    Q_0=5416571)  annotation (Placement(transformation(extent={{-100,-16},{-60,18}})));
  Original.GGOV1 gov(
    R=0.04,
    T_pelec=1,
    maxerr=0.05,
    minerr=-0.05,
    Kpgov=10,
    Kigov=2,
    Kdgov=0,
    Tdgov=1,
    Vmax=1,
    Vmin=0.15,
    Tact=0.5,
    Kturb=1.5,
    Wfnl=0.2,
    Tb=0.1,
    Tc=0,
    Teng=0,
    Tfload=3,
    Kpload=2,
    Kiload=0.67,
    Ldref=1,
    Dm=0,
    Ropen=0.1,
    Rclose=-0.1,
    Kimw=0,
    Aset=0.1,
    Ka=10,
    Ta=0.1,
    Trate=0,
    db=0,
    Tsa=4,
    Tsb=5,
    Rup=99,
    Rdown=-99,
    DELT=0.005) annotation (Placement(transformation(
        extent={{-20,-18},{20,18}},
        rotation=180,
        origin={-68,40})));
equation
  connect(gov.PELEC, generator.PELEC) annotation (Line(
      points={{-47.3793,46.9652},{-34,46.9652},{-34,6.1},{-58,6.1}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(generator.EFD0, generator.EFD) annotation (Line(
      points={{-58,-7.5},{-52,-7.5},{-52,-30},{-102,-30},{-102,-7.5},{-104,-7.5}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(gov.PMECH, generator.PMECH) annotation (Line(points={{-88.8276,
          39.2174},{-106,39.2174},{-106,9.5},{-104,9.5}},                color={0,0,127}));
  connect(generator.SPEED, gov.SPEED) annotation (Line(points={{-58,12.9},{-40,
          12.9},{-40,29.9043},{-47.5172,29.9043}},                      color={0,0,127}));
  connect(generator.p, GEN1.p) annotation (Line(points={{-60,1},{
          -50,1},{-50,0},{-30,0}},        color={0,0,255}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={          Text(
          extent={{38,78},{92,66}},
          textColor={238,46,47},
          textString="Validated!!",
          textStyle={TextStyle.Bold})}),Documentation(revisions="<html>
<!--DISCLAIMER-->
<p>OpenIPSL:</p>
<p>Copyright 2016 SmarTS Lab (Sweden)</p>
<ul>
<li>SmarTS Lab, research group at KTH: <a href=\"https://www.kth.se/en\">https://www.kth.se/en</a></li>
</ul>
<p>The authors can be contacted by email: <a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p>

<p>This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. </p>
<p>If a copy of the MPL was not distributed with this file, You can obtain one at <a href=\"http://mozilla.org/MPL/2.0/\"> http://mozilla.org/MPL/2.0</a>.</p>

<p></p>
<p>iPSL:</p>
<p>Copyright 2015-2016 RTE (France), SmarTS Lab (Sweden), AIA (Spain) and DTU (Denmark)</p>
<ul>
<li>RTE: <a href=\"http://www.rte-france.com\">http://www.rte-france.com</a></li>
<li>SmarTS Lab, research group at KTH: <a href=\"https://www.kth.se/en\">https://www.kth.se/en</a></li>
<li>AIA: <a href=\"http://www.aia.es/en/energy\"> http://www.aia.es/en/energy</a></li>
<li>DTU: <a href=\"http://www.dtu.dk/english\"> http://www.dtu.dk/english</a></li>
</ul>
<p>The authors can be contacted by email: <a href=\"mailto:info@itesla-ipsl.org\">info@itesla-ipsl.org</a></p>

<p>This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. </p>
<p>If a copy of the MPL was not distributed with this file, You can obtain one at <a href=\"http://mozilla.org/MPL/2.0/\"> http://mozilla.org/MPL/2.0</a>.</p>
</html>
"));
end GGOV1_Test;
