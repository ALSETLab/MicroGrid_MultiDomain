within MicroGrid.Examples.BaseClasses;
package SMIB "\"Single Machine Infinite Bus network models\""

  package Partial "Partial models containing the network elements"

    partial model SMIB_Partial_Noise "Partial model containing the network elements, GENROU generator, stochastic load"
      extends Modelica.Icons.Example;

      import Modelica.Constants.pi;
      OpenIPSL.Electrical.Buses.Bus GEN1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-54,-12},{-30,12}})));
      OpenIPSL.Electrical.Buses.Bus BUS1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-18,-12},{6,12}})));
      OpenIPSL.Electrical.Buses.Bus GEN2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{82,-12},{106,12}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer transformer(
        rT=0,
        xT=0.15,
        Sn=10000000,
        V_b=13800,
        Vn=13800)  annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
      OpenIPSL.Electrical.Branches.PwLine line_1(
        G=0,
        B=0,
        R=0.001,
        X=0.2) annotation (Placement(transformation(extent={{34,14},{52,26}})));
      OpenIPSL.Electrical.Branches.PwLine line_2(
        G=0,
        B=0,
        R=0.0003,
        X=0.06) annotation (Placement(transformation(extent={{4,-26},{22,-14}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=10000000,
         fn=50) annotation (Placement(transformation(extent={{-136,76},{-82,96}})));
    protected
      parameter Real S_b=SysData.S_b;
    public
      OpenIPSL.Electrical.Buses.Bus LOAD(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{16,-32},{40,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_3(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{38,-26},{56,-14}})));
    public
      OpenIPSL.Electrical.Buses.Bus BUS2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{48,-32},{72,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_4(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{64,-26},{82,-14}})));
      OpenIPSL.Electrical.Branches.PwLine line_5(
        G=0,
        B=0,
        R=0,
        X=0.00001) annotation (Placement(transformation(
            extent={{-9,-6},{9,6}},
            rotation=270,
            origin={35,-36})));

    public
      OpenIPSL.Electrical.Buses.Bus BUS3(V_b=13800, displayPF=true)
             annotation (Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=90,
            origin={36,-66})));
      OpenIPSL.Electrical.Machines.PSSE.GENCLS infiniteGen(
        angle_0=0,
        Q_0=718540.8,
        P_0=25.69692)   annotation (Placement(transformation(extent={{120,-10},{100,10}})));
      OpenIPSL.Electrical.Loads.NoiseInjections.SineNoiseInjection sineNoiseInjection(
        active_sigma=0.0001,
        freqHz=10,
        amplitude=0)
        annotation (Placement(transformation(extent={{0,-94},{20,-74}})));
      OpenIPSL.Electrical.Loads.PSSE.Load_ExtInput variableLoad(
        V_b=13800,
        v_0=0.9912454,
        angle_0=-0.010623773778329,
        P_0=5000000,
        Q_0=1000000,
        t1=0,
        d_t=0,
        d_P=0,
        characteristic=2)
        annotation (Placement(transformation(extent={{26,-98},{46,-80}})));
      OpenIPSL.Electrical.Sensors.SoftPMU pMU annotation (
          Placement(transformation(
            extent={{-7,-8},{7,8}},
            rotation=-90,
            origin={36,-55})));
    equation
      connect(GEN1.p, transformer.p) annotation (Line(points={{-42,0},{-35,0}}, color={0,0,255}));
      connect(transformer.n, BUS1.p) annotation (Line(points={{-13,0},{-6,0}}, color={0,0,255}));
      connect(BUS1.p, line_1.p) annotation (Line(points={{-6,0},{-6,0},{2,0},{2,20},{34.9,20}}, color={0,0,255}));
      connect(line_1.n, GEN2.p) annotation (Line(points={{51.1,20},{88,20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_2.p, line_1.p) annotation (Line(points={{4.9,-20},{2,-20},{2,20},{34.9,20}}, color={0,0,255}));
      connect(LOAD.p, line_2.n) annotation (Line(points={{28,-20},{21.1,-20}}, color={0,0,255}));
      connect(LOAD.p, line_3.p) annotation (Line(points={{28,-20},{28,-20},{38.9,-20}}, color={0,0,255}));
      connect(BUS2.p, line_3.n) annotation (Line(points={{60,-20},{55.1,-20}}, color={0,0,255}));
      connect(line_4.p, BUS2.p) annotation (Line(points={{64.9,-20},{62.45,-20},{60,-20}}, color={0,0,255}));
      connect(line_4.n, GEN2.p) annotation (Line(points={{81.1,-20},{88,-20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_5.p, line_3.p) annotation (Line(points={{35,-27.9},{35,-20},{38.9,-20}}, color={0,0,255}));
      connect(infiniteGen.p, GEN2.p) annotation (Line(points={{100,0},{97,0},{94,0}}, color={0,0,255}));
      connect(variableLoad.p, BUS3.p) annotation (Line(points={{36,-80},{36,-66}}, color={0,0,255}));
      connect(variableLoad.u, sineNoiseInjection.y) annotation (Line(points={{27.9,-84.05},{24,-84.05},{24,-84.1},{20.9,-84.1}}, color={0,0,127}));
      connect(pMU.n, BUS3.p) annotation (Line(points={{36,-59.9},{36,-62.95},{36,-66}}, color={0,0,255}));
      connect(pMU.p, line_5.n) annotation (Line(points={{36,-50.1},{36,-50.1},{36,-44.1},{35,-44.1}}, color={0,0,255}));
      annotation (
        Placement(transformation(extent={{220,-100},{232,-88}})),
        Diagram(coordinateSystem(extent={{-140,-100},{120,100}}, preserveAspectRatio=false), graphics={Text(
              extent={{-110,80},{110,60}},
              lineColor={0,0,0},
              lineThickness=1,
              fontSize=15,
              textStyle={TextStyle.Bold},
              textString="Single-machine Infinite Bus Model")}),
        Icon(coordinateSystem(extent={{-140,-100},{120,100}})),
        experiment(
          StopTime=10,
          Interval=0.0001,
          Tolerance=1e-006,
          __Dymola_fixedstepsize=0.0001,
          __Dymola_Algorithm="Rkfix2"),
        __Dymola_experimentSetupOutput,
        Documentation(revisions="<html>
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
",     info="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>SMIB PSAT, d_kundur2.mdl, PSAT</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>February 2016</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Maxime Baudette, Ahsan Murad, SmarTS Lab, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p></td>
</tr>
</table>
</html>"));
    end SMIB_Partial_Noise;

    partial model SMIB_Partial_Noise_normal
      "Partial model containing the network elements, GENROU generator, stochastic load"
      extends Modelica.Icons.Example;

      import Modelica.Constants.pi;
      OpenIPSL.Electrical.Buses.Bus GEN1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-54,-12},{-30,12}})));
      OpenIPSL.Electrical.Buses.Bus BUS1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-18,-12},{6,12}})));
      OpenIPSL.Electrical.Buses.Bus GEN2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{82,-12},{106,12}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer transformer(
        rT=0,
        xT=0.15,
        Sn=10000000,
        V_b=13800,
        Vn=13800)  annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
      OpenIPSL.Electrical.Branches.PwLine line_1(
        G=0,
        B=0,
        R=0.001,
        X=0.2) annotation (Placement(transformation(extent={{34,14},{52,26}})));
      OpenIPSL.Electrical.Branches.PwLine line_2(
        G=0,
        B=0,
        R=0.0003,
        X=0.06) annotation (Placement(transformation(extent={{4,-26},{22,-14}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=10000000,
         fn=50) annotation (Placement(transformation(extent={{-60,-60},
      {-6,-40}})));
    protected
      parameter Real S_b=SysData.S_b;
    public
      OpenIPSL.Electrical.Buses.Bus LOAD(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{16,-32},{40,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_3(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{38,-26},{56,-14}})));
    public
      OpenIPSL.Electrical.Buses.Bus BUS2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{48,-32},{72,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_4(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{64,-26},{82,-14}})));
      OpenIPSL.Electrical.Branches.PwLine line_5(
        G=0,
        B=0,
        R=0,
        X=0.00001) annotation (Placement(transformation(
            extent={{-9,-6},{9,6}},
            rotation=270,
            origin={35,-36})));

    public
      OpenIPSL.Electrical.Buses.Bus BUS3(V_b=13800, displayPF=true)
             annotation (Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=90,
            origin={36,-66})));
      OpenIPSL.Electrical.Machines.PSSE.GENCLS infiniteGen(
        angle_0=0,
        Q_0=718540.8,
        P_0=25.69692)    annotation (Placement(transformation(extent={{120,-10},{100,10}})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection whiteNoiseInjection(
          active_sigma=0.0001)
        annotation (Placement(transformation(extent={{0,-94},{20,-74}})));
      OpenIPSL.Electrical.Loads.PSSE.Load_ExtInput variableLoad(
        V_b=13800,
        v_0=0.9912454,
        angle_0=-0.010623773778329,
        P_0=5000000,
        Q_0=1000000,
        t1=0,
        d_t=0,
        d_P=0,
        characteristic=2)
        annotation (Placement(transformation(extent={{26,-98},{46,-80}})));
      OpenIPSL.Electrical.Sensors.SoftPMU pMU annotation (
          Placement(transformation(
            extent={{-7,-8},{7,8}},
            rotation=-90,
            origin={36,-55})));
    equation
      connect(GEN1.p, transformer.p) annotation (Line(points={{-42,0},{-35,0}}, color={0,0,255}));
      connect(transformer.n, BUS1.p) annotation (Line(points={{-13,0},{-6,0}}, color={0,0,255}));
      connect(BUS1.p, line_1.p) annotation (Line(points={{-6,0},{-6,0},{2,0},{2,20},{34.9,20}}, color={0,0,255}));
      connect(line_1.n, GEN2.p) annotation (Line(points={{51.1,20},{88,20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_2.p, line_1.p) annotation (Line(points={{4.9,-20},{2,-20},{2,20},{34.9,20}}, color={0,0,255}));
      connect(LOAD.p, line_2.n) annotation (Line(points={{28,-20},{21.1,-20}}, color={0,0,255}));
      connect(LOAD.p, line_3.p) annotation (Line(points={{28,-20},{28,-20},{38.9,-20}}, color={0,0,255}));
      connect(BUS2.p, line_3.n) annotation (Line(points={{60,-20},{55.1,-20}}, color={0,0,255}));
      connect(line_4.p, BUS2.p) annotation (Line(points={{64.9,-20},{62.45,-20},{60,-20}}, color={0,0,255}));
      connect(line_4.n, GEN2.p) annotation (Line(points={{81.1,-20},{88,-20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_5.p, line_3.p) annotation (Line(points={{35,-27.9},{35,-20},{38.9,-20}}, color={0,0,255}));
      connect(infiniteGen.p, GEN2.p) annotation (Line(points={{100,0},{97,0},{94,0}}, color={0,0,255}));
      connect(variableLoad.p, BUS3.p) annotation (Line(points={{36,-80},{36,-66}}, color={0,0,255}));
      connect(variableLoad.u, whiteNoiseInjection.y) annotation (Line(points={{27.9,
              -84.05},{24,-84.05},{24,-84.1},{20.9,-84.1}}, color={0,0,127}));
      connect(pMU.n, BUS3.p) annotation (Line(points={{36,-59.9},{36,-62.95},{36,-66}}, color={0,0,255}));
      connect(pMU.p, line_5.n) annotation (Line(points={{36,-50.1},{36,-50.1},{36,-44.1},{35,-44.1}}, color={0,0,255}));
      annotation (
        Placement(transformation(extent={{220,-100},{232,-88}})),
        Diagram(coordinateSystem(extent={{-140,-100},{120,100}}, preserveAspectRatio=false)),
        Icon(coordinateSystem(extent={{-140,-100},{120,100}})),
        experiment(
          StopTime=10,
          Interval=0.0001,
          Tolerance=1e-006,
          __Dymola_fixedstepsize=0.0001,
          __Dymola_Algorithm="Rkfix2"),
        __Dymola_experimentSetupOutput,
        Documentation(revisions="<html>
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
",     info="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>SMIB PSAT, d_kundur2.mdl, PSAT</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>February 2016</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Maxime Baudette, Ahsan Murad, SmarTS Lab, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p></td>
</tr>
</table>
</html>"));
    end SMIB_Partial_Noise_normal;

    partial model SMIB_Partial_Noise_normal_limhit
      "Partial model containing the network elements, GENROU generator, stochastic load"
      extends Modelica.Icons.Example;

      import Modelica.Constants.pi;
      OpenIPSL.Electrical.Buses.Bus GEN1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-54,-12},{-30,12}})));
      OpenIPSL.Electrical.Buses.Bus BUS1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-18,-12},{6,12}})));
      OpenIPSL.Electrical.Buses.Bus GEN2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{82,-12},{106,12}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer transformer(
        rT=0,
        xT=0.15,
        Sn=10000000,
        V_b=13800,
        Vn=13800)  annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
      OpenIPSL.Electrical.Branches.PwLine line_1(
        G=0,
        B=0,
        R=0.001,
        X=0.2) annotation (Placement(transformation(extent={{34,14},{52,26}})));
      OpenIPSL.Electrical.Branches.PwLine line_2(
        G=0,
        B=0,
        R=0.0003,
        X=0.06) annotation (Placement(transformation(extent={{4,-26},{22,-14}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=10000000,
         fn=50) annotation (Placement(transformation(extent={{66,-100},
      {120,-80}})));
    protected
      parameter Real S_b=SysData.S_b;
    public
      OpenIPSL.Electrical.Buses.Bus LOAD(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{16,-32},{40,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_3(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{38,-26},{56,-14}})));
    public
      OpenIPSL.Electrical.Buses.Bus BUS2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{48,-32},{72,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_4(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{64,-26},{82,-14}})));
      OpenIPSL.Electrical.Branches.PwLine line_5(
        G=0,
        B=0,
        R=0,
        X=0.00001) annotation (Placement(transformation(
            extent={{-9,-6},{9,6}},
            rotation=270,
            origin={35,-36})));

    public
      OpenIPSL.Electrical.Buses.Bus BUS3(V_b=13800, displayPF=true)
             annotation (Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=90,
            origin={36,-66})));
      OpenIPSL.Electrical.Machines.PSSE.GENCLS infiniteGen(
        angle_0=0,
        Q_0=718540.8,
        P_0=25.69692)   annotation (Placement(transformation(extent={{120,-10},{100,10}})));
      OpenIPSL.Electrical.Loads.PSSE.Load_ExtInput variableLoad(
        V_b=13800,
        v_0=0.9912454,
        angle_0=-0.010623773778329,
        P_0=5000000,
        Q_0=1000000,
        t1=0,
        d_t=0,
        d_P=0,
        characteristic=2)
        annotation (Placement(transformation(extent={{26,-94},{46,-76}})));
      OpenIPSL.Electrical.Sensors.SoftPMU pMU annotation (
          Placement(transformation(
            extent={{-7,-8},{7,8}},
            rotation=-90,
            origin={36,-55})));
    equation
      connect(GEN1.p, transformer.p) annotation (Line(points={{-42,0},{-35,0}}, color={0,0,255}));
      connect(transformer.n, BUS1.p) annotation (Line(points={{-13,0},{-6,0}}, color={0,0,255}));
      connect(BUS1.p, line_1.p) annotation (Line(points={{-6,0},{-6,0},{2,0},{2,20},{34.9,20}}, color={0,0,255}));
      connect(line_1.n, GEN2.p) annotation (Line(points={{51.1,20},{88,20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_2.p, line_1.p) annotation (Line(points={{4.9,-20},{2,-20},{2,20},{34.9,20}}, color={0,0,255}));
      connect(LOAD.p, line_2.n) annotation (Line(points={{28,-20},{21.1,-20}}, color={0,0,255}));
      connect(LOAD.p, line_3.p) annotation (Line(points={{28,-20},{28,-20},{38.9,-20}}, color={0,0,255}));
      connect(BUS2.p, line_3.n) annotation (Line(points={{60,-20},{55.1,-20}}, color={0,0,255}));
      connect(line_4.p, BUS2.p) annotation (Line(points={{64.9,-20},{62.45,-20},{60,-20}}, color={0,0,255}));
      connect(line_4.n, GEN2.p) annotation (Line(points={{81.1,-20},{88,-20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_5.p, line_3.p) annotation (Line(points={{35,-27.9},{35,-20},{38.9,-20}}, color={0,0,255}));
      connect(infiniteGen.p, GEN2.p) annotation (Line(points={{100,0},{97,0},{94,0}}, color={0,0,255}));
      connect(variableLoad.p, BUS3.p) annotation (Line(points={{36,-76},{36,-66}}, color={0,0,255}));
      connect(pMU.n, BUS3.p) annotation (Line(points={{36,-59.9},{36,-62.95},{36,-66}}, color={0,0,255}));
      connect(pMU.p, line_5.n) annotation (Line(points={{36,-50.1},{36,-50.1},{36,-44.1},{35,-44.1}}, color={0,0,255}));
      annotation (
        Placement(transformation(extent={{220,-100},{232,-88}})),
        Diagram(coordinateSystem(extent={{-140,-100},{120,100}}, preserveAspectRatio=false)),
        Icon(coordinateSystem(extent={{-140,-100},{120,100}})),
        experiment(
          StopTime=10,
          Interval=0.0001,
          Tolerance=1e-006,
          __Dymola_fixedstepsize=0.0001,
          __Dymola_Algorithm="Rkfix2"),
        __Dymola_experimentSetupOutput,
        Documentation(revisions="<html>
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
",     info="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>SMIB PSAT, d_kundur2.mdl, PSAT</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>February 2016</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Maxime Baudette, Ahsan Murad, SmarTS Lab, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p></td>
</tr>
</table>
</html>"));
    end SMIB_Partial_Noise_normal_limhit;

    partial model SMIB_Partial_Noise_normal_signalb
      "Partial model containing the network elements, GENROU generator, stochastic load"
      extends Modelica.Icons.Example;

      import Modelica.Constants.pi;
      OpenIPSL.Electrical.Buses.Bus GEN1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-54,-12},{-30,12}})));
      OpenIPSL.Electrical.Buses.Bus BUS1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-18,-12},{6,12}})));
      OpenIPSL.Electrical.Buses.Bus GEN2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{82,-12},{106,12}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer transformer(
        rT=0,
        xT=0.15,
        Sn=10000000,
        V_b=13800,
        Vn=13800)  annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
      OpenIPSL.Electrical.Branches.PwLine line_1(
        G=0,
        B=0,
        R=0.001,
        X=0.2) annotation (Placement(transformation(extent={{34,14},{52,26}})));
      OpenIPSL.Electrical.Branches.PwLine line_2(
        G=0,
        B=0,
        R=0.0003,
        X=0.06) annotation (Placement(transformation(extent={{4,-26},{22,-14}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=10000000,
         fn=50) annotation (Placement(transformation(extent={{-60,-60},
      {-6,-40}})));
    protected
      parameter Real S_b=SysData.S_b;
    public
      OpenIPSL.Electrical.Buses.Bus LOAD(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{16,-32},{40,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_3(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{38,-26},{56,-14}})));
    public
      OpenIPSL.Electrical.Buses.Bus BUS2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{48,-32},{72,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_4(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{64,-26},{82,-14}})));
      OpenIPSL.Electrical.Branches.PwLine line_5(
        G=0,
        B=0,
        R=0,
        X=0.00001) annotation (Placement(transformation(
            extent={{-9,-6},{9,6}},
            rotation=270,
            origin={35,-36})));

    public
      OpenIPSL.Electrical.Buses.Bus BUS3(V_b=13800, displayPF=true)
             annotation (Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=90,
            origin={36,-66})));
      OpenIPSL.Electrical.Machines.PSSE.GENCLS infiniteGen(
        angle_0=0,
        Q_0=718540.8,
        P_0=25.69692)   annotation (Placement(transformation(extent={{120,-10},{100,10}})));
      OpenIPSL.Electrical.Loads.PSSE.Load_ExtInput variableLoad(
        V_b=13800,
        v_0=0.9912454,
        angle_0=-0.010623773778329,
        P_0=5000000,
        Q_0=1000000,
        t1=0,
        d_t=0,
        d_P=0,
        characteristic=2)
        annotation (Placement(transformation(extent={{26,-98},{46,-80}})));
      OpenIPSL.Electrical.Sensors.SoftPMU pMU annotation (
          Placement(transformation(
            extent={{-7,-8},{7,8}},
            rotation=-90,
            origin={36,-55})));
    equation
      connect(GEN1.p, transformer.p) annotation (Line(points={{-42,0},{-35,0}}, color={0,0,255}));
      connect(transformer.n, BUS1.p) annotation (Line(points={{-13,0},{-6,0}}, color={0,0,255}));
      connect(BUS1.p, line_1.p) annotation (Line(points={{-6,0},{-6,0},{2,0},{2,20},{34.9,20}}, color={0,0,255}));
      connect(line_1.n, GEN2.p) annotation (Line(points={{51.1,20},{88,20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_2.p, line_1.p) annotation (Line(points={{4.9,-20},{2,-20},{2,20},{34.9,20}}, color={0,0,255}));
      connect(LOAD.p, line_2.n) annotation (Line(points={{28,-20},{21.1,-20}}, color={0,0,255}));
      connect(LOAD.p, line_3.p) annotation (Line(points={{28,-20},{28,-20},{38.9,-20}}, color={0,0,255}));
      connect(BUS2.p, line_3.n) annotation (Line(points={{60,-20},{55.1,-20}}, color={0,0,255}));
      connect(line_4.p, BUS2.p) annotation (Line(points={{64.9,-20},{62.45,-20},{60,-20}}, color={0,0,255}));
      connect(line_4.n, GEN2.p) annotation (Line(points={{81.1,-20},{88,-20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_5.p, line_3.p) annotation (Line(points={{35,-27.9},{35,-20},{38.9,-20}}, color={0,0,255}));
      connect(infiniteGen.p, GEN2.p) annotation (Line(points={{100,0},{97,0},{94,0}}, color={0,0,255}));
      connect(variableLoad.p, BUS3.p) annotation (Line(points={{36,-80},{36,-66}}, color={0,0,255}));
      connect(pMU.n, BUS3.p) annotation (Line(points={{36,-59.9},{36,-62.95},{36,-66}}, color={0,0,255}));
      connect(pMU.p, line_5.n) annotation (Line(points={{36,-50.1},{36,-50.1},{36,-44.1},{35,-44.1}}, color={0,0,255}));
      annotation (
        Placement(transformation(extent={{220,-100},{232,-88}})),
        Diagram(coordinateSystem(extent={{-140,-100},{120,100}}, preserveAspectRatio=false)),
        Icon(coordinateSystem(extent={{-140,-100},{120,100}})),
        experiment(
          StopTime=10,
          Interval=0.0001,
          Tolerance=1e-006,
          __Dymola_fixedstepsize=0.0001,
          __Dymola_Algorithm="Rkfix2"),
        __Dymola_experimentSetupOutput,
        Documentation(revisions="<html>
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
",     info="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>SMIB PSAT, d_kundur2.mdl, PSAT</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>February 2016</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Maxime Baudette, Ahsan Murad, SmarTS Lab, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p></td>
</tr>
</table>
</html>"));
    end SMIB_Partial_Noise_normal_signalb;

    partial model SMIB_Partial_NoNoise "Partial model containing the network elements, GENROU generator, non stochastic load"
      extends Modelica.Icons.Example;

      import Modelica.Constants.pi;
      OpenIPSL.Electrical.Buses.Bus GEN1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-54,-12},{-30,12}})));
      OpenIPSL.Electrical.Buses.Bus BUS1(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{-18,-12},{6,12}})));
      OpenIPSL.Electrical.Buses.Bus GEN2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{82,-12},{106,12}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer transformer(
        rT=0,
        xT=0.15,
        Sn=10000000,
        V_b=13800,
        Vn=13800)  annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
      OpenIPSL.Electrical.Branches.PwLine line_1(
        G=0,
        B=0,
        R=0.001,
        X=0.2) annotation (Placement(transformation(extent={{34,14},{52,26}})));
      OpenIPSL.Electrical.Branches.PwLine line_2(
        G=0,
        B=0,
        R=0.0003,
        X=0.06) annotation (Placement(transformation(extent={{4,-26},{22,-14}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=10e6, fn=50) annotation (Placement(transformation(extent={{-136,76},{-82,96}})));
    protected
      parameter Real S_b=SysData.S_b;
    public
      OpenIPSL.Electrical.Buses.Bus LOAD(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{16,-32},{40,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_3(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{38,-26},{56,-14}})));
      OpenIPSL.Electrical.Buses.Bus BUS2(V_b=13800, displayPF=true)
             annotation (Placement(transformation(extent={{48,-32},{72,-8}})));
      OpenIPSL.Electrical.Branches.PwLine line_4(
        G=0,
        B=0,
        X=0.07,
        R=0.00035) annotation (Placement(transformation(extent={{64,-26},{82,-14}})));
      OpenIPSL.Electrical.Branches.PwLine line_5(
        G=0,
        B=0,
        R=0,
        X=0.00001) annotation (Placement(transformation(
            extent={{-9,-6},{9,6}},
            rotation=270,
            origin={35,-36})));
      OpenIPSL.Electrical.Buses.Bus BUS3(V_b=13800, displayPF=true)
             annotation (Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=90,
            origin={36,-66})));
      OpenIPSL.Electrical.Machines.PSSE.GENCLS infiniteGen(
        angle_0=0,
        Q_0=0.7185408e6,
        P_0=2.569692e1) annotation (Placement(transformation(extent={{120,-10},{100,10}})));
      OpenIPSL.Electrical.Loads.PSSE.Load_variation variableLoad(
        V_b=13800,
        v_0=0.9912454,
        angle_0=-0.010623773778329,
        P_0=5000000,
        Q_0=1000000,
        characteristic=2,
        d_P=0,
        t1=0,
        d_t=0) annotation (Placement(transformation(extent={{26,-98},{46,-80}})));
      OpenIPSL.Electrical.Sensors.SoftPMU pMU annotation (
          Placement(transformation(
            extent={{-7,-8},{7,8}},
            rotation=-90,
            origin={36,-55})));
    equation
      connect(GEN1.p, transformer.p) annotation (Line(points={{-42,0},{-35,0}}, color={0,0,255}));
      connect(transformer.n, BUS1.p) annotation (Line(points={{-13,0},{-6,0}}, color={0,0,255}));
      connect(BUS1.p, line_1.p) annotation (Line(points={{-6,0},{-6,0},{2,0},{2,20},{34.9,20}}, color={0,0,255}));
      connect(line_1.n, GEN2.p) annotation (Line(points={{51.1,20},{88,20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_2.p, line_1.p) annotation (Line(points={{4.9,-20},{2,-20},{2,20},{34.9,20}}, color={0,0,255}));
      connect(LOAD.p, line_2.n) annotation (Line(points={{28,-20},{21.1,-20}}, color={0,0,255}));
      connect(LOAD.p, line_3.p) annotation (Line(points={{28,-20},{28,-20},{38.9,-20}}, color={0,0,255}));
      connect(BUS2.p, line_3.n) annotation (Line(points={{60,-20},{55.1,-20}}, color={0,0,255}));
      connect(line_4.p, BUS2.p) annotation (Line(points={{64.9,-20},{62.45,-20},{60,-20}}, color={0,0,255}));
      connect(line_4.n, GEN2.p) annotation (Line(points={{81.1,-20},{88,-20},{88,0},{94,0}}, color={0,0,255}));
      connect(line_5.p, line_3.p) annotation (Line(points={{35,-27.9},{35,-20},{38.9,-20}}, color={0,0,255}));
      connect(infiniteGen.p, GEN2.p) annotation (Line(points={{100,0},{97,0},{94,0}}, color={0,0,255}));
      connect(variableLoad.p, BUS3.p) annotation (Line(points={{36,-80},{36,-66}}, color={0,0,255}));
      connect(pMU.p, line_5.n) annotation (Line(points={{36,-50.1},{36,-44.1},{35,-44.1}}, color={0,0,255}));
      connect(pMU.n, BUS3.p) annotation (Line(points={{36,-59.9},{36,-62},{36,-66}}, color={0,0,255}));
      annotation (
        Placement(transformation(extent={{220,-100},{232,-88}})),
        Diagram(coordinateSystem(extent={{-140,-100},{120,100}}, preserveAspectRatio=false), graphics={Text(
              extent={{-110,80},{110,60}},
              lineColor={0,0,0},
              lineThickness=1,
              fontSize=15,
              textStyle={TextStyle.Bold},
              textString="Single-machine Infinite Bus Model")}),
        Icon(coordinateSystem(extent={{-140,-100},{120,100}})),
        experiment(
          StopTime=10,
          Interval=0.0001,
          Tolerance=1e-006,
          __Dymola_fixedstepsize=0.0001,
          __Dymola_Algorithm="Rkfix2"),
        __Dymola_experimentSetupOutput,
        Documentation(revisions="<html>
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
",     info="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>SMIB PSAT, d_kundur2.mdl, PSAT</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>February 2016</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Maxime Baudette, Ahsan Murad, SmarTS Lab, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p><a href=\"mailto:luigiv@kth.se\">luigiv@kth.se</a></p></td>
</tr>
</table>
</html>"));
    end SMIB_Partial_NoNoise;
  end Partial;

  package Records "Records with power flow solutions"

    record Loads "P,Q power flow results of the loads"
      extends Modelica.Icons.Record;
      // Load 23_1
      parameter OpenIPSL.Types.ActivePower PL23_1=5.000000e6;
      parameter OpenIPSL.Types.ReactivePower QL23_1=1.000000e6;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Loads;

    record Machines "P,Q power flow results of the machines"
      extends Modelica.Icons.Record;
      // Machine 3_1
      parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
      parameter OpenIPSL.Types.ReactivePower Q3_1=0.932000e6;
      // Machine 1_1
      parameter OpenIPSL.Types.ActivePower P1_1=5.000000e6;
      parameter OpenIPSL.Types.ReactivePower Q1_1=0.585000e6;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Machines;

    record PF_030
      //Power flow results for the snapshot h30.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=0.054047610946508395;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.997170;
        parameter OpenIPSL.Types.Angle A2=0.00903033354981866;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.995380;
        parameter OpenIPSL.Types.Angle A21=-0.00629714794119554;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.997690;
        parameter OpenIPSL.Types.Angle A22=-0.003145083312093782;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.995380;
        parameter OpenIPSL.Types.Angle A23=-0.00629714794119554;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=0.475000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=3.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.256000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=3.000000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=0.600000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_030;

    record PF_050
      //Power flow results for the snapshot h50.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=0.0907344318234292;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.994050;
        parameter OpenIPSL.Types.Angle A2=0.01521403508963457;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.991240;
        parameter OpenIPSL.Types.Angle A21=-0.010623819156889484;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.995610;
        parameter OpenIPSL.Types.Angle A22=-0.005288347633542819;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.991240;
        parameter OpenIPSL.Types.Angle A23=-0.010623819156889484;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=0.932000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.585000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.000000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.000000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_050;

    record PF_051
      //Power flow results for the snapshot h51.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.303800*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.993900;
        parameter OpenIPSL.Types.Angle A2=0.889400*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.991040;
        parameter OpenIPSL.Types.Angle A21=-0.621100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.995500;
        parameter OpenIPSL.Types.Angle A22=-0.309200*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.991040;
        parameter OpenIPSL.Types.Angle A23=-0.621100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=0.955000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.100000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.603000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.100000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.020000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_051;

    record PF_052
      //Power flow results for the snapshot h52.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.409000*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.993740;
        parameter OpenIPSL.Types.Angle A2=0.907200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.990830;
        parameter OpenIPSL.Types.Angle A21=-0.633500*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.995400;
        parameter OpenIPSL.Types.Angle A22=-0.315300*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.990830;
        parameter OpenIPSL.Types.Angle A23=-0.633500*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=0.979000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.200000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.622000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.200000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.040000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_052;

    record PF_053
      //Power flow results for the snapshot h53.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.514300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.993580;
        parameter OpenIPSL.Types.Angle A2=0.924900*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.990620;
        parameter OpenIPSL.Types.Angle A21=-0.645900*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.995290;
        parameter OpenIPSL.Types.Angle A22=-0.321400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.990620;
        parameter OpenIPSL.Types.Angle A23=-0.645900*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.002000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.300000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.640000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.300000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.060000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_053;

    record PF_054
      //Power flow results for the snapshot h54.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.619600*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.993420;
        parameter OpenIPSL.Types.Angle A2=0.942700*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.990410;
        parameter OpenIPSL.Types.Angle A21=-0.658400*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.995190;
        parameter OpenIPSL.Types.Angle A22=-0.327600*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.990410;
        parameter OpenIPSL.Types.Angle A23=-0.658400*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.026000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.400000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.659000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.400000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.080000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_054;

    record PF_055
      //Power flow results for the snapshot h55.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.724899*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.993260;
        parameter OpenIPSL.Types.Angle A2=0.960500*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.990190;
        parameter OpenIPSL.Types.Angle A21=-0.670800*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.995080;
        parameter OpenIPSL.Types.Angle A22=-0.333800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.990190;
        parameter OpenIPSL.Types.Angle A23=-0.670800*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.049000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.500000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.678000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.500000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.100000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_055;

    record PF_056
      //Power flow results for the snapshot h56.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.830400*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.993090;
        parameter OpenIPSL.Types.Angle A2=0.978300*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.989980;
        parameter OpenIPSL.Types.Angle A21=-0.683300*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994970;
        parameter OpenIPSL.Types.Angle A22=-0.339900*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.989980;
        parameter OpenIPSL.Types.Angle A23=-0.683300*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.073000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.600000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.698000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.600000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.120000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_056;

    record PF_057
      //Power flow results for the snapshot h57.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=5.935900*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.992930;
        parameter OpenIPSL.Types.Angle A2=0.996100*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.989770;
        parameter OpenIPSL.Types.Angle A21=-0.695700*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994860;
        parameter OpenIPSL.Types.Angle A22=-0.346100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.989770;
        parameter OpenIPSL.Types.Angle A23=-0.695700*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.097000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.700000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.717000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.700000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.140000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_057;

    record PF_058
      //Power flow results for the snapshot h58.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.041400*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.992760;
        parameter OpenIPSL.Types.Angle A2=1.013900*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.989550;
        parameter OpenIPSL.Types.Angle A21=-0.708200*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994760;
        parameter OpenIPSL.Types.Angle A22=-0.352300*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.989550;
        parameter OpenIPSL.Types.Angle A23=-0.708200*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.122000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.800000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.737000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.800000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.160000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_058;

    record PF_059
      //Power flow results for the snapshot h59.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.147000*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.992590;
        parameter OpenIPSL.Types.Angle A2=1.031700*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.989330;
        parameter OpenIPSL.Types.Angle A21=-0.720700*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994650;
        parameter OpenIPSL.Types.Angle A22=-0.358400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.989330;
        parameter OpenIPSL.Types.Angle A23=-0.720700*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.146000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=5.900001e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.757000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=5.900001e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.180000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_059;

    record PF_060
      //Power flow results for the snapshot h60.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.252700*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.992420;
        parameter OpenIPSL.Types.Angle A2=1.049600*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.989110;
        parameter OpenIPSL.Types.Angle A21=-0.733200*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994540;
        parameter OpenIPSL.Types.Angle A22=-0.364600*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.989110;
        parameter OpenIPSL.Types.Angle A23=-0.733200*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.171000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.778000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.000000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.200000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_060;

    record PF_061
      //Power flow results for the snapshot h61.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.358500*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.992250;
        parameter OpenIPSL.Types.Angle A2=1.067500*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.988890;
        parameter OpenIPSL.Types.Angle A21=-0.745700*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994420;
        parameter OpenIPSL.Types.Angle A22=-0.370800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.988890;
        parameter OpenIPSL.Types.Angle A23=-0.745700*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.196000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.100000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.799000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.100000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.220000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_061;

    record PF_062
      //Power flow results for the snapshot h62.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.464300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.992080;
        parameter OpenIPSL.Types.Angle A2=1.085400*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.988670;
        parameter OpenIPSL.Types.Angle A21=-0.758300*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994310;
        parameter OpenIPSL.Types.Angle A22=-0.377000*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.988670;
        parameter OpenIPSL.Types.Angle A23=-0.758300*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.221000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.200000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.820000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.200000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.240000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_062;

    record PF_063
      //Power flow results for the snapshot h63.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.570200*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.991900;
        parameter OpenIPSL.Types.Angle A2=1.103300*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.988440;
        parameter OpenIPSL.Types.Angle A21=-0.770800*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994200;
        parameter OpenIPSL.Types.Angle A22=-0.383200*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.988440;
        parameter OpenIPSL.Types.Angle A23=-0.770800*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.246000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.300000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.841000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.300000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.260000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_063;

    record PF_064
      //Power flow results for the snapshot h64.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.676200*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.991720;
        parameter OpenIPSL.Types.Angle A2=1.121200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.988220;
        parameter OpenIPSL.Types.Angle A21=-0.783400*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.994090;
        parameter OpenIPSL.Types.Angle A22=-0.389400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.988220;
        parameter OpenIPSL.Types.Angle A23=-0.783400*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.271000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.400000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.862000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.400000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.280000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_064;

    record PF_065
      //Power flow results for the snapshot h65.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.782200*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.991550;
        parameter OpenIPSL.Types.Angle A2=1.139100*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.987990;
        parameter OpenIPSL.Types.Angle A21=-0.795900*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993970;
        parameter OpenIPSL.Types.Angle A22=-0.395600*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.987990;
        parameter OpenIPSL.Types.Angle A23=-0.795900*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.297000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.500000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.884000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.500000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.300000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_065;

    record PF_066
      //Power flow results for the snapshot h66.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.888300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.991370;
        parameter OpenIPSL.Types.Angle A2=1.157100*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.987770;
        parameter OpenIPSL.Types.Angle A21=-0.808500*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993860;
        parameter OpenIPSL.Types.Angle A22=-0.401800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.987770;
        parameter OpenIPSL.Types.Angle A23=-0.808500*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.323000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.599999e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.906000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.599999e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.320000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_066;

    record PF_067
      //Power flow results for the snapshot h67.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=6.994501*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.991190;
        parameter OpenIPSL.Types.Angle A2=1.175000*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.987540;
        parameter OpenIPSL.Types.Angle A21=-0.821100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993740;
        parameter OpenIPSL.Types.Angle A22=-0.408000*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.987540;
        parameter OpenIPSL.Types.Angle A23=-0.821100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.349000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.700000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.928000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.700000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.340000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_067;

    record PF_068
      //Power flow results for the snapshot h68.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.100700*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.991000;
        parameter OpenIPSL.Types.Angle A2=1.193000*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.987310;
        parameter OpenIPSL.Types.Angle A21=-0.833700*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993630;
        parameter OpenIPSL.Types.Angle A22=-0.414200*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.987310;
        parameter OpenIPSL.Types.Angle A23=-0.833700*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.375000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.800000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.951000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.800000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.360000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_068;

    record PF_069
      //Power flow results for the snapshot h69.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.206999*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.990820;
        parameter OpenIPSL.Types.Angle A2=1.211000*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.987080;
        parameter OpenIPSL.Types.Angle A21=-0.846300*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993510;
        parameter OpenIPSL.Types.Angle A22=-0.420400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.987080;
        parameter OpenIPSL.Types.Angle A23=-0.846300*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.401000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=6.900000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.973000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=6.900000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.380000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_069;

    record PF_070
      //Power flow results for the snapshot h70.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.313399*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.990630;
        parameter OpenIPSL.Types.Angle A2=1.229100*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.986840;
        parameter OpenIPSL.Types.Angle A21=-0.858900*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993390;
        parameter OpenIPSL.Types.Angle A22=-0.426600*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.986840;
        parameter OpenIPSL.Types.Angle A23=-0.858900*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.427000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=0.996000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.000000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.400000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_070;

    record PF_071
      //Power flow results for the snapshot h71.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.419900*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.990450;
        parameter OpenIPSL.Types.Angle A2=1.247100*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.986610;
        parameter OpenIPSL.Types.Angle A21=-0.871600*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993280;
        parameter OpenIPSL.Types.Angle A22=-0.432800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.986610;
        parameter OpenIPSL.Types.Angle A23=-0.871600*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.454000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.100000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.020000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.100000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.420000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_071;

    record PF_072
      //Power flow results for the snapshot h72.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.526400*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.990260;
        parameter OpenIPSL.Types.Angle A2=1.265100*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.986370;
        parameter OpenIPSL.Types.Angle A21=-0.884200*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993160;
        parameter OpenIPSL.Types.Angle A22=-0.439100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.986370;
        parameter OpenIPSL.Types.Angle A23=-0.884200*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.481000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.200000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.043000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.200000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.440000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_072;

    record PF_073
      //Power flow results for the snapshot h73.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.632999*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.990070;
        parameter OpenIPSL.Types.Angle A2=1.283200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.986130;
        parameter OpenIPSL.Types.Angle A21=-0.896900*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.993040;
        parameter OpenIPSL.Types.Angle A22=-0.445300*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.986130;
        parameter OpenIPSL.Types.Angle A23=-0.896900*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.001000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.508000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.300000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.067000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.300000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.460000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_073;

    record PF_074
      //Power flow results for the snapshot h74.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.739699*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.989870;
        parameter OpenIPSL.Types.Angle A2=1.301300*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.985900;
        parameter OpenIPSL.Types.Angle A21=-0.909600*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992920;
        parameter OpenIPSL.Types.Angle A22=-0.451500*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.985900;
        parameter OpenIPSL.Types.Angle A23=-0.909600*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.535000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.400000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.091000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.400000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.480000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_074;

    record PF_075
      //Power flow results for the snapshot h75.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=0.13694725975773506;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.989680;
        parameter OpenIPSL.Types.Angle A2=0.023027874150813185;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.985660;
        parameter OpenIPSL.Types.Angle A21=-0.016095426361891707;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992800;
        parameter OpenIPSL.Types.Angle A22=-0.00799011731563004;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.985660;
        parameter OpenIPSL.Types.Angle A23=-0.016095426361891707;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.563000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.500000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.116000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.500000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.500000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_075;

    record PF_076
      //Power flow results for the snapshot h76.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=7.953300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.989490;
        parameter OpenIPSL.Types.Angle A2=1.337500*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.985420;
        parameter OpenIPSL.Types.Angle A21=-0.935000*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992670;
        parameter OpenIPSL.Types.Angle A22=-0.464000*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.985420;
        parameter OpenIPSL.Types.Angle A23=-0.935000*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.590000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.600000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.140000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.600000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.520000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_076;

    record PF_077
      //Power flow results for the snapshot h77.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.060300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.989290;
        parameter OpenIPSL.Types.Angle A2=1.355700*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.985170;
        parameter OpenIPSL.Types.Angle A21=-0.947700*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992550;
        parameter OpenIPSL.Types.Angle A22=-0.470300*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.985170;
        parameter OpenIPSL.Types.Angle A23=-0.947700*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.618000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.700000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.165000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.700000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.540000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_077;

    record PF_078
      //Power flow results for the snapshot h78.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.167300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.989090;
        parameter OpenIPSL.Types.Angle A2=1.373900*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.984930;
        parameter OpenIPSL.Types.Angle A21=-0.960400*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992430;
        parameter OpenIPSL.Types.Angle A22=-0.476600*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.984930;
        parameter OpenIPSL.Types.Angle A23=-0.960400*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.646000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.800000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.190000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.800000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.560000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_078;

    record PF_079
      //Power flow results for the snapshot h79.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.274401*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.988890;
        parameter OpenIPSL.Types.Angle A2=1.392000*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.984680;
        parameter OpenIPSL.Types.Angle A21=-0.973200*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992310;
        parameter OpenIPSL.Types.Angle A22=-0.482800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.984680;
        parameter OpenIPSL.Types.Angle A23=-0.973200*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.674000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=7.900000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.216000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=7.900000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.580000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_079;

    record PF_080
      //Power flow results for the snapshot h80.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.381599*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.988690;
        parameter OpenIPSL.Types.Angle A2=1.410200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.984440;
        parameter OpenIPSL.Types.Angle A21=-0.985900*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992180;
        parameter OpenIPSL.Types.Angle A22=-0.489100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.984440;
        parameter OpenIPSL.Types.Angle A23=-0.985900*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.703000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.241000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.000000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.600000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_080;

    record PF_081
      //Power flow results for the snapshot h81.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.488800*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.988480;
        parameter OpenIPSL.Types.Angle A2=1.428500*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.984190;
        parameter OpenIPSL.Types.Angle A21=-0.998700*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.992060;
        parameter OpenIPSL.Types.Angle A22=-0.495400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.984190;
        parameter OpenIPSL.Types.Angle A23=-0.998700*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.731000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.100000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.267000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.100000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.620000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_081;

    record PF_082
      //Power flow results for the snapshot h82.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.596200*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.988280;
        parameter OpenIPSL.Types.Angle A2=1.446700*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.983940;
        parameter OpenIPSL.Types.Angle A21=-1.011500*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991930;
        parameter OpenIPSL.Types.Angle A22=-0.501600*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.983940;
        parameter OpenIPSL.Types.Angle A23=-1.011500*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.760000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.200000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.294000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.200000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.640000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_082;

    record PF_083
      //Power flow results for the snapshot h83.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.703600*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.988070;
        parameter OpenIPSL.Types.Angle A2=1.465000*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.983690;
        parameter OpenIPSL.Types.Angle A21=-1.024300*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991800;
        parameter OpenIPSL.Types.Angle A22=-0.507900*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.983690;
        parameter OpenIPSL.Types.Angle A23=-1.024300*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.789000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.300000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.320000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.300000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.660000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_083;

    record PF_084
      //Power flow results for the snapshot h84.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.811100*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.987860;
        parameter OpenIPSL.Types.Angle A2=1.483200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.983430;
        parameter OpenIPSL.Types.Angle A21=-1.037100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991680;
        parameter OpenIPSL.Types.Angle A22=-0.514200*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.983430;
        parameter OpenIPSL.Types.Angle A23=-1.037100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.818000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.400000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.347000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.400000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.680000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_084;

    record PF_085
      //Power flow results for the snapshot h85.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=8.918700*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.987650;
        parameter OpenIPSL.Types.Angle A2=1.501500*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.983180;
        parameter OpenIPSL.Types.Angle A21=-1.049900*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991550;
        parameter OpenIPSL.Types.Angle A22=-0.520500*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.983180;
        parameter OpenIPSL.Types.Angle A23=-1.049900*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.847000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.500000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.374000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.500000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.700000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_085;

    record PF_086
      //Power flow results for the snapshot h86.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.026400*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.987440;
        parameter OpenIPSL.Types.Angle A2=1.519900*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.982920;
        parameter OpenIPSL.Types.Angle A21=-1.062800*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991420;
        parameter OpenIPSL.Types.Angle A22=-0.526800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.982920;
        parameter OpenIPSL.Types.Angle A23=-1.062800*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.877000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.600000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.401000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.600000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.720000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_086;

    record PF_087
      //Power flow results for the snapshot h87.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.134200*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.987230;
        parameter OpenIPSL.Types.Angle A2=1.538200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.982670;
        parameter OpenIPSL.Types.Angle A21=-1.075600*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991290;
        parameter OpenIPSL.Types.Angle A22=-0.533100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.982670;
        parameter OpenIPSL.Types.Angle A23=-1.075600*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.907000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.700000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.429000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.700000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.740000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_087;

    record PF_088
      //Power flow results for the snapshot h88.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.242100*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.987010;
        parameter OpenIPSL.Types.Angle A2=1.556600*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.982410;
        parameter OpenIPSL.Types.Angle A21=-1.088500*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991160;
        parameter OpenIPSL.Types.Angle A22=-0.539400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.982410;
        parameter OpenIPSL.Types.Angle A23=-1.088500*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.937000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.800000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.457000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.800000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.760000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_088;

    record PF_089
      //Power flow results for the snapshot h89.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.350100*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.986800;
        parameter OpenIPSL.Types.Angle A2=1.574900*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.982150;
        parameter OpenIPSL.Types.Angle A21=-1.101400*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.991030;
        parameter OpenIPSL.Types.Angle A22=-0.545700*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.982150;
        parameter OpenIPSL.Types.Angle A23=-1.101400*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.967000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=8.900000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.485000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=8.900000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.780000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_089;

    record PF_090
      //Power flow results for the snapshot h90.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.458100*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.986580;
        parameter OpenIPSL.Types.Angle A2=1.593300*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.981890;
        parameter OpenIPSL.Types.Angle A21=-1.114300*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990900;
        parameter OpenIPSL.Types.Angle A22=-0.552100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.981890;
        parameter OpenIPSL.Types.Angle A23=-1.114300*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=1.997000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.000000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.513000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.000000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.800000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_090;

    record PF_091
      //Power flow results for the snapshot h91.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.566300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.986360;
        parameter OpenIPSL.Types.Angle A2=1.611800*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.981620;
        parameter OpenIPSL.Types.Angle A21=-1.127300*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990760;
        parameter OpenIPSL.Types.Angle A22=-0.558400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.981620;
        parameter OpenIPSL.Types.Angle A23=-1.127300*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.028000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.100000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.542000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.100000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.820000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_091;

    record PF_092
      //Power flow results for the snapshot h92.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.674600*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.986140;
        parameter OpenIPSL.Types.Angle A2=1.630200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.981360;
        parameter OpenIPSL.Types.Angle A21=-1.140200*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990630;
        parameter OpenIPSL.Types.Angle A22=-0.564700*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.981360;
        parameter OpenIPSL.Types.Angle A23=-1.140200*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.058000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.200000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.571000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.200000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.840000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_092;

    record PF_093
      //Power flow results for the snapshot h93.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.782900*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.985920;
        parameter OpenIPSL.Types.Angle A2=1.648700*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.981090;
        parameter OpenIPSL.Types.Angle A21=-1.153100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990500;
        parameter OpenIPSL.Types.Angle A22=-0.571100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.981090;
        parameter OpenIPSL.Types.Angle A23=-1.153100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.002000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.089000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.300000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.600000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.300000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.860000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_093;

    record PF_094
      //Power flow results for the snapshot h94.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.891300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.985690;
        parameter OpenIPSL.Types.Angle A2=1.667200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.980830;
        parameter OpenIPSL.Types.Angle A21=-1.166100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990360;
        parameter OpenIPSL.Types.Angle A22=-0.577400*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.980830;
        parameter OpenIPSL.Types.Angle A23=-1.166100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.003000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.120000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.400000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.630000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.400000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.880000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_094;

    record PF_095
      //Power flow results for the snapshot h95.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=9.999899*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.985460;
        parameter OpenIPSL.Types.Angle A2=1.685700*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.980560;
        parameter OpenIPSL.Types.Angle A21=-1.179100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990230;
        parameter OpenIPSL.Types.Angle A22=-0.583800*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.980560;
        parameter OpenIPSL.Types.Angle A23=-1.179100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.003000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.152000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.500000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.660000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.500000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.900000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_095;

    record PF_096
      //Power flow results for the snapshot h96.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=10.108500*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.985230;
        parameter OpenIPSL.Types.Angle A2=1.704200*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.980290;
        parameter OpenIPSL.Types.Angle A21=-1.192100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.990090;
        parameter OpenIPSL.Types.Angle A22=-0.590100*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.980290;
        parameter OpenIPSL.Types.Angle A23=-1.192100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.003000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.183000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.600000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.690000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.600000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.920000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_096;

    record PF_097
      //Power flow results for the snapshot h97.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=10.217300*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.985000;
        parameter OpenIPSL.Types.Angle A2=1.722800*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.980020;
        parameter OpenIPSL.Types.Angle A21=-1.205100*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.989950;
        parameter OpenIPSL.Types.Angle A22=-0.596500*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.980020;
        parameter OpenIPSL.Types.Angle A23=-1.205100*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.003000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.215000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.700000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.720000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.700000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.940000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_097;

    record PF_098
      //Power flow results for the snapshot h98.0_after_PF

      extends Modelica.Icons.Record;

      record Voltages
        // Bus number 1
        parameter OpenIPSL.Types.PerUnit V1=1.000000;
        parameter OpenIPSL.Types.Angle A1=10.326100*Modelica.Constants.pi/180;
        // Bus number 2
        parameter OpenIPSL.Types.PerUnit V2=0.984770;
        parameter OpenIPSL.Types.Angle A2=1.741400*Modelica.Constants.pi/180;
        // Bus number 3
        parameter OpenIPSL.Types.PerUnit V3=1.000000;
        parameter OpenIPSL.Types.Angle A3=0.000000*Modelica.Constants.pi/180;
        // Bus number 21
        parameter OpenIPSL.Types.PerUnit V21=0.979740;
        parameter OpenIPSL.Types.Angle A21=-1.218200*Modelica.Constants.pi/180;
        // Bus number 22
        parameter OpenIPSL.Types.PerUnit V22=0.989810;
        parameter OpenIPSL.Types.Angle A22=-0.602900*Modelica.Constants.pi/180;
        // Bus number 23
        parameter OpenIPSL.Types.PerUnit V23=0.979740;
        parameter OpenIPSL.Types.Angle A23=-1.218200*Modelica.Constants.pi/180;
      end Voltages;

      record Machines
        // Machine 3_1
        parameter OpenIPSL.Types.ActivePower P3_1=0.003000e6;
        parameter OpenIPSL.Types.ReactivePower Q3_1=2.247000e6;
        // Machine 1_1
        parameter OpenIPSL.Types.ActivePower P1_1=9.800000e6;
        parameter OpenIPSL.Types.ReactivePower Q1_1=1.751000e6;
      end Machines;

      record Loads
        // Load 23_1
        parameter OpenIPSL.Types.ActivePower PL23_1=9.800000e6;
        parameter OpenIPSL.Types.ReactivePower QL23_1=1.960000e6;
      end Loads;

      record Trafos
        // 2WindingTrafo 1_2
        parameter Real t1_1_2=1.000000;
        parameter Real t2_1_2=1.000000;
      end Trafos;
      Voltages voltages;
      Machines machines;
      Loads loads;
      Trafos trafos;
    end PF_098;

    record PF_Results "power flow results"

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end PF_Results;

    package SimpleSMIB "Records for the Simple SMIB power flow results"

      record Loads "P,Q power flow results of the loads"
        extends Modelica.Icons.Record;
        // Load 2_1
        parameter Real PL2_1=5.000000;
        parameter Real QL2_1=1.000000;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Loads;

      record Machines "P,Q power flow results of the machines"
        extends Modelica.Icons.Record;
        // Machine 3_1
        parameter Real P3_1=0.000000;
        parameter Real Q3_1=0.719000;
        // Machine 1_1
        parameter Real P1_1=5.000000;
        parameter Real Q1_1=0.668000;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Machines;

      record Voltages "P,Q power flow results of every bus"
        extends Modelica.Icons.Record;
        // Bus number 1
        parameter Real V1=1.000000;
        parameter Real A1=4.334500;
        // Bus number 2
        parameter Real V2=0.992810;
        parameter Real A2=0.002100;
        // Bus number 3
        parameter Real V3=1.000000;
        parameter Real A3=0.000000;
        // Bus number 22
        parameter Real V22=0.996410;
        parameter Real A22=0.001000;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Voltages;
    end SimpleSMIB;

    record Voltages "P,Q power flow results of every bus"
      extends Modelica.Icons.Record;
      // Bus number 1
      parameter OpenIPSL.Types.PerUnit V1=1.000000;
      parameter OpenIPSL.Types.Angle A1=0.0907344318234292;
      // Bus number 2
      parameter OpenIPSL.Types.PerUnit V2=0.994050;
      parameter OpenIPSL.Types.Angle A2=0.01521403508963457;
      // Bus number 3
      parameter OpenIPSL.Types.PerUnit V3=1.000000;
      parameter OpenIPSL.Types.Angle A3=0.000000;
      // Bus number 21
      parameter OpenIPSL.Types.PerUnit V21=0.991250;
      parameter OpenIPSL.Types.Angle A21=-0.010623819156889484;
      // Bus number 22
      parameter OpenIPSL.Types.PerUnit V22=0.995610;
      parameter OpenIPSL.Types.Angle A22=-0.005288347633542819;
      // Bus number 23
      parameter OpenIPSL.Types.PerUnit V23=0.991250;
      parameter OpenIPSL.Types.Angle A23=-0.010623819156889484;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Voltages;
  end Records;
end SMIB;
