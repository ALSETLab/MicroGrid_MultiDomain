within MicroGrid.Examples.SystemExamples;
package Campuses
  extends Modelica.Icons.ExamplesPackage;

  package CampusA
    extends Modelica.Icons.ExamplesPackage;
    model CampusA
      extends Modelica.Icons.Example;
      OpenIPSL.Electrical.Buses.Bus AENB(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,138})));
      OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,188})));
      OpenIPSL.Electrical.Buses.Bus H2E(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-110,78})));
      OpenIPSL.Electrical.Buses.Bus H4S(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,78})));
      OpenIPSL.Electrical.Buses.Bus H3N(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={50,78})));
      OpenIPSL.Electrical.Buses.Bus H1W(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={110,78})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer HT2E(VB1=69000, VB2=12000)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-110,110})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer HT4S(VNOM1=69000,
          VNOM2=12000)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,110})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer HT1W(VB1=69000, VB2=12000)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={110,110})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer HT3N(VB1=69000, VB2=12000)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={50,110})));
      OpenIPSL.Electrical.Branches.PwLine X1
        annotation (Placement(transformation(extent={{-90,58},{-70,78}})));
      OpenIPSL.Electrical.Branches.PwLine X2
        annotation (Placement(transformation(extent={{-10,58},{10,78}})));
      OpenIPSL.Electrical.Branches.PwLine X4
        annotation (Placement(transformation(extent={{70,58},{90,78}})));
      OpenIPSL.Electrical.Buses.Bus A1W(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,26})));
      OpenIPSL.Electrical.Events.Breaker HEW annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-110,88})));
      OpenIPSL.Electrical.Events.Breaker HSM annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-50,88})));
      OpenIPSL.Electrical.Events.Breaker HNM annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={50,88})));
      OpenIPSL.Electrical.Events.Breaker HWM annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={110,88})));
      OpenIPSL.Electrical.Events.Breaker HEST annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-98,68})));
      OpenIPSL.Electrical.Events.Breaker HSET annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-64,68})));
      OpenIPSL.Electrical.Events.Breaker HNST annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={22,68})));
      OpenIPSL.Electrical.Events.Breaker HSNT annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-22,68})));
      OpenIPSL.Electrical.Events.Breaker HNWT annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={62,68})));
      OpenIPSL.Electrical.Events.Breaker HWNT annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={98,68})));
      OpenIPSL.Electrical.Events.Breaker HE01 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-110,44})));
      OpenIPSL.Electrical.Buses.Bus A2E(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={50,26})));
      OpenIPSL.Electrical.Events.Breaker HW01 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={110,44})));
      OpenIPSL.Electrical.Events.Breaker AWET annotation (Placement(
            transformation(extent={{-4,14},{4,22}}, rotation=0)));
      Generation_Groups.Generator_AVR_PSS_TurbGov CTG8(M_b=36.18, V_b=12000) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,-14})));
      OpenIPSL.Electrical.Branches.PwLine X3
        annotation (Placement(transformation(extent={{-10,44},{10,64}})));
      OpenIPSL.Electrical.Events.Breaker HEW1 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-98,54})));
      OpenIPSL.Electrical.Events.Breaker HWET annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={98,54})));
      OpenIPSL.Electrical.Buses.Bus W1W(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-80,-32})));
      OpenIPSL.Electrical.Events.Breaker WWM annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-80,-24})));
      OpenIPSL.Electrical.Events.Breaker WEM annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={80,-24})));
      OpenIPSL.Electrical.Buses.Bus W2E(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={80,-32})));
      OpenIPSL.Electrical.Events.Breaker WWET annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-70,-42})));
      OpenIPSL.Electrical.Events.Breaker WEWT annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={72,-42})));
      OpenIPSL.Electrical.Branches.PwLine X5
        annotation (Placement(transformation(extent={{-10,-52},{10,-32}})));
      Generation_Groups.Generator_AVR_PSS_TurbGov
        CTG10(M_b=34, V_b=12000) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-80,-80})));
      Generation_Groups.Generator_AVR_PSS_TurbGov
        STG7(M_b=32, V_b=12000)
             annotation (Placement(transformation(
            extent={{-10,-11},{10,11}},
            rotation=90,
            origin={80,-81})));
      OpenIPSL.Electrical.Buses.Bus W3N(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-110,-102})));
      OpenIPSL.Electrical.Buses.Bus W4S(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={110,-102})));
      OpenIPSL.Electrical.Branches.PwLine X6 annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-120,-42})));
      OpenIPSL.Electrical.Events.Breaker WNWT annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-110,-92})));
      OpenIPSL.Electrical.Branches.PwLine X7 annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={120,-42})));
      OpenIPSL.Electrical.Events.Breaker WSET annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={110,-92})));
      OpenIPSL.Electrical.Banks.PSSE.Shunt BC01
        annotation (Placement(transformation(extent={{44,-24},{56,-12}})));
      OpenIPSL.Electrical.Events.Breaker AW10 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-50,8})));
      OpenIPSL.Electrical.Events.Breaker AE01 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={50,6})));
      OpenIPSL.Electrical.Events.Breaker WW04 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-80,-62})));
      OpenIPSL.Electrical.Events.Breaker WE04 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={80,-62})));
      OpenIPSL.Electrical.Events.Breaker WN03 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-110,-122})));
      OpenIPSL.Electrical.Banks.PSSE.Shunt BC02 annotation (Placement(
            transformation(extent={{-116,-152},{-104,-140}})));
      OpenIPSL.Electrical.Buses.Bus B416N(V_b=4.16e3) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,-150})));
      OpenIPSL.Electrical.Buses.Bus B416S(V_b=4.16e3) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={50,-150})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer BI1(
        VB1=12000,
        VB2=4160,
        S_n=7500000)                                              annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-20,-130})));
      OpenIPSL.Electrical.Events.Breaker SNM annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-50,-142})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer BI2(
        VB1=12000,
        VB2=4160,
        S_n=7500000)                                              annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={20,-122})));
      OpenIPSL.Electrical.Events.Breaker WS02 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={50,-142})));
      OpenIPSL.Electrical.Events.Breaker SNST annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-36,-160})));
      OpenIPSL.Electrical.Events.Breaker SN09 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-60,-180})));
      Generation_Groups.Generator_AVR_PSS_TurbGov
        STG5(M_b=7.5, V_b=4160) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,-198})));
      OpenIPSL.Electrical.Events.Breaker SN04 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-36,-180})));
      OpenIPSL.Electrical.Banks.PSSE.Shunt BC03 annotation (Placement(
            transformation(extent={{-42,-200},{-30,-188}})));
      OpenIPSL.Electrical.Events.Breaker SS03 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={50,-178})));
      Generation_Groups.Generator_AVR_PSS_TurbGov
        STG4(M_b=7.8125, V_b=4160) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={50,-196})));
      OpenIPSL.Electrical.Events.Breaker WS01 annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={110,-124})));
      Generation_Groups.Generator_AVR_PSS_TurbGov
        STG9(M_b=32, V_b=12000)   annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={110,-142})));
      OpenIPSL.Electrical.Buses.Bus AENA(V_b=12e3) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,168})));
      OpenIPSL.Electrical.Branches.PwLine L1 annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,152})));
      OpenIPSL.Electrical.Events.Breaker WSNT annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=0,
            origin={-96,-110})));
      inner OpenIPSL.Electrical.SystemBase SysData(fn=60)
        annotation (Placement(transformation(extent={{74,160},{112,188}})));
    equation
      connect(HT2E.p, AENB.p) annotation (Line(points={{-110,121},{-110,128},{0,
              128},{0,138}},    color={0,0,255}));
      connect(HT4S.p, AENB.p) annotation (Line(points={{-50,121},{-50,128},{0,
              128},{0,138}}, color={0,0,255}));
      connect(HT1W.p, AENB.p) annotation (Line(points={{110,121},{110,128},{0,
              128},{0,138}}, color={0,0,255}));
      connect(HT3N.p, AENB.p) annotation (Line(points={{50,121},{50,128},{0,128},
              {0,138}},      color={0,0,255}));
      connect(HEW.s, HT2E.n)
        annotation (Line(points={{-110,92},{-110,99}},  color={0,0,255}));
      connect(HEW.r, H2E.p)
        annotation (Line(points={{-110,84},{-110,78}}, color={0,0,255}));
      connect(HT4S.n, HSM.s)
        annotation (Line(points={{-50,99},{-50,92}},  color={0,0,255}));
      connect(HSM.r, H4S.p)
        annotation (Line(points={{-50,84},{-50,78}}, color={0,0,255}));
      connect(HT3N.n, HNM.s)
        annotation (Line(points={{50,99},{50,92}},  color={0,0,255}));
      connect(H3N.p, HNM.r)
        annotation (Line(points={{50,78},{50,84}}, color={0,0,255}));
      connect(HT1W.n, HWM.s)
        annotation (Line(points={{110,99},{110,92}},  color={0,0,255}));
      connect(H1W.p, HWM.r)
        annotation (Line(points={{110,78},{110,84}}, color={0,0,255}));
      connect(HEST.r, X1.p)
        annotation (Line(points={{-94,68},{-89,68}}, color={0,0,255}));
      connect(HEST.s, H2E.p) annotation (Line(points={{-102,68},{-110,68},{
              -110,78}}, color={0,0,255}));
      connect(X1.n, HSET.s)
        annotation (Line(points={{-71,68},{-68,68}}, color={0,0,255}));
      connect(HSET.r, H4S.p) annotation (Line(points={{-60,68},{-50,68},{-50,
              78}}, color={0,0,255}));
      connect(HSNT.r, X2.p)
        annotation (Line(points={{-18,68},{-9,68}}, color={0,0,255}));
      connect(HSNT.s, H4S.p) annotation (Line(points={{-26,68},{-50,68},{-50,
              78}}, color={0,0,255}));
      connect(X2.n, HNST.s)
        annotation (Line(points={{9,68},{18,68}}, color={0,0,255}));
      connect(HNST.r, H3N.p)
        annotation (Line(points={{26,68},{50,68},{50,78}}, color={0,0,255}));
      connect(X4.p, HNWT.r)
        annotation (Line(points={{71,68},{66,68}}, color={0,0,255}));
      connect(HNWT.s, H3N.p)
        annotation (Line(points={{58,68},{50,68},{50,78}}, color={0,0,255}));
      connect(X4.n, HWNT.s)
        annotation (Line(points={{89,68},{94,68}}, color={0,0,255}));
      connect(HWNT.r, H1W.p) annotation (Line(points={{102,68},{110,68},{110,
              78}}, color={0,0,255}));
      connect(HE01.s, H2E.p)
        annotation (Line(points={{-110,48},{-110,78}}, color={0,0,255}));
      connect(HE01.r, A1W.p) annotation (Line(points={{-110,40},{-110,34},{
              -50,34},{-50,26}}, color={0,0,255}));
      connect(HW01.s, H1W.p)
        annotation (Line(points={{110,48},{110,78}}, color={0,0,255}));
      connect(HW01.r, A2E.p) annotation (Line(points={{110,40},{110,34},{50,
              34},{50,26}}, color={0,0,255}));
      connect(AWET.s, A1W.p) annotation (Line(points={{-4,18},{-50,18},{-50,
              26}}, color={0,0,255}));
      connect(AWET.r, A2E.p)
        annotation (Line(points={{4,18},{50,18},{50,26}}, color={0,0,255}));
      connect(HEW1.r, X3.p)
        annotation (Line(points={{-94,54},{-9,54}}, color={0,0,255}));
      connect(HEW1.s, H2E.p) annotation (Line(points={{-102,54},{-110,54},{
              -110,78}}, color={0,0,255}));
      connect(HWET.s, X3.n)
        annotation (Line(points={{94,54},{9,54}}, color={0,0,255}));
      connect(HWET.r, H1W.p) annotation (Line(points={{102,54},{110,54},{110,
              78}}, color={0,0,255}));
      connect(WWM.r, W1W.p)
        annotation (Line(points={{-80,-28},{-80,-32}}, color={0,0,255}));
      connect(WWM.s, H4S.p) annotation (Line(points={{-80,-20},{-80,46},{-50,
              46},{-50,78}}, color={0,0,255}));
      connect(WEM.s, H3N.p) annotation (Line(points={{80,-20},{80,46},{50,46},
              {50,78}}, color={0,0,255}));
      connect(WWET.s, W1W.p) annotation (Line(points={{-74,-42},{-80,-42},{
              -80,-32}}, color={0,0,255}));
      connect(WEWT.r, W2E.p) annotation (Line(points={{76,-42},{80,-42},{80,
              -32}}, color={0,0,255}));
      connect(X5.p, WWET.r)
        annotation (Line(points={{-9,-42},{-66,-42}}, color={0,0,255}));
      connect(X5.n, WEWT.s)
        annotation (Line(points={{9,-42},{68,-42}}, color={0,0,255}));
      connect(WNWT.r, W3N.p)
        annotation (Line(points={{-110,-96},{-110,-102}}, color={0,0,255}));
      connect(X6.n, A1W.p) annotation (Line(points={{-120,-33},{-120,18},{-50,
              18},{-50,26}}, color={0,0,255}));
      connect(X6.p, WNWT.s) annotation (Line(points={{-120,-51},{-120,-72},{
              -110,-72},{-110,-88}}, color={0,0,255}));
      connect(W4S.p, WSET.r)
        annotation (Line(points={{110,-102},{110,-96}}, color={0,0,255}));
      connect(WSET.s, X7.p) annotation (Line(points={{110,-88},{110,-72},{120,
              -72},{120,-51}}, color={0,0,255}));
      connect(X7.n, A2E.p) annotation (Line(points={{120,-33},{120,18},{50,18},
              {50,26}}, color={0,0,255}));
      connect(AW10.r, CTG8.pwPin)
        annotation (Line(points={{-50,4},{-50,-3}}, color={0,0,255}));
      connect(AW10.s, A1W.p)
        annotation (Line(points={{-50,12},{-50,26}}, color={0,0,255}));
      connect(AE01.r, BC01.p)
        annotation (Line(points={{50,2},{50,-12}}, color={0,0,255}));
      connect(AE01.s, A2E.p)
        annotation (Line(points={{50,10},{50,26}}, color={0,0,255}));
      connect(CTG10.pwPin, WW04.r)
        annotation (Line(points={{-80,-69},{-80,-66}}, color={0,0,255}));
      connect(WW04.s, W1W.p)
        annotation (Line(points={{-80,-58},{-80,-32}}, color={0,0,255}));
      connect(STG7.pwPin, WE04.r)
        annotation (Line(points={{80,-70},{80,-66}}, color={0,0,255}));
      connect(WE04.s, W2E.p)
        annotation (Line(points={{80,-58},{80,-32}}, color={0,0,255}));
      connect(WN03.s, W3N.p)
        annotation (Line(points={{-110,-118},{-110,-102}}, color={0,0,255}));
      connect(WN03.r, BC02.p)
        annotation (Line(points={{-110,-126},{-110,-140}}, color={0,0,255}));
      connect(B416N.p, SNM.r)
        annotation (Line(points={{-50,-150},{-50,-146}}, color={0,0,255}));
      connect(SNM.s, BI1.n) annotation (Line(points={{-50,-138},{-50,-130},{
              -31,-130}}, color={0,0,255}));
      connect(BI1.p, W2E.p) annotation (Line(points={{-9,-130},{60,-130},{60,
              -52},{80,-52},{80,-32}}, color={0,0,255}));
      connect(BI2.p, W1W.p) annotation (Line(points={{9,-122},{-60,-122},{-60,
              -52},{-80,-52},{-80,-32}}, color={0,0,255}));
      connect(B416S.p, WS02.r)
        annotation (Line(points={{50,-150},{50,-146}}, color={0,0,255}));
      connect(WS02.s, BI2.n) annotation (Line(points={{50,-138},{50,-122},{31,
              -122}}, color={0,0,255}));
      connect(SNST.s, B416N.p) annotation (Line(points={{-40,-160},{-50,-160},
              {-50,-150}}, color={0,0,255}));
      connect(SNST.r, B416S.p) annotation (Line(points={{-32,-160},{50,-160},
              {50,-150}}, color={0,0,255}));
      connect(SN09.s, B416N.p) annotation (Line(points={{-60,-176},{-60,-170},
              {-50,-170},{-50,-150}}, color={0,0,255}));
      connect(SN09.r, STG5.pwPin)
        annotation (Line(points={{-60,-184},{-60,-187}}, color={0,0,255}));
      connect(SN04.r, BC03.p)
        annotation (Line(points={{-36,-184},{-36,-188}}, color={0,0,255}));
      connect(SN04.s, B416N.p) annotation (Line(points={{-36,-176},{-36,-170},
              {-50,-170},{-50,-150}}, color={0,0,255}));
      connect(SS03.r, STG4.pwPin)
        annotation (Line(points={{50,-182},{50,-185}}, color={0,0,255}));
      connect(SS03.s, B416S.p)
        annotation (Line(points={{50,-174},{50,-150}}, color={0,0,255}));
      connect(WS01.r, STG9.pwPin)
        annotation (Line(points={{110,-128},{110,-131}}, color={0,0,255}));
      connect(WS01.s, W4S.p)
        annotation (Line(points={{110,-120},{110,-102}}, color={0,0,255}));
      connect(AENA.p, L1.n)
        annotation (Line(points={{0,168},{0,161}}, color={0,0,255}));
      connect(L1.p, AENB.p)
        annotation (Line(points={{0,143},{0,138}}, color={0,0,255}));
      connect(AENA.p, gENCLS.p)
        annotation (Line(points={{0,168},{0,178}}, color={0,0,255}));
      connect(WSNT.s, W3N.p) annotation (Line(points={{-100,-110},{-110,-110},
              {-110,-102}}, color={0,0,255}));
      connect(WSNT.r, W4S.p) annotation (Line(points={{-92,-110},{110,-110},{
              110,-102}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -140,-220},{140,200}})), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-140,-220},{140,200}}), graphics={
            Rectangle(
              extent={{-126,124},{126,82}},
              lineColor={0,140,72},
              pattern=LinePattern.Dash,
              lineThickness=0.5),
              Text(
              extent={{108,136},{146,124}},
              textColor={0,140,72},
              textStyle={TextStyle.Bold},
              textString="Substation")}));
    end CampusA;

    package Generation_Groups

      model Generator "Generator without controls"
        extends OpenIPSL.Electrical.Essentials.pfComponent;
        OpenIPSL.Interfaces.PwPin pwPin annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
        OpenIPSL.Electrical.Machines.PSSE.GENROU gENROU(
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0,
          M_b=M_b,
          V_b=V_b,
          Tpd0=machinePars.Tpd0,
          Tppd0=machinePars.Tppd0,
          Tppq0=machinePars.Tppq0,
          H=machinePars.H,
          D=machinePars.D,
          Xd=machinePars.Xd,
          Xq=machinePars.Xq,
          Xpd=machinePars.Xpd,
          Xppd=machinePars.Xppd,
          Xppq=machinePars.Xppq,
          Xl=machinePars.Xl,
          S10=machinePars.S10,
          S12=machinePars.S12,
          Xpq=machinePars.Xpq,
          Tpq0=machinePars.Tpq0,
          Xpp=machinePars.Xpp,
          R_a=machinePars.R_a) annotation (Placement(transformation(extent={{0,-30},{60,30}})));
        parameter Real M_b "Machine base power (MVA)";
        Modelica.Blocks.Interfaces.RealOutput Pm0 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={60,-106}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={46,-98})));
        Modelica.Blocks.Interfaces.RealInput Pmech annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
        Modelica.Blocks.Interfaces.RealOutput speed annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={106,20}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-44,-98})));
        Modelica.Blocks.Interfaces.RealOutput PELEC
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={94,-38}), iconTransformation(extent={{-82,-62},{-102,-42}})));
        Records.MachinePars1 machinePars
          annotation (Placement(transformation(extent={{-90,66},{-70,86}})));
      equation
        connect(gENROU.p, pwPin) annotation (Line(points={{60,0},{60,0},{110,0}}, color={0,0,255}));
        connect(gENROU.EFD0, gENROU.EFD) annotation (Line(points={{63,-15},{72,-15},{72,-46},{-20,-46},{-20,-15},{-6,-15}},color={0,0,127}));
        connect(Pm0, Pm0) annotation (Line(points={{60,-106},{60,-101},{60,-106}}, color={0,0,127}));
        connect(Pm0, gENROU.PMECH0) annotation (Line(points={{60,-106},{60,-74},{86,-74},{86,15},{63,15}}, color={0,0,127}));
        connect(Pmech, gENROU.PMECH) annotation (Line(points={{-106,0},{-60,0},{-60,15},{-6,15}},color={0,0,127}));
        connect(gENROU.SPEED, speed) annotation (Line(points={{63,21},{83.5,21},{83.5,20},{106,20}}, color={0,0,127}));
        connect(gENROU.PELEC, PELEC) annotation (Line(points={{63,9},{94,9},{94,-38}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-48,2},{-20,56},{2,4},{24,-28},{48,22}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-52,-18},{56,-66}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
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
",       info="<html>
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
      end Generator;

      model Generator_AVR "Generator + AVR"
        extends OpenIPSL.Electrical.Essentials.pfComponent;
        OpenIPSL.Interfaces.PwPin pwPin annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-62,-56},{-42,-36}})));
        OpenIPSL.Electrical.Machines.PSSE.GENROU gENROU(
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0,
          M_b=M_b,
          Tpd0=machinePars.Tpd0,
          Tppd0=machinePars.Tppd0,
          Tppq0=machinePars.Tppq0,
          H=machinePars.H,
          D=machinePars.D,
          Xd=machinePars.Xd,
          Xq=machinePars.Xq,
          Xpd=machinePars.Xpd,
          Xppd=machinePars.Xppd,
          Xppq=machinePars.Xppq,
          Xl=machinePars.Xl,
          S10=machinePars.S10,
          S12=machinePars.S12,
          Xpq=machinePars.Xpq,
          Tpq0=machinePars.Tpq0,
          Xpp=machinePars.Xpp,
          R_a=machinePars.R_a,
          V_b=V_b) annotation (Placement(transformation(extent={{0,-30},{60,30}})));
        parameter Real M_b "Machine base power (MVA)";
        Modelica.Blocks.Interfaces.RealOutput Pm0 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={70,-106}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={46,-98})));
        OpenIPSL.Electrical.Controls.PSSE.ES.IEEET1 avr(
          T_R=aVRPars.T_R,
          K_A=aVRPars.K_A,
          T_A=aVRPars.T_A,
          V_RMAX=aVRPars.V_RMAX,
          V_RMIN=aVRPars.V_RMIN,
          K_E=aVRPars.K_E,
          T_E=aVRPars.T_E,
          K_F=aVRPars.K_F,
          T_F=aVRPars.T_F,
          E_1=aVRPars.E_1,
          S_EE_1=aVRPars.S_EE_1,
          E_2=aVRPars.E_2,
          S_EE_2=aVRPars.S_EE_2) annotation (Placement(transformation(extent={{16,-82},{-16,-54}})));
        Modelica.Blocks.Interfaces.RealInput Pmech annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
        Modelica.Blocks.Interfaces.RealOutput speed annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={106,20}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-44,-98})));
        Records.MachinePars1 machinePars
          annotation (Placement(transformation(extent={{-90,66},{-70,86}})));
        Records.AVRPars aVRPars
          annotation (Placement(transformation(extent={{-68,66},{-48,86}})));
      equation
        connect(gENROU.p, pwPin) annotation (Line(points={{60,0},{110,0}}, color={0,0,255}));
        connect(Pm0, gENROU.PMECH0) annotation (Line(points={{70,-106},{70,-92},{92,-92},{92,15},{63,15}}, color={0,0,127}));
        connect(avr.EFD, gENROU.EFD) annotation (Line(points={{-17.6,-68},{
                -26,-68},{-26,-15},{-6,-15}},                                                                      color={0,0,127}));
        connect(const.y, avr.VOTHSG) annotation (Line(points={{-41,-46},{24,
                -46},{24,-62.4},{17.6,-62.4}},                                                                 color={0,0,127}));
        connect(gENROU.ETERM, avr.ECOMP) annotation (Line(points={{63,-9},{84,
                -9},{84,-68},{17.6,-68}},                                                                       color={0,0,127}));
        connect(gENROU.EFD0, avr.EFD0) annotation (Line(points={{63,-15},{78,
                -15},{78,-73.6},{17.6,-73.6}},                                                                  color={0,0,127}));
        connect(avr.VOEL, avr.VOTHSG) annotation (Line(points={{0,-83.4},{4,
                -83.4},{4,-92},{-34,-92},{-34,-46},{24,-46},{24,-62.4},{17.6,
                -62.4}},                                                                                                                            color={0,0,127}));
        connect(avr.VUEL, avr.VOTHSG) annotation (Line(points={{6.4,-83.4},{
                10,-83.4},{10,-92},{-34,-92},{-34,-46},{24,-46},{24,-62.4},{
                17.6,-62.4}},                                                                                                                          color={0,0,127}));
        connect(Pmech, gENROU.PMECH) annotation (Line(points={{-106,0},{-60,0},{-60,15},{-6,15}}, color={0,0,127}));
        connect(gENROU.SPEED, speed) annotation (Line(points={{63,21},{80.5,21},{80.5,20},{106,20}}, color={0,0,127}));
        connect(avr.VUEL, avr.VOEL) annotation (Line(points={{6.4,-83.4},{10,
                -83.4},{10,-92},{0,-92},{0,-83.4}}, color={0,0,127}));
        connect(gENROU.XADIFD, avr.XADIFD) annotation (Line(points={{63,-27},
                {72,-27},{72,-84},{36,-84},{36,-98},{-12.8,-98},{-12.8,-83.4}},
              color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-48,2},{-20,56},{2,4},{24,-28},{48,22}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-52,-18},{56,-66}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
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
",       info="<html>
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
      end Generator_AVR;

      model Generator_AVR_PSS "Generator + AVR + PSS"
        extends OpenIPSL.Electrical.Essentials.pfComponent;
        OpenIPSL.Interfaces.PwPin pwPin annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
        OpenIPSL.Electrical.Machines.PSSE.GENROU gENROU(
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0,
          M_b=M_b,
          Tpd0=machinePars.Tpd0,
          Tppd0=machinePars.Tppd0,
          Tppq0=machinePars.Tppq0,
          H=machinePars.H,
          D=machinePars.D,
          Xd=machinePars.Xd,
          Xq=machinePars.Xq,
          Xpd=machinePars.Xpd,
          Xppd=machinePars.Xppd,
          Xppq=machinePars.Xppq,
          Xl=machinePars.Xl,
          S10=machinePars.S10,
          S12=machinePars.S12,
          Xpq=machinePars.Xpq,
          Tpq0=machinePars.Tpq0,
          Xpp=machinePars.Xpp,
          R_a=machinePars.R_a,
          V_b=V_b) annotation (Placement(transformation(extent={{0,-30},{60,30}})));
        parameter Real M_b "Machine base power (MVA)";
        Modelica.Blocks.Interfaces.RealOutput Pm0 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,-106}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={46,-98})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-62,-98},{-42,-78}})));
        OpenIPSL.Electrical.Controls.PSSE.ES.IEEET1 avr(
          T_R=aVRPars.T_R,
          K_A=aVRPars.K_A,
          T_A=aVRPars.T_A,
          V_RMAX=aVRPars.V_RMAX,
          V_RMIN=aVRPars.V_RMIN,
          K_E=aVRPars.K_E,
          T_E=aVRPars.T_E,
          K_F=aVRPars.K_F,
          T_F=aVRPars.T_F,
          E_1=aVRPars.E_1,
          S_EE_1=aVRPars.S_EE_1,
          E_2=aVRPars.E_2,
          S_EE_2=aVRPars.S_EE_2) annotation (Placement(transformation(extent={{8,-76},{-24,-48}})));
        OpenIPSL.Electrical.Controls.PSSE.PSS.PSS2A pss(
          T_w1=pSSPars.T_w1,
          T_w2=pSSPars.T_w2,
          T_6=pSSPars.T_6,
          T_w3=pSSPars.T_w3,
          T_w4=pSSPars.T_w4,
          T_7=pSSPars.T_7,
          K_S2=pSSPars.K_S2,
          K_S3=pSSPars.K_S3,
          T_8=pSSPars.T_8,
          T_9=pSSPars.T_9,
          K_S1=pSSPars.K_S1,
          T_1=pSSPars.T_1,
          T_2=pSSPars.T_2,
          T_3=pSSPars.T_3,
          T_4=pSSPars.T_4,
          V_STMAX=pSSPars.V_STMAX,
          V_STMIN=pSSPars.V_STMIN,
          M=pSSPars.M,
          N=pSSPars.N) annotation (Placement(transformation(extent={{60,-48},{22,-40}})));
        Modelica.Blocks.Interfaces.RealInput Pmech annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
        Modelica.Blocks.Interfaces.RealOutput speed annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={106,20}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-44,-98})));
        Records.MachinePars1 machinePars
          annotation (Placement(transformation(extent={{-90,66},{-70,86}})));
        Records.AVRPars aVRPars
          annotation (Placement(transformation(extent={{-68,66},{-48,86}})));
        Records.PSSPars pSSPars
          annotation (Placement(transformation(extent={{-46,66},{-26,86}})));
      equation
        connect(gENROU.p, pwPin) annotation (Line(points={{60,0},{60,0},{110,0}}, color={0,0,255}));
        connect(Pm0, gENROU.PMECH0) annotation (Line(points={{80,-106},{80,-88},{92,-88},{92,15},{63,15}}, color={0,0,127}));
        connect(avr.EFD, gENROU.EFD) annotation (Line(points={{-25.6,-62},{
                -34,-62},{-40,-62},{-40,-60},{-40,-32},{-40,-15},{-6,-15}},                                                                           color={0,0,127}));
        connect(gENROU.EFD0, avr.EFD0) annotation (Line(points={{63,-15},{76,
                -15},{76,-67.6},{9.6,-67.6}},                                                                  color={0,0,127}));
        connect(gENROU.ETERM, avr.ECOMP) annotation (Line(points={{63,-9},{82,
                -9},{82,-62},{9.6,-62}},                                                                       color={0,0,127}));
        connect(const.y, avr.VOEL) annotation (Line(points={{-41,-88},{-41,
                -88},{-8,-88},{-8,-77.4}},                                                              color={0,0,127}));
        connect(avr.VUEL, avr.VOEL) annotation (Line(points={{-1.6,-77.4},{2,
                -77.4},{2,-88},{-8,-88},{-8,-77.4}},                                                           color={0,0,127}));
        connect(pss.VOTHSG, avr.VOTHSG) annotation (Line(points={{20.1,-44},{
                14,-44},{14,-56.4},{9.6,-56.4}},                                                                  color={0,0,127}));
        connect(gENROU.PELEC, pss.V_S2) annotation (Line(points={{63,9},{96,9},
                {96,-45.6},{61.9,-45.6}},                                                          color={0,0,127}));
        connect(Pmech, gENROU.PMECH) annotation (Line(points={{-106,0},{-60,0},{-60,15},{-6,15}}, color={0,0,127}));
        connect(speed, gENROU.SPEED) annotation (Line(points={{106,20},{88,20},{88,21},{63,21}}, color={0,0,127}));
        connect(pss.V_S1, gENROU.SPEED) annotation (Line(points={{61.9,-42.4},
                {88,-42.4},{88,21},{63,21}},                                                         color={0,0,127}));
        connect(gENROU.XADIFD, avr.XADIFD) annotation (Line(points={{63,-27},
                {70,-27},{70,-94},{-20.8,-94},{-20.8,-77.4}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-48,2},{-20,56},{2,4},{24,-28},{48,22}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-52,-18},{56,-66}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
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
",       info="<html>
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
      end Generator_AVR_PSS;

      model Generator_TurbGov "Generator + Turbine + Governor"
        extends OpenIPSL.Electrical.Essentials.pfComponent;
        OpenIPSL.Interfaces.PwPin pwPin annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
        OpenIPSL.Electrical.Machines.PSSE.GENROU gENROU(
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0,
          M_b=M_b,
          V_b=V_b,
          Tpd0=machinePars.Tpd0,
          Tppd0=machinePars.Tppd0,
          Tppq0=machinePars.Tppq0,
          H=machinePars.H,
          D=machinePars.D,
          Xd=machinePars.Xd,
          Xq=machinePars.Xq,
          Xpd=machinePars.Xpd,
          Xppd=machinePars.Xppd,
          Xppq=machinePars.Xppq,
          Xl=machinePars.Xl,
          S10=machinePars.S10,
          S12=machinePars.S12,
          Xpq=machinePars.Xpq,
          Tpq0=machinePars.Tpq0,
          Xpp=machinePars.Xpp,
          R_a=machinePars.R_a) annotation (Placement(transformation(extent={{0,-30},{60,30}})));
        parameter Real M_b "Machine base power (MVA)";
        Modelica.Blocks.Interfaces.RealOutput Pm0 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={60,-106}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={46,-98})));
        Modelica.Blocks.Interfaces.RealOutput speed annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={106,20}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-44,-98})));
        Modelica.Blocks.Interfaces.RealOutput PELEC
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={94,-88}), iconTransformation(extent={{-82,-62},{-102,-42}})));
        Electrical.Controls.TG.GGOV1.Simplified.GGOV2                  govturb(
          R=0.04,
          T_pelec=1,
          maxerr=0.05,
          minerr=-0.05,
          Kpgov=10,
          Kigov=5,
          Kdgov=0,
          Tdgov=1,
          Dm=0,
          Kimw=0,
          db=0,
          Vmax=1,
          Vmin=0.1,
          Tact=4,
          Teng=0,
          Tfload=3,
          Tsa=4,
          Tsb=5,
          DELT=0.005,
          Trate=10,
          Rup=99,
          Rdown=-99,
          Ropen=0.1,
          Rclose=-0.1,
          Flag=0,
          Pref=P_0/M_b,
          Kturb=1.5,
          Wfnl=0.15,
          Tb=0.14101,
          Tc=0.11514)
          annotation (Placement(transformation(extent={{-80,40},{-40,80}})));
        Records.MachinePars1 machinePars
          annotation (Placement(transformation(extent={{40,60},{60,80}})));
      equation
        connect(gENROU.p, pwPin) annotation (Line(points={{60,0},{60,0},{110,0}}, color={0,0,255}));
        connect(gENROU.EFD0, gENROU.EFD) annotation (Line(points={{63,-15},{72,-15},{72,-46},{-20,-46},{-20,-15},{-6,-15}},color={0,0,127}));
        connect(Pm0, Pm0) annotation (Line(points={{60,-106},{60,-101},{60,-106}}, color={0,0,127}));
        connect(Pm0, gENROU.PMECH0) annotation (Line(points={{60,-106},{60,-74},{86,-74},{86,15},{63,15}}, color={0,0,127}));
        connect(gENROU.SPEED, speed) annotation (Line(points={{63,21},{83.5,21},{83.5,20},{106,20}}, color={0,0,127}));
        connect(gENROU.PELEC, PELEC) annotation (Line(points={{63,9},{94,9},{94,-88}}, color={0,0,127}));
        connect(govturb.PELEC, PELEC) annotation (Line(points={{-81.125,49.8571},
                {-90,49.8571},{-90,-52},{94,-52},{94,-88}},                                                                  color={0,0,127}));
        connect(govturb.SPEED, speed) annotation (Line(points={{-81.125,73},{-90,73},{-90,88},{80,88},{80,22},{83.5,21},{83.5,20},{106,20}}, color={0,0,127}));
        connect(govturb.PMECH, gENROU.PMECH) annotation (Line(points={{-38.5,50},{-20,50},{-20,15},{-6,15}},color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-48,2},{-20,56},{2,4},{24,-28},{48,22}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-52,-18},{56,-66}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
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
",       info="<html>
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
      end Generator_TurbGov;

      model Generator_AVR_PSS_TurbGov
        "Generator + AVR + PSS + Turbine + Governor"
        extends OpenIPSL.Electrical.Essentials.pfComponent;
        OpenIPSL.Interfaces.PwPin pwPin annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
        OpenIPSL.Electrical.Machines.PSSE.GENROU gENROU(
          v_0=v_0,
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0,
          M_b=M_b,
          Tpd0=machinePars.Tpd0,
          Tppd0=machinePars.Tppd0,
          Tppq0=machinePars.Tppq0,
          H=machinePars.H,
          D=machinePars.D,
          Xd=machinePars.Xd,
          Xq=machinePars.Xq,
          Xpd=machinePars.Xpd,
          Xppd=machinePars.Xppd,
          Xppq=machinePars.Xppq,
          Xl=machinePars.Xl,
          S10=machinePars.S10,
          S12=machinePars.S12,
          Xpq=machinePars.Xpq,
          Tpq0=machinePars.Tpq0,
          Xpp=machinePars.Xpp,
          R_a=machinePars.R_a,
          V_b=V_b) annotation (Placement(transformation(extent={{0,-26},{60,34}})));
        parameter Real M_b "Machine base power (MVA)";
        Modelica.Blocks.Interfaces.RealOutput Pm0 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,-106}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={46,-98})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-62,-98},{-42,-78}})));
        OpenIPSL.Electrical.Controls.PSSE.ES.IEEET1 avr(
          T_R=aVRPars.T_R,
          K_A=aVRPars.K_A,
          T_A=aVRPars.T_A,
          V_RMAX=aVRPars.V_RMAX,
          V_RMIN=aVRPars.V_RMIN,
          K_E=aVRPars.K_E,
          T_E=aVRPars.T_E,
          K_F=aVRPars.K_F,
          T_F=aVRPars.T_F,
          E_1=aVRPars.E_1,
          S_EE_1=aVRPars.S_EE_1,
          E_2=aVRPars.E_2,
          S_EE_2=aVRPars.S_EE_2) annotation (Placement(transformation(extent={{8,-76},{-24,-48}})));
        OpenIPSL.Electrical.Controls.PSSE.PSS.PSS2A pss(
          T_w1=pSSPars.T_w1,
          T_w2=pSSPars.T_w2,
          T_6=pSSPars.T_6,
          T_w3=pSSPars.T_w3,
          T_w4=pSSPars.T_w4,
          T_7=pSSPars.T_7,
          K_S2=pSSPars.K_S2,
          K_S3=pSSPars.K_S3,
          T_8=pSSPars.T_8,
          T_9=pSSPars.T_9,
          K_S1=pSSPars.K_S1,
          T_1=pSSPars.T_1,
          T_2=pSSPars.T_2,
          T_3=pSSPars.T_3,
          T_4=pSSPars.T_4,
          V_STMAX=pSSPars.V_STMAX,
          V_STMIN=pSSPars.V_STMIN,
          M=pSSPars.M,
          N=pSSPars.N) annotation (Placement(transformation(extent={{60,-48},{22,-40}})));
        Modelica.Blocks.Interfaces.RealOutput speed annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={106,32}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-44,-98})));
        Electrical.Controls.TG.GGOV1.Simplified.GGOV2                  govturb(
          R=0.04,
          T_pelec=1,
          maxerr=0.05,
          minerr=-0.05,
          Kpgov=10,
          Kigov=5,
          Kdgov=0,
          Tdgov=1,
          Dm=0,
          Kimw=0,
          db=0,
          Vmax=1,
          Vmin=0.1,
          Tact=4,
          Teng=0,
          Tfload=3,
          Tsa=4,
          Tsb=5,
          DELT=0.005,
          Trate=10,
          Rup=99,
          Rdown=-99,
          Ropen=0.1,
          Rclose=-0.1,
          Flag=0,
          Pref=P_0/M_b,
          Kturb=1.5,
          Wfnl=0.15,
          Tb=0.14101,
          Tc=0.11514)
          annotation (Placement(transformation(extent={{-80,40},{-40,80}})));
        Records.MachinePars1 machinePars
          annotation (Placement(transformation(extent={{0,60},{20,80}})));
        Records.AVRPars aVRPars
          annotation (Placement(transformation(extent={{22,60},{42,80}})));
        Records.PSSPars pSSPars
          annotation (Placement(transformation(extent={{44,60},{64,80}})));
      equation
        connect(gENROU.p, pwPin) annotation (Line(points={{60,4},{110,4},{110,0}},color={0,0,255}));
        connect(Pm0, gENROU.PMECH0) annotation (Line(points={{80,-106},{80,-88},{92,-88},{92,19},{63,19}}, color={0,0,127}));
        connect(avr.EFD, gENROU.EFD) annotation (Line(points={{-25.6,-62},{
                -34,-62},{-40,-62},{-40,-60},{-40,-32},{-40,-11},{-6,-11}},                                                                           color={0,0,127}));
        connect(gENROU.EFD0, avr.EFD0) annotation (Line(points={{63,-11},{76,
                -11},{76,-67.6},{9.6,-67.6}},                                                                  color={0,0,127}));
        connect(gENROU.ETERM, avr.ECOMP) annotation (Line(points={{63,-5},{82,
                -5},{82,-62},{9.6,-62}},                                                                       color={0,0,127}));
        connect(const.y, avr.VOEL) annotation (Line(points={{-41,-88},{-41,
                -88},{-8,-88},{-8,-77.4}},                                                              color={0,0,127}));
        connect(avr.VUEL, avr.VOEL) annotation (Line(points={{-1.6,-77.4},{2,
                -77.4},{2,-88},{-8,-88},{-8,-77.4}},                                                           color={0,0,127}));
        connect(pss.VOTHSG, avr.VOTHSG) annotation (Line(points={{20.1,-44},{
                14,-44},{14,-56.4},{9.6,-56.4}},                                                                  color={0,0,127}));
        connect(gENROU.PELEC, pss.V_S2) annotation (Line(points={{63,13},{96,
                13},{96,-45.6},{61.9,-45.6}},                                                        color={0,0,127}));
        connect(speed, gENROU.SPEED) annotation (Line(points={{106,32},{88,32},{88,25},{63,25}}, color={0,0,127}));
        connect(pss.V_S1, gENROU.SPEED) annotation (Line(points={{61.9,-42.4},
                {88,-42.4},{88,25},{63,25}},                                                         color={0,0,127}));
        connect(govturb.PMECH, gENROU.PMECH) annotation (Line(points={{-38.5,50},{-20,50},{-20,19},{-6,19}}, color={0,0,127}));
        connect(govturb.SPEED, gENROU.SPEED) annotation (Line(points={{-81.125,73},{-90,73},{-90,88},{80,88},{80,25},{63,25}}, color={0,0,127}));
        connect(govturb.PELEC, pss.V_S2) annotation (Line(points={{-81.125,
                49.8571},{-90,49.8571},{-90,-34},{96,-34},{96,-45.6},{61.9,
                -45.6}},                                                                                                                 color={0,0,127}));
        connect(gENROU.XADIFD, avr.XADIFD) annotation (Line(points={{63,-23},
                {70,-23},{70,-94},{-20.8,-94},{-20.8,-77.4}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-48,2},{-20,56},{2,4},{24,-28},{48,22}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-52,-18},{56,-66}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
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
",       info="<html>
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
      end Generator_AVR_PSS_TurbGov;

        model Generator_AVR_PSS_TurbGov_external_pmech
        "Generator + AVR + PSS + Turbine + Governor"
        extends OpenIPSL.Electrical.Essentials.pfComponent;
        OpenIPSL.Interfaces.PwPin pwPin annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
        OpenIPSL.Electrical.Machines.PSSE.GENROU gENROU(
          v_0=v_0,
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0,
          M_b=M_b,
          Tpd0=machinePars.Tpd0,
          Tppd0=machinePars.Tppd0,
          Tppq0=machinePars.Tppq0,
          H=machinePars.H,
          D=machinePars.D,
          Xd=machinePars.Xd,
          Xq=machinePars.Xq,
          Xpd=machinePars.Xpd,
          Xppd=machinePars.Xppd,
          Xppq=machinePars.Xppq,
          Xl=machinePars.Xl,
          S10=machinePars.S10,
          S12=machinePars.S12,
          Xpq=machinePars.Xpq,
          Tpq0=machinePars.Tpq0,
          Xpp=machinePars.Xpp,
          R_a=machinePars.R_a,
          V_b=V_b) annotation (Placement(transformation(extent={{0,-26},{60,34}})));
        parameter Real M_b "Machine base power (MVA)";
        Modelica.Blocks.Interfaces.RealOutput Pm0 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,-106}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={46,-98})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-62,-98},{-42,-78}})));
        OpenIPSL.Electrical.Controls.PSSE.ES.IEEET1 avr(
          T_R=aVRPars.T_R,
          K_A=aVRPars.K_A,
          T_A=aVRPars.T_A,
          V_RMAX=aVRPars.V_RMAX,
          V_RMIN=aVRPars.V_RMIN,
          K_E=aVRPars.K_E,
          T_E=aVRPars.T_E,
          K_F=aVRPars.K_F,
          T_F=aVRPars.T_F,
          E_1=aVRPars.E_1,
          S_EE_1=aVRPars.S_EE_1,
          E_2=aVRPars.E_2,
          S_EE_2=aVRPars.S_EE_2) annotation (Placement(transformation(extent={{8,-76},{-24,-48}})));
        OpenIPSL.Electrical.Controls.PSSE.PSS.PSS2A pss(
          T_w1=pSSPars.T_w1,
          T_w2=pSSPars.T_w2,
          T_6=pSSPars.T_6,
          T_w3=pSSPars.T_w3,
          T_w4=pSSPars.T_w4,
          T_7=pSSPars.T_7,
          K_S2=pSSPars.K_S2,
          K_S3=pSSPars.K_S3,
          T_8=pSSPars.T_8,
          T_9=pSSPars.T_9,
          K_S1=pSSPars.K_S1,
          T_1=pSSPars.T_1,
          T_2=pSSPars.T_2,
          T_3=pSSPars.T_3,
          T_4=pSSPars.T_4,
          V_STMAX=pSSPars.V_STMAX,
          V_STMIN=pSSPars.V_STMIN,
          M=pSSPars.M,
          N=pSSPars.N) annotation (Placement(transformation(extent={{60,-48},{22,-40}})));
        Modelica.Blocks.Interfaces.RealOutput speed annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={106,32}), iconTransformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-44,-98})));
        Electrical.Controls.TG.GGOV1.Simplified.GGOV2                  govturb(
          R=0.04,
          T_pelec=1,
          maxerr=0.05,
          minerr=-0.05,
          Kpgov=10,
          Kigov=5,
          Kdgov=0,
          Tdgov=1,
          Dm=0,
          Kimw=0,
          db=0,
          Vmax=1,
          Vmin=0.1,
          Tact=4,
          Teng=0,
          Tfload=3,
          Tsa=4,
          Tsb=5,
          DELT=0.005,
          Trate=10,
          Rup=99,
          Rdown=-99,
          Ropen=0.1,
          Rclose=-0.1,
          Flag=0,
          Pref=P_0/M_b,
          Kturb=1.5,
          Wfnl=0.15,
          Tb=0.14101,
          Tc=0.11514)
          annotation (Placement(transformation(extent={{-80,40},{-40,80}})));
        Records.MachinePars1 machinePars
          annotation (Placement(transformation(extent={{0,60},{20,80}})));
        Records.AVRPars aVRPars
          annotation (Placement(transformation(extent={{22,60},{42,80}})));
        Records.PSSPars pSSPars
          annotation (Placement(transformation(extent={{44,60},{64,80}})));
        Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(
                extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},
                  {-100,20}})));
        Modelica.Blocks.Math.Add add
          annotation (Placement(transformation(extent={{-118,36},{-98,56}})));
        equation
        connect(gENROU.p, pwPin) annotation (Line(points={{60,4},{110,4},{110,0}},color={0,0,255}));
        connect(Pm0, gENROU.PMECH0) annotation (Line(points={{80,-106},{80,-88},{92,-88},{92,19},{63,19}}, color={0,0,127}));
        connect(avr.EFD, gENROU.EFD) annotation (Line(points={{-25.6,-62},{
                -34,-62},{-40,-62},{-40,-60},{-40,-32},{-40,-11},{-6,-11}},                                                                           color={0,0,127}));
        connect(gENROU.EFD0, avr.EFD0) annotation (Line(points={{63,-11},{76,
                -11},{76,-67.6},{9.6,-67.6}},                                                                  color={0,0,127}));
        connect(gENROU.ETERM, avr.ECOMP) annotation (Line(points={{63,-5},{82,
                -5},{82,-62},{9.6,-62}},                                                                       color={0,0,127}));
        connect(const.y, avr.VOEL) annotation (Line(points={{-41,-88},{-41,
                -88},{-8,-88},{-8,-77.4}},                                                              color={0,0,127}));
        connect(avr.VUEL, avr.VOEL) annotation (Line(points={{-1.6,-77.4},{2,
                -77.4},{2,-88},{-8,-88},{-8,-77.4}},                                                           color={0,0,127}));
        connect(pss.VOTHSG, avr.VOTHSG) annotation (Line(points={{20.1,-44},{
                14,-44},{14,-56.4},{9.6,-56.4}},                                                                  color={0,0,127}));
        connect(gENROU.PELEC, pss.V_S2) annotation (Line(points={{63,13},{96,
                13},{96,-45.6},{61.9,-45.6}},                                                        color={0,0,127}));
        connect(speed, gENROU.SPEED) annotation (Line(points={{106,32},{88,32},{88,25},{63,25}}, color={0,0,127}));
        connect(pss.V_S1, gENROU.SPEED) annotation (Line(points={{61.9,-42.4},
                {88,-42.4},{88,25},{63,25}},                                                         color={0,0,127}));
        connect(govturb.PMECH, gENROU.PMECH) annotation (Line(points={{-38.5,50},{-20,50},{-20,19},{-6,19}}, color={0,0,127}));
        connect(govturb.SPEED, gENROU.SPEED) annotation (Line(points={{-81.125,73},{-90,73},{-90,88},{80,88},{80,25},{63,25}}, color={0,0,127}));
        connect(add.y, govturb.PELEC) annotation (Line(points={{-97,46},{-89.5,
                46},{-89.5,49.8571},{-81.125,49.8571}},
                                                   color={0,0,127}));
        connect(add.u2, pss.V_S2) annotation (Line(points={{-120,40},{-88,40},
                {-88,-44},{20,-44},{20,-45.6},{61.9,-45.6}},
                                                  color={0,0,127}));
        connect(u, add.u1)
          annotation (Line(points={{-120,0},{-120,52}}, color={0,0,127}));
        connect(gENROU.XADIFD, avr.XADIFD) annotation (Line(points={{63,-23},{72,-23},
                {72,-92},{-20.8,-92},{-20.8,-77.4}}, color={0,0,127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-48,2},{-20,56},{2,4},{24,-28},{48,22}},
                color={0,0,0},
                smooth=Smooth.Bezier),
              Text(
                extent={{-52,-18},{56,-66}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
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
",       info="<html>
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
        end Generator_AVR_PSS_TurbGov_external_pmech;

      package Records "Sets of generator parameters"

        record MachinePars1 "Original OpenIPSL GENROU Model Parameters"
          extends Modelica.Icons.Record;

          parameter Real M_b=10 "Machine base power (MVA)";
          parameter Real Tpd0=5 "d-axis transient open-circuit time constant (s)";
          parameter Real Tppd0=0.50000E-01 "d-axis sub-transient open-circuit time constant (s)";
          parameter Real Tppq0=0.1 "q-axis transient open-circuit time constant (s)";
          parameter Real H=4.0000 "Inertia constant (s)";
          parameter Real D=0 "Speed damping";
          parameter Real Xd=1.41 "d-axis reactance";
          parameter Real Xq=1.3500 "q-axis reactance";
          parameter Real Xpd=0.3 "d-axis transient reactance";
          parameter Real Xppd=0.2 "d-axis sub-transient reactance";
          parameter Real Xppq=0.2 "q-axis sub-transient reactance";
          parameter Real Xl=0.12 "leakage reactance";
          parameter Real S10=0.1 "Saturation factor at 1.0 pu";
          parameter Real S12=0.5 "Saturation factor at 1.2 pu";
          parameter Real R_a=0 "amature resistance";
          parameter Real Xpq=0.6 "q-axis transient reactance (pu)";
          parameter Real Tpq0=0.7 "q-axis transient open-circuit time constant (s)";
          parameter Real Xpp=0.2 "Sub-transient reactance (pu)";

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end MachinePars1;

        record MachinePars2 "Parameters taken from Anderson book"
          extends Modelica.Icons.Record;

          parameter Real M_b=10 "Machine base power (MVA)";
          parameter Real Tpd0=7.5 "d-axis transient open-circuit time constant (s)";
          parameter Real Tppd0=0.054 "d-axis sub-transient open-circuit time constant (s)";
          parameter Real Tppq0=0.107 "q-axis transient open-circuit time constant (s)";
          parameter Real H=4.28 "Inertia constant (s)";
          parameter Real D=2 "Speed damping";
          parameter Real Xd=1.64 "d-axis reactance";
          parameter Real Xq=1.575 "q-axis reactance";
          parameter Real Xpd=0.159 "d-axis transient reactance";
          parameter Real Xppd=0.102 "d-axis sub-transient reactance";
          parameter Real Xppq=0.1 "q-axis sub-transient reactance";
          parameter Real Xl=0.113 "leakage reactance";
          parameter Real S10=0.087 "Saturation factor at 1.0 pu";
          parameter Real S12=0.2681 "Saturation factor at 1.2 pu";
          parameter Real R_a=0.034 "amature resistance";
          parameter Real Xpq=0.306 "q-axis transient reactance (pu)";
          parameter Real Tpq0=1.5 "q-axis transient open-circuit time constant (s)";
          parameter Real Xpp=Xppd "Sub-transient reactance (pu)";

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end MachinePars2;

        record AVRPars "Example of AVR parameters"
          extends Modelica.Icons.Record;

          parameter Real T_R=1 "Voltage input time constant (s)";
          parameter Real K_A=40 "AVR gain";
          parameter Real T_A=0.04 "AVR time constant (s)";
          parameter Real V_RMAX=7.3 "Maximum AVR output (pu)";
          parameter Real V_RMIN=-7.3 "Minimum AVR output (pu)";
          parameter Real K_E=1 "Exciter field gain, s";
          parameter Real T_E=0.8 "Exciter time constant (s)";
          parameter Real K_F=0.03 "Rate feedback gain (pu)";
          parameter Real T_F=1 "Rate feedback time constant (s)";
          parameter Real E_1=2.400 "Exciter saturation point 1 (pu)";
          parameter Real S_EE_1=0.30000E-01 "Saturation at E1";
          parameter Real E_2=5.0000 "Exciter saturation point 2 (pu)";
          parameter Real S_EE_2=0.50000 "Saturation at E2";

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end AVRPars;

        record PSSPars "Example of PSS parameters"
          extends Modelica.Icons.Record;

          parameter Real T_w1=10 "Washout 1 time constant";
          parameter Real T_w2=10 "Washout 2 time constant";
          parameter Real T_6=1e-9 "Lag 1 time constant";
          parameter Real T_w3=10 "Washout 3 time constant";
          parameter Real T_w4=1e-9 "Washout 4 time constant";
          parameter Real T_7=10 "Lag 2 time constant";
          parameter Real K_S2=0.99 "Lag 2 gain";
          parameter Real K_S3=1 "gain";
          parameter Real T_8=0.5 "Ramp-tracking filter time constant";
          parameter Real T_9=0.1 "Ramp-tracking filter time constant";
          parameter Real K_S1=20 "PSS gain";
          parameter Real T_1=0.15 "Leadlag1 time constant (data from IEEE std, not representive, need to be tuned following system parameters)";
          parameter Real T_2=0.025 "Leadlag1 time constant";
          parameter Real T_3=0.15 "Leadlag2 time constant";
          parameter Real T_4=0.025 "Leadlag2 time constant";
          parameter Real V_STMAX=0.1 "PSS output limiation";
          parameter Real V_STMIN=-0.1 "PSS output limiation";
          parameter Integer M=0 "Ramp tracking filter coefficient";
          parameter Integer N=0 "Ramp tracking filter coefficient";

          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end PSSPars;
      end Records;
    end Generation_Groups;
  end CampusA;
end Campuses;
