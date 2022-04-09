within MicroGrid.Examples;
package SMIB
  extends Modelica.Icons.ExamplesPackage;

  package NoNoiseOnLoad
    "Models where the load does not have variability"
    extends Modelica.Icons.ExamplesPackage;

    model SMIB "SMIB network without controls (no noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_NoNoise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13800,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator generator(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
      BaseClasses.SMIB.Records.PF_050 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
    equation
      connect(generator.pwPin, GEN1.p) annotation (Line(points={{-51,0},{-42,0}}, color={0,0,255}));
      connect(generator.Pm0, generator.Pmech) annotation (Line(points={{-57.4,-9.8},
    {-57.4,-20},{-80,-20},{-80,0},{-72.6,0}}, color={0,0,127}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(StopTime=50),
        __Dymola_experimentSetupOutput);
    end SMIB;

    model SMIB_TurbGov "SMIB network with governor and turbine dynamics, no exciter (load without noise)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_NoNoise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23,
          d_P=0.2,
          t1=30,
          d_t=20),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_TurbGov generator(
        V_b=13800,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
    equation
      connect(generator.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(StopTime=100, __Dymola_NumberOfIntervals=5000),
        __Dymola_experimentSetupOutput);
    end SMIB_TurbGov;

    model SMIB_TurbGov2 "Model with GGOV1 turbine and ThermoPower PID controller, change of power dispatch"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_NoNoise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator generator(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Modelica.Blocks.Math.Add add annotation (Placement(transformation(extent={{-54,-60},{-66,-48}})));
      Modelica.Blocks.Sources.Step step(startTime=30, height=0.3) annotation (Placement(transformation(extent={{-30,-70},{-40,-60}})));
      Electrical.Controls.TG.GGOV1.Simplified.GGOV2B gGOV2B(
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
        Tb=0.5,
        Tc=0,
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
        Kturb=1.5,
        Wfnl=0.15)
        annotation (Placement(transformation(extent={{-126,-28},{-94,0}})));
      BaseClasses.SMIB.Records.PF_050 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
    equation
      connect(GEN1.p, generator.pwPin) annotation (Line(points={{-42,0},{-49,0}}, color={0,0,255}));
      connect(generator.Pm0, add.u1) annotation (Line(points={{-55.4,-9.8},{-55.4,-34},{-44,-34},{-44,-50.4},{-52.8,-50.4}}, color={0,0,127}));
      connect(step.y, add.u2) annotation (Line(points={{-40.5,-65},{-46,-65},{-46,-57.6},{-52.8,-57.6}}, color={0,0,127}));
      connect(gGOV2B.PMECH, generator.Pmech) annotation (Line(points={{-92.8,-21},{-82,-21},{-82,0},{-70.6,0}}, color={0,0,127}));
      connect(generator.PELEC, gGOV2B.PELEC) annotation (Line(points={{-69.2,-5.2},{-76,-5.2},{-76,-42},{-132,-42},{-132,-21.1},{-126.9,-21.1}}, color={0,0,127}));
      connect(generator.speed, gGOV2B.SPEED) annotation (Line(points={{-64.4,-9.8},{-64.4,-16},{-90,-16},{-90,6},{-134,6},{-134,-4.9},{-126.9,-4.9}}, color={0,0,127}));
      connect(add.y, gGOV2B.PSP) annotation (Line(points={{-66.6,-54},{-136,-54},{-136,-13.3},{-126.9,-13.3}}, color={0,0,127}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(StopTime=100),
        __Dymola_experimentSetupOutput);
    end SMIB_TurbGov2;

    model SMIB_TurbNoGov "SMIB network and turbine dynamics, no exciter (load without noise), no governor"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_NoNoise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23,
          d_P=0.2,
          t1=30,
          d_t=20),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator generator(
        V_b=13800,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
      Electrical.Controls.TG.GGOV1.Simplified.Turbine turbine(
        Flag=0,
        Tact=4,
        Kturb=1.5,
        Tb=0.14101,
        Tc=0.1151,
        Teng=0,
        Dm=0,
        Wfnl=0.15,
        Tfload=3,
        Ropen=0.1,
        Rclose=-0.1,
        Tsa=4,
        Tsb=5,
        DELT=0.005)
        annotation (Placement(transformation(extent={{-108,-14},{-80,14}})));
      Modelica.Blocks.Sources.Constant const(k=0.65) annotation (Placement(transformation(extent={{-148,-18},{-132,-2}})));
    equation
      connect(generator.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(turbine.PMECH, generator.Pmech) annotation (Line(points={{-78.8,0},{-78,0},{-70.6,0}}, color={0,0,127}));
      connect(generator.speed, turbine.SPEED) annotation (Line(points={{-64.4,-9.8},{-64.4,-38},{-64,-38},{-126,-38},{-126,10},{-109,10}}, color={0,0,127}));
      connect(generator.PELEC, turbine.PELEC) annotation (Line(points={{-69.2,-5.2},{-68,-5.2},{-68,-28},{-122,-28},{-122,0.1},{-108.9,0.1}}, color={0,0,127}));
      connect(const.y, turbine.FSR) annotation (Line(points={{-131.2,-10},{-109.1,-10},{-109.1,-10.1}}, color={0,0,127}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(StopTime=100, __Dymola_NumberOfIntervals=5000),
        __Dymola_experimentSetupOutput);
    end SMIB_TurbNoGov;
  end NoNoiseOnLoad;

  package NoiseOnLoad "Models where non-deterministic load is employed"
    extends Modelica.Icons.ExamplesPackage;

    model input_plusfault_variacechange
      "SMIB network without controls (noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise_normal_limhit(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23),
        line_3(opening=1));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_AVR_PSS_TurbGov_external_pmech generator_AVR_PSS_TurbGov_external_pmech(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-76,-118},{-56,-98}})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection
                                          whiteNoiseInjection(
          active_sigma=0.001) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-150,10})));

      Modelica.Blocks.Sources.Constant
     const(k=0)
        annotation (Placement(transformation(extent={{-194,-110},{-174,-90}})));
      Modelica.Blocks.Math.Add add
        annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
      Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=5)
        annotation (Placement(transformation(extent={{-162,-110},{-142,-90}})));
      Modelica.Blocks.Discrete.FirstOrderHold firstOrderHold(samplePeriod=0.02)
        annotation (Placement(transformation(extent={{-134,-110},{-114,-90}})));
      Modelica.Blocks.Logical.Switch switch1
        annotation (Placement(transformation(extent={{-142,-38},{-122,-18}})));
      Modelica.Blocks.Logical.Greater greater
        annotation (Placement(transformation(extent={{-178,-38},{-158,-18}})));
      Modelica.Blocks.Sources.ContinuousClock
                                    clock
        annotation (Placement(transformation(extent={{-220,-38},{-200,-18}})));
      Modelica.Blocks.Sources.Constant const1(k=1)
        annotation (Placement(transformation(extent={{-220,-70},{-200,-50}})));
      Modelica.Blocks.Sources.Constant const2(k=0) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-166,-62})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection
                                          whiteNoiseInjection1(
          active_sigma=0.001) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-70,-62})));

      Modelica.Blocks.Logical.Switch switch2
        annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
      OpenIPSL.Electrical.Events.PwFault pwFault(
        R=0.001,
        X=0.001,
        t1=90,
        t2=90.08) annotation (Placement(transformation(extent={{-6,-48},{6,-36}})));
      Modelica.Blocks.Sources.Trapezoid trapezoid(
        amplitude=0.3,
        rising=100,
        width=100,
        falling=100,
        period=300,
        offset=-0.15,
        startTime=1)
        annotation (Placement(transformation(extent={{-80,-46},{-60,-26}})));
      Modelica.Blocks.Math.Add add1
        annotation (Placement(transformation(extent={{-46,-54},{-26,-34}})));
    equation
      connect(generator_AVR_PSS_TurbGov_external_pmech.pwPin, GEN1.p)
        annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(add.y, generator_AVR_PSS_TurbGov_external_pmech.u)
        annotation (Line(points={{-77,0},{-72,0}}, color={0,0,127}));
      connect(sampler.y, firstOrderHold.u)
        annotation (Line(points={{-141,-100},{-136,-100}}, color={0,0,127}));
      connect(sampler.u, const.y)
        annotation (Line(points={{-164,-100},{-173,-100}}, color={0,0,127}));
      connect(clock.y, greater.u1)
        annotation (Line(points={{-199,-28},{-180,-28}}, color={0,0,127}));
      connect(const1.y, greater.u2) annotation (Line(points={{-199,-60},{-190,-60},
    {-190,-36},{-180,-36}}, color={0,0,127}));
      connect(greater.y, switch1.u2)
        annotation (Line(points={{-157,-28},{-144,-28}}, color={255,0,255}));
      connect(const2.y, switch1.u3) annotation (Line(points={{-155,-62},{-150,-62},
    {-150,-36},{-144,-36}}, color={0,0,127}));
      connect(switch1.y, add.u1) annotation (Line(points={{-121,-28},{-110,-28},{
    -110,6},{-100,6}}, color={0,0,127}));
      connect(whiteNoiseInjection.y, switch1.u1) annotation (Line(points={{-150.1,
    -0.9},{-150.1,-20},{-144,-20}}, color={0,0,127}));
      connect(firstOrderHold.y, add.u2) annotation (Line(points={{-113,-100},{-104,
    -100},{-104,-6},{-100,-6}}, color={0,0,127}));
      connect(switch2.y, variableLoad.u) annotation (Line(points={{1,-80},{14,-80},
    {14,-80.05},{27.9,-80.05}}, color={0,0,127}));
      connect(switch2.u2, switch1.u2) annotation (Line(points={{-22,-80},{-86,-80},
    {-86,-28},{-144,-28}}, color={255,0,255}));
      connect(switch2.u3, switch1.u3) annotation (Line(points={{-22,-88},{-92,-88},
    {-92,-52},{-150,-52},{-150,-36},{-144,-36}}, color={0,0,127}));
      connect(pwFault.p, BUS1.p) annotation (Line(points={{-7,-42},{-12,-42},{-12,0},
    {-6,0}}, color={0,0,255}));
      connect(trapezoid.y, add1.u1) annotation (Line(points={{-59,-36},{-52,-36},{
    -52,-38},{-48,-38}}, color={0,0,127}));
      connect(whiteNoiseInjection1.y, add1.u2) annotation (Line(points={{-59.1,
    -62.1},{-59.1,-56.05},{-48,-56.05},{-48,-50}}, color={0,0,127}));
      connect(add1.y, switch2.u1) annotation (Line(points={{-25,-44},{-22,-44},{-22,
    -72},{-22,-72}}, color={0,0,127}));
      annotation (
        experiment(
          StopTime=10,
          Interval=0.02,
          __Dymola_Algorithm="Dassl"),
        __Dymola_experimentSetupOutput,
        Diagram(coordinateSystem(extent={{-240,-120},{160,40}}), graphics={Text(
    extent={{-226,38},{-66,18}},
    lineColor={28,108,200},
    textString="Note: this model uses a full generator+controls, it simulates the ramp-up or down of the generator by varying the input of Pmech.
To do this, the ramp has a height of -0.25 for ramp up, and 0.25 for ramp up. Ramp is set to zero in this example.
Stochastic input is added also.
A fault block is added and line 4 is tripped. Simulates a contingency in the network.

"),                                                                        Text(
    extent={{-26,36},{134,16}},
    lineColor={238,46,47},
              textString="Note2: The previous Modelica.Blocks.Sources.Clock block was replaced by a Modelica.Blocks.Sources.ContinuousClock to be compliant with MSL4.

")}),             Icon(coordinateSystem(extent={{-240,-120},{160,40}})));
    end input_plusfault_variacechange;

    model SignalB
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=true)
        annotation (Placement(transformation(extent={{112,86},{132,106}})));
      Modelica.Blocks.Noise.NormalNoise normalNoise(samplePeriod=0.01, sigma=1)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=0,
            origin={10,98})));
      Modelica.Blocks.Sources.Ramp ramp(height=0.2, duration=15)
        annotation (Placement(transformation(extent={{-32,88},{-12,108}})));
      Modelica.Blocks.Sources.Ramp ramp1(
        height=-0.2,
        duration=15,
        startTime=30)
        annotation (Placement(transformation(extent={{-64,88},{-44,108}})));
      Modelica.Blocks.Math.MultiSum multiSum(nu=5)
        annotation (Placement(transformation(extent={{36,66},{48,78}})));
      Modelica.Blocks.Sources.Pulse pulse(
        amplitude=1,
        width=30,
        period=1,
        nperiod=1,
        startTime=70)
        annotation (Placement(transformation(extent={{-94,88},{-74,108}})));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{134,62},{154,82}})));
      Modelica.Blocks.Continuous.SecondOrder secondOrder1(
        k=1,
        w=10,
        D=5)
        annotation (Placement(transformation(extent={{76,62},{96,82}})));
      Modelica.Blocks.Logical.Switch switch1 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-22,-4})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=120)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=180,
            origin={34,-22})));
      Modelica.Blocks.Sources.RealExpression realExpression(y=time)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=180,
            origin={80,-22})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=180,
            origin={56,4})));
      Modelica.Blocks.Sources.RealExpression realExpression1(y=time)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-64,-26})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1(threshold=200)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-64,8})));
      Modelica.Blocks.Logical.Switch switch2 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-28,38})));
      Modelica.Blocks.Sources.Sine sine(
        amplitude=2,
        f=15,
        phase=0,
        startTime=120) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={2,-28})));
      Modelica.Blocks.Math.Gain gain(k=5)  annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={20,38})));
      Modelica.Blocks.Sources.Constant const1(k=0)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-84,46})));
    equation
      connect(normalNoise.y, multiSum.u[1]) annotation (Line(points={{21,98},{28,
    98},{28,76},{36,76},{36,75.36}},   color={0,0,
    127}));
      connect(ramp.y, multiSum.u[2]) annotation (Line(points={{-11,98},{-6,98},{
    -6,73.68},{36,73.68}},         color={0,0,127}));
      connect(ramp1.y, multiSum.u[3]) annotation (Line(points={{-43,98},{-36,98},
    {-36,72},{36,72}},        color={0,0,127}));
      connect(pulse.y, multiSum.u[4]) annotation (Line(points={{-73,98},{-68,98},
    {-68,70.32},{36,70.32}},color={0,0,127}));
      connect(secondOrder1.y, y)
        annotation (Line(points={{97,72},{144,72}},
          color={0,0,127}));
      connect(greaterThreshold.y, switch1.u2) annotation (Line(points={{23,-22},{
    23,-4},{-10,-4}},    color={255,0,255}));
      connect(multiSum.y, secondOrder1.u)
        annotation (Line(points={{49.02,72},{74,72}},
            color={0,0,127}));
      connect(realExpression.y, greaterThreshold.u) annotation (Line(points={{69,-22},
    {46,-22}}, color={0,0,127}));
      connect(realExpression1.y, greaterThreshold1.u)
        annotation (Line(points={{-64,-15},{-64,-4}},  color={0,0,127}));
      connect(greaterThreshold1.y, switch2.u2) annotation (Line(points={{-64,19},
    {-64,38},{-40,38}},    color={255,0,255}));
      connect(sine.y, switch1.u1) annotation (Line(points={{2,-17},{2,-12},{-10,
    -12}},       color={0,0,127}));
      connect(const.y, switch1.u3) annotation (Line(points={{45,4},{-10,4}},
            color={0,0,127}));
      connect(switch1.y, switch2.u3) annotation (Line(points={{-33,-4},{-46,-4},{
    -46,30},{-40,30}},       color={0,0,127}));
      connect(switch2.y, gain.u) annotation (Line(points={{-17,38},{8,38}},
          color={0,0,127}));
      connect(gain.y, multiSum.u[5]) annotation (Line(points={{31,38},{31,68.64},
    {36,68.64}},   color={0,0,127}));
      connect(const1.y, switch2.u1)
        annotation (Line(points={{-73,46},{-40,46}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{140,
      120}})),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{
      140,120}})),
        experiment(
          StopTime=300,
          __Dymola_NumberOfIntervals=9000,
          __Dymola_fixedstepsize=0.001,
          __Dymola_Algorithm="Rkfix4"));
    end SignalB;

    model SMIB "SMIB network without controls (noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        sineNoiseInjection(amplitude=0.05, freqHz=0.02),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator generator(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
    equation
      connect(generator.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(generator.Pm0, generator.Pmech) annotation (Line(points={{-55.4,-9.8},{-55.4,-20},{-80,-20},{-80,0},{-70.6,0}}, color={0,0,127}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(StopTime=50),
        __Dymola_experimentSetupOutput);
    end SMIB;

    model SMIB_normal_noise
      "SMIB network without controls (noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise_normal(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23),
        whiteNoiseInjection(active_sigma=1e-3, samplePeriod=0.01),
        line_3(opening=1));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_AVR_PSS_TurbGov generator_AVR_PSS_TurbGov(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-60,-86},{-40,-66}})));
    equation
      connect(generator_AVR_PSS_TurbGov.pwPin, GEN1.p)
        annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      annotation (
        experiment(
          StopTime=300,
          Interval=0.02,
          __Dymola_Algorithm="Dassl"),
        __Dymola_experimentSetupOutput,
        Diagram(coordinateSystem(extent={{-80,-120},{120,40}}), graphics={Text(
    extent={{-60,40},{100,20}},
    lineColor={28,108,200},
    textString="Note: this model uses a full generator+controls, it simulates the ambient response of the system to a stochastically varying load.
")}),             Icon(coordinateSystem(extent={{-80,-120},{120,40}})));
    end SMIB_normal_noise;

    model SMIB_normal_noise_gen_input
      "SMIB network without controls (noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise_normal_limhit(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23),
        line_3(opening=1));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_AVR_PSS_TurbGov_external_pmech generator_AVR_PSS_TurbGov_external_pmech(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-76,-118},{-56,-98}})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection whiteNoiseInjection(
          active_sigma=0.01) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-150,10})));

      Modelica.Blocks.Sources.Ramp ramp(
        height=0.25,
        duration=120,
        startTime=60)
        annotation (Placement(transformation(extent={{-194,-110},{-174,-90}})));
      Modelica.Blocks.Math.Add add
        annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
      Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=5)
        annotation (Placement(transformation(extent={{-162,-110},{-142,-90}})));
      Modelica.Blocks.Discrete.FirstOrderHold firstOrderHold(samplePeriod=0.02)
        annotation (Placement(transformation(extent={{-134,-110},{-114,-90}})));
      Modelica.Blocks.Logical.Switch switch1
        annotation (Placement(transformation(extent={{-142,-38},{-122,-18}})));
      Modelica.Blocks.Logical.Greater greater
        annotation (Placement(transformation(extent={{-178,-38},{-158,-18}})));
      Modelica.Blocks.Sources.ContinuousClock clock
        annotation (Placement(transformation(extent={{-220,-38},{-200,-18}})));
      Modelica.Blocks.Sources.Constant const1(k=1)
        annotation (Placement(transformation(extent={{-220,-70},{-200,-50}})));
      Modelica.Blocks.Sources.Constant const2(k=0) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-166,-62})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection whiteNoiseInjection1(
          active_sigma=0.001) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-4,-80})));

    equation
      connect(generator_AVR_PSS_TurbGov_external_pmech.pwPin, GEN1.p)
        annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(add.y, generator_AVR_PSS_TurbGov_external_pmech.u)
        annotation (Line(points={{-77,0},{-72,0}}, color={0,0,127}));
      connect(sampler.y, firstOrderHold.u)
        annotation (Line(points={{-141,-100},{-136,-100}}, color={0,0,127}));
      connect(sampler.u, ramp.y)
        annotation (Line(points={{-164,-100},{-173,-100}}, color={0,0,127}));
      connect(clock.y, greater.u1)
        annotation (Line(points={{-199,-28},{-180,-28}}, color={0,0,127}));
      connect(const1.y, greater.u2) annotation (Line(points={{-199,-60},{-190,-60},
              {-190,-36},{-180,-36}}, color={0,0,127}));
      connect(greater.y, switch1.u2)
        annotation (Line(points={{-157,-28},{-144,-28}}, color={255,0,255}));
      connect(const2.y, switch1.u3) annotation (Line(points={{-155,-62},{-150,-62},
              {-150,-36},{-144,-36}}, color={0,0,127}));
      connect(switch1.y, add.u1) annotation (Line(points={{-121,-28},{-110,-28},{
              -110,6},{-100,6}}, color={0,0,127}));
      connect(whiteNoiseInjection.y, switch1.u1) annotation (Line(points={{-150.1,
              -0.9},{-150.1,-20},{-144,-20}}, color={0,0,127}));
      connect(firstOrderHold.y, add.u2) annotation (Line(points={{-113,-100},{-104,
              -100},{-104,-6},{-100,-6}}, color={0,0,127}));
      connect(whiteNoiseInjection1.y, variableLoad.u) annotation (Line(points={{6.9,
              -80.1},{8.45,-80.1},{8.45,-80.05},{27.9,-80.05}}, color={0,0,127}));
      annotation (
        experiment(
          StopTime=10,
          Interval=0.02,
          __Dymola_Algorithm="Dassl"),
        __Dymola_experimentSetupOutput,
        Diagram(coordinateSystem(extent={{-200,-120},{160,40}}), graphics={Text(
              extent={{-184,38},{-24,18}},
              lineColor={28,108,200},
              textString="Note: this model uses a full generator+controls, it simulates the ramp-up or down of the generator by varying the input of Pmech.
To do this, the ramp has a height of -0.25 for ramp up, and 0.25 for ramp up.
Stochastic input is added also.
"),                                                                        Text(
    extent={{0,36},{154,22}},
    lineColor={238,46,47},
              textString="Note2: The previous Modelica.Blocks.Sources.Clock block was replaced by
a Modelica.Blocks.Sources.ContinuousClock to be compliant with MSL4.

")}),   Icon(coordinateSystem(extent={{-200,-120},{160,40}})));
    end SMIB_normal_noise_gen_input;

    model SMIB_normal_noise_gen_input_plusfault
      "SMIB network without controls (noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise_normal_limhit(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23),
        line_3(opening=1));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_AVR_PSS_TurbGov_external_pmech generator_AVR_PSS_TurbGov_external_pmech(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-76,-118},{-56,-98}})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection whiteNoiseInjection(
          active_sigma=0.01) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-150,10})));

      Modelica.Blocks.Sources.Ramp ramp(
        height=-0.25,
        duration=120,
        startTime=60)
        annotation (Placement(transformation(extent={{-194,-110},{-174,-90}})));
      Modelica.Blocks.Math.Add add
        annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
      Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=5)
        annotation (Placement(transformation(extent={{-162,-110},{-142,-90}})));
      Modelica.Blocks.Discrete.FirstOrderHold firstOrderHold(samplePeriod=0.02)
        annotation (Placement(transformation(extent={{-134,-110},{-114,-90}})));
      Modelica.Blocks.Logical.Switch switch1
        annotation (Placement(transformation(extent={{-142,-38},{-122,-18}})));
      Modelica.Blocks.Logical.Greater greater
        annotation (Placement(transformation(extent={{-178,-38},{-158,-18}})));
      Modelica.Blocks.Sources.ContinuousClock clock
        annotation (Placement(transformation(extent={{-220,-38},{-200,-18}})));
      Modelica.Blocks.Sources.Constant const1(k=1)
        annotation (Placement(transformation(extent={{-220,-70},{-200,-50}})));
      Modelica.Blocks.Sources.Constant const2(k=0) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-166,-62})));
      OpenIPSL.Electrical.Loads.NoiseInjections.WhiteNoiseInjection whiteNoiseInjection1(
          active_sigma=0.001) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-58,-54})));

      Modelica.Blocks.Logical.Switch switch2
        annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
      OpenIPSL.Electrical.Events.PwFault pwFault(
        R=0.001,
        X=0.001,
        t1=90,
        t2=90.08) annotation (Placement(transformation(extent={{-6,-48},{6,-36}})));
    equation
      connect(generator_AVR_PSS_TurbGov_external_pmech.pwPin, GEN1.p)
        annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(add.y, generator_AVR_PSS_TurbGov_external_pmech.u)
        annotation (Line(points={{-77,0},{-72,0}}, color={0,0,127}));
      connect(sampler.y, firstOrderHold.u)
        annotation (Line(points={{-141,-100},{-136,-100}}, color={0,0,127}));
      connect(sampler.u, ramp.y)
        annotation (Line(points={{-164,-100},{-173,-100}}, color={0,0,127}));
      connect(clock.y, greater.u1)
        annotation (Line(points={{-199,-28},{-180,-28}}, color={0,0,127}));
      connect(const1.y, greater.u2) annotation (Line(points={{-199,-60},{-190,-60},
    {-190,-36},{-180,-36}}, color={0,0,127}));
      connect(greater.y, switch1.u2)
        annotation (Line(points={{-157,-28},{-144,-28}}, color={255,0,255}));
      connect(const2.y, switch1.u3) annotation (Line(points={{-155,-62},{-150,-62},
    {-150,-36},{-144,-36}}, color={0,0,127}));
      connect(switch1.y, add.u1) annotation (Line(points={{-121,-28},{-110,-28},{
    -110,6},{-100,6}}, color={0,0,127}));
      connect(whiteNoiseInjection.y, switch1.u1) annotation (Line(points={{-150.1,
    -0.9},{-150.1,-20},{-144,-20}}, color={0,0,127}));
      connect(firstOrderHold.y, add.u2) annotation (Line(points={{-113,-100},{-104,
    -100},{-104,-6},{-100,-6}}, color={0,0,127}));
      connect(switch2.y, variableLoad.u) annotation (Line(points={{1,-80},{14,-80},
    {14,-80.05},{27.9,-80.05}}, color={0,0,127}));
      connect(whiteNoiseInjection1.y, switch2.u1) annotation (Line(points={{-47.1,
    -54.1},{-32.55,-54.1},{-32.55,-72},{-22,-72}}, color={0,0,127}));
      connect(switch2.u2, switch1.u2) annotation (Line(points={{-22,-80},{-86,-80},
    {-86,-28},{-144,-28}}, color={255,0,255}));
      connect(switch2.u3, switch1.u3) annotation (Line(points={{-22,-88},{-92,-88},
    {-92,-52},{-150,-52},{-150,-36},{-144,-36}}, color={0,0,127}));
      connect(pwFault.p, BUS1.p) annotation (Line(points={{-7,-42},{-12,-42},{-12,0},
    {-6,0}}, color={0,0,255}));
      annotation (
        experiment(
          StopTime=10,
          Interval=0.02,
          __Dymola_Algorithm="Dassl"),
        __Dymola_experimentSetupOutput,
        Diagram(coordinateSystem(extent={{-240,-120},{160,40}}), graphics={Text(
    extent={{-226,38},{-66,18}},
    lineColor={28,108,200},
    textString="Note: this model uses a full generator+controls, it simulates the ramp-up or down of the generator by varying the input of Pmech.
To do this, the ramp has a height of -0.25 for ramp up, and 0.25 for ramp up. Ramp is set to zero in this example.
Stochastic input is added also.
A fault block is added and line 4 is tripped. Simulates a contingency in the network.

"),                                                                        Text(
    extent={{-26,36},{134,16}},
    lineColor={238,46,47},
              textString="Note2: The previous Modelica.Blocks.Sources.Clock block was replaced by a Modelica.Blocks.Sources.ContinuousClock to be compliant with MSL4.

")}),             Icon(coordinateSystem(extent={{-240,-120},{160,40}})));
    end SMIB_normal_noise_gen_input_plusfault;

    model SMIB_normal_noise_signalb
      "SMIB network without controls (noise on load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise_normal_signalb(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          d_P=0,
          t1=0,
          d_t=0,
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23),
        line_3(opening=1));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_AVR_PSS_TurbGov generator_AVR_PSS_TurbGov(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-60,-86},{-40,-66}})));
      SignalB signalB
        annotation (Placement(transformation(extent={{-24,-98},{0,-82}})));
    equation
      connect(generator_AVR_PSS_TurbGov.pwPin, GEN1.p)
        annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(signalB.y, variableLoad.u) annotation (Line(points={{0.4,-86.8},{22,-86.8},
    {22,-84.05},{27.9,-84.05}}, color={0,0,127}));
      annotation (
        experiment(
          StopTime=300,
          Interval=0.01,
          __Dymola_fixedstepsize=0.001,
          __Dymola_Algorithm="Rkfix4"),
        __Dymola_experimentSetupOutput,
        Diagram(coordinateSystem(extent={{-80,-120},{120,40}}), graphics={Text(
    extent={{-60,40},{100,20}},
    lineColor={28,108,200},
    textString="Note: this model uses a full generator+controls, it simulates the ambient response of the system to a stochastically varying load.
")}),             Icon(coordinateSystem(extent={{-80,-120},{120,40}})));
    end SMIB_normal_noise_signalb;

    model SMIB_TurbGov "SMIB network with governor and turbine dynamics, no exciter (stochastic load)"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23,
          d_P=0.2,
          t1=30,
          d_t=20),
        sineNoiseInjection(
          amplitude=0.05,
          freqHz=0.02,
          active_sigma=0.0005),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator_TurbGov generator(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.V1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=true) annotation (Placement(transformation(extent={{-34,-80},{-14,-60}})));
    equation
      connect(generator.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(
          StopTime=100,
          Interval=0.02,
          Tolerance=0.001,
          __Dymola_fixedstepsize=0.02,
          __Dymola_Algorithm="Rkfix4"),
        __Dymola_experimentSetupOutput);
    end SMIB_TurbGov;

    model SMIB_TurbGov2 "Model with GGOV1 turbine and ThermoPower PID controller, change of power dispatch"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23,
          d_P=0.1,
          t1=80,
          d_t=5),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator generator(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.A1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Modelica.Blocks.Math.Add add annotation (Placement(transformation(extent={{-54,-60},{-66,-48}})));
      Modelica.Blocks.Sources.Step step(height=0.1, startTime=30) annotation (Placement(transformation(extent={{-30,-70},{-40,-60}})));
      Electrical.Controls.TG.GGOV1.Simplified.GGOV2B gGOV2B(
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
        Tb=0.5,
        Tc=0,
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
        Kturb=1.5,
        Wfnl=0.15)
        annotation (Placement(transformation(extent={{-126,-28},{-94,0}})));
      BaseClasses.SMIB.Records.PF_050 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
    equation
      connect(GEN1.p, generator.pwPin) annotation (Line(points={{-42,0},{-49,0}}, color={0,0,255}));
      connect(generator.Pm0, add.u1) annotation (Line(points={{-55.4,-9.8},{-55.4,-34},{-44,-34},{-44,-50.4},{-52.8,-50.4}}, color={0,0,127}));
      connect(step.y, add.u2) annotation (Line(points={{-40.5,-65},{-46,-65},{-46,-57.6},{-52.8,-57.6}}, color={0,0,127}));
      connect(gGOV2B.PMECH, generator.Pmech) annotation (Line(points={{-92.8,-21},{-82,-21},{-82,0},{-70.6,0}}, color={0,0,127}));
      connect(generator.PELEC, gGOV2B.PELEC) annotation (Line(points={{-69.2,-5.2},{-76,-5.2},{-76,-42},{-132,-42},{-132,-21.1},{-126.9,-21.1}}, color={0,0,127}));
      connect(generator.speed, gGOV2B.SPEED) annotation (Line(points={{-64.4,-9.8},{-64.4,-16},{-90,-16},{-90,6},{-134,6},{-134,-4.9},{-126.9,-4.9}}, color={0,0,127}));
      connect(add.y, gGOV2B.PSP) annotation (Line(points={{-66.6,-54},{-136,-54},{-136,-13.3},{-126.9,-13.3}}, color={0,0,127}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)")}),
        experiment(StopTime=100),
        __Dymola_experimentSetupOutput);
    end SMIB_TurbGov2;

    model SMIB_TurbNoGov "SMIB network and turbine dynamics, no exciter (stochastic load), no governor"
      extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise(
        transformer(V_b=13.8e3, Vn=13.8e3),
        LOAD(V_b=13.8e3),
        GEN1(V_b=13.8e3),
        BUS1(V_b=13.8e3),
        BUS2(V_b=13.8e3),
        BUS3(V_b=13.8e3),
        GEN2(V_b=13.8e3),
        infiniteGen(
          V_b=13.8e3,
          M_b=1000e6,
          P_0=pf_results.machines.P3_1,
          Q_0=pf_results.machines.Q3_1,
          v_0=pf_results.voltages.V3,
          angle_0=pf_results.voltages.A3),
        variableLoad(
          P_0=pf_results.loads.PL23_1,
          Q_0=pf_results.loads.QL23_1,
          v_0=pf_results.voltages.V23,
          angle_0=pf_results.voltages.A23,
          d_P=0.2,
          t1=30,
          d_t=20),
        sineNoiseInjection(
          amplitude=0.05,
          freqHz=0.02,
          active_sigma=0.0005),
        pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
      import Modelica.Constants.pi;

      Electrical.Generation_Groups.SMIB.Generator generator(
        V_b=13.8e3,
        M_b=10e6,
        Q_0=pf_results.machines.Q1_1,
        P_0=pf_results.machines.P1_1,
        v_0=pf_results.voltages.A1,
        angle_0=pf_results.voltages.A1)
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      BaseClasses.SMIB.Records.PF_075 pf_results
        annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
      inner Modelica.Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=true) annotation (Placement(transformation(extent={{-34,-80},{-14,-60}})));
      Electrical.Controls.TG.GGOV1.Simplified.Turbine turbine(
        Flag=0,
        Tact=4,
        Kturb=1.5,
        Tb=0.14101,
        Tc=0.1151,
        Teng=0,
        Dm=0,
        Wfnl=0.15,
        Tfload=3,
        Ropen=0.1,
        Rclose=-0.1,
        Tsa=4,
        Tsb=5,
        DELT=0.005)
        annotation (Placement(transformation(extent={{-114,-14},{-86,14}})));
      Modelica.Blocks.Sources.Constant const(k=0.65) annotation (Placement(transformation(extent={{-154,-18},{-138,-2}})));
    equation
      connect(generator.pwPin, GEN1.p) annotation (Line(points={{-49,0},{-45.5,0},{-42,0}}, color={0,0,255}));
      connect(turbine.PMECH, generator.Pmech) annotation (Line(points={{-84.8,0},{-70.6,0}}, color={0,0,127}));
      connect(generator.speed, turbine.SPEED) annotation (Line(points={{-64.4,-9.8},{-64.4,-36},{-64.4,-48},{-132,-48},{-132,10},{-115,10}}, color={0,0,127}));
      connect(generator.PELEC, turbine.PELEC) annotation (Line(points={{-69.2,-5.2},{-74,-5.2},{-74,-40},{-128,-40},{-128,0.1},{-114.9,0.1}}, color={0,0,127}));
      connect(const.y, turbine.FSR) annotation (Line(points={{-137.2,-10},{-115.1,-10},{-115.1,-10.1}}, color={0,0,127}));
      annotation (
        Diagram(graphics={Text(
    extent={{-112,68},{108,48}},
    lineColor={0,0,0},
    lineThickness=1,
    fillPattern=FillPattern.Solid,
    fontSize=15,
    textStyle={TextStyle.Bold},
    textString="(Constant Efd)"), Text(
    extent={{58,50},{112,38}},
    textColor={238,46,47},
    textString="Validated!!",
    textStyle={TextStyle.Bold})}),
        experiment(
          StopTime=100,
          Interval=0.02,
          __Dymola_fixedstepsize=0.02),
        __Dymola_experimentSetupOutput);
    end SMIB_TurbNoGov;
  end NoiseOnLoad;

  package OpenLoopTests
    "Open-Loop Tests on the GGov1 Turbine with the SMIB networks"
  extends Modelica.Icons.ExamplesPackage;

    package NoiseOnLoad "Models with stochastic load"

      model SMIB_TM "Open-Loop Test of the Single-Shaft Gas Turbine Model driving generator of the SMIB network"
        extends BaseClasses.SMIB.Partial.SMIB_Partial_Noise(
          transformer(V_b=13.8e3, Vn=13.8e3),
          LOAD(V_b=13.8e3),
          GEN1(V_b=13.8e3),
          BUS1(V_b=13.8e3),
          BUS2(V_b=13.8e3),
          BUS3(V_b=13.8e3),
          GEN2(V_b=13.8e3),
          sineNoiseInjection(freqHz=0.1, active_sigma=0),
          infiniteGen(
            V_b=13.8e3,
            P_0=machines.P3_1,
            Q_0=machines.Q3_1,
            v_0=voltages.V3,
            angle_0=voltages.A3,
            M_b=1000e6),
          variableLoad(
            d_P=0,
            t1=0,
            d_t=0,
            P_0=loads.PL23_1,
            Q_0=loads.QL23_1,
            v_0=voltages.V23,
            angle_0=voltages.A23),
          pMU(V_0=voltages.V23, angle_0=voltages.A23));
        import Modelica.Constants.pi;

        Modelica.Blocks.Sources.Step step(
          height=0.2,
          startTime=30,
          offset=0.483333) annotation (Placement(transformation(extent={{-138,4},{-124,18}})));
        Electrical.Generation_Groups.SMIB.Generator genGroup(
          V_b=13.8e3,
          M_b=10e6,
          Q_0=machines.Q1_1,
          P_0=machines.P1_1,
          v_0=voltages.V1,
          angle_0=voltages.A1)
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
        BaseClasses.SMIB.Records.Machines machines
          annotation (Placement(transformation(extent={{-138,50},{-118,70}})));
        BaseClasses.SMIB.Records.Loads loads
          annotation (Placement(transformation(extent={{-116,50},{-96,70}})));
        BaseClasses.SMIB.Records.Voltages voltages
          annotation (Placement(transformation(extent={{-94,50},{-74,70}})));
        Electrical.Controls.TG.GGOV1.Simplified.GTPlant_GGov1_PU gt(
          Kturb=1.5,
          Teng=0,
          Dm=0,
          Tb=0.14101,
          Tc=0.11514)
          annotation (Placement(transformation(extent={{-110,-10},{-82,10}})));
      equation
        connect(GEN1.p, genGroup.pwPin) annotation (Line(points={{-42,0},{-49,0}}, color={0,0,255}));
        connect(genGroup.Pmech, gt.Pmech) annotation (Line(points={{-70.6,0},{-81.4,0}}, color={0,0,127}));
        connect(genGroup.speed, gt.speedRef) annotation (Line(points={{-64.4,-9.8},{-64.4,-28},{-120,-28},{-120,-4},{-110.4,-4}}, color={0,0,127}));
        connect(step.y, gt.valvePosition) annotation (Line(points={{-123.3,11},{-118,11},{-118,4},{-110.4,4}}, color={0,0,127}));
        annotation (
          Diagram(graphics={Text(
                extent={{-112,68},{108,48}},
                lineColor={0,0,0},
                lineThickness=1,
                fillPattern=FillPattern.Solid,
                fontSize=15,
                textStyle={TextStyle.Bold},
                textString="(Constant Efd)"),
          Text( extent={{54,50},{108,38}},
                textColor={238,46,47},
                textString="Validated!!",
                textStyle={TextStyle.Bold})}),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end SMIB_TM;
    end NoiseOnLoad;

    package NoNoiseOnLoad "Models with non stochastic load"

      model SMIB_TM "Open-Loop Test of the Single-Shaft Gas Turbine Model driving generator of the SMIB network"
        extends BaseClasses.SMIB.Partial.SMIB_Partial_NoNoise(
          transformer(V_b=13.8e3, Vn=13.8e3),
          LOAD(V_b=13.8e3),
          GEN1(V_b=13.8e3),
          BUS1(V_b=13.8e3),
          BUS2(V_b=13.8e3),
          BUS3(V_b=13.8e3),
          GEN2(V_b=13.8e3),
          infiniteGen(
            V_b=13.8e3,
            M_b=1000e6,
            P_0=pf_results.machines.P3_1,
            Q_0=pf_results.machines.Q3_1,
            v_0=pf_results.voltages.V3,
            angle_0=pf_results.voltages.A3),
          variableLoad(
            d_P=0,
            t1=0,
            d_t=0,
            P_0=pf_results.loads.PL23_1,
            Q_0=pf_results.loads.QL23_1,
            v_0=pf_results.voltages.V23,
            angle_0=pf_results.voltages.A23),
          pMU(V_0=pf_results.voltages.V23, angle_0=pf_results.voltages.A23));
        import Modelica.Constants.pi;

        Electrical.Generation_Groups.SMIB.Generator genGroup(
          V_b=13.8e3,
          M_b=10e6,
          Q_0=pf_results.machines.Q1_1,
          P_0=pf_results.machines.P1_1,
          v_0=pf_results.voltages.V1,
          angle_0=pf_results.voltages.A1)
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
        Modelica.Blocks.Sources.Step step(
          height=0.2,
          startTime=30,
          offset=0.483333) annotation (Placement(transformation(extent={{-138,4},{-124,18}})));
        Electrical.Controls.TG.GGOV1.Simplified.GTPlant_GGov1_PU gt(
          Kturb=1.5,
          Teng=0,
          Dm=0,
          Tb=0.14101,
          Tc=0.11514)
          annotation (Placement(transformation(extent={{-110,-10},{-82,10}})));
        BaseClasses.SMIB.Records.PF_050 pf_results
          annotation (Placement(transformation(extent={{-136,50},{-116,70}})));
      equation
        connect(GEN1.p, genGroup.pwPin) annotation (Line(points={{-42,0},{-49,0}}, color={0,0,255}));
        connect(genGroup.Pmech, gt.Pmech) annotation (Line(points={{-70.6,0},{-70.6,0},{-81.4,0}}, color={0,0,127}));
        connect(genGroup.speed, gt.speedRef) annotation (Line(points={{-64.4,-9.8},{-64.4,-28},{-120,-28},{-120,-4},{-110.4,-4}}, color={0,0,127}));
        connect(step.y, gt.valvePosition) annotation (Line(points={{-123.3,11},{-118,11},{-118,4},{-110.4,4}}, color={0,0,127}));
        annotation (
          Diagram(graphics={Text(
                extent={{-112,68},{108,48}},
                lineColor={0,0,0},
                lineThickness=1,
                fillPattern=FillPattern.Solid,
                fontSize=15,
                textStyle={TextStyle.Bold},
                textString="(Constant Efd)"),
          Text( extent={{54,56},{108,44}},
                textColor={238,46,47},
                textString="Validated!!",
                textStyle={TextStyle.Bold})}),
          experiment(StopTime=100),
          __Dymola_experimentSetupOutput);
      end SMIB_TM;
    end NoNoiseOnLoad;
  end OpenLoopTests;
end SMIB;
