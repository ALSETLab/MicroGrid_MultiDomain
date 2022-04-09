within MicroGrid.Electrical.Renewables.WECC;
package GridForming
  model GFM_REGC "Generic grid forming renewable generator/converter model."
    extends OpenIPSL.Electrical.Essentials.pfComponent(
      final enabledisplayPF=false,
      final enableangle_0=true,
      final enablev_0=true,
      final enableQ_0=true,
      final enableP_0=true,
      final enablefn=false,
      final enableV_b=false,
      final enableS_b=false);

  parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "IBR rating" annotation (Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.PerUnit Rf = 0.0015 "Filter resistance" annotation (Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.PerUnit Xf = 0.15 "Filter reactance"
                                                               annotation (Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.PerUnit Imax = 1.2 "Maximum current magnitude" annotation (Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.AngularVelocity dwmax =  75 "Maximum value of frequency deviation" annotation(Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.AngularVelocity dwmin = -75 "Minimum value of frequency deviation" annotation(Dialog(group= "Input Parameters"));
  parameter Real wdrp = 0.033 "Frequeny droop percent" annotation(Dialog(group= "Input Parameters"));
  parameter Real Qdrp = 0.045 "Voltage droop percent" annotation(Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.Time Tr=0.005 "Transducer time constant"
                                                                   annotation (Dialog(group= "Input Parameters"));
  parameter OpenIPSL.Types.Time Te=0.01 "Output state time constant "
                                                                     annotation (Dialog(group= "Input Parameters"));
  parameter Real mf = 0.15 "VSM inertia constant" annotation(Dialog(group= "Input Parameters"));
  parameter Real dd = 0.11 "VSM damping factor" annotation(Dialog(group= "Input Parameters"));
  parameter Real Kppll = 20 "PLL proportional gain" annotation(Dialog(group= "Input Parameters"));
  parameter Real Kipll = 700 "PLL integral gain" annotation(Dialog(group= "Input Parameters"));
  parameter Real Kpi = 0.5 "Current control proportional gain" annotation(Dialog(group= "Input Parameters"));
  parameter Real Kii = 20 "Current control integral gain" annotation(Dialog(group= "Input Parameters"));

  parameter Integer GFM = 0 "Grid Forming Method: (0) PLL, (1) Droop, (2) VSM, (3) DVOC" annotation (choices(choice=0, choice=1, choice=2, choice=3));

    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag2(K=1, T=Tr)
      annotation (Placement(transformation(extent={{-152,-126},{-132,-106}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag3(K=1, T=Tr)
      annotation (Placement(transformation(extent={{-152,-156},{-132,-136}})));
    Modelica.Blocks.Math.Add3 add3_1(k3=-1)
      annotation (Placement(transformation(extent={{-88,-118},{-68,-98}})));
    Modelica.Blocks.Sources.RealExpression PREF
      annotation (Placement(transformation(extent={{-124,-118},{-104,-98}})));
    Modelica.Blocks.Interfaces.RealInput Paux "Connector of Real input signal 2"
      annotation (Placement(transformation(extent={{-220,-50},{-200,-30}}),
          iconTransformation(extent={{-220,-50},{-200,-30}})));
    Modelica.Blocks.Math.Add3 add3_2(k3=-1)
      annotation (Placement(transformation(extent={{-88,-144},{-68,-164}})));
    Modelica.Blocks.Math.Gain gain2(k=K1)
      annotation (Placement(transformation(extent={{-58,-118},{-38,-98}})));
    Modelica.Blocks.Math.Gain gain3(k=K2)
      annotation (Placement(transformation(extent={{-58,-164},{-38,-144}})));
    Modelica.Blocks.Sources.RealExpression QREF
      annotation (Placement(transformation(extent={{-124,-144},{-104,-164}})));
    Modelica.Blocks.Interfaces.RealInput Qaux "Connector of Real input signal 1"
      annotation (Placement(transformation(extent={{-220,-100},{-200,-80}}),
          iconTransformation(extent={{-220,-100},{-200,-80}})));
    Modelica.Blocks.Math.Add add6(k1=-1)
      annotation (Placement(transformation(extent={{38,-38},{58,-18}})));
    Modelica.Blocks.Math.Add add7
      annotation (Placement(transformation(extent={{38,-38},{58,-58}})));
    Modelica.Blocks.Sources.RealExpression Vq(y=Vxy2Vdq.q_output)
      annotation (Placement(transformation(extent={{-172,-62},{-152,-42}})));
    Modelica.Blocks.Math.Gain gain4(k=Kppll)
      annotation (Placement(transformation(extent={{-136,-46},{-116,-26}})));
    Modelica.Blocks.Continuous.Integrator integrator2(k=Kipll)
      annotation (Placement(transformation(extent={{-136,-58},{-116,-78}})));
    Modelica.Blocks.Math.Add add8
      annotation (Placement(transformation(extent={{-106,-64},{-86,-44}})));
    Modelica.Blocks.Math.Gain gain5(k=Kd)
      annotation (Placement(transformation(extent={{-72,-64},{-52,-44}})));
    Modelica.Blocks.Math.Add add9
      annotation (Placement(transformation(extent={{68,-48},{88,-28}})));
    Modelica.Blocks.Sources.RealExpression WREF(y=2*Modelica.Constants.pi*fn)
      annotation (Placement(transformation(extent={{6,-32},{26,-52}})));
    Modelica.Blocks.Continuous.Integrator integrator10(k=1/Tf)
      annotation (Placement(transformation(extent={{106,-48},{126,-28}})));
    Modelica.Blocks.Logical.Switch WFLAG_switch
      annotation (Placement(transformation(extent={{38,2},{58,22}})));

    Modelica.Blocks.Sources.BooleanExpression WFLAG(y=if GFM == 0 then True else
          False)
      annotation (Placement(transformation(extent={{6,22},{26,2}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=dwmax, uMin=dwmin)
      annotation (Placement(transformation(extent={{74,2},{94,22}})));
    Modelica.Blocks.Continuous.Integrator theta_inv(y_start=angle_0)
      annotation (Placement(transformation(extent={{118,2},{138,22}})));
    Modelica.Blocks.Math.Add add10
      annotation (Placement(transformation(extent={{38,-66},{58,-86}})));
    Modelica.Blocks.Continuous.Integrator VREF_INV_d(k=1/Tv)
      annotation (Placement(transformation(extent={{76,-86},{96,-66}})));
    Modelica.Blocks.Interfaces.RealOutput vref_inv_d
      "Connector of Real output signal"
      annotation (Placement(transformation(extent={{200,-120},{220,-100}}),
          iconTransformation(extent={{200,-120},{220,-100}})));
    Modelica.Blocks.Interfaces.RealOutput Qgen
      annotation (Placement(transformation(extent={{200,30},{220,50}})));
    Modelica.Blocks.Sources.RealExpression APower(y=Active_Power.y)
      annotation (Placement(transformation(extent={{174,80},{194,100}})));
    Modelica.Blocks.Sources.RealExpression RPower(y=Reactive_Power.y)
      annotation (Placement(transformation(extent={{174,30},{194,50}})));
    Modelica.Blocks.Math.Gain gain6(k=K2dvoc)
      annotation (Placement(transformation(extent={{142,-110},{122,-90}})));
    Modelica.Blocks.Logical.Switch WFLAG_switch1
      annotation (Placement(transformation(extent={{82,-118},{62,-98}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=1) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={72,-128})));
    Modelica.Blocks.Sources.BooleanExpression WFlag(y=if GFM == 3 then True else
          False)
      annotation (Placement(transformation(extent={{110,-118},{90,-98}})));
    Modelica.Blocks.Sources.RealExpression Const(y=if GFM == 0 then 0 else 1)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={72,-136})));
    Modelica.Blocks.Math.Product product1 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={40,-114})));
    Modelica.Blocks.Math.Product product2 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={16,-96})));
    Modelica.Blocks.Sources.RealExpression VREF(y=v_0)
                                                annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={186,-190})));
    Modelica.Blocks.Math.Add add11(k1=-1) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={162,-170})));
    Modelica.Blocks.Sources.RealExpression Vref_INV_d(y=VREF_INV_d.y)
      annotation (Placement(transformation(extent={{98,-180},{118,-200}})));
    Modelica.Blocks.Math.Add add12(k1=+1) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={134,-170})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=1) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={62,-186})));
    Modelica.Blocks.Logical.Switch WFLAG_switch2
      annotation (Placement(transformation(extent={{72,-176},{52,-156}})));
    Modelica.Blocks.Sources.BooleanExpression WFlag1(y=if GFM == 3 then True
           else False)
      annotation (Placement(transformation(extent={{110,-180},{90,-160}})));
    Modelica.Blocks.Math.Product product3 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={24,-154})));

    OpenIPSL.Interfaces.PwPin p(
      vr(start=vr0),
      vi(start=vi0),
      ir(start=ir0_Sb),
      ii(start=ii0_Sb))
      annotation (Placement(transformation(extent={{190,-10},{210,10}})));
    Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2 Ixy2Idq
      annotation (Placement(transformation(extent={{-156,164},{-136,184}})));
    Modelica.Blocks.Sources.RealExpression Ix(y=p.ir)
      annotation (Placement(transformation(extent={{-184,170},{-164,190}})));
    Modelica.Blocks.Sources.RealExpression Iy(y=p.ii)
      annotation (Placement(transformation(extent={{-184,178},{-164,158}})));
    Modelica.Blocks.Sources.RealExpression angle_inv(y=theta_inv.y)
      annotation (Placement(transformation(extent={{-170,138},{-150,158}})));
    Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2 Vxy2Vdq
      annotation (Placement(transformation(extent={{-84,164},{-64,184}})));
    Modelica.Blocks.Sources.RealExpression Vx(y=p.vr)
      annotation (Placement(transformation(extent={{-112,170},{-92,190}})));
    Modelica.Blocks.Sources.RealExpression Vy(y=p.vi)
      annotation (Placement(transformation(extent={{-112,178},{-92,158}})));
    Modelica.Blocks.Interfaces.RealInput Iq_ref annotation (Placement(
          transformation(extent={{-220,80},{-200,100}}),
          iconTransformation(extent={{-220,80},{-200,100}})));
    Modelica.Blocks.Interfaces.RealInput Id_ref annotation (Placement(
          transformation(extent={{-220,30},{-200,50}}),
          iconTransformation(extent={{-220,30},{-200,50}})));
    Modelica.Blocks.Sources.RealExpression Active_Power(y=-(1/CoB)*(p.vr*p.ir + p.vi
          *p.ii))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-172,-116})));
    Modelica.Blocks.Sources.RealExpression Reactive_Power(y=-(1/CoB)*(p.vi*p.ir -
          p.vr*p.ii))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-172,-146})));
    Modelica.Blocks.Interfaces.RealOutput Pgen "Value of Real output" annotation (
       Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={210,90}),  iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={210,110})));
    Modelica.Blocks.Interfaces.RealOutput dwinv "Value of Real output"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={210,-40}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={210,-40})));
    Modelica.Blocks.Math.Add add(k2=-1)
      annotation (Placement(transformation(extent={{-120,68},{-100,88}})));
    Modelica.Blocks.Sources.RealExpression Iq(y=Ixy2Idq.q_output)
      annotation (Placement(transformation(extent={{-148,82},{-128,62}})));
    Modelica.Blocks.Continuous.Integrator integrator(k=Kii, y_start=Exy2Edq_init.q_output
           - (Vxy2Vdq_init.q_output + Ixy2Idq_init.q_output*Rf + Ixy2Idq_init.d_output
          *Xf))
      annotation (Placement(transformation(extent={{-92,72},{-72,52}})));
    Modelica.Blocks.Math.Gain gain(k=Kip)
      annotation (Placement(transformation(extent={{-92,88},{-72,108}})));
    Modelica.Blocks.Math.Add add1(k2=+1)
      annotation (Placement(transformation(extent={{-64,84},{-44,104}})));
    Modelica.Blocks.Math.Add add2(k2=+1)
      annotation (Placement(transformation(extent={{-32,84},{-12,104}})));
    Modelica.Blocks.Sources.RealExpression Eq(y=Vxy2Vdq.q_output + Ixy2Idq.q_output
          *Rf + Ixy2Idq.d_output*Xf)
      annotation (Placement(transformation(extent={{-64,86},{-44,66}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(
      K=1,
      T=Te,
      y_start=Exy2Edq_init.q_output)
      annotation (Placement(transformation(extent={{8,84},{28,104}})));
    Logic_Blocks.REGCC_Logic_Blocks.aB_conversion Exy
      annotation (Placement(transformation(extent={{100,78},{120,98}})));
    Modelica.Blocks.Sources.RealExpression theta_inv2(y=theta_inv.y)
      annotation (Placement(transformation(extent={{80,40},{100,60}})));
    Modelica.Blocks.Math.Add add3(k2=-1)
      annotation (Placement(transformation(extent={{-120,0},{-100,20}})));
    Modelica.Blocks.Sources.RealExpression Id(y=Ixy2Idq.d_output)
      annotation (Placement(transformation(extent={{-148,14},{-128,-6}})));
    Modelica.Blocks.Continuous.Integrator integrator1(k=Kii, y_start=
          Exy2Edq_init.d_output - (Vxy2Vdq_init.d_output + Ixy2Idq_init.d_output
          *Rf - Ixy2Idq_init.q_output*Xf))
      annotation (Placement(transformation(extent={{-92,4},{-72,-16}})));
    Modelica.Blocks.Math.Gain gain1(k=Kip)
      annotation (Placement(transformation(extent={{-92,16},{-72,36}})));
    Modelica.Blocks.Math.Add add4(k2=+1)
      annotation (Placement(transformation(extent={{-64,0},{-44,20}})));
    Modelica.Blocks.Math.Add add5(k2=+1)
      annotation (Placement(transformation(extent={{-32,64},{-12,44}})));
    Modelica.Blocks.Sources.RealExpression Ed(y=Vxy2Vdq.d_output + Ixy2Idq.d_output
          *Rf - Ixy2Idq.q_output*Xf)
      annotation (Placement(transformation(extent={{-64,70},{-44,50}})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag1(
      K=1,
      T=Te,
      y_start=Exy2Edq_init.d_output)
      annotation (Placement(transformation(extent={{8,54},{28,74}})));
    Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2 Exy2Edq_init
      annotation (Placement(transformation(extent={{160,162},{180,182}})));
    Modelica.Blocks.Sources.RealExpression Ex_0(y=Ex0)
      annotation (Placement(transformation(extent={{132,168},{152,188}})));
    Modelica.Blocks.Sources.RealExpression Ey_0(y=Ey0)
      annotation (Placement(transformation(extent={{132,176},{152,156}})));
    Modelica.Blocks.Sources.RealExpression Theta_init3(y=angle_0)
      annotation (Placement(transformation(extent={{142,136},{162,156}})));
    Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2 Ixy2Idq_init
      annotation (Placement(transformation(extent={{12,164},{32,184}})));
    Modelica.Blocks.Sources.RealExpression Ix_0(y=ir0_Mb)
      annotation (Placement(transformation(extent={{-16,170},{4,190}})));
    Modelica.Blocks.Sources.RealExpression Iy_0(y=p.ii)
      annotation (Placement(transformation(extent={{-16,178},{4,158}})));
    Modelica.Blocks.Sources.RealExpression Theta_init1(y=angle_0)
      annotation (Placement(transformation(extent={{-2,138},{18,158}})));
    Logic_Blocks.REGCC_Logic_Blocks.dq_conversion2 Vxy2Vdq_init
      annotation (Placement(transformation(extent={{86,164},{106,184}})));
    Modelica.Blocks.Sources.RealExpression Vx_0(y=p.vr)
      annotation (Placement(transformation(extent={{58,170},{78,190}})));
    Modelica.Blocks.Sources.RealExpression Vy_0(y=p.vi)
      annotation (Placement(transformation(extent={{58,178},{78,158}})));
    Modelica.Blocks.Sources.RealExpression Theta_init2(y=angle_0)
      annotation (Placement(transformation(extent={{68,138},{88,158}})));

  protected
    parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
    parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
    parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0) "Inverter Terminal Initial real voltage";
    parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0) "Inverter Terminal Initial imaginary voltage";
    parameter OpenIPSL.Types.PerUnit Ex0 = vr0 + Rf*ir0_Mb - Xf*ii0_Mb "Initial imaginary droop voltage";
    parameter OpenIPSL.Types.PerUnit Ey0 = vi0 + Rf*ii0_Mb + Xf*ir0_Mb "Initial real droop voltage";
    parameter OpenIPSL.Types.PerUnit ir0_Sb = -CoB*(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2) "Inverter Terminal Initial real current on System Rating";
    parameter OpenIPSL.Types.PerUnit ii0_Sb = -CoB*(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2) "Inverter Terminal Initial imaginary current on System Rating";
    parameter OpenIPSL.Types.PerUnit ir0_Mb = (p0*vr0 + q0*vi0)/(vr0^2 + vi0^2) "Inverter Terminal Initial real current on Source Rating";
    parameter OpenIPSL.Types.PerUnit ii0_Mb = (p0*vi0 - q0*vr0)/(vr0^2 + vi0^2) "Inverter Terminal Initial imaginary current on Source Rating";
    parameter Real CoB=M_b/S_b "Base Conversion ratio";
    parameter OpenIPSL.Types.AngularVelocity w0 = 2*Modelica.Constants.pi*fn "Initial angular speed";
    parameter OpenIPSL.Types.Time Tf = if GFM == 0 then 1 elseif GFM == 1 then 0    elseif GFM == 2 then mf*wdrp else 0;
    parameter OpenIPSL.Types.Time Tv = if GFM == 0 then 1 elseif GFM == 1 then 0    elseif GFM == 2 then 0       else 1/w0;
    parameter Real Kd =                if GFM == 0 then 0 elseif GFM == 1 then 0    elseif GFM == 2 then dd*wdrp else 0;
    parameter Real K1 =                if GFM == 0 then 1 elseif GFM == 1 then wdrp elseif GFM == 2 then wdrp    else wdrp;
    parameter Real K2 =                if GFM == 0 then 1 elseif GFM == 1 then Qdrp elseif GFM == 2 then Qdrp    else wdrp;
    parameter Real K2dvoc =            if GFM == 0 then 1 elseif GFM == 1 then 1    elseif GFM == 2 then 1       else 4*100^4/(100^4 - (2*(100 - 100*Qdrp)^2-100^2)^2);

  equation

  // Inveter Terminal Current Calculations
  p.ir = -CoB*( Exy.real_output*Rf + Exy.imaginary_output*Xf - p.vr*Rf - p.vi*Xf)/(Rf^2 + Xf^2);
  p.ii = -CoB*(-Exy.real_output*Xf + Exy.imaginary_output*Rf + p.vr*Xf - p.vi*Rf)/(Rf^2 + Xf^2);

    connect(Ix.y, Ixy2Idq.real_input)
      annotation (Line(points={{-163,180},{-158,180}}, color={0,0,127}));
    connect(Iy.y, Ixy2Idq.imaginary_input)
      annotation (Line(points={{-163,168},{-158,168}}, color={0,0,127}));
    connect(angle_inv.y, Ixy2Idq.angle) annotation (Line(points={{-149,148},{-146,
            148},{-146,162}}, color={0,0,127}));
    connect(Vx.y, Vxy2Vdq.real_input)
      annotation (Line(points={{-91,180},{-86,180}},
                                                  color={0,0,127}));
    connect(Vy.y, Vxy2Vdq.imaginary_input)
      annotation (Line(points={{-91,168},{-86,168}},
                                                  color={0,0,127}));
    connect(add.u1, Iq_ref)
      annotation (Line(points={{-122,84},{-180,84},{-180,90},{-210,90}},
                                                     color={0,0,127}));
    connect(Iq.y, add.u2)
      annotation (Line(points={{-127,72},{-122,72}}, color={0,0,127}));
    connect(add.y, integrator.u) annotation (Line(points={{-99,78},{-98,78},{-98,62},
            {-94,62}},     color={0,0,127}));
    connect(gain.u, add.y) annotation (Line(points={{-94,98},{-98,98},{-98,78},{-99,
            78}},      color={0,0,127}));
    connect(gain.y, add1.u1)
      annotation (Line(points={{-71,98},{-68,98},{-68,100},{-66,100}},
                                                            color={0,0,127}));
    connect(integrator.y, add1.u2)
      annotation (Line(points={{-71,62},{-66,62},{-66,88}}, color={0,0,127}));
    connect(add1.y, add2.u1)
      annotation (Line(points={{-43,94},{-38,94},{-38,100},{-34,100}},
                                                   color={0,0,127}));
    connect(Eq.y, add2.u2) annotation (Line(points={{-43,76},{-40,76},{-40,88},{-34,
            88}}, color={0,0,127}));
    connect(add2.y, simpleLag.u)
      annotation (Line(points={{-11,94},{6,94}},  color={0,0,127}));
    connect(simpleLag.y, Exy.q_input) annotation (Line(points={{29,94},{98,94}},
                                     color={0,0,127}));
    connect(theta_inv2.y, Exy.angle) annotation (Line(points={{101,50},{110,50},{110,
            76}},            color={0,0,127}));
    connect(Id.y, add3.u2)
      annotation (Line(points={{-127,4},{-122,4}},   color={0,0,127}));
    connect(add3.y, integrator1.u) annotation (Line(points={{-99,10},{-98,10},{-98,
            -6},{-94,-6}},                   color={0,0,127}));
    connect(gain1.u, add3.y) annotation (Line(points={{-94,26},{-98,26},{-98,10},{
            -99,10}},              color={0,0,127}));
    connect(gain1.y, add4.u1) annotation (Line(points={{-71,26},{-66,26},{-66,16}},
                      color={0,0,127}));
    connect(integrator1.y, add4.u2) annotation (Line(points={{-71,-6},{-68,-6},{-68,
            4},{-66,4}},         color={0,0,127}));
    connect(add4.y,add5. u1)
      annotation (Line(points={{-43,10},{-34,10},{-34,48}},
                                                   color={0,0,127}));
    connect(Ed.y, add5.u2) annotation (Line(points={{-43,60},{-34,60}},
                                 color={0,0,127}));
    connect(add5.y, simpleLag1.u)
      annotation (Line(points={{-11,54},{-2,54},{-2,64},{6,64}},
                                                   color={0,0,127}));
    connect(add3.u1, Id_ref) annotation (Line(points={{-122,16},{-180,16},{-180,40},
            {-210,40}},                  color={0,0,127}));
    connect(simpleLag1.y, Exy.d_input) annotation (Line(points={{29,64},{74,64},{74,
            82},{98,82}},              color={0,0,127}));
    connect(Ex_0.y, Exy2Edq_init.real_input)
      annotation (Line(points={{153,178},{158,178}}, color={0,0,127}));
    connect(Ey_0.y, Exy2Edq_init.imaginary_input)
      annotation (Line(points={{153,166},{158,166}}, color={0,0,127}));
    connect(Theta_init3.y, Exy2Edq_init.angle)
      annotation (Line(points={{163,146},{170,146},{170,160}}, color={0,0,127}));
    connect(Ix_0.y, Ixy2Idq_init.real_input)
      annotation (Line(points={{5,180},{10,180}},    color={0,0,127}));
    connect(Iy_0.y, Ixy2Idq_init.imaginary_input)
      annotation (Line(points={{5,168},{10,168}},    color={0,0,127}));
    connect(Theta_init1.y, Ixy2Idq_init.angle)
      annotation (Line(points={{19,148},{22,148},{22,162}},    color={0,0,127}));
    connect(Vx_0.y, Vxy2Vdq_init.real_input)
      annotation (Line(points={{79,180},{84,180}}, color={0,0,127}));
    connect(Vy_0.y, Vxy2Vdq_init.imaginary_input)
      annotation (Line(points={{79,168},{84,168}}, color={0,0,127}));
    connect(Theta_init2.y, Vxy2Vdq_init.angle)
      annotation (Line(points={{89,148},{96,148},{96,162}}, color={0,0,127}));
    connect(Reactive_Power.y, simpleLag3.u)
      annotation (Line(points={{-161,-146},{-154,-146}}, color={0,0,127}));
    connect(Active_Power.y, simpleLag2.u)
      annotation (Line(points={{-161,-116},{-154,-116}}, color={0,0,127}));
    connect(simpleLag2.y, add3_1.u3)
      annotation (Line(points={{-131,-116},{-90,-116}}, color={0,0,127}));
    connect(PREF.y, add3_1.u2)
      annotation (Line(points={{-103,-108},{-90,-108}},color={0,0,127}));
    connect(Paux, add3_1.u1) annotation (Line(points={{-210,-40},{-180,-40},{-180,
            -94},{-90,-94},{-90,-100}},            color={0,0,127}));
    connect(add3_1.y, gain2.u)
      annotation (Line(points={{-67,-108},{-60,-108}}, color={0,0,127}));
    connect(add3_2.y, gain3.u)
      annotation (Line(points={{-67,-154},{-60,-154}}, color={0,0,127}));
    connect(simpleLag3.y, add3_2.u3)
      annotation (Line(points={{-131,-146},{-90,-146}}, color={0,0,127}));
    connect(QREF.y, add3_2.u2)
      annotation (Line(points={{-103,-154},{-90,-154}},color={0,0,127}));
    connect(add3_2.u1, Qaux) annotation (Line(points={{-90,-162},{-100,-162},{-100,
            -168},{-200,-168},{-200,-90},{-210,-90}},
                                                  color={0,0,127}));
    connect(Vq.y, gain4.u) annotation (Line(points={{-151,-52},{-148,-52},{-148,-36},
            {-138,-36}}, color={0,0,127}));
    connect(integrator2.u, gain4.u) annotation (Line(points={{-138,-68},{-148,-68},
            {-148,-36},{-138,-36}}, color={0,0,127}));
    connect(gain4.y, add8.u1) annotation (Line(points={{-115,-36},{-108,-36},{-108,
            -48}}, color={0,0,127}));
    connect(integrator2.y, add8.u2) annotation (Line(points={{-115,-68},{-108,-68},
            {-108,-60}}, color={0,0,127}));
    connect(gain2.y, add7.u1)
      annotation (Line(points={{-37,-108},{-20,-108},{-20,-54},{36,-54}},
                                                      color={0,0,127}));
    connect(add6.y, add9.u1)
      annotation (Line(points={{59,-28},{66,-28},{66,-32}}, color={0,0,127}));
    connect(add7.y, add9.u2)
      annotation (Line(points={{59,-48},{66,-48},{66,-44}},   color={0,0,127}));
    connect(add8.y, gain5.u)
      annotation (Line(points={{-85,-54},{-74,-54}}, color={0,0,127}));
    connect(gain5.y, add6.u2) annotation (Line(points={{-51,-54},{-40,-54},{-40,-34},
            {36,-34}},color={0,0,127}));
    connect(WREF.y, add7.u2) annotation (Line(points={{27,-42},{36,-42}},
                      color={0,0,127}));
    connect(add9.y, integrator10.u)
      annotation (Line(points={{89,-38},{104,-38}}, color={0,0,127}));
    connect(integrator10.y, add6.u1) annotation (Line(points={{127,-38},{142,-38},
            {142,-12},{30,-12},{30,-22},{36,-22}}, color={0,0,127}));
    connect(WFLAG_switch.u3, add6.u1) annotation (Line(points={{36,4},{30,4},{30,-22},
            {36,-22}}, color={0,0,127}));
    connect(WFLAG_switch.u1, gain5.u) annotation (Line(points={{36,20},{-28,20},{-28,
            -26},{-78,-26},{-78,-54},{-74,-54}}, color={0,0,127}));
    connect(WFLAG.y, WFLAG_switch.u2)
      annotation (Line(points={{27,12},{36,12}}, color={255,0,255}));
    connect(WFLAG_switch.y, limiter.u)
      annotation (Line(points={{59,12},{72,12}}, color={0,0,127}));
    connect(limiter.y, theta_inv.u)
      annotation (Line(points={{95,12},{116,12}}, color={0,0,127}));
    connect(Vxy2Vdq.angle, Ixy2Idq.angle) annotation (Line(points={{-74,162},{-74,
            148},{-146,148},{-146,162}}, color={0,0,127}));
    connect(dwinv, theta_inv.u) annotation (Line(points={{210,-40},{160,
            -40},{160,-6},{108,-6},{108,12},{116,12}},
                                             color={0,0,127}));
    connect(gain3.y, add10.u2) annotation (Line(points={{-37,-154},{-2,-154},{-2,-70},
            {36,-70}}, color={0,0,127}));
    connect(add10.y, VREF_INV_d.u)
      annotation (Line(points={{59,-76},{74,-76}}, color={0,0,127}));
    connect(VREF_INV_d.y, vref_inv_d) annotation (Line(points={{97,-76},{
            160,-76},{160,-110},{210,-110}},
                                  color={0,0,127}));
    connect(APower.y, Pgen)
      annotation (Line(points={{195,90},{210,90}}, color={0,0,127}));
    connect(RPower.y, Qgen)
      annotation (Line(points={{195,40},{210,40}}, color={0,0,127}));
    connect(gain6.u, vref_inv_d) annotation (Line(points={{144,-100},{160,
            -100},{160,-110},{210,-110}},
                             color={0,0,127}));
    connect(gain6.y, WFLAG_switch1.u1)
      annotation (Line(points={{121,-100},{84,-100}}, color={0,0,127}));
    connect(realExpression.y, WFLAG_switch1.u3) annotation (Line(points={{83,-128},
            {94,-128},{94,-116},{84,-116}}, color={0,0,127}));
    connect(WFLAG_switch1.u2, WFlag.y)
      annotation (Line(points={{84,-108},{89,-108}}, color={255,0,255}));
    connect(Const.y, product1.u1)
      annotation (Line(points={{61,-136},{52,-136},{52,-120}}, color={0,0,127}));
    connect(WFLAG_switch1.y, product1.u2)
      annotation (Line(points={{61,-108},{52,-108}}, color={0,0,127}));
    connect(product2.y, add10.u1)
      annotation (Line(points={{16,-85},{16,-82},{36,-82}}, color={0,0,127}));
    connect(product1.y, product2.u2)
      annotation (Line(points={{29,-114},{22,-114},{22,-108}}, color={0,0,127}));
    connect(VREF.y, add11.u2) annotation (Line(points={{175,-190},{168,-190},{168,
            -182}}, color={0,0,127}));
    connect(Vref_INV_d.y, add11.u1) annotation (Line(points={{119,-190},{156,-190},
            {156,-182}}, color={0,0,127}));
    connect(add12.u2, add11.u2) annotation (Line(points={{140,-182},{140,-196},{168,
            -196},{168,-182}}, color={0,0,127}));
    connect(add12.u1, add11.u1) annotation (Line(points={{128,-182},{128,-190},{156,
            -190},{156,-182}}, color={0,0,127}));
    connect(WFLAG_switch2.u2, WFlag1.y) annotation (Line(points={{74,-166},{80,-166},
            {80,-170},{89,-170}}, color={255,0,255}));
    connect(add12.y, WFLAG_switch2.u1) annotation (Line(points={{134,-159},{134,-158},
            {74,-158}}, color={0,0,127}));
    connect(realExpression1.y, WFLAG_switch2.u3) annotation (Line(points={{73,-186},
            {84,-186},{84,-174},{74,-174}}, color={0,0,127}));
    connect(WFLAG_switch2.y, product3.u1) annotation (Line(points={{51,-166},{44,-166},
            {44,-160},{36,-160}}, color={0,0,127}));
    connect(add11.y, product3.u2) annotation (Line(points={{162,-159},{162,-148},{
            36,-148}}, color={0,0,127}));
    connect(product3.y, product2.u1)
      annotation (Line(points={{13,-154},{10,-154},{10,-108}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},
              {200,200}}), graphics={Rectangle(extent={{-200,200},{200,
                -200}}, lineColor={28,108,200}), Text(
            extent={{-180,40},{180,-40}},
            lineColor={28,108,200},
            textString="%name")}),                                 Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},{200,200}}),
          graphics={
          Rectangle(
            extent={{-188,194},{-130,132}},
            lineColor={255,0,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{-158,142},{-128,132}},
            lineColor={255,0,0},
            textString="Ixy-Idq"),
          Rectangle(
            extent={{-116,194},{-58,132}},
            lineColor={0,127,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{-90,142},{-60,132}},
            lineColor={0,127,0},
            textString="Vxy-Vdq"),
          Rectangle(
            extent={{128,194},{188,132}},
            lineColor={0,127,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{156,142},{186,132}},
            lineColor={0,127,0},
            textString="Exy0-Edq0"),
          Rectangle(
            extent={{-20,194},{38,132}},
            lineColor={255,0,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{10,146},{36,126}},
            lineColor={255,0,0},
            textString="Ixy0-Idq0"),
          Rectangle(
            extent={{54,194},{112,132}},
            lineColor={0,127,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{80,142},{110,130}},
            lineColor={0,127,0},
            textString="Vxy0-Vdq0")}));
  end GFM_REGC;

  model GFM_REEC "Generic grid forming renewable electrical control model."
  extends OpenIPSL.Electrical.Essentials.pfComponent(
      final enabledisplayPF=false,
      final enableangle_0=true,
      final enablev_0=true,
      final enableQ_0=true,
      final enableP_0=true,
      final enablefn=false,
      final enableV_b=false,
      final enableS_b=false);

  parameter OpenIPSL.Types.ApparentPower M_b = 100e6 "IBR rating" annotation (Dialog(group= "Input Parameters"));
  parameter Real wdrp = 0.033 "Frequeny droop percent" annotation(Dialog(group= "Input Parameters"));
  parameter Real Qdrp = 0.045 "Voltage droop percent" annotation(Dialog(group= "Input Parameters"));
  parameter Real KPv = if GFM == 0 then 0.5 else 3.0 "Voltage control proportional gain" annotation(Dialog(group= "Input Parameters"));
  parameter Real KIp = 20.0 "Active power integral gain" annotation(Dialog(group= "Input Parameters"));
  parameter Real KPp = 0.5  "Active power proportional gain" annotation(Dialog(group= "Input Parameters"));
  parameter Real KIv = if GFM == 0 then 150 else 10  "Voltage control integral gain" annotation(Dialog(group= "Input Parameters"));

  parameter Integer GFM = 0 "Grid Forming Method: (0) PLL, (1) Droop, (2) VSM, (3) DVOC" annotation (choices(choice=0, choice=1, choice=2, choice=3));
    Modelica.Blocks.Interfaces.RealInput Paux annotation (Placement(
          transformation(extent={{-220,-50},{-200,-30}}), iconTransformation(
            extent={{-220,-50},{-200,-30}})));
    Modelica.Blocks.Interfaces.RealInput vref_inv_d annotation (Placement(
          transformation(extent={{-220,-140},{-200,-120}}), iconTransformation(
            extent={{-220,-140},{-200,-120}})));
    Modelica.Blocks.Interfaces.RealInput p annotation (Placement(transformation(
            extent={{-220,-110},{-200,-90}}),iconTransformation(extent={{-220,
              -110},{-200,-90}})));
    Modelica.Blocks.Interfaces.RealInput dwinv annotation (Placement(
          transformation(extent={{-220,-80},{-200,-60}}),  iconTransformation(
            extent={{-220,-80},{-200,-60}})));
    Modelica.Blocks.Interfaces.RealInput v_inv_d annotation (Placement(
          transformation(extent={{-220,-170},{-200,-150}}), iconTransformation(
            extent={{-220,-170},{-200,-150}})));
    Modelica.Blocks.Interfaces.RealInput v_inv_q annotation (Placement(
          transformation(extent={{-220,40},{-200,60}}), iconTransformation(extent={{-220,30},
              {-200,50}})));
    Modelica.Blocks.Interfaces.RealInput q annotation (Placement(transformation(
            extent={{-220,90},{-200,110}}), iconTransformation(extent={{-220,90},
              {-200,110}})));
    Modelica.Blocks.Interfaces.RealInput Qaux annotation (Placement(
          transformation(extent={{-220,140},{-200,160}}), iconTransformation(
            extent={{-220,150},{-200,170}})));
    Modelica.Blocks.Math.Add add(k2=-1)
      annotation (Placement(transformation(extent={{-140,120},{-120,140}})));
    Modelica.Blocks.Math.Gain QDRP
      annotation (Placement(transformation(extent={{-106,120},{-86,140}})));
    Modelica.Blocks.Math.Add add1(k2=+1)
      annotation (Placement(transformation(extent={{-70,126},{-50,146}})));
    Modelica.Blocks.Math.Add add2(k1=-1, k2=+1)
      annotation (Placement(transformation(extent={{-70,96},{-50,76}})));
    Modelica.Blocks.Math.Add add3(k2=+1)
      annotation (Placement(transformation(extent={{-42,100},{-22,120}})));
    Modelica.Blocks.Sources.RealExpression VREF
      annotation (Placement(transformation(extent={{-140,82},{-120,102}})));
    Modelica.Blocks.Sources.RealExpression QREF
      annotation (Placement(transformation(extent={{-180,126},{-160,146}})));
    Modelica.Blocks.Math.Gain gain
      annotation (Placement(transformation(extent={{0,118},{20,138}})));
    Modelica.Blocks.Continuous.Integrator integrator
      annotation (Placement(transformation(extent={{0,84},{20,104}})));
    Modelica.Blocks.Math.Add add4(k2=+1)
      annotation (Placement(transformation(extent={{42,102},{62,122}})));
    Modelica.Blocks.Math.Division division
      annotation (Placement(transformation(extent={{-140,-116},{-120,-96}})));
    Modelica.Blocks.Sources.RealExpression WDRP
      annotation (Placement(transformation(extent={{-180,-102},{-160,-122}})));
    Modelica.Blocks.Logical.Switch switch1
      annotation (Placement(transformation(extent={{84,64},{104,84}})));
    Modelica.Blocks.Sources.BooleanExpression WFLAG
      annotation (Placement(transformation(extent={{46,64},{66,84}})));
    Modelica.Blocks.Nonlinear.Limiter limiter
      annotation (Placement(transformation(extent={{128,64},{148,84}})));
    Modelica.Blocks.Interfaces.RealOutput Iqref "Connector of Real output signal"
      annotation (Placement(transformation(extent={{200,60},{220,80}})));
    Modelica.Blocks.Sources.RealExpression VINV_D(y=v_inv_d)
      annotation (Placement(transformation(extent={{-140,90},{-120,70}})));
    Modelica.Blocks.Math.Gain NEG_ONE(k=-1)
      annotation (Placement(transformation(extent={{-106,30},{-86,50}})));
    Modelica.Blocks.Math.Gain gain1
      annotation (Placement(transformation(extent={{0,46},{20,66}})));
    Modelica.Blocks.Continuous.Integrator integrator1
      annotation (Placement(transformation(extent={{0,14},{20,34}})));
    Modelica.Blocks.Math.Add add5(k2=+1)
      annotation (Placement(transformation(extent={{42,30},{62,50}})));
    Modelica.Blocks.Math.Add add6(k2=-1)
      annotation (Placement(transformation(extent={{-140,-156},{-120,-136}})));
    Modelica.Blocks.Math.Gain gain2
      annotation (Placement(transformation(extent={{-94,-140},{-74,-120}})));
    Modelica.Blocks.Continuous.Integrator integrator2
      annotation (Placement(transformation(extent={{-94,-176},{-74,-156}})));
    Modelica.Blocks.Math.Add add7(k2=+1)
      annotation (Placement(transformation(extent={{-60,-158},{-40,-138}})));
    Modelica.Blocks.Logical.Switch switch2
      annotation (Placement(transformation(extent={{84,-108},{104,-88}})));
    Modelica.Blocks.Sources.BooleanExpression WFLAG1
      annotation (Placement(transformation(extent={{46,-108},{66,-88}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1
      annotation (Placement(transformation(extent={{128,-108},{148,-88}})));
    Modelica.Blocks.Math.Add add8(k2=+1)
      annotation (Placement(transformation(extent={{-100,-44},{-80,-24}})));
    Modelica.Blocks.Math.Add add9(k1=-1, k2=-1)
      annotation (Placement(transformation(extent={{-100,-86},{-80,-66}})));
    Modelica.Blocks.Sources.RealExpression PREF
      annotation (Placement(transformation(extent={{-180,-38},{-160,-18}})));
    Modelica.Blocks.Math.Add add10(k2=+1)
      annotation (Placement(transformation(extent={{-60,-66},{-40,-46}})));
    Modelica.Blocks.Math.Gain gain3
      annotation (Placement(transformation(extent={{0,-48},{20,-28}})));
    Modelica.Blocks.Continuous.Integrator integrator3
      annotation (Placement(transformation(extent={{0,-84},{20,-64}})));
    Modelica.Blocks.Math.Add add11(k2=+1)
      annotation (Placement(transformation(extent={{34,-66},{54,-46}})));
    Modelica.Blocks.Interfaces.RealOutput Idref
      "Connector of Real output signal" annotation (Placement(
          transformation(extent={{200,-80},{220,-60}})));
  equation
    connect(add.y, QDRP.u)
      annotation (Line(points={{-119,130},{-108,130}}, color={0,0,127}));
    connect(add1.y, add3.u1) annotation (Line(points={{-49,136},{-46,136},{-46,116},
            {-44,116}}, color={0,0,127}));
    connect(QDRP.y, add1.u2)
      annotation (Line(points={{-85,130},{-72,130}}, color={0,0,127}));
    connect(VREF.y, add2.u2)
      annotation (Line(points={{-119,92},{-72,92}}, color={0,0,127}));
    connect(QREF.y, add.u1)
      annotation (Line(points={{-159,136},{-142,136}}, color={0,0,127}));
    connect(gain.y, add4.u1) annotation (Line(points={{21,128},{38,128},{38,118},{
            40,118}}, color={0,0,127}));
    connect(integrator.y, add4.u2) annotation (Line(points={{21,94},{38,94},{38,106},
            {40,106}}, color={0,0,127}));
    connect(WDRP.y, division.u2)
      annotation (Line(points={{-159,-112},{-142,-112}}, color={0,0,127}));
    connect(integrator.u, add3.y) annotation (Line(points={{-2,94},{-12,94},{-12,110},
            {-21,110}}, color={0,0,127}));
    connect(gain.u, add3.y) annotation (Line(points={{-2,128},{-12,128},{-12,110},
            {-21,110}}, color={0,0,127}));
    connect(add2.y, add3.u2) annotation (Line(points={{-49,86},{-46,86},{-46,104},
            {-44,104}}, color={0,0,127}));
    connect(add4.y, switch1.u1) annotation (Line(points={{63,112},{72,112},{72,82},
            {82,82}}, color={0,0,127}));
    connect(WFLAG.y, switch1.u2)
      annotation (Line(points={{67,74},{82,74}}, color={255,0,255}));
    connect(q, add.u2) annotation (Line(points={{-210,100},{-176,100},{-176,124},{
            -142,124}}, color={0,0,127}));
    connect(Qaux, add1.u1) annotation (Line(points={{-210,150},{-80,150},{-80,142},
            {-72,142}}, color={0,0,127}));
    connect(switch1.y, limiter.u)
      annotation (Line(points={{105,74},{126,74}}, color={0,0,127}));
    connect(limiter.y, Iqref) annotation (Line(points={{149,74},{180,74},{180,70},
            {210,70}}, color={0,0,127}));
    connect(VINV_D.y, add2.u1)
      annotation (Line(points={{-119,80},{-72,80}}, color={0,0,127}));
    connect(v_inv_q, NEG_ONE.u) annotation (Line(points={{-210,50},{-160,50},{-160,
            40},{-108,40}}, color={0,0,127}));
    connect(gain1.y, add5.u1) annotation (Line(points={{21,56},{38,56},{38,46},{40,
            46}}, color={0,0,127}));
    connect(integrator1.y, add5.u2) annotation (Line(points={{21,24},{38,24},{38,34},
            {40,34}}, color={0,0,127}));
    connect(NEG_ONE.y, gain1.u) annotation (Line(points={{-85,40},{-20,40},{-20,56},
            {-2,56}}, color={0,0,127}));
    connect(integrator1.u, gain1.u) annotation (Line(points={{-2,24},{-20,24},{-20,
            56},{-2,56}}, color={0,0,127}));
    connect(add5.y, switch1.u3) annotation (Line(points={{63,40},{72,40},{72,66},{
            82,66}}, color={0,0,127}));
    connect(vref_inv_d, add6.u1) annotation (Line(points={{-210,-130},{-150,-130},
            {-150,-140},{-142,-140}}, color={0,0,127}));
    connect(v_inv_d, add6.u2) annotation (Line(points={{-210,-160},{-150,-160},{-150,
            -152},{-142,-152}}, color={0,0,127}));
    connect(add6.y, gain2.u) annotation (Line(points={{-119,-146},{-108,-146},{-108,
            -130},{-96,-130}}, color={0,0,127}));
    connect(integrator2.u, gain2.u) annotation (Line(points={{-96,-166},{-108,-166},
            {-108,-130},{-96,-130}}, color={0,0,127}));
    connect(gain2.y, add7.u1) annotation (Line(points={{-73,-130},{-62,-130},{-62,
            -142}}, color={0,0,127}));
    connect(integrator2.y, add7.u2) annotation (Line(points={{-73,-166},{-62,-166},
            {-62,-154}}, color={0,0,127}));
    connect(WFLAG1.y, switch2.u2)
      annotation (Line(points={{67,-98},{82,-98}}, color={255,0,255}));
    connect(switch2.y, limiter1.u)
      annotation (Line(points={{105,-98},{126,-98}}, color={0,0,127}));
    connect(add7.y, switch2.u3) annotation (Line(points={{-39,-148},{70,-148},{70,
            -106},{82,-106}}, color={0,0,127}));
    connect(dwinv, division.u1)
      annotation (Line(points={{-210,-70},{-176,-70},{-176,-100},{-142,
            -100}},                                      color={0,0,127}));
    connect(division.y, add9.u2) annotation (Line(points={{-119,-106},{-102,-106},
            {-102,-82}}, color={0,0,127}));
    connect(p, add9.u1)
      annotation (Line(points={{-210,-100},{-156,-100},{-156,-70},{-102,
            -70}},                                     color={0,0,127}));
    connect(Paux, add8.u2)
      annotation (Line(points={{-210,-40},{-156,-40},{-156,-40},{-102,-40}},
                                                       color={0,0,127}));
    connect(PREF.y, add8.u1)
      annotation (Line(points={{-159,-28},{-102,-28}}, color={0,0,127}));
    connect(add8.y, add10.u1)
      annotation (Line(points={{-79,-34},{-62,-34},{-62,-50}}, color={0,0,127}));
    connect(add9.y, add10.u2)
      annotation (Line(points={{-79,-76},{-62,-76},{-62,-62}}, color={0,0,127}));
    connect(gain3.y, add11.u1)
      annotation (Line(points={{21,-38},{32,-38},{32,-50}}, color={0,0,127}));
    connect(integrator3.y, add11.u2)
      annotation (Line(points={{21,-74},{32,-74},{32,-62}}, color={0,0,127}));
    connect(add10.y, gain3.u) annotation (Line(points={{-39,-56},{-20,-56},{-20,-38},
            {-2,-38}}, color={0,0,127}));
    connect(integrator3.u, gain3.u) annotation (Line(points={{-2,-74},{-20,-74},{-20,
            -38},{-2,-38}}, color={0,0,127}));
    connect(add11.y, switch2.u1) annotation (Line(points={{55,-56},{70,-56},{70,-90},
            {82,-90}}, color={0,0,127}));
    connect(limiter1.y, Idref) annotation (Line(points={{149,-98},{192,-98},
            {192,-70},{210,-70}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},
              {200,200}}), graphics={Rectangle(extent={{-200,200},{200,
                -200}}, lineColor={28,108,200}), Text(
            extent={{-180,40},{160,-40}},
            lineColor={28,108,200},
            textString="%name")}),                                 Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},{200,200}})));
  end GFM_REEC;
end GridForming;
