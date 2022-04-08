within MicroGrid.Optimization.MPComponents;
model Breaker_MPC "Circuit breaker with time or signal control"

  parameter OpenIPSL.Types.Time t_o = 10 "Opening time"
    annotation (Dialog(enable=not enableTrigger));
     parameter OpenIPSL.Types.Time t_f = 20 "Opening time"
    annotation (Dialog(enable=not enableTrigger));

  OpenIPSL.Interfaces.PwPin s "Sending pin" annotation (Placement(
        transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent=
           {{-110,-10},{-90,10}})));
  OpenIPSL.Interfaces.PwPin r "Receiving pin" annotation (Placement(
        transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{
            90,-10},{110,10}})));

  Complex vs(re=s.vr, im=s.vi);
  Complex vr(re=r.vr, im=r.vi);
  Complex is(re=s.ir, im=s.ii);
  Complex ir(re=r.ir, im=r.ii);

equation

   if time >= t_o then
    is = Complex(0);
    ir = Complex(0);
   elseif time >= t_f then
    vs = vr;
    is = -ir;
   else
    vs = vr;
    is = -ir;
   end if;

  annotation (
    Icon(graphics={Rectangle(
          extent={{-40,40},{40,-40}},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None), Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-90,2},{-44,2}}, color={0,0,255}),
        Ellipse(extent={{-44,6},{-36,-2}}, lineColor={0,0,255}),
        Line(points={{-37,4},{40,42}}, color={0,0,255}),
        Line(points={{40,2},{90,2}}, color={0,0,255}),
        Line(points={{40,22},{40,2}}, color={0,0,255}),
        Text(
          extent={{-150,90},{150,50}},
          textString="%name",
          lineColor={0,0,255})}),
    Documentation(info="<html>
<p>This is an <strong>opening</strong> circuit breaker which can either be parametrised with an opening and closing time or controlled via an external trigger. If the external trigger is active (i.e.,  <code>Trigger=true</code>)> then the circuit breaker is open.</p>
</html>"));
end Breaker_MPC;
