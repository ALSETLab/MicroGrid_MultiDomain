within MicroGrid.Examples.MPC;
model slip_eq

  parameter Modelica.Units.SI.PerUnit Rr1=0.01;
  parameter Modelica.Units.SI.PerUnit Rs=0.05;
  parameter Modelica.Units.SI.PerUnit Xs=0.001;
  parameter Modelica.Units.SI.PerUnit Xr1=0.005;
  parameter Modelica.Units.SI.PerUnit V=1;
  parameter Modelica.Units.SI.PerUnit ws=2*Modelica.Constants.pi*60;
  parameter Modelica.Units.SI.PerUnit Hm;
  Modelica.Blocks.Interfaces.RealInput Pm;
  Modelica.Units.SI.PerUnit Pe;
  Modelica.Units.SI.PerUnit s;
  Modelica.Units.SI.AngularVelocity wr;

protected
  parameter Modelica.Units.SI.PerUnit s0=0.01;
  parameter Modelica.Units.SI.AngularVelocity w0=310;

equation

  s = (ws - wr)/ws;
  der(s) = (Pm - Pe)/(2*Hm);
  Pe = (Rr1*V^2/s)/((Rs + Rr1/s)^2 + (Xs + Xr1)^2);
end slip_eq;
