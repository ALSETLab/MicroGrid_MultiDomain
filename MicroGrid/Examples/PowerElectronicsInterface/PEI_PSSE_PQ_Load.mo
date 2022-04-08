within MicroGrid.Examples.PowerElectronicsInterface;
model PEI_PSSE_PQ_Load
  extends BaseClasses.VSC_partial(Fault(
      R=0.01,
      X=0.01,
      t1=101,
      t2=102));
  OpenIPSL.Electrical.Loads.PSSE.Load load(P_0=25000000, Q_0=10000000)
    annotation (Placement(transformation(extent={{60,-40},{80,-20}})));
equation
  connect(load.p, AC_2_DC_and_DC_2_AC.n)
    annotation (Line(points={{70,-20},{70,0},{46,0}}, color={0,0,255}));
end PEI_PSSE_PQ_Load;
