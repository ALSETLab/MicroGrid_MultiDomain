within MicroGrid.Examples.PowerElectronicsInterface;
model PEI_PSAT_PQ_Load
  extends BaseClasses.VSC_partial(Fault(t1=100, t2=101));
  OpenIPSL.Electrical.Loads.PSAT.PQ pQ(P_0=25000000, Q_0=10000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={70,-30})));
equation
  connect(AC_2_DC_and_DC_2_AC.n, pQ.p)
    annotation (Line(points={{46,0},{70,0},{70,-20}}, color={0,0,255}));
end PEI_PSAT_PQ_Load;
