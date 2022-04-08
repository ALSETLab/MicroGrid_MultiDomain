within MicroGrid.Optimization.MPComponents;
model PQ "Pl - Constant PQ Load"
  extends OpenIPSL.Electrical.Loads.PSAT.BaseClasses.baseLoad;

equation
    P = P_0/S_b;
    Q = Q_0/S_b;

  annotation (
    Documentation(revisions="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Last update</p></td>
<td>2015-09</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Joan Russinol, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p>see <a href=\"modelica://OpenIPSL.UsersGuide.Contact\">UsersGuide.Contact</a></p></td>
</tr>
</table></html>"));
end PQ;
