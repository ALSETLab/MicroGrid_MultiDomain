within OpenIPSL.DataSets.Anderson.ESData;
record ExcSystemDataCT1
  extends ExcSystemDataTemplate( VR_type = "D",
    IEEE_type = "ST2x",
    Name = "SCPT",
    RR = Modelica.Constants.small,
    T_R = 0.000,
    K_A = 120.000,
    T_A1 = 0.050,
    T_A2 = 0.000,
    V_RMAX = 1.200,
    V_RMIN = -1.200,
    K_E = 1.000,
    T_E = 0.500,
    E_1 = Modelica.Constants.small,
    E_2 = Modelica.Constants.small,
    S_EE_1 = Modelica.Constants.small,
    S_EE_2 = Modelica.Constants.small,
    A_ex = Modelica.Constants.small,
    B_ex = Modelica.Constants.small,
    Efd_max = Modelica.Constants.small,
    Efd_min = Modelica.Constants.small,
    K_F = 0.020,
    T_F1 = 1.000,
    T_F2 = 0.000);
end ExcSystemDataCT1;
