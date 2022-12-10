within OpenIPSL.DataSets.Anderson.ESData;
record ExcSystemDataH6
  extends ExcSystemDataTemplate( VR_type = "A",
    IEEE_type = "DC1x",
    Name = "REGULUX",
    RR = 0.5,
    T_R = 0.000,
    K_A = 25.000,
    T_A1 = 0.200,
    T_A2 = 0.000,
    V_RMAX = 1.000,
    V_RMIN = -1.000,
    K_E = -0.057,
    T_E = 0.646,
    E_1 = 2.610,
    E_2 = 3.480,
    S_EE_1 = 0.0885,
    S_EE_2 = 0.3480,
    A_ex = 0.0015,
    B_ex = 1.5738,
    Efd_max = 3.480,
    Efd_min = -3.480,
    K_F = 0.103,
    T_F1 = 1.000,
    T_F2 = 0.000);
end ExcSystemDataH6;
