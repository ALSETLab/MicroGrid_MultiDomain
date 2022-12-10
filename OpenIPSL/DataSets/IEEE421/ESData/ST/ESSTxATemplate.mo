within OpenIPSL.DataSets.IEEE421.ESData.ST;
partial record ESSTxATemplate "Type ST-A Excitation System Data"
  parameter String Type "Type of ESSST excitation system model";
  parameter OpenIPSL.Types.Time T_R "Regulator input filter time constant";
  parameter OpenIPSL.Types.PerUnit V_IMAX "Maximum voltage error (regulator input)";
  parameter OpenIPSL.Types.PerUnit V_IMIN "Minimum voltage error (regulator input)";
  parameter OpenIPSL.Types.Time T_C "Regulator numerator (lead) time constant. First lead-lag";
  parameter OpenIPSL.Types.Time T_B "Regulator denominator (lag) time constant. First lead-lag";
  parameter OpenIPSL.Types.Time T_C1 "Regulator numerator (lead) time constant. Second lead-lag";
  parameter OpenIPSL.Types.Time T_B1 "Regulator denominator (lag) time constant. Second lead-lag";
  parameter OpenIPSL.Types.PerUnit K_A "Voltage regulator gain";
  parameter OpenIPSL.Types.Time T_A "Voltage regulator time constant";
  parameter OpenIPSL.Types.PerUnit V_AMAX "Maximum regulator output";
  parameter OpenIPSL.Types.PerUnit V_AMIN "Minimum regulator output";
  parameter OpenIPSL.Types.PerUnit V_RMAX "Maximum exciter output";
  parameter OpenIPSL.Types.PerUnit V_RMIN "Minimum exciter output";
  parameter OpenIPSL.Types.PerUnit K_C "Rectifier loading factor proportional to commutating reactance";
  parameter OpenIPSL.Types.PerUnit K_F "Rate feedback gain";
  parameter OpenIPSL.Types.Time T_F "Rate feedback time constant";
  parameter OpenIPSL.Types.PerUnit K_LR "Exciter output current limiter gain";
  parameter OpenIPSL.Types.PerUnit I_LR "Exciter output current limit reference";
  parameter OpenIPSL.Types.PerUnit K_P "Potential circuit (voltage) gain coefficient";
  parameter OpenIPSL.Types.PerUnit K_I "Compound circuit (current) gain coefficient";
  parameter OpenIPSL.Types.PerUnit K_E "Excitation power source output gain";
  parameter OpenIPSL.Types.Time T_E "Excitation power source output time constant";
  parameter OpenIPSL.Types.PerUnit EFD_MAX "Maximum exciter output";

end ESSTxATemplate;
