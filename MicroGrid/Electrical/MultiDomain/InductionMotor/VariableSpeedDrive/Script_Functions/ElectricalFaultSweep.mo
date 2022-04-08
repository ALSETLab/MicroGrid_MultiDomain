within MicroGrid.Electrical.MultiDomain.InductionMotor.VariableSpeedDrive.Script_Functions;
function ElectricalFaultSweep

algorithm
  pathToModel := "MicroGrid.Induction_Motor.VSD_NEW.Testing_Motors.GM_";
  translateModel(pathToModel);
  simulateModel(pathToModel, stopTime=50, numberOfIntervals=10000, tolerance=0.01, resultFile="Banana");
  pwLine.R :=100;
  simulateModel(pathToModel, stopTime=50, numberOfIntervals=10000, tolerance=0.01, resultFile="Banana");
  pwLine.R :=1000;
  simulateModel(pathToModel, stopTime=50, numberOfIntervals=10000, tolerance=0.01, resultFile="Banana");
end ElectricalFaultSweep;
