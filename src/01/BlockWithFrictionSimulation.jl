using FMI, Plots, DifferentialEquations
tStart = 0.0;
tStop = 5.0;
fmu = loadFMU(
    pwd()*raw"\modelica\ControlChallenges\ControlChallenges.BlockOnSlope_Challenges.Examples.WithFriction.fmu",
)
c = fmi2Instantiate!(fmu; loggingOn = true)
fmi2SetupExperiment(c, tStart, tStop)
# params = ["BlockOnSlope_Challenges.FullStateFeedback.K1", "BlockOnSlope_Challenges.FullStateFeedback.K2"]
# fmi2EnterInitializationMode(c)
# getValue(c, params)
# fmi2ExitInitializationMode(c)
params = ["Kone", "Ktwo"];
# setValue(c, params, [337.0 , 64.0])

fmi2EnterInitializationMode(c)
fmi2SetReal(c, "Kone", 337.0)
fmi2SetReal(c, "Ktwo", 64.0)
getValue(c, params)
fmi2ExitInitializationMode(c)

simData = simulateME(
    c,
    (tStart, tStop);
    recordValues = ["blockOnSlope.x", "blockOnSlope.xd", "blockOnSlope.usat"]
);
plot(simData, states = true, timeEvents = false)
# p = plot(simData, series_keyword = 5)
# simDatau = simulateME(fmu, (tStart, tStop); recordValues=["blockOnSlope.usat"]);
# plot(simDatau, states=false)
# unloadFMU(fmu)
