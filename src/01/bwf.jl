using CCS
using ControlSystems, Plots, LinearAlgebra, RobustAndOptimalControl
CCS.setupEnv()

contSys = CCS.blockModel.csys(; g = 0, α = 0, μ = 1, τ = 20)

pzmap(contSys)
bodeplot(contSys)


plot!(bodeplot(contSys[1, 1]), pzmap(contSys))
display(eigvals(contSys.A))
controllability(contSys.A, contSys.B).iscontrollable || error("System is not controllable")

ε = 0.01;
pp = 15.0;
poles_cont = - [pp + ε, pp - ε, pp];
L = real(place(contSys, poles_cont, :c));


poles_obs = poles_cont * 10.0;
K = place(contSys, poles_obs, :o)
obs_controller = observer_controller(contSys, L, K; direct = false);
fsf_controller = named_ss(obs_controller, u = [:ref_S, :ref_V], y = [:u]);
closedLoop = feedback(contSys * fsf_controller);
print(poles(closedLoop));
setPlotScale("dB")
plot!(bodeplot(closedLoop[1, 1], 0.1:40), pzmap(closedLoop))


using CCS: blockModel
using ControlSystems, Plots, LinearAlgebra, RobustAndOptimalControl

contSys = blockModel.csys(; g = 0, α = 0, μ = 1, τ = 20)

plot!(bodeplot(contSys[1, 1]), pzmap(contSys))
display(eigvals(contSys.A))

observability(contSys.A, contSys.C).isobservable || error("System is not observable")
controllability(contSys.A, contSys.B).iscontrollable || error("System is not controllable")


ε = 0.01;
pp = 10.0;
poles_cont = - [pp + ε, pp - ε, pp];
L = real(place(contSys, poles_cont, :c));


poles_obs = poles_cont * 10.0;
K = place(contSys, poles_obs, :o)
obs_controller = observer_controller(contSys, L, K; direct = false);
fsf_controller = named_ss(obs_controller, u = [:ref_S, :ref_V], y = [:u])



closedLoop = feedback(contSys * fsf_controller);
print(poles(closedLoop));
setPlotScale("dB")
plot!(bodeplot(closedLoop[1, 1], 0.1:40), pzmap(closedLoop))


# simulation

using DiscretePIDs
Ts = 0.02 # sampling time
Tf = 2.5; #final simulation time

K = L[1];
Ti = 0;
Td = L[2] / L[1];

K, Ti, Td = parallel2standard(L[1], 0, L[2])
vmax = 20.0;

pid = DiscretePID(; K, Ts, Ti, Td, umax = vmax, umin = -vmax);

onlyEncoder = blockModel.dsys(; g = 0, α = 0, μ = 1, τ = 20, y2 = false)

ctrl2 = function (x, t)
    y = (onlyEncoder.C*x)[] # measurement
    d = 0.0         # disturbance
    r = 2.0*(t >= 0) # reference
    u = pid(r, y) # control signal
    u + d # Plant input is control signal + disturbance
end
res2 = lsim(onlyEncoder, ctrl2, t)
plot(res2, plotu = true, plotx = true);
ylabel!("u + d", sp = 2)


ctrl = function (x, t)
    # y = (contSys.C*x)[] # measurement
    d = 0 * [1.0]        # disturbance
    r = 2.0 * (t >= 1) # reference
    # u = pid(r, y) # control signal
    # u + d # Plant input is control signal + disturbance
    # u =1
    e = x - [r; 0.0; 0.0]
    e[3] = 0.0 # torque not observable, just ignore it in the final feedback
    u = -L * e + d
    u = [maximum([-20.0 minimum([20.0 u])])]
end
t = 0:Ts:Tf

res = lsim(contSys, ctrl, t)

display(plot(res, plotu = true, plotx = true, ploty = false))

## Non linear simulator

using ControlSystemsBase: saturation

nl = nonlinearity(x->clamp(x, -20, 20)) # a saturating nonlinearity
satPID = nl*pid
Csat = saturation(20.0) * pid
