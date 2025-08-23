using DiscretePIDs, ControlSystems, Plots
using Plots
using LinearAlgebra

# System parameters
Ts = 0.02 # sampling time
Tf = 2; #final simulation time
g = 9.81 #gravity
α = 0.0 # slope
μ = 1.0 # friction coefficient
x_0 = -2.0 # starting position
dx_0 = 0.0 # starting velocity
τ = 20.0 # torque constant

# State Space Matrix
A = Float64[0 1 0
            0 -μ 1
            0 0 -τ];
B = Float64[0
            0
            τ];
C = Float64[1 0 0
            0 1 0];

sys = ss(A, B, C, 0.0)      # Continuous

bodeplot(tf(sys))
# current_poles = poles(sys)

ϵ = 0.01;
pp = 15;
p = - [pp + ϵ, pp - ϵ, pp]
observability(A, C)
controllability(A, B)
L = real(place(sys, p, :c));

Kp = place(sys, 5.0*p; opt = :o)
place(A', C', p)
R1 = diagm([0.01, 0.02, 0.03])
R2 = diagm([0.005, 0.002])
K = kalman(sys, R1, R2; direct = false)

cont = observer_controller(sys, L, Kp; direct = false)
filter = observer_filter(sys, K)

closedLoop = feedback(sys, cont)

clpoles = poles(closedLoop)
setPlotScale("dB")

gangoffourplot(sys, cont; minimal = true)

bodeplot(closedLoop[1, 1], 0.1:40)

K = L[1]
Ti = 0;
Td = L[2] / L[1]

pid = DiscretePID(; K, Ts, Ti, Td)

sysreal = ss(A, B, [1 0 0], 0)

ctrl = function (x, t)
    y = (sysreal.C * x)[] # measurement
    d = 0 * [1.0]        # disturbance
    r = 2 * (t >= 0) # reference
    # u = pid(r, y) # control signal
    # u + d # Plant input is control signal + disturbance
    # u =1
    e = x - [r; 0; 0]
    e[3] = 0 # torque not observable, just ignore it in the final feedback
    u = -L * e + d
    u = [maximum([-20 minimum([20 u])])]
end
t = 0:Ts:Tf

res = lsim(sysreal, ctrl, t)

plot(res, plotu = true, plotx = true, ploty = false);
ylabel!("u", sp = 1);
ylabel!("x", sp = 2);
ylabel!("v", sp = 3);
ylabel!("T", sp = 4);

#  ylabel!(["u", "x","v","T"]);

#
si = stepinfo(res);
plot(si);
title!("Step Response");
