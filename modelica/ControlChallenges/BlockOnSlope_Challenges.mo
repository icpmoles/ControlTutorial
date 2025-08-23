within ControlChallenges;

package BlockOnSlope_Challenges
   
model BlockOnSlope
    final parameter SI.Voltage sat = 20;
    final parameter SI.Acceleration g = 9.81;
    final parameter SI.Mass m = 1;
    parameter SI.CoefficientOfFriction mu = 1 "coulomb friction coefficient";
    parameter Real slope = 1 "slope in percentage";
    final parameter SI.ElectricalForceConstant kf = 20 "torque constant";
    parameter SI.Position x0 = 0 "init position";
    final parameter Real r = sin(slope) "slope coefficient conversion";
    Real usat;
    SI.Position x;
    SI.Velocity xd;
    SI.Force F;
  Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flange_b annotation(
      Placement(transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
 
 initial equation
    x = x0;
  equation
//der(x) = xd;
    usat = max(min(u,sat),-sat);
    
    der(xd) = F/m - r*g/m - mu*xd/m;
    der(F) = -kf*F + kf*usat;
//y[1] = x;
//y[2] = xd;
    der(flange_b.s) = xd;
    flange_b.s = x;
    annotation(
      Diagram,
      Icon(graphics = {Rectangle(origin = {1, -10}, rotation = 45, lineThickness = 1.5, extent = {{-71, 44}, {71, -44}}), Polygon(origin = {-33, 34}, rotation = 45, lineThickness = 0.75, points = {{-79, -12}, {63, -12}, {79, 12}, {-53, 12}, {-79, -12}}), Polygon(origin = {49, 56}, rotation = 45, lineThickness = 0.75, points = {{-7, -54}, {9, -28}, {7, 54}, {-9, 32}, {-7, -54}})}));
  end BlockOnSlope;

  package Examples
  model WithFriction
  parameter Real Ts = 0.001;
  parameter Real qf = 5;
  parameter Real Kpp[2,2] = -[337, 0; 0, 64];
  BlockOnSlope blockOnSlope(mu = 1, slope = 0, x0 = 0)  annotation(
        Placement(transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Discrete.ZeroOrderHold zohControl(samplePeriod = Ts, ySample(fixed = false), startTime = Ts/3) annotation(
        Placement(transformation(origin = {166, -4}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor annotation(
        Placement(transformation(origin = {128, 28}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor annotation(
        Placement(transformation(origin = {114, -6}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Routing.Multiplex2 multiplex2 annotation(
        Placement(transformation(origin = {-126, 2}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Math.MatrixGain K(K = Kpp)  annotation(
        Placement(transformation(origin = {-72, 2}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Math.Sum sum1(nin = 2)  annotation(
        Placement(transformation(origin = {-30, 2}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Math.Add add(k1 = -1)  annotation(
        Placement(transformation(origin = {-190, 14}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Sources.KinematicPTP kinematicPTP(deltaq = {qf}, qd_max = {10}, qdd_max = {1.5}, startTime = 1)  annotation(
        Placement(transformation(origin = {-342, 20}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Discrete.ZeroOrderHold zohPosition(samplePeriod = Ts, ySample(fixed = false), startTime = Ts/2) annotation(
        Placement(transformation(origin = {180, 30}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Discrete.FirstOrderHold fohControl(samplePeriod = Ts)  annotation(
        Placement(transformation(origin = {8, 2}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Continuous.Integrator integrator annotation(
        Placement(transformation(origin = {-288, 22}, extent = {{-10, -10}, {10, 10}})));
 Modelica.Blocks.Continuous.Integrator integrator1 annotation(
        Placement(transformation(origin = {-254, 20}, extent = {{-10, -10}, {10, 10}})));
    equation
      connect(positionSensor.flange, blockOnSlope.flange_b) annotation(
        Line(points = {{118, 28}, {72, 28}, {72, 0}, {68, 0}}, color = {0, 127, 0}));
      connect(blockOnSlope.flange_b, speedSensor.flange) annotation(
        Line(points = {{68, 0}, {82, 0}, {82, -3}, {94, -3}, {94, -6}, {104, -6}}, color = {0, 127, 0}));
 connect(zohPosition.u, positionSensor.s) annotation(
        Line(points = {{168, 30}, {140, 30}, {140, 28}}, color = {0, 0, 127}));
 connect(zohPosition.y, add.u2) annotation(
        Line(points = {{192, 30}, {238, 30}, {238, -118}, {-202, -118}, {-202, 8}}, color = {0, 0, 127}));
 connect(speedSensor.v, zohControl.u) annotation(
        Line(points = {{126, -6}, {156, -6}, {156, -4}}, color = {0, 0, 127}));
 connect(zohControl.y, multiplex2.u2[1]) annotation(
        Line(points = {{174, -4}, {224, -4}, {224, -106}, {-154, -106}, {-154, -4}, {-138, -4}}, color = {0, 0, 127}));
 connect(sum1.y, fohControl.u) annotation(
        Line(points = {{-18, 2}, {-4, 2}}, color = {0, 0, 127}));
 connect(fohControl.y, blockOnSlope.u) annotation(
        Line(points = {{20, 2}, {48, 2}, {48, 0}}, color = {0, 0, 127}));
 connect(add.y, multiplex2.u1[1]) annotation(
        Line(points = {{-178, 14}, {-154, 14}, {-154, 8}, {-138, 8}}, color = {0, 0, 127}));
 connect(multiplex2.y, K.u) annotation(
        Line(points = {{-114, 2}, {-84, 2}}, color = {0, 0, 127}, thickness = 0.5));
 connect(K.y, sum1.u) annotation(
        Line(points = {{-60, 2}, {-42, 2}}, color = {0, 0, 127}, thickness = 0.5));
 connect(integrator1.u, integrator.y) annotation(
        Line(points = {{-266, 20}, {-276, 20}, {-276, 22}}, color = {0, 0, 127}));
 connect(add.u1, integrator1.y) annotation(
        Line(points = {{-202, 20}, {-242, 20}}, color = {0, 0, 127}));
 connect(kinematicPTP.y[1], integrator.u) annotation(
        Line(points = {{-330, 20}, {-300, 20}, {-300, 22}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-6, Interval = 0.01),
 Diagram(coordinateSystem(extent = {{-360, 40}, {240, -120}})));
end WithFriction;
  end Examples;

  model FullStateFeedback
  
  parameter Real K1;
  parameter Real K2;
  Modelica.Blocks.Interfaces.RealInput ref annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealVectorInput y[2] annotation(
      Placement(transformation(origin = {0, -98}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {0, -98}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealOutput u annotation(
      Placement(transformation(origin = {102, 2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 2}, extent = {{-10, -10}, {10, 10}})));
  equation
  u = (ref-y[1])*K1  - y[2]*K2; 
  

  annotation(
      Diagram,
  Icon(graphics = {Text(origin = {112, -4}, extent = {{-226, -96}, {0, 96}}, textString = "-K", fontName = "Georgia")}));
end FullStateFeedback;
end BlockOnSlope_Challenges;
