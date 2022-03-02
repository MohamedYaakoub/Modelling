within BasicAeroEngines;

package Components "Components of aeroengines"
  extends Modelica.Icons.Package;

  model Environment "Outer model providing environmental conditions to components"
    package Air = Media.Air "Medium model";
    //parameter Modelica.SIunits.Temperature temperature = 288.15 "Air Temperature";
    parameter Modelica.SIunits.Length altitude = 0 "Altitude";
    parameter Modelica.SIunits.Velocity airspeed = 0 "Airspeed";
    parameter Real Mach = 0 "Mach nr.";
    parameter Boolean useMach = false "if true use Mach nr. to calculate speed";
    parameter Modelica.SIunits.Pressure Pb = 101325 "Base value of pressure" annotation(
      Dialog(group = "Atmospheric pressure model"));
    parameter Modelica.SIunits.Temperature Tb = 288.15 "Base value of temperature" annotation(
      Dialog(group = "Atmospheric pressure model"));
    parameter Modelica.SIunits.Length Hb = 0 "Base value of altitude" annotation(
      Dialog(group = "Atmospheric pressure model"));
    parameter Real Tlr = -0.0065 "Temperature lapse rate" annotation(
      Dialog(group = "Atmospheric pressure model"));
    constant Modelica.SIunits.Acceleration g = Modelica.Constants.g_n "Acceleration of gravity";
    constant Real R(final unit = "J/(mol.K)") = 8.3144598 "Molar gas constant";
    Modelica.SIunits.Temperature T "Air temperature";
    Modelica.SIunits.Length H = altitude "Height of the engine above sea level";
    Modelica.SIunits.Velocity v = if useMach then Mach * c else airspeed "Airspeed";
    Modelica.SIunits.Pressure P(start = 101325) "Air pressure";
    Modelica.SIunits.SpecificEntropy s "Air specific entropy";
    Air.BaseProperties airProps "Air properties";
    Air.BaseProperties stagnationProps "Air properties in stagnation conditions";
    Modelica.SIunits.Velocity c "Velocity of sound at the ambient conditions";
  equation
// Standard atmospheric model - temperature lapse rate = 0
//P = Pb * exp(-g * airProps.MM * (H - Hb) / (R * Tb));
// Standard atmospheric model - temperature lapse rate < 0
    T = Tb + Tlr * (H - Hb);
    P = Pb * (Tb / T) ^ (g * airProps.MM / (R * Tlr));
// Air properties
    airProps.p = P;
    airProps.T = T;
    s = Air.specificEntropy(airProps.state);
    c = Air.velocityOfSound(airProps.state);
// Intake properties
    stagnationProps.h = airProps.h + v ^ 2 / 2;
    s = Air.specificEntropy(stagnationProps.state);
// Connector variables
//if useMach then v = Mach*c; else v = airspeed; end if;
    annotation(
      defaultComponentName = "environment",
      defaultComponentPrefixes = "inner",
      missingInnerMessage = "No \"environment\" component is defined. Please drag one into your system model and set it according to your needs.",
      Icon(graphics = {Rectangle(fillColor = {110, 156, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Ellipse(origin = {-33, -15}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-41, 19}, {41, -19}}, endAngle = 360), Ellipse(origin = {-1, -41}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-41, 19}, {31, -11}}, endAngle = 360), Ellipse(origin = {25, -3}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-41, 19}, {31, -11}}, endAngle = 360), Ellipse(origin = {57, 58}, lineColor = {255, 255, 0}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{25, 24}, {-25, -24}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
  end Environment;

  model EnvironmentTakeOff "Outer model providing environmental conditions to components based on mission"
    extends Components.Environment(H = altitude + climbheight * (time - timeTO - timeSet) / timeTOclimb * b, v = speedTO * (b + a * (time - timeSet) / timeTO));
    parameter Modelica.SIunits.Time timeTO = 30 "takeoff time" annotation(
      Dialog(group = "Mission profile"));
    parameter Modelica.SIunits.Time timeTOclimb = 40 "climb time" annotation(
      Dialog(group = "Mission profile"));
    parameter Modelica.SIunits.Time timeSet = 30 "settling time - to eliminate initial dummy transient" annotation(
      Dialog(group = "Mission profile"));
    parameter Modelica.SIunits.Length climbheight = 500 "climb height" annotation(
      Dialog(group = "Mission profile"));
    parameter Modelica.SIunits.Velocity speedTO = 87.4555 "Speed Takeoff" annotation(
      Dialog(group = "Mission profile"));
    Real a;
    Real b;
  equation
// v = speedTO*(b+a*(time-timeSet)/timeTO);
    if time > timeTO + timeSet then
// Interpolation interval is not big enough, use "next" value
      a = 0;
      b = 1;
    elseif time > timeSet then
//Acceleration
      a = 1;
      b = 0;
    else
//During settling time
      a = 0;
      b = 0;
    end if;
    annotation(
      defaultComponentName = "environment",
      defaultComponentPrefixes = "inner",
      missingInnerMessage = "No \"environment\" component is defined. Please drag one into your system model and set it according to your needs.",
      Icon(graphics = {Rectangle(fillColor = {110, 156, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Ellipse(origin = {-33, -15}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-41, 19}, {41, -19}}, endAngle = 360), Ellipse(origin = {-1, -41}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-41, 19}, {31, -11}}, endAngle = 360), Ellipse(origin = {25, -3}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-41, 19}, {31, -11}}, endAngle = 360), Ellipse(origin = {57, 58}, lineColor = {255, 255, 0}, fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{25, 24}, {-25, -24}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
  end EnvironmentTakeOff;

  model AirIntake "Intake model converting forward speed into static pressure"
    // no pressure losses
    outer Components.Environment environment;
    Interfaces.AirPort outlet annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.Power W "Mechanical power input";
    Modelica.SIunits.MassFlowRate f "Mass flow rate through the intake";
    Modelica.Blocks.Interfaces.RealOutput drag "Drag of the engine intake" annotation(
      Placement(visible = true, transformation(origin = {64, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 7.10543e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    W = f * environment.v ^ 2 / 2;
    drag = f * environment.v;
    f = -outlet.f;
    outlet.P = environment.stagnationProps.p;
    outlet.h_L = environment.stagnationProps.h;
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 100}, {-60, -100}, {60, -60}, {60, 60}, {60, 60}, {-60, 100}})}),
      experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
  end AirIntake;

  model CompressorMapsBetaLines "Compressor model using non-dimensional maps and beta lines"
    extends BaseClasses.BaseCompressor(redeclare parameter Types.CompressorMapsBetaLinesData data);
    parameter Boolean enforceBetaLimits = false "Strictly enforce beta limits if true";
    parameter Boolean useHomotopy = false "Use simplified map for initialization";
    final parameter Integer idx = div(size(data.Phi_n_table,1), 2) + 1 "Index of table rows used to compute simplified slope";
    final parameter Real q_Phi = abs(data.Phi_n_table[idx,end] - data.Phi_n_table[idx, 2])/(data.beta_choke - data.beta_surge)
      "slope of simplified Phi curve at nominal speed: (Phi_n_choke-Phi_n_surge)/(beta_choke-beta_surge)";
    final parameter Real q_PR = -abs(data.PR_n_table[idx,end] - data.PR_n_table[idx, 2])/(data.beta_choke - data.beta_surge)
      "slope of simplified PR curve at nominal speed: (PR_n_choke-PR_n_surge)/(beta_choke-beta_surge)";
    Modelica.SIunits.PerUnit beta(start = data.beta_nom) "Beta line value";
    Modelica.Blocks.Tables.CombiTable2D Phi_n_interp(table = data.Phi_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D PR_n_interp(table = data.PR_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D eta_interp(table = data.eta_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Phi_n = Phi_n(N_n, beta)
    Phi_n_interp.u1 = N_n;
    Phi_n_interp.u2 = beta;
    if useHomotopy then
      Phi_n = homotopy(
        Phi_n_interp.y,
        1 + (N_n - 1) + (beta - data.beta_nom)*q_Phi);
    else
      Phi_n = Phi_n_interp.y;
    end if;
// PR_n = PR_n(N_n, beta)
    PR_n_interp.u1 = N_n;
    PR_n_interp.u2 = beta;
    if useHomotopy then
      PR_n = homotopy(
        PR_n_interp.y,
        1 + (N_n - 1) + (beta - data.beta_nom)*q_PR);
    else
      PR_n = PR_n_interp.y;
    end if;
// eta_n = eta(N_n, beta)
    eta_interp.u1 = N_n;
    eta_interp.u2 = beta;
    if useHomotopy then
      eta_n = homotopy(eta_interp.y, 1);
    else
      eta_n = eta_interp.y;
    end if;
    if enforceBetaLimits then
      if data.beta_choke > data.beta_surge then
        assert(beta >= data.beta_choke, "Choke line crossed");
        assert(beta <= data.beta_surge, "Surge line crossed");
      else
        assert(beta <= data.beta_choke, "Choke line crossed");
        assert(beta >= data.beta_surge, "Surge line crossed");
      end if;
    end if;
  end CompressorMapsBetaLines;

  model CompressorBleed "Compressor model with bleed ports and using non-dimensional maps and beta lines"
  
  extends BaseClasses.BaseCompressorBleed(redeclare parameter Types.CompressorMapsBetaLinesData data);
    parameter Boolean enforceBetaLimits = false "Strictly enforce beta limits if true";
    parameter Boolean useHomotopy = false "Use simplified map for initialization";
    final parameter Integer idx = div(size(data.Phi_n_table, 1), 2) + 1 "Index of table rows used to compute simplified slope";
    final parameter Real q_Phi = abs(data.Phi_n_table[idx, end] - data.Phi_n_table[idx, 2]) / (data.beta_choke - data.beta_surge) "slope of simplified Phi curve at nominal speed: (Phi_n_choke-Phi_n_surge)/(beta_choke-beta_surge)";
    final parameter Real q_PR = -abs(data.PR_n_table[idx, end] - data.PR_n_table[idx, 2]) / (data.beta_choke - data.beta_surge) "slope of simplified PR curve at nominal speed: (PR_n_choke-PR_n_surge)/(beta_choke-beta_surge)";
    Modelica.SIunits.PerUnit beta(start = data.beta_nom) "Beta line value";
    Modelica.Blocks.Tables.CombiTable2D Phi_n_interp(table = data.Phi_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D PR_n_interp(table = data.PR_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D eta_interp(table = data.eta_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Phi_n = Phi_n(N_n, beta)
    Phi_n_interp.u1 = N_n;
    Phi_n_interp.u2 = beta;
    if useHomotopy then
      Phi_n = homotopy(Phi_n_interp.y, 1 + N_n - 1 + (beta - data.beta_nom) * q_Phi);
    else
      Phi_n = Phi_n_interp.y;
    end if;
// PR_n = PR_n(N_n, beta)
    PR_n_interp.u1 = N_n;
    PR_n_interp.u2 = beta;
    if useHomotopy then
      PR_n = homotopy(PR_n_interp.y, 1 + N_n - 1 + (beta - data.beta_nom) * q_PR);
    else
      PR_n = PR_n_interp.y;
    end if;
// eta_n = eta(N_n, beta)
    eta_interp.u1 = N_n;
    eta_interp.u2 = beta;
    if useHomotopy then
      eta_n = homotopy(eta_interp.y, 1);
    else
      eta_n = eta_interp.y;
    end if;
    if enforceBetaLimits then
      if data.beta_choke > data.beta_surge then
        assert(beta >= data.beta_choke, "Choke line crossed");
        assert(beta <= data.beta_surge, "Surge line crossed");
      else
        assert(beta <= data.beta_choke, "Choke line crossed");
        assert(beta >= data.beta_surge, "Surge line crossed");
      end if;
    end if;
  end CompressorBleed;

  model TurbineStodola "Turbine model using Stodola's ellipse law and non-dimensional efficiency maps"
    parameter Boolean ConstantEta = true;
    extends BaseClasses.BaseTurbine(redeclare parameter Types.TurbineStodolaData data);
    final parameter Modelica.SIunits.PerUnit k_s = 1 / sqrt(1 - 1 / (data.P_E_nom / data.P_L_nom) ^ 2) "Normalized Stodola's law coefficient";
    Modelica.Blocks.Tables.CombiTable2D eta_interp(table = data.eta_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Stodola' ellipse law
    Phi_n = k_s * sqrt(1 - 1 / PR ^ 2);
// eta = eta(N_n, Phi_n)
    eta_interp.u1 = N_n;
    eta_interp.u2 = Phi_n;
    if ConstantEta then
      eta_n = 1;
    else
      eta_interp.y = eta_n;
    end if;
  end TurbineStodola;

  model TurbineMapsBetaLines "Compressor model using non-dimensional maps and beta lines"
    extends BaseClasses.BaseTurbine(redeclare parameter Types.TurbineMapsBetaLinesData data);
    Modelica.SIunits.PerUnit beta(start = data.beta_nom) "Beta line value";
    Modelica.Blocks.Tables.CombiTable2D Phi_n_interp(table = data.Phi_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D PR_n_interp(table = data.PR_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D eta_interp(table = data.eta_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Phi_n = Phi_n(N_n, beta)
    Phi_n_interp.u1 = N_n;
    Phi_n_interp.u2 = beta;
    Phi_n_interp.y = Phi_n;
// PR_n = PR_n(N_n, beta)
    PR_n_interp.u1 = N_n;
    PR_n_interp.u2 = beta;
    PR_n_interp.y = PR_n;
// eta = eta(N_n, beta)
    eta_interp.u1 = N_n;
    eta_interp.u2 = beta;
    eta_interp.y = eta_n;
  end TurbineMapsBetaLines;

  model NozzleExhaust "Nozzle exhaust model converting static pressure into thrust"
    outer Components.Environment environment;
    package ExhaustGas = Media.ExhaustGas;
    Interfaces.ExhaustPort inlet annotation(
      Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput thrust(unit = "N") annotation(
      Placement(visible = true, transformation(origin = {62, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Modelica.SIunits.Area A "Area of nozzle exhaust cross-section";
    parameter Modelica.SIunits.Velocity v_start = 250 "Tentative value of exhaust gas velocity";
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.Pressure P_E(start = 1.1e5) "Entering pressure";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering specific enthalpy";
    Modelica.SIunits.SpecificEnthalpy X_E[ExhaustGas.nX] "Entering gas composition";
    Modelica.SIunits.SpecificEntropy s "Specific entropy of exhaust gas";
    Modelica.SIunits.Velocity v(start = v_start) "Velocity at the nozzle exhaust";
    Modelica.SIunits.Velocity c "Velocity of sound at the nozzle exhaust conditions";
    Modelica.SIunits.Power W "Mechanical power output";
    ExhaustGas.BaseProperties inletProps "Fluid properties at the nozzle inlet";
    ExhaustGas.BaseProperties outletProps "Fluid properties at the nozzle outlet";
  equation
// Nozzle inlet conditions (stagnation assumed)
    inletProps.p = P_E;
    inletProps.h = h_E;
    inletProps.X = X_E;
    s = ExhaustGas.specificEntropy(inletProps.state);
// Nozzle outlet conditions
    outletProps.p = environment.P;
    outletProps.X = X_E;
    inletProps.h = outletProps.h + v ^ 2 / 2;
    s = ExhaustGas.specificEntropy(outletProps.state);
    outletProps.d * v * A = f_E;
    c = ExhaustGas.velocityOfSound(outletProps.state);
    assert(v < c, "Invalid supersonic conditions at nozzle outlet (not a Laval nozzle)", AssertionLevel.warning);
// Generated thrust and power
    thrust = f_E * v;
    W = f_E * v ^ 2 / 2;
// Boundary conditions
    f_E = inlet.f;
    P_E = inlet.P;
    h_E = inStream(inlet.h_L);
    X_E = inStream(inlet.X_L);
    inlet.h_L = 0 "Not used, no flow reversal";
    inlet.X_L = ExhaustGas.reference_X "Not used, no flow reversal";
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 60}, {-60, -60}, {60, -100}, {60, 100}, {60, 100}, {-60, 60}})}, coordinateSystem(initialScale = 0.1)));
  end NozzleExhaust;

  model NozzleAir "Nozzle exhaust model converting static pressure into thrust"
    outer Components.Environment environment;
    package BypassFlow = Media.Air;
    Modelica.Blocks.Interfaces.RealOutput thrustBypass(unit = "N") annotation(
      Placement(visible = true, transformation(origin = {62, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    parameter Modelica.SIunits.Area A "Area of nozzle exhaust cross-section";
    parameter Modelica.SIunits.Velocity v_start = 250 "Tentative value of exhaust gas velocity";
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.Pressure P_E(start = 1.1e5) "Entering pressure";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering specific enthalpy";
    //Modelica.SIunits.SpecificEnthalpy X_E[BypassFlow.nX] "Entering gas composition";
    //Modelica.SIunits.SpecificEnthalpy X_L[BypassFlow.nX] "Entering gas composition";
    Modelica.SIunits.SpecificEntropy s "Specific entropy of exhaust gas";
    Modelica.SIunits.Velocity v(start = v_start) "Velocity at the nozzle exhaust";
    Modelica.SIunits.Velocity c "Velocity of sound at the nozzle exhaust conditions";
    Modelica.SIunits.Power W "Mechanical power output";
    BypassFlow.BaseProperties inletProps "Fluid properties at the nozzle inlet";
    BypassFlow.BaseProperties outletProps "Fluid properties at the nozzle outlet";
    Interfaces.AirPort inlet annotation(
      Placement(transformation(extent = {{-68, 46}, {-48, 66}})));
  equation
// Nozzle inlet conditions (stagnation assumed)
    inletProps.p = P_E;
    inletProps.h = h_E;
//inletProps.X = X_E;
    s = BypassFlow.specificEntropy(inletProps.state);
// Nozzle outlet conditions
    outletProps.p = environment.P;
//outletProps.X = X_E;
    inletProps.h = outletProps.h + v ^ 2 / 2;
    s = BypassFlow.specificEntropy(outletProps.state);
    outletProps.d * v * A = f_E;
    c = BypassFlow.velocityOfSound(outletProps.state);
    assert(v < c, "Invalid supersonic conditions at nozzle outlet (not a Laval nozzle)", AssertionLevel.warning);
// Generated thrust and power
    thrustBypass = f_E * v;
    W = f_E * v ^ 2 / 2;
// Boundary conditions
    P_E = inlet.P;
    f_E = inlet.f;
    h_E = inStream(inlet.h_L);
//X_E = inStream(inlet.X_L);
//X_E = {0.768,0.232};
//X_L = X_E;
    inlet.h_L = 0 "Not used, no flow reversal";
//inlet.X_L = BypassFlow.reference_X "Not used, no flow reversal";
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 60}, {-60, -60}, {60, -100}, {60, 100}, {60, 100}, {-60, 60}})}, coordinateSystem(initialScale = 0.1)));
  end NozzleAir;

  model Fan "Model of a fan"
    parameter BasicAeroEngines.Data.Compressors.GSP_FanCore data_core;
    parameter BasicAeroEngines.Data.Compressors.GSP_FanDuct data_bypass;
    parameter Boolean useHomotopy = false "Use simplified map for initialization";
    Interfaces.AirPort inlet annotation(
      Placement(visible = true, transformation(origin = {-72, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Interfaces.AirPort outlet_bypass annotation(
      Placement(visible = true, transformation(origin = {-22, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Interfaces.AirPort outlet_core annotation(
      Placement(visible = true, transformation(origin = {48, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a fan_mechanical_interface annotation(
      Placement(visible = true, transformation(origin = {46, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.CompressorMapsBetaLines compressor_bypass(
      data = data_bypass, enforceBetaLimits = false, 
      useHomotopy = useHomotopy) annotation(
      Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Components.CompressorMapsBetaLines compressor_core(
      data = data_core, enforceBetaLimits = false,
      useHomotopy = useHomotopy) annotation(
      Placement(transformation(extent = {{-6, -10}, {14, 10}})));
  equation
    connect(compressor_bypass.outlet, outlet_bypass) annotation(
      Line(points = {{-46, 6}, {-44, 6}, {-44, 22}, {-22, 22}}, color = {170, 223, 255}));
    connect(outlet_bypass, outlet_bypass) annotation(
      Line(points = {{-22, 22}, {-22, 22}}, color = {170, 223, 255}));
    connect(inlet, compressor_bypass.inlet) annotation(
      Line(points = {{-72, 36}, {-72, 37}, {-58, 37}, {-58, 10}}, color = {170, 223, 255}));
    connect(compressor_core.inlet, compressor_bypass.inlet) annotation(
      Line(points = {{-2, 10}, {-2, 36}, {-58, 36}, {-58, 10}}, color = {170, 223, 255}));
    connect(outlet_core, compressor_core.outlet) annotation(
      Line(points = {{48, 36}, {10, 36}, {10, 6}}, color = {170, 223, 255}));
    connect(compressor_core.shaft_a, compressor_bypass.shaft_b) annotation(
      Line(points = {{-2, 0}, {-46, 0}}, color = {0, 0, 0}));
    connect(fan_mechanical_interface, compressor_core.shaft_b) annotation(
      Line(points = {{46, 0}, {10, 0}}, color = {0, 0, 0}));
    annotation(
      Icon(coordinateSystem(extent = {{-80, -80}, {80, 80}}), graphics = {Polygon(points = {{-24, 72}, {-24, -56}, {16, -56}, {16, 52}, {-4, 62}, {-24, 72}}, lineColor = {0, 0, 0}, fillColor = {135, 135, 135}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-24, -56}, {36, -66}}, lineColor = {0, 0, 0}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid)}));
  end Fan;

  model CombustionChamberLHVSimple "Simple model of combustion chamber with given fuel LHV -  constant reference outlet composition assumed"
    package ExhaustGas = Media.ExhaustGas;
    parameter Modelica.SIunits.SpecificEnergy LHV = 42.8e6 "Lower heating value of fuel";
    parameter Modelica.SIunits.MassFraction X_L[ExhaustGas.nX] = ExhaustGas.reference_X "Composition of exhaust gases";
    parameter Modelica.SIunits.Volume V "Internal volume of combustion chamber";
    parameter Modelica.SIunits.Pressure P_start "Start value of pressure";
    parameter Modelica.SIunits.Temperature T_start "Start value of temperature";
    parameter Boolean steadyStateInit = false "Initialize in steady state if true";
    parameter Real eta_comb = 0.995 "combustion chamber efficiency";
    parameter Real P_Loss = 4 "Relative pressure loss in percentage";
    BasicAeroEngines.Interfaces.AirPort airInlet annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    BasicAeroEngines.Interfaces.ExhaustPort exhaust annotation(
      Placement(visible = true, transformation(origin = {100, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 7.10543e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput fuelFlow annotation(
      Placement(visible = true, transformation(origin = {-8, 110}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    ExhaustGas.BaseProperties prop "Properties of exhaust gas";
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.MassFlowRate f_L "Leaving mass flow rate";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering specific enthalpy";
    Modelica.SIunits.SpecificEnthalpy h_L "Leaving specific enthalpy";
    Modelica.SIunits.Pressure P(start = P_start) "Exhaust gas pressure";
    Modelica.SIunits.Temperature T(start = T_start, stateSelect = StateSelect.prefer) "Exhaust gas temperature";
    Modelica.SIunits.Mass M(stateSelect = StateSelect.avoid) "Total mass";
    Modelica.SIunits.Energy E(stateSelect = StateSelect.avoid) "Total internal energy";
    Modelica.SIunits.Power Q "Thermal power released by combustion";
  initial equation
    if steadyStateInit then
      der(T) = 0;
      der(P) = 0;
    else
      T = T_start;
      P = P_start;
    end if;
  equation
// Conservation equations
    der(M) = f_E - f_L + fuelFlow;
    der(E) = f_E * h_E - f_L * h_L + Q;
// Constitutive equations and fluid properties
    M = prop.d * V;
    E = prop.u * M;
    Q = LHV * fuelFlow * eta_comb;
    prop.p = P;
    prop.T = T;
    prop.X = X_L;
    h_L = prop.h;
// Boundary conditions
    P = airInlet.P * (1 - P_Loss / 100);
    P = exhaust.P;
    f_E = airInlet.f;
    f_L = -exhaust.f;
    h_E = inStream(airInlet.h_L);
    exhaust.h_L = h_L;
    exhaust.X_L = X_L;
    airInlet.h_L = 0 "Unused, no flow reversal";
    annotation(
      Icon(graphics = {Ellipse(origin = {0, -1}, fillColor = {129, 170, 194}, fillPattern = FillPattern.Solid, extent = {{-100, 101}, {100, -99}}, endAngle = 360), Polygon(origin = {-2, 19}, fillColor = {218, 74, 25}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0, 75}, {-82, 3}, {-14, 35}, {-64, -55}, {-12, -1}, {4, -75}, {16, 1}, {62, -67}, {34, 31}, {82, 17}, {0, 75}})}));
  end CombustionChamberLHVSimple;

  model CombustionChamberLHV "Model of combustion chamber with given fuel LHV"
    //   import Modelica.Media.Interfaces.Choices.ReferenceEnthalpy;
    //   constant ReferenceEnthalpy referenceChoice=ReferenceEnthalpy.ZeroAt25C
    //     "Choice of reference enthalpy";
    package ExhaustGas = Media.ExhaustGas;
    parameter Modelica.SIunits.SpecificEnergy LHV = 42.8e6 "Lower heating value of fuel";
    Modelica.SIunits.MassFraction X_L[ExhaustGas.nX] "Composition of exhaust gases";
    //ExhaustGas.reference_X
    parameter Modelica.SIunits.Volume V "Internal volume of combustion chamber";
    parameter Modelica.SIunits.Pressure P_start "Start value of pressure";
    parameter Modelica.SIunits.Temperature T_start "Start value of temperature";
    parameter Boolean steadyStateInit = false "Initialize in steady state if true";
    parameter Real ZC "number of carbon atoms in fuel equivalent chemical formula";
    parameter Real ZH "number of hydrogen atoms in fuel equivalent chemical formula";
    final parameter Real a_stoich = ZC + ZH / 4 "stoichiometric ratio of the fuel combustion reaction";
    final parameter Modelica.SIunits.MolarMass MMfuel = ZC * 12 + ZH "Eq. molar mass of the fuel";
    parameter Real eta_comb = 0.995 "Combustion chamber efficiency";
    parameter Real P_Loss = 4 "Relative pressure loss in percentage";
    Interfaces.AirPort airInlet annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 3.55271e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.ExhaustPort exhaust annotation(
      Placement(visible = true, transformation(origin = {100, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 7.10543e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput fuelFlow annotation(
      Placement(visible = true, transformation(origin = {-8, 110}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    ExhaustGas.BaseProperties prop "Properties of exhaust gas";
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.MassFlowRate f_L "Leaving mass flow rate";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering specific enthalpy";
    Modelica.SIunits.SpecificEnthalpy h_L "Leaving specific enthalpy";
    Modelica.SIunits.Pressure P(start = P_start, stateSelect = StateSelect.prefer) "Exhaust gas pressure";
    Modelica.SIunits.Temperature T(start = T_start, stateSelect = StateSelect.prefer) "Exhaust gas temperature";
    Modelica.SIunits.Mass M(stateSelect = StateSelect.avoid) "Total mass";
    Modelica.SIunits.Energy E(stateSelect = StateSelect.avoid) "Total internal energy";
    Modelica.SIunits.Power Q "Thermal power released by combustion";
    Modelica.SIunits.MassFraction X_E[Media.Air.nX] "Composition of air";
  initial equation
    if steadyStateInit then
      der(T) = 0;
      der(P) = 0;
      der(X_L[2:4]) = zeros(ExhaustGas.nX - 1);
    else
      T = T_start;
      P = P_start;
      X_L[2:4] = ExhaustGas.reference_X[2:4];
    end if;
  equation
// Conservation equations
    der(M) = f_E + fuelFlow - f_L;
    der(E) = f_E * h_E - f_L * h_L + Q;
//"Nitrogen","Oxygen","Water", "Carbondioxide"
    X_L[1] = 1 - X_L[2] - X_L[3] - X_L[4];
    der(M * X_L[2]) = f_E * X_E[2] - f_L * X_L[2] - a_stoich * fuelFlow / MMfuel * ExhaustGas.data[2].MM * 1000 "oxygen";
    der(M * X_L[3]) = (-f_L * X_L[3]) + ZH / 2 * fuelFlow / MMfuel * ExhaustGas.data[3].MM * 1000 "water";
    der(M * X_L[4]) = (-f_L * X_L[4]) + ZC * fuelFlow / MMfuel * ExhaustGas.data[4].MM * 1000 "carbondioxide";
// Constitutive equations and fluid properties
    M = prop.d * V;
    E = prop.u * M;
    Q = LHV * fuelFlow * eta_comb;
    prop.p = P;
    prop.T = T;
    prop.X = X_L;
    h_L = prop.h;
// Boundary conditions
    P = airInlet.P;
    P * (1 - P_Loss / 100) = exhaust.P;
    f_E = airInlet.f;
    f_L = -exhaust.f;
    h_E = inStream(airInlet.h_L);
    exhaust.h_L = h_L;
    exhaust.X_L = X_L;
    X_E = Media.Air.reference_X;
    airInlet.h_L = 0 "Unused, no flow reversal";
    annotation(
      Icon(graphics = {Ellipse(origin = {0, -1}, fillColor = {129, 170, 194}, fillPattern = FillPattern.Solid, extent = {{-100, 101}, {100, -99}}, endAngle = 360), Polygon(origin = {-2, 19}, fillColor = {218, 74, 25}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0, 75}, {-82, 3}, {-14, 35}, {-64, -55}, {-12, -1}, {4, -75}, {16, 1}, {62, -67}, {34, 31}, {82, 17}, {0, 75}})}));
  end CombustionChamberLHV;

  model PressureSourceAir
    package Air = Media.Air;
    parameter Modelica.SIunits.Pressure referencePressure = 101325 "Reference pressure value";
    parameter Modelica.SIunits.Temperature referenceTemperature = 288.15 "Reference temperature value";
    BasicAeroEngines.Interfaces.AirPort fluidPort annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.Pressure P = referencePressure "Source pressure (can be changed via modifier)";
    Modelica.SIunits.Temperature T = referenceTemperature "Source temperature (can be changed via modifier)";
    Air.BaseProperties sourceProps "Source fluid properties";
  equation
    sourceProps.p = P;
    sourceProps.T = T;
// Boundary conditions
    fluidPort.P = sourceProps.p;
    fluidPort.h_L = sourceProps.h;
    annotation(
      Icon(graphics = {Ellipse(fillColor = {170, 223, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
  end PressureSourceAir;

  model PressureSourceExhaust
    package ExhaustGas = Media.ExhaustGas;
    parameter Modelica.SIunits.Pressure referencePressure = 101325 "Reference pressure value";
    parameter Modelica.SIunits.Temperature referenceTemperature = 288.15 "Reference temperature value";
    parameter Modelica.SIunits.MassFraction referenceComposition[ExhaustGas.nX] = ExhaustGas.reference_X "Reference composition vector";
    BasicAeroEngines.Interfaces.ExhaustPort fluidPort annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.Pressure P = referencePressure "Source pressure (can be changed via modifier)";
    Modelica.SIunits.Temperature T = referenceTemperature "Source temperature (can be changed via modifier)";
    Modelica.SIunits.MassFraction X[ExhaustGas.nX] = referenceComposition "Source composition (can be changed via modifier)";
    ExhaustGas.BaseProperties sourceProps "Source fluid properties";
  equation
    sourceProps.p = P;
    sourceProps.T = T;
    sourceProps.X = X;
// Boundary conditions
    fluidPort.P = sourceProps.p;
    fluidPort.h_L = sourceProps.h;
    fluidPort.X_L = sourceProps.X;
    annotation(
      Icon(graphics = {Ellipse(fillColor = {129, 170, 194}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
  end PressureSourceExhaust;

  model FlowSourceAir
    package Air = Media.Air;
    parameter Modelica.SIunits.MassFlowRate referenceMassFlowRate "Reference pressure value";
    parameter Modelica.SIunits.Temperature referenceTemperature = 288.15 "Reference temperature value";
    BasicAeroEngines.Interfaces.AirPort fluidPort annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.MassFlowRate f = referenceMassFlowRate "Source pressure (can be changed via modifier)";
    Modelica.SIunits.Temperature T = referenceTemperature "Source temperature (can be changed via modifier)";
    Modelica.SIunits.Pressure P "Source pressure";
    Air.BaseProperties sourceProps "Source fluid properties";
  equation
    sourceProps.p = P;
    sourceProps.T = T;
// Boundary conditions
    fluidPort.P = P;
    fluidPort.f = -f;
    fluidPort.h_L = sourceProps.h;
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {1, 0}, fillColor = {170, 223, 255}, fillPattern = FillPattern.Solid, extent = {{-101, 40}, {99, -40}}), Line(origin = {10.3427, 0.421704}, points = {{-28.5288, 19.9969}, {31.4712, -0.00309983}, {-30.5288, -20.0031}})}));
  end FlowSourceAir;

  model FlowSourceExhaust
    package ExhaustGas = Media.ExhaustGas;
    parameter Modelica.SIunits.MassFlowRate referenceMassFlowRate "Reference pressure value";
    parameter Modelica.SIunits.Temperature referenceTemperature = 288.15 "Reference temperature value";
    parameter Modelica.SIunits.MassFraction referenceComposition[ExhaustGas.nX] = ExhaustGas.reference_X "Reference composition vector";
    BasicAeroEngines.Interfaces.ExhaustPort fluidPort annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.MassFlowRate f = referenceMassFlowRate "Source pressure (can be changed via modifier)";
    Modelica.SIunits.Temperature T = referenceTemperature "Source temperature (can be changed via modifier)";
    Modelica.SIunits.MassFraction X[ExhaustGas.nX] = referenceComposition "Source composition (can be changed via modifier)";
    Modelica.SIunits.Pressure P "Source pressure";
    ExhaustGas.BaseProperties sourceProps "Source fluid properties";
  equation
    sourceProps.p = P;
    sourceProps.T = T;
    sourceProps.X = X;
// Boundary conditions
    fluidPort.P = P;
    fluidPort.f = -f;
    fluidPort.h_L = sourceProps.h;
    fluidPort.X_L = X;
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {1, 0}, fillColor = {129, 170, 194}, fillPattern = FillPattern.Solid, extent = {{-101, 40}, {99, -40}}), Line(origin = {10.3427, 0.421704}, points = {{-28.5288, 19.9969}, {31.4712, -0.00309983}, {-30.5288, -20.0031}})}));
  end FlowSourceExhaust;

  model LinearPressureDropExhaust
    parameter Modelica.SIunits.MassFlowRate referenceMassFlowRate "Reference pressure value";
    parameter Modelica.SIunits.Pressure referencePressureDrop "Reference temperature value";
    Modelica.SIunits.MassFlowRate f "Mass flow rate";
    Modelica.SIunits.PressureDifference dp "Pressure difference";
    Interfaces.ExhaustPort inlet annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Interfaces.ExhaustPort outlet annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    f = referenceMassFlowRate * dp / referencePressureDrop;
// Boundary conditions
    dp = inlet.P - outlet.P;
    inlet.f = f;
    outlet.f = -f;
    outlet.h_L = inStream(inlet.h_L);
    inlet.h_L = inStream(outlet.h_L);
    outlet.X_L = inStream(inlet.X_L);
    inlet.X_L = inStream(outlet.X_L);
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {1, 0}, fillColor = {129, 170, 194}, fillPattern = FillPattern.Solid, extent = {{-101, 40}, {99, -40}}), Line(origin = {10.3427, 0.421704}, points = {{-28.5288, 19.9969}, {31.4712, -0.00309983}, {-30.5288, -20.0031}})}));
  end LinearPressureDropExhaust;

  model LinearPressureDropAir
    parameter Modelica.SIunits.MassFlowRate referenceMassFlowRate "Reference pressure value";
    parameter Modelica.SIunits.Pressure referencePressureDrop "Reference temperature value";
    Modelica.SIunits.MassFlowRate f "Mass flow rate";
    Modelica.SIunits.PressureDifference dp "Pressure difference";
    Interfaces.AirPort inlet annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Interfaces.AirPort outlet annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    f = referenceMassFlowRate * dp / referencePressureDrop;
// Boundary conditions
    dp = inlet.P - outlet.P;
    inlet.f = f;
    outlet.f = -f;
    outlet.h_L = inStream(inlet.h_L);
    inlet.h_L = inStream(outlet.h_L);
    annotation(
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(origin = {1, 0}, fillColor = {170, 223, 255}, fillPattern = FillPattern.Solid, extent = {{-101, 40}, {99, -40}}), Line(origin = {10.3427, 0.421704}, points = {{-28.5288, 19.9969}, {31.4712, -0.00309983}, {-30.5288, -20.0031}})}));
  end LinearPressureDropAir;
  
  model ShaftInitializer
    package Air = Media.Air;
    parameter Boolean useHomotopy = false "=true to activate homotopy-based initialization";
    parameter Modelica.SIunits.AngularVelocity w_start "Start value of angular velocity";
    parameter Modelica.SIunits.Power W_nom "Nominal value of power (for scaling)";
    parameter Modelica.SIunits.PerUnit sigma = 1e-3 "slip ratio to provide nominal power";
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Types.Torque tau "Driving torque";
    Modelica.SIunits.AngularVelocity w "Angular velocity";
  equation
    if useHomotopy then
// Correct implementation
//  0 = homotopy(tau,
//               tau - (w - w_start)*W_nom/(w_start^2*sigma));
//
// Workaround implementation for bug #8376
    0 = homotopy(tau - (w - w_start)*W_nom/(w_start^2*sigma*1e8),
                 tau - (w - w_start)*W_nom/(w_start^2*sigma));
    else
      flange.tau = 0;
    end if;
    w = der(flange.phi);
    tau = -flange.tau;
    annotation(
      Icon(graphics = {Ellipse(fillColor = {76, 76, 76}, fillPattern = FillPattern.Solid, extent = {{-60, 60}, {60, -60}}), Ellipse(origin = {0, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-40, 41}, {40, -41}}), Rectangle(origin = {72, -1}, fillColor = {76, 76, 76}, fillPattern = FillPattern.Solid, extent = {{-12, 5}, {12, -5}})}, coordinateSystem(initialScale = 0.1)),
      Documentation(info = "<html>
<p>This component facilitates the steady-state initialization of gas turbine systems, by means of the homotopy() operator. The simplified model nearly constrains the shaft speed by introducing a large fictitious torque if the rotational speed deviates from the given start value; this avoids large deviations of the rotational speed during the first iterations of nonlinear solvers, improving the solver covergence.</p>
<p>During simulation, the component applies a null torque, thus having no influence on the system behaviour.</p>
</html>"));
  end ShaftInitializer;

model BleedAirDistributor

    parameter Integer Nbleed=3 "Number of bleed offtakes from the compressor";

  parameter Boolean useHandBleed = true "= true if handling bleed port to be connected"     annotation(Evaluate=true,
    HideResult=true,
    choices(checkBox=true));
  parameter Boolean useOverBleed = false "= true if overboard bleed port to be connected"  annotation(Evaluate=true,
    HideResult=true,
    choices(checkBox=true));
  parameter Boolean useLPTBleed = true "= true if LPT bleed port to be connected" annotation(Evaluate=true,
    HideResult=true,
    choices(checkBox=true));
  parameter Boolean useHPTBleed = false "= true if HPT bleed port to be connected" annotation(Evaluate=true,
    HideResult=true,
    choices(checkBox=true));

  parameter Integer NPortsHandBleed[max(HandBleedPorts,1)]={1} "Ports corresponding to Handling Bleed" annotation (Dialog(enable=useHandBleed));
  parameter Integer NPortsOverBleed[max(OverBleedPorts,1)]={1} "Ports corresponding to OverBoard Bleed" annotation (Dialog(enable=useOverBleed));
  parameter Integer NPortsLPTBleed[max(LPTBleedPorts,1)]= {2,3} "Ports corresponding to LPT Bleed" annotation (Dialog(enable=useLPTBleed));
  parameter Integer NPortsHPTBleed[max(HPTBleedPorts,1)]={1} "Ports corresponding to HPT Bleed" annotation (Dialog(enable=useHPTBleed));

  Interfaces.AirPort Bl_port[Nbleed] annotation (Placement(
        transformation(extent={{-8,70},{12,90}}), iconTransformation(extent={
            {-8,70},{12,90}})));

  Interfaces.AirPort HandBleed[HandBleedPorts] if useHandBleed "Handling bleed" annotation (
      Placement(
      visible=true,
      transformation(
        origin={-72,0},
        extent={{-10,-10},{10,10}},
        rotation=0),
      iconTransformation(
        origin={-42,0},
        extent={{-10, -10},{10, 10}},
        rotation=0)));
  Interfaces.AirPort OverBleed[OverBleedPorts] if useOverBleed "Overboard bleed" annotation (
      Placement(
      visible=true,
      transformation(
        origin={-26,0},
        extent={{-10,-10},{10,10}},
        rotation=0),
      iconTransformation(
        origin={-14,0},
        extent={{-10, -10},{10, 10}},
        rotation=0)));
  Interfaces.AirPort LPTBleed[LPTBleedPorts] if useLPTBleed "LPT cooling bleed" annotation (
      Placement(
      visible=true,
      transformation(
        origin={26,0},
        extent={{-10,-10},{10,10}},
        rotation=0),
      iconTransformation(
        origin={14,0},
        extent={{-10, -10},{10, 10}},
        rotation=0)));
  Interfaces.AirPort HPTBleed[HPTBleedPorts] if useHPTBleed "HPT cooling bleed" annotation (
      Placement(
      visible=true,
      transformation(
        origin={72,0},
        extent={{-10,-10},{10,10}},
        rotation=0),
      iconTransformation(
        origin={44,0},
        extent={{-10, -10},{10, 10}},
        rotation=0)));

  parameter Integer HandBleedPorts = 1 "Nr. of ports associated to Handling bleed" annotation (Dialog(enable=useHandBleed));
  parameter Integer OverBleedPorts = 0 "Nr. of ports associated to Overboard bleed" annotation (Dialog(enable=useOverBleed));
  parameter Integer LPTBleedPorts = 2 "Nr. of ports associated to LPT bleed" annotation (Dialog(enable=useLPTBleed));
  parameter Integer HPTBleedPorts = 0 "Nr. of ports associated to HPT bleed"annotation (Dialog(enable=useHPTBleed));


equation 

  if useHandBleed then
    connect(Bl_port[NPortsHandBleed], HandBleed);
  end if;
  if useOverBleed then
    connect(Bl_port[NPortsOverBleed], OverBleed);
  end if;
  if useLPTBleed then
    connect(Bl_port[NPortsLPTBleed], LPTBleed);
  end if;
  if useHPTBleed then
    connect(Bl_port[NPortsHPTBleed], HPTBleed);
  end if;
  
annotation (
Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(lineColor = {28, 108, 200}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, extent = {{-60, 80}, {64, 0}}), Text(lineColor = {255, 255, 255}, fillColor = {95, 95, 95}, extent = {{-24, 50}, {30, 24}}, textString = "Bleed Air"), Text(lineColor = {28, 108, 200}, fillColor = {95, 95, 95}, extent = {{-20, -40}, {-18, -40}}, textString = "Edit Here")}), Diagram(coordinateSystem(preserveAspectRatio=false)));

end BleedAirDistributor;
  
  model CooledTurbine "Compressor model using non-dimensional maps and beta lines"
    extends BasicAeroEngines.BaseClasses.BaseCooledTurbine(redeclare parameter Types.TurbineMapsBetaLinesData data);
    Modelica.SIunits.PerUnit beta(start = data.beta_nom) "Beta line value";
    Modelica.Blocks.Tables.CombiTable2D Phi_n_interp(table = data.Phi_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D PR_n_interp(table = data.PR_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Tables.CombiTable2D eta_interp(table = data.eta_n_table, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative) annotation(
      Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Phi_n = Phi_n(N_n, beta)
    Phi_n_interp.u1 = N_n;
    Phi_n_interp.u2 = beta;
    Phi_n_interp.y = Phi_n;
// PR_n = PR_n(N_n, beta)
    PR_n_interp.u1 = N_n;
    PR_n_interp.u2 = beta;
    PR_n_interp.y = PR_n;
// eta = eta(N_n, beta)
    eta_interp.u1 = N_n;
    eta_interp.u2 = beta;
    eta_interp.y = eta_n;
  end CooledTurbine;

  package TurbineCoolingModel
    model Gauntner
  "Gauntner, J. W. (1980). Algorithm for calculating turbine cooling flow and the resulting decrease in turbine efficiency (No. NASA-TM-81453)"
    Modelica.SIunits.PerUnit eps_cooling "Cooling effectiveness";
    Modelica.SIunits.PerUnit mfrac "Cooling flow mass fraction";
    input Modelica.SIunits.Temperature Tblade;
    input Modelica.SIunits.Temperature T_ca "Cooling air temperature";
    input Modelica.SIunits.Temperature T_E "Ehaust gas temperature";
    input Integer CoolingTech "Define Cooling Tech of stator blades: 1 uncooled, 2 convection+coating, 3 film + conv.";
    
    final parameter Modelica.SIunits.PerUnit F_Gauntner[3] = {0, 1.5, 1};
    
    equation
      eps_cooling = max(0, (T_E - Tblade) / (T_E - T_ca));
      mfrac = F_Gauntner[CoolingTech] * 0.022 * (eps_cooling / (1 - eps_cooling)) ^ 1.25;
    end Gauntner;
  end TurbineCoolingModel;
end Components;
