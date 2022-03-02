within BasicAeroEngines;

package BaseClasses "Base models"
  extends Modelica.Icons.BasesPackage;

  partial model BaseCompressor
    outer Components.Environment environment;
    package Air = Media.Air;
    replaceable parameter Types.TurbomachineData data "Compressor data";
    final parameter Modelica.SIunits.SpecificHeatCapacity cp_nom = 1000 "Nominal cp";
    final parameter Modelica.SIunits.PerUnit e = 0.28 "(gamma-1)/gamma";
    final parameter Modelica.SIunits.Power tau_nom = cp_nom * data.T_E_nom * ((data.P_L_nom / data.P_E_nom) ^ e - 1) / data.eta_nom * data.f_nom / data.omega_nom "Nominal torque";
    Interfaces.AirPort inlet "Air inlet port" annotation(
      Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.AirPort outlet "Air outlet port" annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.MassFlowRate f_L "Leaving mass flow rate";
    Modelica.SIunits.Pressure P_E(start = data.P_E_nom, fixed = false) "Entering pressure";
    Modelica.SIunits.Pressure P_L "Leaving pressure";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering pressure";
    Modelica.SIunits.SpecificEnthalpy h_L "Leaving pressure";
    Modelica.SIunits.SpecificEnthalpy h_iso "Isentropic enthalpy at outlet pressure";
    Modelica.SIunits.Temperature T_E "Inlet temperature";
    Modelica.SIunits.Temperature T_L "Outlet temperature";
    Modelica.SIunits.PerUnit PR "Pressure ratio";
    Modelica.SIunits.PerUnit eta "Isentropic efficiency";
    Modelica.SIunits.Power W "Mechanical power input to the compressor";
    Modelica.SIunits.AngularVelocity omega "Angular velocity of shaft";
    Modelica.SIunits.Torque tau(start = tau_nom) "Torque applied onto the shaft (positive)";
    Modelica.SIunits.PerUnit PR_n "Normalized pressure ratio";
    Modelica.SIunits.PerUnit Phi_n "Normalized flow number";
    //Real Phi_c_DP "Flow number @ design point";
    Real Phi_c "Flow number";
    Modelica.SIunits.PerUnit N_n(start = 1, fixed = false) "Normalized corrected speed";
    Modelica.SIunits.PerUnit eta_n "Normalized Isentropic efficiency";
    Air.BaseProperties propIn "Fluid properties at the inlet";
    Air.BaseProperties propOut "Fluid properties at the outlet";
    Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_a annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_b annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Balance equations
    f_E = f_L "Mass balance";
    W = f_E * (h_L - h_E) "Energy balance";
// Definition of auxiliary quantities
    W = omega * tau;
    PR = P_L / P_E;
    PR_n = PR / (data.P_L_nom / data.P_E_nom);
    eta = (h_iso - h_E) / (h_L - h_E);
    eta_n = eta / data.eta_nom;
    Phi_c = f_E * sqrt(T_E) / P_E;
//Phi_c_DP = (data.f_nom*sqrt(data.T_E_nom)/data.P_E_nom);
    Phi_n = Phi_c / (data.f_nom * sqrt(data.T_E_nom) / data.P_E_nom);
    N_n = omega / sqrt(T_E) / (data.omega_nom / sqrt(data.T_E_nom));
// Fluid properties
    propIn.p = P_E;
    propIn.h = h_E;
    propOut.p = P_L;
    propOut.h = h_L;
    T_E = propIn.T;
    T_L = propOut.T;
    h_iso = Air.isentropicEnthalpy(P_L, propIn.state);
// Boundary conditions
    f_E = inlet.f;
    f_L = -outlet.f;
    P_E = inlet.P;
    P_L = outlet.P;
    h_E = inStream(inlet.h_L);
    h_L = outlet.h_L;
    inlet.h_L = 0 "Not used, no flow reversal";
    shaft_a.phi = shaft_b.phi;
    omega = der(shaft_a.phi);
    tau = shaft_a.tau + shaft_b.tau;
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 100}, {-60, -100}, {60, -60}, {60, 60}, {60, 60}, {-60, 100}})}));
  end BaseCompressor;

  partial model BaseTurbine
    package ExhaustGas = Media.ExhaustGas;
    outer Components.Environment environment;
    replaceable parameter Types.TurbomachineData data "Turbine data";
    final parameter Modelica.SIunits.SpecificHeatCapacity cp_nom = 1000 "Nominal cp";
    final parameter Modelica.SIunits.PerUnit e = 0.28 "(gamma-1)/gamma";
    final parameter Modelica.SIunits.Power tau_nom = cp_nom * data.T_E_nom * (1 - (data.P_L_nom / data.P_E_nom) ^ e) * data.eta_nom * data.f_nom / data.omega_nom "Nominal torque";
    Interfaces.ExhaustPort inlet annotation(
      Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.ExhaustPort outlet annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.MassFlowRate f_L "Leaving mass flow rate";
    Modelica.SIunits.Pressure P_E "Entering pressure";
    Modelica.SIunits.Pressure P_L "Leaving pressure";
    Modelica.SIunits.MassFraction X_E[ExhaustGas.nX] "Entering gas composition";
    Modelica.SIunits.MassFraction X_L[ExhaustGas.nX] "Leaving gas composition";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering specific enthalpy";
    Modelica.SIunits.SpecificEnthalpy h_L "Leaving specific enthaly";
    Modelica.SIunits.SpecificEnthalpy h_iso "Isentropic enthalpy at outlet pressure";
    Modelica.SIunits.Temperature T_E "Inlet temperature";
    Modelica.SIunits.Temperature T_L "Outlet temperature";
    Modelica.SIunits.PerUnit PR(start = data.P_E_nom / data.P_L_nom) "Pressure ratio";
    Modelica.SIunits.PerUnit eta(start = 0.8) "Isentropic efficiency";
    Modelica.SIunits.Power W "Mechanical power";
    Modelica.SIunits.AngularVelocity omega "Angular velocity of shaft";
    Modelica.SIunits.Torque tau(start = tau_nom) "Torque applied onto the shaft (positive)";
    Modelica.SIunits.PerUnit PR_n(start = 1) "Normalized pressure ratio";
    Modelica.SIunits.PerUnit Phi_n(start = 1) "Normalized flow number";
    Modelica.SIunits.PerUnit N_n(start = 1) "Normalized corrected speed";
    Modelica.SIunits.PerUnit eta_n "Normalized Isentropic efficiency";
    ExhaustGas.BaseProperties propIn "Fluid properties at the inlet";
    ExhaustGas.BaseProperties propOut "Fluid properties at the outlet";
    parameter Real eta_mech = 0.99 "Mechanical efficiency";
    Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
// Balance equations
    f_E = f_L "Total mass balance";
    X_E = X_L "Single components mass balances";
    W = f_E * (h_E - h_L) "Energy balance";
// Definition of auxiliary quantities
    W = omega * tau;
    PR = P_E / P_L;
    PR_n = PR / (data.P_E_nom / data.P_L_nom);
    eta = (h_E - h_L) / (h_E - h_iso);
    eta_n = eta / data.eta_nom;
    Phi_n = f_E * sqrt(T_E) / P_E / (data.f_nom * sqrt(data.T_E_nom) / data.P_E_nom);
    N_n = omega / sqrt(T_E) / (data.omega_nom / sqrt(data.T_E_nom));
// Fluid properties
    propIn.p = P_E;
    propIn.h = h_E;
    propIn.X = X_E;
    propOut.p = P_L;
    propOut.h = h_L;
    propOut.X = X_E;
    T_E = propIn.T;
    T_L = propOut.T;
    h_iso = ExhaustGas.isentropicEnthalpy(P_L, propIn.state);
// Boundary conditions
    f_E = inlet.f;
    f_L = -outlet.f;
    P_E = inlet.P;
    P_L = outlet.P;
    h_E = inStream(inlet.h_L);
    h_L = outlet.h_L;
    X_E = inStream(inlet.X_L);
    X_L = outlet.X_L;
    inlet.h_L = 0 "Not used, no flow reversal";
    inlet.X_L = ExhaustGas.reference_X;
    omega = der(shaft.phi);
    tau = -shaft.tau / eta_mech;
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 60}, {-60, -60}, {60, -100}, {60, 100}, {60, 100}, {-60, 60}})}, coordinateSystem(initialScale = 0.1)));
  end BaseTurbine;

  partial model BaseCompressorBleed
  "Inspired from: Rick Hackney, Theoklis Nikolaidis, Alvise Pellegrini,
  A method for modelling compressor bleed in gas turbine analysis software,
  Applied Thermal Engineering, Volume 172, 2020"
    outer Components.Environment environment;
    package Air = Media.Air;
    replaceable parameter Types.TurbomachineData data "Compressor data";
    // For model initialization
    final parameter Modelica.SIunits.SpecificHeatCapacity cp_nom = 1000 "Nominal cp";
    final parameter Modelica.SIunits.PerUnit e = 0.28 "(gamma-1)/gamma";
    final parameter Modelica.SIunits.Power tau_nom = cp_nom * data.T_E_nom * ((data.P_L_nom / data.P_E_nom) ^ e - 1) / data.eta_nom * data.f_nom / data.omega_nom "Nominal torque";
    //For bleed offtakes
    parameter Integer Nbleed = 2 "Number of bleed offtake points within the compressor";
    parameter Integer Nstages = 6 "Number of stages";
    parameter Integer Nstages_Bleeds[Nbleed] "Stage after which bleed offtakes are taken, from compressor inlet to outlet";
    Interfaces.AirPort inlet "Air inlet port" annotation(
      Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.AirPort outlet "Air outlet port" annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.AirPort Bl_port[Nbleed] annotation(
      Placement(transformation(extent = {{-8, 70}, {12, 90}}), iconTransformation(extent = {{-8, 70}, {12, 90}})));
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.MassFlowRate f_L "Leaving mass flow rate";
    Modelica.SIunits.Pressure P_E(start = data.P_E_nom, fixed = false) "Entering pressure";
    Modelica.SIunits.Pressure P_L "Leaving pressure";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering pressure";
    Modelica.SIunits.SpecificEnthalpy h_L "Leaving pressure";
    Modelica.SIunits.SpecificEnthalpy h_iso "Isentropic enthalpy at outlet pressure";
    Modelica.SIunits.Temperature T_E "Inlet temperature";
    Modelica.SIunits.Temperature T_L "Outlet temperature";
    Modelica.SIunits.PerUnit PR "Pressure ratio";
    Modelica.SIunits.PerUnit eta "Isentropic efficiency";
    Modelica.SIunits.Power W "Mechanical power input to the compressor";
    Modelica.SIunits.AngularVelocity omega "Angular velocity of shaft";
    Modelica.SIunits.Torque tau(start = tau_nom) "Torque applied onto the shaft (positive)";
    Modelica.SIunits.PerUnit PR_n "Normalized pressure ratio";
    Modelica.SIunits.PerUnit Phi_n "Normalized flow number";
    //Real Phi_c_DP "Flow number @ design point";
    Real Phi_c "Flow number";
    Modelica.SIunits.PerUnit N_n(start = 1, fixed = false) "Normalized corrected speed";
    Modelica.SIunits.PerUnit eta_n "Normalized Isentropic efficiency";
    Air.BaseProperties propIn "Fluid properties at the inlet";
    Air.BaseProperties propOut "Fluid properties at the outlet";
    Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_a annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_b annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    //Variables pertaining to bleed offtakes calculations
    Modelica.SIunits.MassFlowRate f_bl[Nbleed] "Bleed air mass flow rate";
    Modelica.SIunits.SpecificEnthalpy h_bl[Nbleed] "Bleed air specific enthalpy";
    Modelica.SIunits.Pressure P_bl[Nbleed] "Bleed air pressure";
    Modelica.SIunits.Temperature DT_stage "Temperature increase per compressor stage";
    Modelica.SIunits.Temperature T_bl[Nbleed] "Bleed air temperature";
    Modelica.SIunits.PerUnit eta_poly "Polytropic efficiency";
    Modelica.SIunits.PerUnit gammaAVG;
    Air.BaseProperties prop_bl[Nbleed] "Fluid properties of bleed air";
  equation
// Balance equations
    f_E = f_L + sum(f_bl) "Mass balance";
    W = f_L * (h_L - h_E) + sum(f_bl .* (h_bl .- h_E)) "Energy balance";
//Definition of auxiliary quantities of bleed air offtakes
    DT_stage = (T_L - T_E) / Nstages;
    for i in 1:Nbleed loop
      T_bl[i] = T_E + DT_stage * Nstages_Bleeds[i];
      T_bl[i] / T_E = (P_bl[i] / P_E) ^ ((gammaAVG - 1) / gammaAVG / eta_poly);
    end for;
    eta = ((P_L / P_E) ^ ((gammaAVG - 1) / gammaAVG) - 1) / ((P_L / P_E) ^ ((gammaAVG - 1) / (eta_poly * gammaAVG)) - 1);
    gammaAVG = 0.5 * (Air.specificHeatCapacityCp(propIn.state) / Air.specificHeatCapacityCv(propIn.state) + Air.specificHeatCapacityCp(propOut.state) / Air.specificHeatCapacityCv(propOut.state));
// Definition of auxiliary quantities
    W = omega * tau;
    PR = P_L / P_E;
    PR_n = PR / (data.P_L_nom / data.P_E_nom);
    eta = (h_iso - h_E) / (h_L - h_E);
    eta_n = eta / data.eta_nom;
    Phi_c = f_E * sqrt(T_E) / P_E;
//Phi_c_DP = (data.f_nom*sqrt(data.T_E_nom)/data.P_E_nom);
    Phi_n = Phi_c / (data.f_nom * sqrt(data.T_E_nom) / data.P_E_nom);
    N_n = omega / sqrt(T_E) / (data.omega_nom / sqrt(data.T_E_nom));
// Fluid properties
    propIn.p = P_E;
    propIn.h = h_E;
    propOut.p = P_L;
    propOut.h = h_L;
    T_E = propIn.T;
    T_L = propOut.T;
    h_iso = Air.isentropicEnthalpy(P_L, propIn.state);
//Fluid properties Bleed air
    for i in 1:Nbleed loop
      prop_bl[i].p = P_bl[i];
      prop_bl[i].h = h_bl[i];
      prop_bl[i].T = T_bl[i];
    end for;
// Boundary conditions
    f_E = inlet.f;
    f_L = -outlet.f;
    P_E = inlet.P;
    P_L = outlet.P;
    h_E = inStream(inlet.h_L);
    h_L = outlet.h_L;
    inlet.h_L = 0 "Not used, no flow reversal";
    shaft_a.phi = shaft_b.phi;
    omega = der(shaft_a.phi);
    tau = shaft_a.tau + shaft_b.tau;
//Boundary conditions bleed ports
    for i in 1:Nbleed loop
      f_bl[i] = -Bl_port[i].f;
      h_bl[i] = Bl_port[i].h_L;
      P_bl[i] = Bl_port[i].P;
    end for;
    assert(Nstages_Bleeds[end] <= Nstages, "Bleed port location unfeasible");
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 100}, {-60, -100}, {60, -60}, {60, 60}, {60, 60}, {-60, 100}})}));
  end BaseCompressorBleed;

  partial model BaseCooledTurbine
    package ExhaustGas = Media.ExhaustGas;
    outer Components.Environment environment;
    replaceable parameter Types.TurbomachineData data "Turbine data";
    final parameter Modelica.SIunits.SpecificHeatCapacity cp_nom = 1000 "Nominal cp";
    final parameter Modelica.SIunits.PerUnit e = 0.28 "(gamma-1)/gamma";
    final parameter Modelica.SIunits.Power tau_nom = cp_nom * data.T_E_nom * (1 - (data.P_L_nom / data.P_E_nom) ^ e) * data.eta_nom * data.f_nom / data.omega_nom "Nominal torque";
    Interfaces.ExhaustPort inlet annotation(
      Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.ExhaustPort outlet annotation(
      Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Interfaces.AirPort Bl_port[Nbleed] annotation (Placement(
            transformation(extent={{-8,70},{12,90}}), iconTransformation(extent={
                {-8,70},{12,90}})));
    
    Modelica.SIunits.MassFlowRate f_E "Entering mass flow rate";
    Modelica.SIunits.MassFlowRate f_L "Leaving mass flow rate";
    Modelica.SIunits.Pressure P_E "Entering pressure";
    Modelica.SIunits.Pressure P_L "Leaving pressure";
    Modelica.SIunits.MassFraction X_E[ExhaustGas.nX] "Entering gas composition";
    Modelica.SIunits.MassFraction X_L[ExhaustGas.nX] "Leaving gas composition";
    Modelica.SIunits.SpecificEnthalpy h_E "Entering specific enthalpy";
    Modelica.SIunits.SpecificEnthalpy h_L "Leaving specific enthaly";
    Modelica.SIunits.SpecificEnthalpy h_iso "Isentropic enthalpy at outlet pressure";
    Modelica.SIunits.Temperature T_E "Inlet temperature";
    Modelica.SIunits.Temperature T_L "Outlet temperature";
    Modelica.SIunits.PerUnit PR(start = data.P_E_nom / data.P_L_nom) "Pressure ratio";
    Modelica.SIunits.PerUnit eta(start = 0.8) "Isentropic efficiency";
    Modelica.SIunits.Power W "Mechanical power";
    Modelica.SIunits.AngularVelocity omega "Angular velocity of shaft";
    Modelica.SIunits.Torque tau(start = tau_nom) "Torque applied onto the shaft (positive)";
    Modelica.SIunits.PerUnit PR_n(start = 1) "Normalized pressure ratio";
    Modelica.SIunits.PerUnit Phi_n(start = 1) "Normalized flow number";
    Modelica.SIunits.PerUnit N_n(start = 1) "Normalized corrected speed";
    Modelica.SIunits.PerUnit eta_n "Normalized Isentropic efficiency";
    ExhaustGas.BaseProperties propIn "Fluid properties at the inlet";
    ExhaustGas.BaseProperties propOut "Fluid properties at the outlet";
    parameter Real eta_mech = 0.99 "Mechanical efficiency";
    Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    //Additional parameters for cooling flow prediction
    parameter Integer Nstages = 4 "Number of stages";
    parameter Integer CoolingTechStat[Nstages] = ones(Nstages) "Define Cooling Tech of stator blades: 1 uncooled, 2 convection+coating, 3 film + conv.";
    parameter Integer CoolingTechRot[Nstages] = ones(Nstages) "Define Cooling Tech of rotor blades. Same convention of stator";
    parameter Modelica.SIunits.PerUnit Deta_stat[3] = {0, 0.1, 0.18};
    parameter Modelica.SIunits.PerUnit Deta_rot[3] = {0, 0.2, 0.36};
    parameter Boolean CoolingModel = true "if false, the cooling mass fraction is imposed";
    parameter Modelica.SIunits.Temperature Tblade = 1300;
    //For bleed offtakes
    parameter Integer Nbleed = 2 "Number of bleed air supplies";
    parameter Integer Nstages_Bleeds[Nbleed] = {2, Nstages} "The integers in the array indicate the last stage cooled by each bleed air supply. The last integer must be equal to Nstages, though the last stages may be  not cooled";
    parameter Modelica.SIunits.MassFraction Xair[ExhaustGas.nX] = ExhaustGas.reference_X;
    parameter Modelica.SIunits.MassFraction Xi_cool_stat[Nstages] = zeros(Nstages) "if CoolingModel = false, cooling fraction in each stator row";
    parameter Modelica.SIunits.MassFraction Xi_cool_rot[Nstages] = zeros(Nstages) "if CoolingModel = false, cooling fraction in each rotor row"; 
    //Additional variables for cooling flow prediction
    ExhaustGas.BaseProperties propInStage[Nstages] "Fluid properties at each stage inlet";
    ExhaustGas.BaseProperties propInStageCooled[Nstages] "Fluid properties at stage inlet along cooled expansion";
    ExhaustGas.BaseProperties propInRot[Nstages] "Fluid properties at rotor inlet along cooled expansion";
    Modelica.SIunits.Pressure P_E_stage[Nstages] "Entering pressure of each stage";
    parameter Modelica.SIunits.PerUnit PRnom = 1.3;
    parameter Modelica.SIunits.Pressure P_L_iso_start[Nstages] = data.P_E_nom ./PRnom.^linspace(1,Nstages,Nstages);
    Modelica.SIunits.Pressure P_L_iso[Nstages](start = P_L_iso_start) "Equivalent leaving pressure of an isentropic expansion";
    Modelica.SIunits.SpecificEnthalpy h_iso_stage[Nstages] "Isentropic Enthalpy at stage outlet for uncooled expansion";
    Modelica.SIunits.SpecificEnthalpy h_iso_stage_cooled[Nstages] "Isentropic Enthalpy at stage outlet for cooled expansion";
    Modelica.SIunits.SpecificEnthalpy h_E_rot[Nstages] "Enthalpy at rotor inlet  for cooled expansion";
    Modelica.SIunits.SpecificEnthalpy h_L_rot[Nstages] "Enthalpy at rotor outlet  for cooled expansion";
    Modelica.SIunits.SpecificEnthalpy h_L_uncooled "Leaving specific enthaly";
    Modelica.SIunits.Temperature T_E_rot[Nstages] "Temperature at rotor inlet  for cooled expansion";
    Modelica.SIunits.Temperature T_E_stat[Nstages] "Temperature at stator inlet  for cooled expansion";
    Modelica.SIunits.SpecificEnthalpy Dh_stage "Constant Dh through stages";
    Modelica.SIunits.PerUnit eta_poly(start = 0.8) "Polytropic efficiency of the uncooled expansion";
    Modelica.SIunits.PerUnit eta_is_stage[Nstages](each start = 0.8) "Isentropic efficiency of each uncooled stage";
    Modelica.SIunits.PerUnit eta_stage_cooled[Nstages](each start = 0.8) "Isentropic efficiency of each cooled stage";
    Modelica.SIunits.PerUnit mfrac_stat[Nstages] "Cooling flow mass fraction in the stator";
    Modelica.SIunits.PerUnit mfrac_rot[Nstages] "Cooling flow mass fraction in the rotor";
    // cooling model
    replaceable Components.TurbineCoolingModel.Gauntner StatorCooling[Nstages](each Tblade = Tblade, T_ca = T_ca, T_E = T_E_stat, CoolingTech = CoolingTechStat);
    replaceable Components.TurbineCoolingModel.Gauntner RotorCooling[Nstages](each Tblade = Tblade, T_ca = T_ca, T_E = T_E_rot, CoolingTech = CoolingTechRot);
    
    //cooling air
    Modelica.SIunits.MassFlowRate f_bl[Nbleed] "Bleed air mass flow rate";
    //Modelica.SIunits.Pressure P_bl[Nbleed] "Bleed air pressure";
    Modelica.SIunits.SpecificEnthalpy h_ca[Nstages] "Cooling air enthalpy";
    Media.Air.BaseProperties propInCoolAir[Nbleed] "Fluid properties of the cooling air streams";
    Modelica.SIunits.Temperature T_ca[Nstages] "Cooling air temperature";
  
  equation
// Balance equations
    f_E + sum(f_bl)= f_L "Total mass balance";
//X_E = X_L "Single components mass balances";
//W = f_E * (h_E - h_L) "Energy balance";
    W = f_E * (h_E - h_L) + sum(f_bl.*( inStream(Bl_port.h_L) - ones(Nbleed).*h_L)) "Energy balance";

  // Definition of auxiliary quantities
    W = omega * tau;
    PR = P_E / P_L;
    PR_n = PR / (data.P_E_nom / data.P_L_nom);
    // definition of isentropic efficiency for the uncooled machine
    eta = (h_E - h_L_uncooled) / (h_E - h_iso);
    eta_n = eta / data.eta_nom;
    Phi_n = f_E * sqrt(T_E) / P_E / (data.f_nom * sqrt(data.T_E_nom) / data.P_E_nom);
    N_n = omega / sqrt(T_E) / (data.omega_nom / sqrt(data.T_E_nom));

  // Fluid properties
    propIn.p = P_E;
    propIn.h = h_E;
    propIn.X = X_E;
    propOut.p = P_L;
    propOut.h = h_L;
    propOut.X = X_E;
    T_E = propIn.T;
    T_L = propOut.T;
    h_iso = ExhaustGas.isentropicEnthalpy(P_L, propIn.state);

  //Cooling air properties and mass flow rate
    for i in 1:Nbleed loop
      propInCoolAir[i].p = Bl_port[i].P;
      propInCoolAir[i].h = inStream(Bl_port[i].h_L);
      if i == 1 then
       f_bl[i]=(sum(mfrac_stat[1:Nstages_Bleeds[i]]) + sum(mfrac_rot[1:Nstages_Bleeds[i]]))*f_E;
        for j in 1:Nstages_Bleeds[i] loop
          h_ca[j] = propInCoolAir[i].h;
          T_ca[j] = propInCoolAir[i].T;
        end for;
      else
      f_bl[i]=(sum(mfrac_stat[Nstages_Bleeds[i-1]+1:Nstages_Bleeds[i]]) + sum(mfrac_rot[Nstages_Bleeds[i-1]+1:Nstages_Bleeds[i]]))*f_E;
        for j in Nstages_Bleeds[i - 1] + 1:Nstages_Bleeds[i] loop
          h_ca[j] = propInCoolAir[i].h;
          T_ca[j] = propInCoolAir[i].T;
        end for;
      end if;
    end for;
    
    
// Uncooled Expansion along the single stages
    Dh_stage = (h_E - h_L_uncooled) / Nstages;
    for i in 1:Nstages loop
      if i == Nstages then
        h_L_uncooled = ExhaustGas.isentropicEnthalpy(P_L_iso[i], propInStage[i].state);
// Definition of eta_poly for a gas with varying cp / gamma
        log(P_E_stage[i] / P_L) * eta_poly = log(P_E_stage[i] / P_L_iso[i]);
        propInStage[i].p = P_E_stage[i];
        if Nstages ==1 then
          P_E_stage[i]= P_E;
        end if;
        propInStage[i].h = h_L_uncooled + Dh_stage;
        h_iso_stage[i] = ExhaustGas.isentropicEnthalpy(P_L, propInStage[i].state);
      elseif i == 1 then
        propInStage[i].h = h_E;
        propInStage[i].p = P_E;
        P_E_stage[i] = P_E;
        propInStage[i + 1].h = ExhaustGas.isentropicEnthalpy(P_L_iso[i], propInStage[i].state);
        log(P_E / P_E_stage[i + 1]) * eta_poly = log(P_E / P_L_iso[i]);
        h_iso_stage[i] = ExhaustGas.isentropicEnthalpy(P_E_stage[i + 1], propInStage[i].state);
      else
        propInStage[i + 1].h = ExhaustGas.isentropicEnthalpy(P_L_iso[i], propInStage[i].state);
        log(P_E_stage[i] / P_E_stage[i + 1]) * eta_poly = log(P_E_stage[i] / P_L_iso[i]);
        propInStage[i].p = P_E_stage[i];
        propInStage[i].h = propInStage[i + 1].h + Dh_stage;
        h_iso_stage[i] = ExhaustGas.isentropicEnthalpy(P_E_stage[i + 1], propInStage[i].state);
      end if;
      propInStage[i].X = X_E;
      eta_is_stage[i] = Dh_stage / (propInStage[i].h - h_iso_stage[i]);
    end for;
    
//Cooled expansion through each blade row. A subset of turbine stages may be cooled, always starting from the first stage.
    for i in 1:Nstages loop
      if i == 1 then
        propInStageCooled[i].X = X_E;
        propInStageCooled[i].p = P_E;
        propInStageCooled[i].h = h_E;
      end if;
      //T_E_stat[i] = propInStageCooled[i].T;
      T_E_stat[i] = ExhaustGas.T_hX(propInStageCooled[i].h,propInStageCooled[i].X);
          
      if CoolingModel then
      // Cooling flow model
        mfrac_stat[i] = StatorCooling[i].mfrac;
        mfrac_rot[i] = RotorCooling[i].mfrac;
  
      else
      // Imposed mass fraction
        mfrac_stat[i] = Xi_cool_stat[i];
        mfrac_rot[i] = Xi_cool_rot[i];
      end if;

      //Mixing at stator outlet - calculation of total enthalpy at the rotor inlet
      (1 + mfrac_stat[i]) * h_E_rot[i] = propInStageCooled[i].h + mfrac_stat[i] * h_ca[i];
      propInRot[i].h = h_E_rot[i];
      (1 + mfrac_stat[i]) * propInRot[i].X = propInStageCooled[i].X + mfrac_stat[i] * Xair;
      // Total temperature at rotot inlet; 0.92 factor accounts for lower relative speed at rotor inlet (from Gauntner 1980)
      propInRot[i].p = propInStageCooled[i].p;
      //Total pressure not conserved, but irrelevant for calculation of h_tot
      T_E_rot[i] = 0.92*propInRot[i].T;
      
      // Determine efficiency of the cooled stage (from Gauntner)
      eta_stage_cooled[i] = eta_is_stage[i] - mfrac_stat[i] * Deta_stat[CoolingTechStat[i]] * eta_is_stage[i] - mfrac_rot[i] * Deta_rot[CoolingTechRot[i]] * eta_is_stage[i];

      //Work extraction
      // It is assumed that PR does not change with respect to the uncooled stage
// Ptot is not conserved --> approximantion in estimation of entropy at the stage inlet. Better approach not possible, as velocities of the 2 streams are unknown.
      if i == Nstages then
        h_iso_stage_cooled[i] = ExhaustGas.isentropicEnthalpy(P_L, propInRot[i].state);
      else
        h_iso_stage_cooled[i] = ExhaustGas.isentropicEnthalpy(P_E_stage[i + 1], propInRot[i].state);
      end if;
      eta_stage_cooled[i] = (h_E_rot[i] - h_L_rot[i]) / (h_E_rot[i] - h_iso_stage_cooled[i]);
    
      //Mixing at rotor outlet
      if i == Nstages then
        (1 + mfrac_rot[i]) * h_L = h_L_rot[i] + mfrac_rot[i] * h_ca[i];
        (1 + mfrac_rot[i]) * X_L = propInRot[i].X + mfrac_rot[i] * Xair;                        
      else
        (1 + mfrac_rot[i]) * propInStageCooled[i + 1].h = h_L_rot[i] + mfrac_rot[i] * h_ca[i];
        (1 + mfrac_rot[i]) * propInStageCooled[i + 1].X = propInRot[i].X + mfrac_rot[i] * Xair;
        propInStageCooled[i + 1].p = P_E_stage[i + 1];
      end if;
    end for;
    
// Boundary conditions
    f_E = inlet.f;
    f_L = -outlet.f;
    P_E = inlet.P;
    P_L = outlet.P;
    h_E = inStream(inlet.h_L);
    h_L = outlet.h_L;
    X_E = inStream(inlet.X_L);
    X_L = outlet.X_L;
    inlet.h_L = 0 "Not used, no flow reversal";
    inlet.X_L = ExhaustGas.reference_X;
    omega = der(shaft.phi);
    tau = -shaft.tau / eta_mech;

  //Boundary conditions bleed ports
  for i in 1:Nbleed loop
    f_bl[i] = Bl_port[i].f;
    Bl_port[i].h_L = 0 "Not used, no flow reversal";
//    //h_bl[i] = Bl_port[i].h_L;
//  //P_bl[i] = Bl_port[i].P;
  end for;
    annotation(
      Icon(graphics = {Polygon(fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-60, 60}, {-60, -60}, {60, -100}, {60, 100}, {60, 100}, {-60, 60}})}, coordinateSystem(initialScale = 0.1)));
  end BaseCooledTurbine;
end BaseClasses;
