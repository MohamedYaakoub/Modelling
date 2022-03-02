within BasicAeroEngines;

package Tests "Test models"
  extends Modelica.Icons.ExamplesPackage;

  package TestComponents "Test benches for individual library components"
    extends Modelica.Icons.ExamplesPackage;

    model TestAirIntake "Test bench of air intake model"
      extends Modelica.Icons.Example;
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-20, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir flowSource(referenceMassFlowRate = 10) annotation(
        Placement(visible = true, transformation(origin = {18, 8}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      inner BasicAeroEngines.Components.Environment environment(altitude = 10000, airspeed = 100, Mach = 0.8, useMach = true) annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(airIntake.outlet, flowSource.fluidPort) annotation(
        Line(points = {{-14, 8}, {6, 8}, {6, 8}, {8, 8}}, color = {170, 223, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Documentation(info = "<html>
<p>Test of the <a href=\"modelica://BasicAeroEngines.Components.AirIntake\">AirIntake</a> component.</p>
<p>The air intake moves at 100 m/s; correspondingly, the pressure at the outlet is increased by about 6000 Pa and the temperature by about 5 K with respect to the environment values.</p>
</html>"));
    end TestAirIntake;

    model TestFan "Test bench of Fan model"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir sources(P = P_E, each referencePressure = GSP_c.P_E_nom, each referenceTemperature = GSP_c.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {-56, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir sinks(each referencePressure = GSP_c.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {32, -44}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.SIunits.Pressure P_E = GSP_c.P_E_nom * (1 + time / 10) "Inlet pressures are free to change";
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed(w_fixed = 355) annotation(
        Placement(visible = true, transformation(origin = {46, -64}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
      Components.Fan fan(data_bypass = GSP_d, data_core = GSP_c) annotation(
        Placement(transformation(extent = {{-48, -74}, {30, 4}})));
      Components.PressureSourceAir sinks1(each referencePressure = GSP_d.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {42, -16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Data.Compressors.GSP_FanCore GSP_c annotation(
        Placement(visible = true, transformation(origin = {-40, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Data.Compressors.GSP_FanDuct GSP_d annotation(
        Placement(visible = true, transformation(origin = {36, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sources.fluidPort, fan.inlet) annotation(
        Line(points = {{-46, -36}, {-34, -36}, {-34, -35}, {-23.625, -35}}, color = {170, 223, 255}));
      connect(sinks.fluidPort, fan.outlet_core) annotation(
        Line(points = {{22, -44}, {14, -44}, {14, -44.75}, {0.75, -44.75}}, color = {170, 223, 255}));
      connect(constSpeed.flange, fan.fan_mechanical_interface) annotation(
        Line(points = {{40, -64}, {22, -64}, {22, -64.25}, {10.5, -64.25}}, color = {0, 0, 0}));
      connect(fan.outlet_bypass, sinks1.fluidPort) annotation(
        Line(points = {{0.75, -15.5}, {15.2, -15.5}, {15.2, -16}, {32, -16}}, color = {170, 223, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Documentation);
    end TestFan;

    model TestNozzleExhaust "Test bench of Nozzle Exhaust model"
      extends Modelica.Icons.Example;
      BasicAeroEngines.Components.NozzleExhaust nozzle(A = 0.2) annotation(
        Placement(visible = true, transformation(origin = {16, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.FlowSourceExhaust flowSource(referenceMassFlowRate = 30, referenceTemperature = 673.15) annotation(
        Placement(visible = true, transformation(origin = {-22, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner BasicAeroEngines.Components.Environment environment(P(displayUnit = "Pa"), useMach = true) annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(flowSource.fluidPort, nozzle.inlet) annotation(
        Line(points = {{-12, 0}, {10, 0}, {10, 0}, {10, 0}}, color = {129, 170, 194}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Documentation(info = "<html><head></head><body><p>This model tests the <a href=\"modelica://BasicAeroEngines.Components.NozzleExhaust\">NozzleExhaust</a> model.</p>
<p>The flow source at 400 degC injects 30 kg/s of exhaust gases into a nozzle with outlet cross section of 0.2 m2, discharging at one atmosphere. The resulting inlet pressure is 1.233 bar, while the outlet temperature is reduced to 365.3 degC after the expansion. The velocity at the nozzle exhaust is 272.4 m/s, and the thrust is the exhaust gas is 8173 N:</p></body></html>"));
    end TestNozzleExhaust;

    model TestCombustionChamberLHVSimple "Test bench of CombustionChamberLHVSimple"
      extends Modelica.Icons.Example;
      Components.FlowSourceAir airFlowSource(referenceMassFlowRate = 30, referenceTemperature = 673.15) annotation(
        Placement(visible = true, transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.CombustionChamberLHVSimple cc(P_start = 1e+06, T_start = 773.15, V = 0.1) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant fuelFlow(k = 0.4) annotation(
        Placement(visible = true, transformation(origin = {-32, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.LinearPressureDropExhaust pressureDrop(referenceMassFlowRate = 30, referencePressureDrop = 900000) annotation(
        Placement(visible = true, transformation(origin = {42, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust sink annotation(
        Placement(visible = true, transformation(origin = {80, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    equation
      connect(pressureDrop.outlet, sink.fluidPort) annotation(
        Line(points = {{52, 0}, {68, 0}, {68, 0}, {70, 0}}, color = {129, 170, 194}));
      connect(cc.exhaust, pressureDrop.inlet) annotation(
        Line(points = {{10, 0}, {30, 0}, {30, 0}, {32, 0}, {32, 0}}, color = {129, 170, 194}));
      connect(airFlowSource.fluidPort, cc.airInlet) annotation(
        Line(points = {{-34, 0}, {-12, 0}, {-12, 0}, {-10, 0}}, color = {170, 223, 255}));
      connect(fuelFlow.y, cc.fuelFlow) annotation(
        Line(points = {{-21, 32}, {0, 32}, {0, 12}, {0, 12}, {0, 10}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 0.2, Tolerance = 1e-6, Interval = 0.0004),
        Documentation(info = "<html><head></head><body><p>Test of <a href=\"modelica://BasicAeroEngines.Components.CombustionChamberLHVSimple\">CombustionChamberLHVSimple</a>.</p>
<p>The combustion chamber starts at T = 500 degC, then it reaches the equilibrium temperature of 891 degC in about 0.1 seconds.</p>
</body></html>"));
    end TestCombustionChamberLHVSimple;

    model TestCombustionChamberLHV "Test bench of CombustionChamberLHV"
      extends Modelica.Icons.Example;
      Components.CombustionChamberLHV cc(LHV = 41.6e6, P_start = 1000000, T_start = 673.15, V = 0.1, ZC = 1, ZH = 4) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.FlowSourceAir airFlowSource(referenceMassFlowRate = 158, referenceTemperature = 616.95) annotation(
        Placement(visible = true, transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.LinearPressureDropExhaust pressureDrop(referenceMassFlowRate = 54, referencePressureDrop = 1200000) annotation(
        Placement(visible = true, transformation(origin = {42, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.PressureSourceExhaust sink annotation(
        Placement(visible = true, transformation(origin = {80, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Pulse pulse(amplitude = -0.3, period = 0.5, nperiod = 1, offset = 3.1, startTime = 0.5) annotation(
        Placement(transformation(extent = {{-42, 20}, {-22, 40}})));
    equation
      connect(pressureDrop.outlet, sink.fluidPort) annotation(
        Line(points = {{52, 0}, {68, 0}, {68, 0}, {70, 0}}, color = {129, 170, 194}));
      connect(cc.exhaust, pressureDrop.inlet) annotation(
        Line(points = {{10, 0}, {30, 0}, {30, 0}, {32, 0}, {32, 0}}, color = {129, 170, 194}));
      connect(airFlowSource.fluidPort, cc.airInlet) annotation(
        Line(points = {{-34, 0}, {-12, 0}, {-12, 0}, {-10, 0}}, color = {170, 223, 255}));
      connect(pulse.y, cc.fuelFlow) annotation(
        Line(points = {{-21, 30}, {0, 30}, {0, 10}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 0.2, Tolerance = 1e-6, Interval = 0.0004),
        Documentation(info = "<html><head></head><body><p>Test of <a href=\"modelica://BasicAeroEngines.Components.CombustionChamberLHV\">CombustionChamberLHV</a>.</p>
<p>The combustion chamber starts at T = 500 degC, then it reaches the equilibrium temperature of 996 degC in about 0.1 seconds.</p>
</body></html>"));
    end TestCombustionChamberLHV;

    model TestSampleCompressorReplaceable "Compressor test bench with replaceable CompressorData dataset, defaults to SampleCompressor"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      replaceable record CompressorData = Data.Compressors.SampleCompressor constrainedby BasicAeroEngines.Types.CompressorMapsBetaLinesData;
      parameter CompressorData compressorData annotation(
        Placement(transformation(extent = {{-14, 48}, {6, 68}})));
      parameter Integer Nv = size(compressorData.Phi_n_table, 1) - 1 "Number of values for N_n in the data record";
      parameter Modelica.SIunits.PerUnit N_n_min = compressorData.Phi_n_table[2, 1] "Minimum value of N_n in the data record";
      parameter Modelica.SIunits.PerUnit N_n_max = compressorData.Phi_n_table[end, 1] "Minimum value of N_n in the data record";
      parameter Integer N = 2 * Nv - 1 "Number of test cases, one for each N_n value plus one for each intermediate value";
      parameter Modelica.SIunits.PerUnit N_n[N] = linspace(N_n_min, N_n_max, N) "Angular velocity will be fixed for each test instance";
      BasicAeroEngines.Components.PressureSourceAir sources[N](P = P_E, each referencePressure = compressorData.P_E_nom, each referenceTemperature = compressorData.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {-42, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines compressors[N](each data = compressorData) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir sinks[N](each referencePressure = compressorData.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {66, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.SIunits.Pressure P_E[N] "Inlet pressures are free to change";
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed[N](w_fixed = N_n * compressorData.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-52, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      compressors.beta = (compressorData.beta_surge + time * (compressorData.beta_choke - compressorData.beta_surge)) * ones(N);
      connect(compressors.outlet, sinks.fluidPort) annotation(
        Line(points = {{6, 6}, {14, 6}, {14, 20}, {56, 20}, {56, 20}}, color = {170, 223, 255}, thickness = 0.5));
      connect(sources.fluidPort, compressors.inlet) annotation(
        Line(points = {{-32, 10}, {-6, 10}}, color = {170, 223, 255}, thickness = 0.5));
      connect(compressors.shaft_a, constSpeed.flange) annotation(
        Line(points = {{-6, 0}, {-26, 0}, {-26, -20}, {-46, -20}}, color = {0, 0, 0}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Documentation(info = "<html><head></head><body><p>Test of the <a href=\"modelica://BasicAeroEngines.Components.CompressorMapsBetaLines\">CompressorMapsBetaLines</a> component, with data taken from <a href=\"modelica://BasicAeroEngines.Data.Compressors.TestCompressor\">TestCompressor</a>.</p>
  <p>The test is automatically set up based on the data record, to simulate <code>N</code> parallel instances, each with a different value of <code>N_n</code>, spanning from the minimum to the maximum and including some intermediate values besides the ones included in the map data.</p>
  <p>To check the model, you can use 2D plots with <code>Pcompressors[j].Phi_n</code>on the abscissa and <code>Pcompressors[j].PR_n</code>, one plot for each j corresponding to a specific value of the reduced speed. You can also plot the values of <code>eta</code> against, e.g., <code>beta</code> values.</p>
  <p>In order to test other compressor data sets, you can define a new data record for your compressor, e.g. <code>MyCompressorData</code>, extend this model and redeclare the data record:
  </p><pre>  model TestMyCompressor
  extends BasicAeroEngines.Test.TestCompressor(
    redeclare record CompressorData = MyCompressorData);
  annotation(experiment(StopTime = 1, Interval = 0.01));
  end TestMyCompressor;
  </pre><p></p>
  </body></html>"));
    end TestSampleCompressorReplaceable;

    model Test_GSP_FanCore_Compressor "Compressor test bench with GSP_FanCore dataset"
      extends BasicAeroEngines.Tests.TestComponents.TestSampleCompressorReplaceable(redeclare record CompressorData = BasicAeroEngines.Data.Compressors.GSP_FanCore);
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Documentation(info = "<html><head></head><body><p>Test of the <a href=\"modelica://BasicAeroEngines.Components.CompressorMapsBetaLines\">CompressorMapsBetaLines</a> component, with data taken from&nbsp;<a href=\"modelica://BasicAeroEngines.Data.Compressors.GSP_FanCore\">GSP_FanCore</a>.</p>
<p>This test model extends <a href=\"modelica://BasicAeroEngines.Tests.TestCompressor\">TestCompressor</a>, by redeclaring the compressor map.</p>
<p>To check the model, you can use 2D plots with <code>Pcompressors[j].Phi_n</code>on the abscissa and <code>Pcompressors[j].PR_n</code>, one plot for each j corresponding to a specific value of the reduced speed. You can also plot the values of <code>eta</code> against, e.g., <code>beta</code> values.</p>
</body></html>"));
    end Test_GSP_FanCore_Compressor;

    model Test_GSP_HPC_Compressor "Compressor test bench with GSP_HPC dataset"
      extends BasicAeroEngines.Tests.TestComponents.TestSampleCompressorReplaceable(redeclare record CompressorData = BasicAeroEngines.Data.Compressors.GSP_HPC, each compressors.useHomotopy = true);
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Documentation(info = "<html><head></head><body><p>Test of the <a href=\"modelica://BasicAeroEngines.Components.CompressorMapsBetaLines\">CompressorMapsBetaLines</a> component, with data taken from&nbsp;<a href=\"modelica://BasicAeroEngines.Data.Compressors.GSP_HPC\">GSP_HPC</a>.</p>
  <p>This test model extends <a href=\"modelica://BasicAeroEngines.Tests.TestCompressor\">TestCompressor</a>, by redeclaring the compressor map.</p>
  <p>To check the model, you can use 2D plots with <code>Pcompressors[j].Phi_n</code>on the abscissa and <code>Pcompressors[j].PR_n</code>, one plot for each j corresponding to a specific value of the reduced speed. You can also plot the values of <code>eta</code> against, e.g., <code>beta</code> values.</p>
  </body></html>"));
    end Test_GSP_HPC_Compressor;

    model Test_GSP_LPC_Compressor "Compressor test bench with GSP_LPC dataset"
      extends BasicAeroEngines.Tests.TestComponents.TestSampleCompressorReplaceable(redeclare record CompressorData = BasicAeroEngines.Data.Compressors.GSP_LPC);
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Documentation(info = "<html><head></head><body><p>Test of the <a href=\"modelica://BasicAeroEngines.Components.CompressorMapsBetaLines\">CompressorMapsBetaLines</a> component, with data taken from&nbsp;<a href=\"modelica://BasicAeroEngines.Data.Compressors.GSP_LPC\">GSP_LPC</a>.</p>
  <p>This test model extends <a href=\"modelica://BasicAeroEngines.Tests.TestCompressor\">TestCompressor</a>, by redeclaring the compressor map.</p>
  <p>To check the model, you can use 2D plots with <code>Pcompressors[j].Phi_n</code>on the abscissa and <code>Pcompressors[j].PR_n</code>, one plot for each j corresponding to a specific value of the reduced speed. You can also plot the values of <code>eta</code> against, e.g., <code>beta</code> values.</p>
  </body></html>"));
    end Test_GSP_LPC_Compressor;

    model TestSampleTurbine "Turbine test bench with SampleTurbine data"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Turbines.SampleTurbine turbineData annotation(
        Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Integer Nv = size(turbineData.eta_table, 1) - 1 "Number of values for N_n in the data record";
      parameter Modelica.SIunits.PerUnit N_n_min = turbineData.eta_table[2, 1] "Minimum value of N_n in the data record";
      parameter Modelica.SIunits.PerUnit N_n_max = turbineData.eta_table[end, 1] "Minimum value of N_n in the data record";
      parameter Modelica.SIunits.PerUnit Phi_n_min = 0.4 "Minimum value of Phi_n for the experiment";
      parameter Modelica.SIunits.PerUnit Phi_n_max = 1.0 "Minimum value of Phi_n for the experiment";
      parameter Integer N = 2 * Nv - 1 "Number of test cases, one for each N_n value plus one for each intermediate value";
      parameter Modelica.SIunits.PerUnit N_n[N] = linspace(N_n_min, N_n_max, N) "Angular velocity will be fixed for each test instance";
      BasicAeroEngines.Components.PressureSourceExhaust sources[N](P = P_E, each referencePressure = turbineData.P_E_nom, each referenceTemperature = turbineData.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {-42, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.TurbineStodola turbines[N](each data = turbineData) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust sinks[N](each referencePressure = turbineData.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {66, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.SIunits.Pressure P_E[N] "Inlet pressures are free to change";
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed[N](w_fixed = N_n * turbineData.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-42, -16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      turbines.Phi_n = (Phi_n_min + time * (Phi_n_max - Phi_n_min)) * ones(N);
      connect(turbines.shaft, constSpeed.flange) annotation(
        Line(points = {{-6, 0}, {-20, 0}, {-20, -16}, {-36, -16}}, thickness = 0.5));
      connect(turbines.outlet, sinks.fluidPort) annotation(
        Line(points = {{6, 10}, {14, 10}, {14, 20}, {56, 20}, {56, 20}}, color = {170, 223, 255}, thickness = 0.5));
      connect(sources.fluidPort, turbines.inlet) annotation(
        Line(points = {{-32, 10}, {-20, 10}, {-20, 6}, {-6, 6}}, color = {170, 223, 255}, thickness = 0.5));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Documentation(info = "<html><head></head><body><p>Test of the <a href=\"modelica://BasicAeroEngines.Components.TurbineStodolaEtaMap\">TurbineStodolaEtaMap</a> component, with data taken from <a href=\"modelica://BasicAeroEngines.Data.Turbines.TestTurbine\">TestTurbine</a>.</p>
  <p>The test is automatically set up based on the data record, to simulate <code>N</code> parallel instances, each with a different value of <code>N_n</code>, spanning from the minimum to the maximum and including some intermediate values besides the ones included in the map data.</p>
  <p>To check the model, you can use 2D plots with <code>turbines[j].PR</code>&nbsp;on the abscissa and <code>turbines[j].Phi</code>&nbsp;on the ordinate, one plot for each j corresponding to a specific value of the reduced speed. In fact, since the inlet temperature is the same, these curves will be superimposed. You can also plot the values of <code>eta</code> against, e.g., <code>Phi_n</code> values; in this case, you get different curves for each instance, i.e. for each value of the normalized corrected speed <code>N_m</code>.</p>
  <p>In order to test other turbine data sets, you can define a new data set record, e.g. <code>MyTurbineData</code>, extend this model and redeclare the data record:
  </p><pre>  model TestMyTurbine
  extends BasicAeroEnginesTest.TestTurbine(
    redeclare MyTurbineData turbineData);
  end TestMyCompressor;
  </pre><p></p>
</body></html>"));
    end TestSampleTurbine;

    model TestSecondaryAirDistribution "Test bench of air intake model"
      extends Modelica.Icons.Example;
      parameter Integer Nbleed = 2;
      parameter Integer Nbleed_2 = 4;
      parameter Modelica.SIunits.Pressure P_E_2[Nbleed_2] = {2e5, 4e5, 6e5, 8e5} "Bleed pressure";
      parameter Modelica.SIunits.Pressure P_E[Nbleed] = {2e5, 4e5} "Bleed pressure";
      parameter Modelica.SIunits.Temperature T_E[Nbleed] = {298, 398} "Bleed Temperature";
      parameter Modelica.SIunits.Temperature T_E_2[Nbleed_2] = {298, 398, 498, 598} "Bleed Temperature";
      parameter Modelica.SIunits.MassFlowRate f_E[2] = {-3, -4};
      //1st test
      BasicAeroEngines.Components.PressureSourceAir sources[Nbleed](P = P_E, T = T_E) annotation(
        Placement(visible = true, transformation(origin = {-86, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir LPT[2](each referenceMassFlowRate = 1, f = f_E) annotation(
        Placement(visible = true, transformation(extent = {{-86, -30}, {-66, -10}}, rotation = 0)));
      BasicAeroEngines.Components.BleedAirDistributor bleedAirDistributor(HandBleedPorts = 0, NPortsLPTBleed = {1, 2}, Nbleed = 2, useHandBleed = false, useLPTBleed = true) annotation(
        Placement(visible = true, transformation(extent = {{-70, -18}, {-22, 12}}, rotation = 0)));
      //2nd tests
      BasicAeroEngines.Components.BleedAirDistributor bleedAirDistributor_2(HandBleedPorts = 1, NPortsHandBleed = {1}, NPortsLPTBleed = {3, 4}, NPortsOverBleed = {2}, Nbleed = 4, OverBleedPorts = 1, useHandBleed = true, useLPTBleed = true, useOverBleed = true) annotation(
        Placement(visible = true, transformation(extent = {{46, -10}, {94, 20}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir LPT_2[2](f = f_E, each referenceMassFlowRate = 1) annotation(
        Placement(visible = true, transformation(extent = {{24, -42}, {44, -22}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir sources_2[Nbleed_2](P = P_E_2, T = T_E_2) annotation(
        Placement(visible = true, transformation(origin = {22, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir HandlingBleed[1](each referenceMassFlowRate = -1) annotation(
        Placement(visible = true, transformation(origin = {8, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir OverboardBleed[1](each referenceMassFlowRate = -2) annotation(
        Placement(visible = true, transformation(origin = {8, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(sources.fluidPort, bleedAirDistributor.Bl_port) annotation(
        Line(points = {{-76, 28}, {-46, 28}, {-46, 9}, {-45.52, 9}}, color = {170, 223, 255}));
      connect(bleedAirDistributor.LPTBleed, LPT.fluidPort) annotation(
        Line(points = {{-42.64, -3}, {-42.64, -21}, {-66, -21}, {-66, -20}}, color = {170, 223, 255}, thickness = 0.5));
      connect(bleedAirDistributor_2.LPTBleed, LPT_2.fluidPort) annotation(
        Line(points = {{74, 6}, {72, 6}, {72, -32}, {44, -32}}, color = {170, 223, 255}, thickness = 0.5));
      connect(sources_2.fluidPort, bleedAirDistributor_2.Bl_port) annotation(
        Line(points = {{32, 44}, {70, 44}, {70, 18}}, color = {170, 223, 255}, thickness = 0.5));
      connect(HandlingBleed.fluidPort, bleedAirDistributor_2.HandBleed) annotation(
        Line(points = {{18, -4}, {60, -4}, {60, 6}}, color = {170, 223, 255}, thickness = 0.5));
      connect(bleedAirDistributor_2.OverBleed, OverboardBleed.fluidPort) annotation(
        Line(points = {{66, 6}, {66, -24}, {18, -24}}, color = {170, 223, 255}, thickness = 0.5));
    end TestSecondaryAirDistribution;

    model TestCompressorBleeds
      extends Modelica.Icons.Example;
      parameter Modelica.SIunits.MassFlowRate fnom = 70.955;
      parameter Modelica.SIunits.Efficiency EisL = 0.82;
      parameter Modelica.SIunits.Efficiency EisH = 0.85;
      parameter Modelica.SIunits.Pressure Plpc_E = 82700;
      parameter Modelica.SIunits.Temperature Tlpc_E = 253.03;
      parameter Modelica.SIunits.Pressure Plpc_L = 3.95 * Plpc_E;
      parameter Modelica.SIunits.Pressure Phpc_E = Plpc_L * 0.98;
      parameter Modelica.SIunits.Pressure Phpc_L = 3.5 * Phpc_E;
      parameter Modelica.SIunits.Temperature Thpc_E = Tlpc_E * (1 + 1 / 0.823 * ((Plpc_L / Plpc_E) ^ (0.4 / 1.4) - 1));
      BasicAeroEngines.Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-74, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir flowSourceAir(referenceMassFlowRate = (-0.88 * fnom) + 8, referenceTemperature = 871.15) annotation(
        Placement(visible = true, transformation(origin = {30, 4}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed1(w_fixed = LPCmap.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-12, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed2(w_fixed = HPCmap.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {42, -16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = LPCmap) annotation(
        Placement(visible = true, transformation(origin = {-46, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_HPC HPCmap(P_E_nom(displayUnit = "Pa") = Phpc_E, P_L_nom = Phpc_L, T_E_nom(displayUnit = "K") = Thpc_E, eta_nom = EisH, f_nom = fnom, omega_nom = 9235) annotation(
        Placement(visible = true, transformation(origin = {-6, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC LPCmap(P_E_nom(displayUnit = "Pa") = Plpc_E, P_L_nom = Plpc_L, T_E_nom(displayUnit = "K") = Tlpc_E, eta_nom = EisL, f_nom = fnom, omega_nom = 5955) annotation(
        Placement(visible = true, transformation(origin = {-36, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner BasicAeroEngines.Components.Environment environment(Mach = 0.147, P(displayUnit = "Pa"), Pb(displayUnit = "Pa") = 103900, Tb(displayUnit = "K") = 263.93, altitude = 1844, useMach = true) annotation(
        Placement(visible = true, transformation(origin = {66, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.FlowSourceAir HPTBleed(referenceMassFlowRate = -0.12 * fnom, referenceTemperature(displayUnit = "K") = 598) annotation(
        Placement(visible = true, transformation(origin = {30, 22}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.LinearPressureDropAir PressureDrop(referenceMassFlowRate = fnom, referencePressureDrop = 0.02 * Plpc_L) annotation(
        Placement(visible = true, transformation(origin = {-24, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorBleed HPC(Nbleed = 3, Nstages = 9, Nstages_Bleeds = {1, 4, 9}, data = HPCmap) annotation(
        Placement(visible = true, transformation(origin = {-2, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.BleedAirDistributor SecondaryAirDistributor(useHandBleed = true, useLPTBleed = true) annotation(
        Placement(visible = true, transformation(origin = {-32, 38}, extent = {{-24, -20}, {24, 20}}, rotation = 180)));
      BasicAeroEngines.Components.FlowSourceAir LPT[2](f = f_E, each referenceMassFlowRate = 1) annotation(
        Placement(visible = true, transformation(extent = {{-82, 48}, {-62, 68}}, rotation = 0)));
      parameter Modelica.SIunits.MassFlowRate f_E[2] = {-3, -4};
      BasicAeroEngines.Components.FlowSourceAir HandlingBleed[1](each referenceMassFlowRate = -1) annotation(
        Placement(visible = true, transformation(origin = {2, 56}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    equation
      connect(airIntake.outlet, LPC.inlet) annotation(
        Line(points = {{-68, -40}, {-52, -40}}, color = {170, 223, 255}));
      connect(LPC.outlet, PressureDrop.inlet) annotation(
        Line(points = {{-40, -44}, {-40, -6}, {-34, -6}}, color = {170, 223, 255}));
      connect(HPC.inlet, PressureDrop.outlet) annotation(
        Line(points = {{-8, -6}, {-14, -6}}, color = {170, 223, 255}));
      connect(constantSpeed1.flange, LPC.shaft_b) annotation(
        Line(points = {{-22, -50}, {-40, -50}}));
      connect(HPC.shaft_b, constantSpeed2.flange) annotation(
        Line(points = {{4, -16}, {32, -16}}));
      connect(flowSourceAir.fluidPort, HPC.outlet) annotation(
        Line(points = {{20, 4}, {4, 4}, {4, -10}}, color = {170, 223, 255}));
      connect(HPTBleed.fluidPort, HPC.outlet) annotation(
        Line(points = {{20, 22}, {4, 22}, {4, -10}}, color = {170, 223, 255}));
      connect(SecondaryAirDistributor.Bl_port, HPC.Bl_port) annotation(
        Line(points = {{-32, 22}, {-32, 10}, {-2, 10}, {-2, -8}}, color = {170, 223, 255}, thickness = 0.5));
      connect(HandlingBleed.fluidPort, SecondaryAirDistributor.HandBleed) annotation(
        Line(points = {{-8, 56}, {-22, 56}, {-22, 38}}, color = {170, 223, 255}, thickness = 0.5));
      connect(LPT.fluidPort, SecondaryAirDistributor.LPTBleed) annotation(
        Line(points = {{-62, 58}, {-36, 58}, {-36, 38}}, color = {170, 223, 255}, thickness = 0.5));
    end TestCompressorBleeds;

    model TestCooledVSUncooledTurbine
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CooledTurbine cooledTurbine(CoolingModel = false, CoolingTechRot = {3, 2, 2, 1}, CoolingTechStat = {3, 2, 2, 1}, Nstages = 4, Nstages_Bleeds = {2, 4}, Xi_cool_rot = {0.04, 0.01, 0, 0}, Xi_cool_stat = {0.06, 0.02, 0, 0}, data = turbineData) annotation(
        Placement(visible = true, transformation(origin = {47, -15}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust sources(referencePressure = turbineData.P_E_nom, referenceTemperature = turbineData.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust sinks(referencePressure = turbineData.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {90, 18}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed(w_fixed = turbineData.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {1, -33}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines turbineData annotation(
        Placement(visible = true, transformation(origin = {52, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir pressureSourceAir[2](each referencePressure = 10e5, each referenceTemperature = 287.15) annotation(
        Placement(visible = true, transformation(origin = {28, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed = turbineData.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-117, -31}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaust(referencePressure = turbineData.P_E_nom, referenceTemperature = turbineData.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {-108, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaust1(referencePressure = turbineData.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {-28, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.TurbineMapsBetaLines UncooledTurbine(data = turbineData) annotation(
        Placement(visible = true, transformation(origin = {-71, -13}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
    equation
      connect(sources.fluidPort, cooledTurbine.inlet) annotation(
        Line(points = {{20, 20}, {36, 20}, {36, -4}}, color = {129, 170, 194}));
      connect(sinks.fluidPort, cooledTurbine.outlet) annotation(
        Line(points = {{80, 18}, {58, 18}, {58, 2}}, color = {129, 170, 194}));
      connect(constSpeed.flange, cooledTurbine.shaft) annotation(
        Line(points = {{12, -33}, {18, -33}, {18, -14}, {36, -14}}));
      connect(pressureSourceAir.fluidPort, cooledTurbine.Bl_port) annotation(
        Line(points = {{38, 46}, {48, 46}, {48, -2}}, color = {170, 223, 255}, thickness = 0.5));
      connect(pressureSourceExhaust.fluidPort, UncooledTurbine.inlet) annotation(
        Line(points = {{-98, 22}, {-82, 22}, {-82, -2}}, color = {129, 170, 194}));
      connect(pressureSourceExhaust1.fluidPort, UncooledTurbine.outlet) annotation(
        Line(points = {{-38, 20}, {-60, 20}, {-60, 4}}, color = {129, 170, 194}));
      connect(constantSpeed.flange, UncooledTurbine.shaft) annotation(
        Line(points = {{-106, -31}, {-100, -31}, {-100, -12}, {-82, -12}}));
    end TestCooledVSUncooledTurbine;

    model TestCooledVSUncooledTurbine_GauntnerModel
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CooledTurbine cooledTurbine(CoolingModel = true, CoolingTechRot = {3, 2, 2, 1}, CoolingTechStat = {3, 2, 2, 1}, Nstages = 4, Nstages_Bleeds = {2, 4}, Xi_cool_rot = {0.04, 0.01, 0, 0}, Xi_cool_stat = {0.06, 0.02, 0, 0}, data = turbineData) annotation(
        Placement(visible = true, transformation(origin = {47, -15}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust sources(referencePressure = turbineData.P_E_nom, referenceTemperature = turbineData.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust sinks(referencePressure = turbineData.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {90, 18}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed(w_fixed = turbineData.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {1, -33}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines turbineData annotation(
        Placement(visible = true, transformation(origin = {52, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir pressureSourceAir[2](each referencePressure = 10e5, each referenceTemperature = 287.15) annotation(
        Placement(visible = true, transformation(origin = {28, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed = turbineData.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-117, -31}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaust(referencePressure = turbineData.P_E_nom, referenceTemperature = turbineData.T_E_nom) annotation(
        Placement(visible = true, transformation(origin = {-108, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaust1(referencePressure = turbineData.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {-28, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.TurbineMapsBetaLines UncooledTurbine(data = turbineData) annotation(
        Placement(visible = true, transformation(origin = {-71, -13}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
    equation
      connect(sources.fluidPort, cooledTurbine.inlet) annotation(
        Line(points = {{20, 20}, {36, 20}, {36, -4}}, color = {129, 170, 194}));
      connect(sinks.fluidPort, cooledTurbine.outlet) annotation(
        Line(points = {{80, 18}, {58, 18}, {58, 2}}, color = {129, 170, 194}));
      connect(constSpeed.flange, cooledTurbine.shaft) annotation(
        Line(points = {{12, -33}, {18, -33}, {18, -14}, {36, -14}}));
      connect(pressureSourceAir.fluidPort, cooledTurbine.Bl_port) annotation(
        Line(points = {{38, 46}, {48, 46}, {48, -2}}, color = {170, 223, 255}, thickness = 0.5));
      connect(pressureSourceExhaust.fluidPort, UncooledTurbine.inlet) annotation(
        Line(points = {{-98, 22}, {-82, 22}, {-82, -2}}, color = {129, 170, 194}));
      connect(pressureSourceExhaust1.fluidPort, UncooledTurbine.outlet) annotation(
        Line(points = {{-38, 20}, {-60, 20}, {-60, 4}}, color = {129, 170, 194}));
      connect(constantSpeed.flange, UncooledTurbine.shaft) annotation(
        Line(points = {{-106, -31}, {-100, -31}, {-100, -12}, {-82, -12}}));
    end TestCooledVSUncooledTurbine_GauntnerModel;
  end TestComponents;

  package TestPartialAssemblies "Test of partial assemblies of systems"
    extends Modelica.Icons.ExamplesPackage;

    model TestFanNozzle "Test of fan-nozzle aggregate"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir sinks(each referencePressure = GSP_c.P_L_nom) annotation(
        Placement(visible = true, transformation(origin = {29, -9}, extent = {{7, -7}, {-7, 7}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed(w_fixed = 355) annotation(
        Placement(visible = true, transformation(origin = {52, -28}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
      parameter Data.Compressors.GSP_FanCore GSP_c annotation(
        Placement(visible = true, transformation(origin = {-40, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Data.Compressors.GSP_FanDuct GSP_d annotation(
        Placement(visible = true, transformation(origin = {36, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 350) annotation(
        Placement(transformation(extent = {{18, 12}, {38, 32}})));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-38, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.Fan fan(data_bypass = GSP_d, data_core = GSP_c) annotation(
        Placement(transformation(extent = {{-44, -38}, {34, 40}})));
    equation
      connect(airIntake.outlet, fan.inlet) annotation(
        Line(points = {{-32, 4}, {-26, 4}, {-26, 1}, {-19.625, 1}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{22.2, 27.6}, {15.1, 27.6}, {15.1, 20.5}, {4.75, 20.5}}, color = {170, 223, 255}));
      connect(fan.fan_mechanical_interface, constSpeed.flange) annotation(
        Line(points = {{14.5, -28.25}, {30, -28.25}, {30, -28}, {46, -28}}, color = {0, 0, 0}));
      connect(fan.outlet_core, sinks.fluidPort) annotation(
        Line(points = {{4.75, -8.75}, {14, -8.75}, {14, -9}, {22, -9}}, color = {170, 223, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Documentation(info = "<html>
<p>Test of the <a href=\"modelica://BasicAeroEngines.Components.CompressorMapsBetaLines\">CompressorMapsBetaLines</a> component, with data taken from <a href=\"modelica://BasicAeroEngines.Data.Compressors.TestCompressor\">TestCompressor</a>.</p>
<p>The test is automatically set up based on the data record, to simulate <code>N</code> parallel instances, each with a different value of <code>N_n</code>, spanning from the minimum to the maximum and including some intermediate values besides the ones included in the map data.</p>
<p>To check the model, you can use 2D plots with <code>Phi_n[j]</code> on the abscissa and <code>PR_n[j]</code>, one plot for each j corresponding to a specific value of the reduced speed. You can also plot the values of <code>eta</code> against, e.g., <code>beta</code> values.</p>
<p>In order to test other compressor data sets, you can extend this model and redeclare the data record:
<pre>
model TestMyCompressor
  extends BasicAeroEnginesTest.TestCompressor(
    redeclare MyCompressorData data);
end TestMyCompressor;
</pre></p>
</html>"));
    end TestFanNozzle;

    model TestFanLowSpeedShaft "Test of low-speed part of turbofan engine"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-150, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.NozzleExhaust nozzle(A = 0.6383, v_start = 459.7) annotation(
        Placement(visible = true, transformation(origin = {64, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftInertia1(J = 1000, phi(fixed = true, start = 0), w(fixed = true, start = 355)) annotation(
        Placement(visible = true, transformation(origin = {2, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.TurbineStodola turbine(data = GSP_LPT) annotation(
        Placement(visible = true, transformation(origin = {38, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = GSP_LPC) annotation(
        Placement(transformation(extent = {{-90, -44}, {-70, -24}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_Stodola GSP_LPT annotation(
        Placement(transformation(extent = {{34, -10}, {54, 10}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-126, 12}, {-106, 32}})));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 284.6) annotation(
        Placement(transformation(extent = {{-102, 14}, {-82, -6}})));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct annotation(
        Placement(visible = true, transformation(origin = {-146, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-78, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.PressureSourceExhaust pressureSourceExhaust(referencePressure(displayUnit = "bar") = 704959, referenceTemperature(displayUnit = "K") = 1081.53) annotation(
        Placement(transformation(extent = {{4, -4}, {24, 16}})));
      Components.FlowSourceAir flowSourceAir(referenceMassFlowRate = -123) annotation(
        Placement(transformation(extent = {{-44, -20}, {-62, -2}})));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {-20, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore) annotation(
        Placement(transformation(extent = {{-146, -40}, {-96, 10}})));
    equation
      connect(LPC.outlet, flowSourceAir.fluidPort) annotation(
        Line(points = {{-74, -28}, {-72, -28}, {-72, -11}, {-62, -11}}, color = {170, 223, 255}));
      connect(shaftInertia1.flange_b, turbine.shaft) annotation(
        Line(points = {{12, -34}, {32, -34}}, color = {0, 0, 0}));
      connect(pressureSourceExhaust.fluidPort, turbine.inlet) annotation(
        Line(points = {{24, 6}, {30, 6}, {30, -28}, {32, -28}}, color = {129, 170, 194}));
      connect(turbine.outlet, nozzle.inlet) annotation(
        Line(points = {{44, -24}, {58, -24}}, color = {129, 170, 194}));
      connect(airIntake.outlet, fan.inlet) annotation(
        Line(points = {{-144, -14}, {-136, -14}, {-136, -15}, {-130.375, -15}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-97.8, -1.6}, {-104.9, -1.6}, {-104.9, -2.5}, {-114.75, -2.5}}, color = {170, 223, 255}));
      connect(LPC.inlet, fan.outlet_core) annotation(
        Line(points = {{-86, -24}, {-100, -24}, {-100, -21.25}, {-114.75, -21.25}}, color = {170, 223, 255}));
      connect(LPC.shaft_b, shaftInertia1.flange_a) annotation(
        Line(points = {{-74, -34}, {-8, -34}}, color = {0, 0, 0}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-108.5, -33.75}, {-97.25, -33.75}, {-97.25, -34}, {-86, -34}}, color = {0, 0, 0}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html>
<p>Simple test model of a turbojet engine, using the TestTurbine and TestCompressor data sets. Please not that this model only has testing purposes for the library components, it does not represent a real-life, well-designed aeroengine.</p>
<p>The simulation starts near steady state on the ground at zero airspeed, with a fuel flow of 0.279 kg/s. At time 10, the fuel flow is increased with a two-second ramp to 0.35 kg/s.</p>
</html>"),
        Diagram(coordinateSystem(extent = {{-160, -100}, {100, 100}})));
    end TestFanLowSpeedShaft;

    model TestFanTwoShaftsTakeOff "Test of turbofan engine with some prescribed intermediate conditions, take-off"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-154, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines compressor(data = GSP_HPC) annotation(
        Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.TurbineStodola turbine(ConstantEta = true, data = GSP_HPT) annotation(
        Placement(visible = true, transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.NozzleExhaust nozzle(A = 0.6383, v_start = 459.7) annotation(
        Placement(visible = true, transformation(origin = {82, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftInertia(J = 1000, phi(fixed = true, start = 0), w(fixed = true, start = 1078.6)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {-20, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftInertia1(J = 1000, phi(fixed = true, start = 0), w(fixed = true, start = 355)) annotation(
        Placement(visible = true, transformation(origin = {2, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.TurbineStodola turbine1(ConstantEta = true, data = GSP_LPT) annotation(
        Placement(visible = true, transformation(origin = {58, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = GSP_LPC) annotation(
        Placement(transformation(extent = {{-106, -44}, {-86, -24}})));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 284.6) annotation(
        Placement(transformation(extent = {{-114, 2}, {-94, 22}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_Stodola GSP_LPT annotation(
        Placement(transformation(extent = {{48, -68}, {68, -48}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_Stodola GSP_HPT annotation(
        Placement(transformation(extent = {{26, 22}, {46, 42}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-154, 40}, {-134, 60}})));
      parameter Data.Compressors.GSP_HPC GSP_HPC annotation(
        Placement(transformation(extent = {{-58, -22}, {-38, -2}})));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct annotation(
        Placement(visible = true, transformation(origin = {-146, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-94, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.FlowSourceAir flowSourceAir(referenceMassFlowRate = -132.7) annotation(
        Placement(transformation(extent = {{-42, 20}, {-60, 38}})));
      Components.PressureSourceExhaust pressureSourceExhaust(referencePressure(displayUnit = "bar") = 2874500, referenceTemperature(displayUnit = "K") = 1459) annotation(
        Placement(transformation(extent = {{-16, 16}, {4, 36}})));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore) annotation(
        Placement(transformation(extent = {{-150, -40}, {-100, 10}})));
    equation
      connect(shaftInertia.flange_b, turbine.shaft) annotation(
        Line(points = {{10, 0}, {32, 0}}));
      connect(turbine1.inlet, turbine.outlet) annotation(
        Line(points = {{52, -28}, {52, 10}, {44, 10}}, color = {129, 170, 194}));
      connect(turbine1.outlet, nozzle.inlet) annotation(
        Line(points = {{64, -24}, {76, -24}}, color = {129, 170, 194}));
      connect(turbine1.shaft, shaftInertia1.flange_b) annotation(
        Line(points = {{52, -34}, {12, -34}}, color = {0, 0, 0}));
      connect(LPC.outlet, compressor.inlet) annotation(
        Line(points = {{-90, -28}, {-90, 10}, {-76, 10}}, color = {170, 223, 255}));
      connect(flowSourceAir.fluidPort, compressor.outlet) annotation(
        Line(points = {{-60, 29}, {-60, 26.5}, {-64, 26.5}, {-64, 6}}, color = {170, 223, 255}));
      connect(turbine.inlet, pressureSourceExhaust.fluidPort) annotation(
        Line(points = {{32, 6}, {18, 6}, {18, 26}, {4, 26}}, color = {129, 170, 194}));
      connect(airIntake.outlet, fan.inlet) annotation(
        Line(points = {{-148, -16}, {-144, -16}, {-144, -15}, {-134.375, -15}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-109.8, 17.6}, {-109.8, 16.8}, {-118.75, 16.8}, {-118.75, -2.5}}, color = {170, 223, 255}));
      connect(fan.outlet_core, LPC.inlet) annotation(
        Line(points = {{-118.75, -21.25}, {-111.375, -21.25}, {-111.375, -24}, {-102, -24}}, color = {170, 223, 255}));
      connect(LPC.shaft_b, shaftInertia1.flange_a) annotation(
        Line(points = {{-90, -34}, {-8, -34}}, color = {0, 0, 0}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-112.5, -33.75}, {-107.25, -33.75}, {-107.25, -34}, {-102, -34}}, color = {0, 0, 0}));
      connect(compressor.shaft_b, shaftInertia.flange_a) annotation(
        Line(points = {{-64, 0}, {-10, 0}}, color = {0, 0, 0}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html>
<p>Simple test model of a turbojet engine, using the TestTurbine and TestCompressor data sets. Please not that this model only has testing purposes for the library components, it does not represent a real-life, well-designed aeroengine.</p>
<p>The simulation starts near steady state on the ground at zero airspeed, with a fuel flow of 0.279 kg/s. At time 10, the fuel flow is increased with a two-second ramp to 0.35 kg/s.</p>
</html>"),
        Diagram(coordinateSystem(extent = {{-160, -100}, {100, 100}})));
    end TestFanTwoShaftsTakeOff;

    model TestFanTakeOffMaps "Test of turbofan engine with some prescribed intermediate conditions, take-off"
      extends Modelica.Icons.Example;
      inner Components.Environment environment(altitude = 0, airspeed = 0) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-148, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines compressor(data = GSP_HPC) annotation(
        Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftHP(J = 74, phi(fixed = true, start = 0), w(fixed = true, start = 1078.6)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftLP(J = 380, phi(fixed = true, start = 0), w(fixed = true, start = 355)) annotation(
        Placement(visible = true, transformation(origin = {2, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = GSP_LPC, f_E(fixed = false, start = 51), PR(fixed = false, start = 1.495)) annotation(
        Placement(transformation(extent = {{-90, -44}, {-70, -24}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-130, -62}, {-110, -42}})));
      parameter Data.Compressors.GSP_HPC GSP_HPC annotation(
        Placement(transformation(extent = {{-74, 24}, {-54, 44}})));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct annotation(
        Placement(visible = true, transformation(origin = {-126, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-80, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.TurbineMapsBetaLines turbineMapsBetaLines1(data = GSP_LPT) annotation(
        Placement(transformation(extent = {{50, -46}, {70, -26}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_BetaLines GSP_LPT annotation(
        Placement(transformation(extent = {{52, -70}, {72, -50}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines GSP_HPT annotation(
        Placement(transformation(extent = {{42, 22}, {62, 42}})));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 314.9, P_E(start = 74559, displayUnit = "Pa")) annotation(
        Placement(transformation(extent = {{-102, -18}, {-82, 2}})));
      Components.NozzleExhaust nozzle(A = 0.6383, v_start = 518.3) annotation(
        Placement(visible = true, transformation(origin = {86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore) annotation(
        Placement(transformation(extent = {{-148, -40}, {-98, 10}})));
      BasicAeroEngines.Components.FlowSourceAir flowSourceAir(referenceMassFlowRate = -132.7) annotation(
        Placement(transformation(extent = {{-18, 12}, {-36, 30}})));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaust(referencePressure(displayUnit = "bar") = 2874500, referenceTemperature(displayUnit = "K") = 1459) annotation(
        Placement(transformation(extent = {{2, 20}, {22, 40}})));
      BasicAeroEngines.Components.TurbineMapsBetaLines turbineMapsBetaLines(data = GSP_HPT) annotation(
        Placement(transformation(extent = {{26, -10}, {46, 10}})));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {-20, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(LPC.outlet, compressor.inlet) annotation(
        Line(points = {{-74, -28}, {-74, 10}, {-58, 10}}, color = {170, 223, 255}));
      connect(shaftLP.flange_b, turbineMapsBetaLines1.shaft) annotation(
        Line(points = {{12, -36}, {54, -36}}, color = {0, 0, 0}));
      connect(turbineMapsBetaLines1.outlet, nozzle.inlet) annotation(
        Line(points = {{66, -26}, {80, -26}}, color = {129, 170, 194}));
      connect(fan.inlet, airIntake.outlet) annotation(
        Line(points = {{-132.375, -15}, {-136.188, -15}, {-136.188, -14}, {-142, -14}}, color = {170, 223, 255}));
      connect(fan.outlet_core, LPC.inlet) annotation(
        Line(points = {{-116.75, -21.25}, {-101.375, -21.25}, {-101.375, -24}, {-86, -24}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-97.8, -2.4}, {-105.9, -2.4}, {-105.9, -2.5}, {-116.75, -2.5}}, color = {170, 223, 255}));
      connect(flowSourceAir.fluidPort, compressor.outlet) annotation(
        Line(points = {{-36, 21}, {-36, 20.5}, {-46, 20.5}, {-46, 6}}, color = {170, 223, 255}));
      connect(turbineMapsBetaLines.outlet, turbineMapsBetaLines1.inlet) annotation(
        Line(points = {{42, 10}, {48, 10}, {48, -30}, {54, -30}}, color = {129, 170, 194}));
      connect(pressureSourceExhaust.fluidPort, turbineMapsBetaLines.inlet) annotation(
        Line(points = {{22, 30}, {26, 30}, {26, 6}, {30, 6}}, color = {129, 170, 194}));
      connect(shaftHP.flange_b, turbineMapsBetaLines.shaft) annotation(
        Line(points = {{10, 0}, {30, 0}}, color = {0, 0, 0}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-110.5, -33.75}, {-98.25, -33.75}, {-98.25, -34}, {-86, -34}}, color = {0, 0, 0}));
      connect(LPC.shaft_b, shaftLP.flange_a) annotation(
        Line(points = {{-74, -34}, {-42, -34}, {-42, -36}, {-8, -36}}, color = {0, 0, 0}));
      connect(compressor.shaft_b, shaftHP.flange_a) annotation(
        Line(points = {{-46, 0}, {-10, 0}}, color = {0, 0, 0}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html>
<p>Simple test model of a turbojet engine, using the TestTurbine and TestCompressor data sets. Please not that this model only has testing purposes for the library components, it does not represent a real-life, well-designed aeroengine.</p>
<p>The simulation starts near steady state on the ground at zero airspeed, with a fuel flow of 0.279 kg/s. At time 10, the fuel flow is increased with a two-second ramp to 0.35 kg/s.</p>
</html>"),
        Diagram(coordinateSystem(extent = {{-160, -100}, {100, 100}})));
    end TestFanTakeOffMaps;

    model TestTurbinesMaps "Test of connected turbines"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed(w_fixed = GSP_HPT.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-36, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Components.TurbineMapsBetaLines turbineMapsBetaLines(data = GSP_HPT) annotation(
        Placement(transformation(extent = {{-10, -22}, {10, -2}})));
      Components.TurbineMapsBetaLines turbineMapsBetaLines1(data = GSP_LPT) annotation(
        Placement(transformation(extent = {{14, -58}, {34, -38}})));
      Components.NozzleExhaust nozzle(A = 0.6383, v_start = 518.3) annotation(
        Placement(visible = true, transformation(origin = {50, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.PressureSourceExhaust pressureSourceExhaust(referencePressure(displayUnit = "bar") = 2874500, referenceTemperature(displayUnit = "K") = 1459) annotation(
        Placement(transformation(extent = {{-34, 8}, {-14, 28}})));
      parameter Data.Turbines.GSP_HPT_BetaLines GSP_HPT annotation(
        Placement(transformation(extent = {{6, 10}, {26, 30}})));
      parameter Data.Turbines.GSP_LPT_BetaLines GSP_LPT annotation(
        Placement(transformation(extent = {{16, -82}, {36, -62}})));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constSpeed1(w_fixed = GSP_LPT.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-16, -46}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(constSpeed1.flange, turbineMapsBetaLines1.shaft) annotation(
        Line(points = {{-10, -46}, {4, -46}, {4, -48}, {18, -48}}, color = {0, 0, 0}));
      connect(turbineMapsBetaLines1.outlet, nozzle.inlet) annotation(
        Line(points = {{30, -38}, {44, -38}}, color = {129, 170, 194}));
      connect(constSpeed.flange, turbineMapsBetaLines.shaft) annotation(
        Line(points = {{-30, -14}, {-18, -14}, {-18, -12}, {-6, -12}}, color = {0, 0, 0}));
      connect(turbineMapsBetaLines.outlet, turbineMapsBetaLines1.inlet) annotation(
        Line(points = {{6, -2}, {12, -2}, {12, -42}, {18, -42}}, color = {129, 170, 194}));
      connect(pressureSourceExhaust.fluidPort, turbineMapsBetaLines.inlet) annotation(
        Line(points = {{-14, 18}, {-10, 18}, {-10, -6}, {-6, -6}}, color = {129, 170, 194}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Documentation(info = "<html>
    <p>Test of the <a href=\"modelica://BasicAeroEngines.Components.CompressorMapsBetaLines\">CompressorMapsBetaLines</a> component, with data taken from <a href=\"modelica://BasicAeroEngines.Data.Compressors.TestCompressor\">TestCompressor</a>.</p>
    <p>The test is automatically set up based on the data record, to simulate <code>N</code> parallel instances, each with a different value of <code>N_n</code>, spanning from the minimum to the maximum and including some intermediate values besides the ones included in the map data.</p>
    <p>To check the model, you can use 2D plots with <code>Phi_n[j]</code> on the abscissa and <code>PR_n[j]</code>, one plot for each j corresponding to a specific value of the reduced speed. You can also plot the values of <code>eta</code> against, e.g., <code>beta</code> values.</p>
    <p>In order to test other compressor data sets, you can extend this model and redeclare the data record:
    <pre>
    model TestMyCompressor
    extends BasicAeroEnginesTest.TestCompressor(
    redeclare MyCompressorData data);
    end TestMyCompressor;
    </pre></p>
    </html>"));
    end TestTurbinesMaps;

    model CooledTurbineTest
      parameter Modelica.SIunits.MassFlowRate fnom = 63.477;
      parameter Modelica.SIunits.MassFlowRate flpt = 70.43;
      parameter Modelica.SIunits.MassFlowRate w25 = 70.955;
      parameter Modelica.SIunits.Efficiency EisHT = 0.865;
      parameter Modelica.SIunits.Efficiency EisLT = 0.882;
      parameter Modelica.SIunits.Pressure Phpt_E = 1058899;
      parameter Modelica.SIunits.Temperature Thpt_E = 1199;
      parameter Modelica.SIunits.Pressure Phpt_L = 462389;
      parameter Modelica.SIunits.Temperature Tlpt_E = 967.53;
      //Phpt_E/2.29;
      parameter Modelica.SIunits.Temperature Thpt_L = 967.53;
      parameter Modelica.SIunits.Pressure Plpt_E = Phpt_L;
      parameter Modelica.SIunits.Pressure Plpt_L = 237732;
      //Plpt_E/1.945;
      parameter Modelica.SIunits.Temperature Tlpt_L = 831.49;
      //  parameter Modelica.SIunits.Temperature Thpc_E = Tlpc_E * (1 + 1 / 0.823 * ((Plpc_L / Plpc_E) ^ (0.4 / 1.4) - 1));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeedLPT(w_fixed = LPTdata.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-40, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeedHPT(w_fixed = HPTdata.omega_nom) annotation(
        Placement(visible = true, transformation(origin = {-60, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Components.Environment environment(Mach = 0.147, P(displayUnit = "Pa"), Pb(displayUnit = "Pa") = 103900, Tb(displayUnit = "K") = 263.93, altitude = 1844, useMach = true) annotation(
        Placement(visible = true, transformation(origin = {60, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines HPTdata(P_E_nom = Phpt_E, P_L_nom = Phpt_L, T_E_nom = Thpt_E, eta_nom = 0.865, f_nom = fnom, omega_nom = 9235) annotation(
        Placement(visible = true, transformation(origin = {-24, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CooledTurbine HPT(CoolingModel = false, CoolingTechRot = {2}, CoolingTechStat = {2}, Deta_rot = {0, 0.2, 0.36}, Deta_stat = {0, 0.1, 0.18}, Nbleed = 1, Nstages = 1, Nstages_Bleeds = {1}, Tblade(displayUnit = "K"), Xi_cool_rot = {0.048 * w25 / fnom}, Xi_cool_stat = {0.05 * w25 / fnom}, data = HPTdata, PRnom = 2, P_L_iso_start = Phpt_L .* ones(HPT.Nstages), f_E(start = 63.48)) annotation(
        Placement(visible = true, transformation(origin = {-18, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir pressureSourceAirHPT[1](each referencePressure(displayUnit = "Pa") = 1120528, each referenceTemperature(displayUnit = "K") = 598.4) annotation(
        Placement(visible = true, transformation(origin = {-30, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaustCC(referencePressure(displayUnit = "Pa") = Phpt_E, referenceTemperature(displayUnit = "K") = Thpt_E) annotation(
        Placement(visible = true, transformation(origin = {-64, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceAir pressureSourceAirLPT[1](each referencePressure(displayUnit = "Pa") = 697346, each referenceTemperature(displayUnit = "K") = 513.0999999999999) annotation(
        Placement(visible = true, transformation(origin = {34, 12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CooledTurbine LPT(CoolingModel = false, CoolingTechRot = {1, 1}, CoolingTechStat = {2, 1}, Deta_rot = {0, 0.2, 0.36}, Deta_stat = {0, 0.1, 0.18}, Nbleed = 1, Nstages = 2, Nstages_Bleeds = {2}, PRnom = 1.39, Tblade(displayUnit = "K"), Xi_cool_rot = {0.0, 0.0}, Xi_cool_stat = {0.017 * w25 / fnom, 0.0}, data = LPTdata) annotation(
        Placement(visible = true, transformation(origin = {0, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Data.Turbines.GSP_LPT_BetaLines LPTdata(P_E_nom = Plpt_E, P_L_nom = Plpt_L, T_E_nom(displayUnit = "K") = Tlpt_E, eta_nom = EisLT, f_nom = flpt, omega_nom = 5955) annotation(
        Placement(visible = true, transformation(origin = {10, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.PressureSourceExhaust pressureSourceExhaust(referencePressure(displayUnit = "Pa") = Plpt_L, referenceTemperature(displayUnit = "K") = Tlpt_L) annotation(
        Placement(visible = true, transformation(origin = {40, -26}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    equation
      connect(constantSpeedHPT.flange, HPT.shaft) annotation(
        Line(points = {{-50, 12}, {-37, 12}, {-37, 14}, {-24, 14}}));
      connect(pressureSourceExhaustCC.fluidPort, HPT.inlet) annotation(
        Line(points = {{-54, 44}, {-44, 44}, {-44, 20}, {-24, 20}}, color = {129, 170, 194}));
      connect(constantSpeedLPT.flange, LPT.shaft) annotation(
        Line(points = {{-30, -26}, {-20, -26}, {-20, -28}, {-6, -28}}));
      connect(pressureSourceAirHPT[1].fluidPort, HPT.Bl_port[1]) annotation(
        Line(points = {{-20, 44}, {-17.8, 44}, {-17.8, 22}}, color = {170, 223, 255}, thickness = 0.5));
      connect(pressureSourceAirLPT[1].fluidPort, LPT.Bl_port[1]) annotation(
        Line(points = {{24, 12}, {0.2, 12}, {0.2, -20}}, color = {170, 223, 255}, thickness = 0.5));
      connect(HPT.outlet, LPT.inlet) annotation(
        Line(points = {{-12, 24}, {-6, 24}, {-6, -22}}, color = {129, 170, 194}));
      connect(LPT.outlet, pressureSourceExhaust.fluidPort) annotation(
        Line(points = {{6, -18}, {22, -18}, {22, -26}, {30, -26}}, color = {129, 170, 194}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {75, 138, 73}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Polygon(lineColor = {0, 0, 255}, fillColor = {75, 138, 73}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-36, 60}, {64, 0}, {-36, -60}, {-36, 60}})}));
    end CooledTurbineTest;
  end TestPartialAssemblies;

  package TestSystems "Test cases for partially or fully assembled systems"
    extends Modelica.Icons.ExamplesPackage;

    model TestTurboJet "Simple turbojet model based on the sample compressor and turbine, takeoff conditions"
      extends Modelica.Icons.Example;
      inner Components.Environment environment annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-62, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines compressor(data = compressorData) annotation(
        Placement(visible = true, transformation(origin = {-34, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CombustionChamberLHVSimple combustor(P_start = 957000, T_start = 1098.5, V = 0.1) annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Components.TurbineStodola turbine(data = turbineData) annotation(
        Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.NozzleExhaust nozzle(A = 0.1, v_start = 350) annotation(
        Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftInertia(J = 1000, phi(fixed = true, start = 0), w(fixed = false, start = 100)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.SampleCompressor compressorData annotation(
        Placement(visible = true, transformation(origin = {-43, 23}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Turbines.SampleTurbine turbineData annotation(
        Placement(visible = true, transformation(origin = {41, 23}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression thermalEfficiency(y = (nozzle.W - airIntake.W) / combustor.Q) annotation(
        Placement(visible = true, transformation(origin = {48, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression overallEfficiency(y = netThrust.y * environment.v / combustor.Q) annotation(
        Placement(visible = true, transformation(origin = {48, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable fuelFlow(table = [0, 0.279; 10, 0.279; 12, 0.35; 1000, 0.35]) annotation(
        Placement(visible = true, transformation(origin = {-32, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag) annotation(
        Placement(visible = true, transformation(origin = {48, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(fuelFlow.y, combustor.fuelFlow) annotation(
        Line(points = {{-21, 58}, {0, 58}, {0, 32}}, color = {0, 0, 127}));
      connect(combustor.exhaust, turbine.inlet) annotation(
        Line(points = {{8, 24}, {30, 24}, {30, 6}}, color = {129, 170, 194}));
      connect(compressor.outlet, combustor.airInlet) annotation(
        Line(points = {{-28, 6}, {-28, 24}, {-8, 24}}, color = {170, 223, 255}));
      connect(compressor.shaft_a, shaftInertia.flange_a) annotation(
        Line(points = {{-40, 0}, {-10, 0}}));
      connect(airIntake.outlet, compressor.inlet) annotation(
        Line(points = {{-56, 6}, {-56, 10}, {-40, 10}}, color = {170, 223, 255}));
      connect(shaftInertia.flange_b, turbine.shaft) annotation(
        Line(points = {{10, 0}, {30, 0}, {30, 0}, {30, 0}}));
      connect(turbine.outlet, nozzle.inlet) annotation(
        Line(points = {{42, 10}, {54, 10}, {54, 6}, {54, 6}}, color = {129, 170, 194}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html>
<p>Simple test model of a turbojet engine, using the TestTurbine and TestCompressor data sets. Please not that this model only has testing purposes for the library components, it does not represent a real-life, well-designed aeroengine.</p>
<p>The simulation starts near steady state on the ground at zero airspeed, with a fuel flow of 0.279 kg/s. At time 10, the fuel flow is increased with a two-second ramp to 0.35 kg/s.</p>
</html>"));
    end TestTurboJet;

    model TestTurboJetCruising "Same as TestTurboJet but at cruising conditions"
      extends TestSystems.TestTurboJet(environment(airspeed = 230, altitude = 10000, P(start = 40000)), combustor(P_start = 412000, T_start = 923.15), fuelFlow(table = [0, 0.10; 10, 0.10; 11, 0.12; 1000, 0.12]), nozzle(P_E(start = 50000)), shaftInertia(w(start = 100)));
    equation

      annotation(
        experiment(StartTime = 0, StopTime = 30, Tolerance = 1e-6, Interval = 0.06),
        Documentation(info = "<html>
<p>Extends the <a href=\"modelica://BasicAeroEngines.Test.TestTurboJet\">TestTurboJet</a>, adapting the operating conditions to high-altitude cruise operation at 10000 m and -40 degC.</p>
<p>The simulation starts close to steady state. At time = 10, a ramp increase of fuel flow is applied.</p>
</html>"));
    end TestTurboJetCruising;

    model TestTurboJetPrescribedSpeed "Same as TestTurboJet, but with fixed shaft speed"
      extends TestSystems.TestTurboJet(shaftInertia(phi(fixed = false), w(fixed = false)), fuelFlow(table = [0, 0.279; 1000, 0.32; 10000, 0.32]));
      Modelica.Mechanics.Rotational.Sources.Speed prescribedAngularVelocity(exact = true) annotation(
        Placement(visible = true, transformation(origin = {-36, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable shaftAngularVelocity(table = [0, 100; 2000, 100; 3000, 110; 10000, 110]) annotation(
        Placement(visible = true, transformation(origin = {-74, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(prescribedAngularVelocity.flange, shaftInertia.flange_b) annotation(
        Line(points = {{-26, -46}, {12, -46}, {12, 0}, {10, 0}, {10, 0}}));
      connect(shaftAngularVelocity.y, prescribedAngularVelocity.w_ref) annotation(
        Line(points = {{-63, -46}, {-48, -46}}, color = {0, 0, 127}));
      annotation(
        Documentation(info = "<html>
<p>This model extends <a href=\"modelica://BasicAeroEngines.Tests.TestTurboJet\">TestTurboJet</a> by adding a prescribed velocity source. This allows to test and tune the various components of the original system and to study how they react to changes of shaft speed and fuel flow, without worrying about the power balance.</p>
</html>"),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
    end TestTurboJetPrescribedSpeed;

    model TestTurboFanTakeOff "Test of full turbofan engine, take-off conditions"
      extends Modelica.Icons.Example;
      inner Components.Environment environment(altitude = 0, airspeed = 0) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-148, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines HPC(data = GSP_HPC) annotation(
        Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CombustionChamberLHV combustor(P_start(displayUnit = "Pa") = 2.9943e+6, LHV = 43.031e6, V = 0.1, T_start = 1373.15, steadyStateInit = false, ZC = 1, ZH = 1.9167) annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftHP(J = 74, phi(fixed = true, start = 0), w(fixed = false, start = 1078.6)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable fuelFlow(table = [0, 2.204; 20, 2.204; 20, 2.0; 100, 2.0]) annotation(
        Placement(visible = true, transformation(origin = {-32, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftLP(J = 380, phi(fixed = true, start = 0), w(fixed = true, start = 355)) annotation(
        Placement(visible = true, transformation(origin = {2, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = GSP_LPC, f_E(fixed = true, start = 132), PR(fixed = false, start = 1.495)) annotation(
        Placement(transformation(extent = {{-90, -44}, {-70, -24}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-130, -62}, {-110, -42}})));
      parameter Data.Compressors.GSP_HPC GSP_HPC annotation(
        Placement(transformation(extent = {{-74, 24}, {-54, 44}})));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct annotation(
        Placement(visible = true, transformation(origin = {-126, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-80, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.TurbineMapsBetaLines HPT(data = GSP_HPT) annotation(
        Placement(transformation(extent = {{26, -10}, {46, 10}})));
      Components.TurbineMapsBetaLines LPT(data = GSP_LPT) annotation(
        Placement(transformation(extent = {{50, -46}, {70, -26}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_BetaLines GSP_LPT annotation(
        Placement(transformation(extent = {{52, -70}, {72, -50}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines GSP_HPT annotation(
        Placement(transformation(extent = {{42, 22}, {62, 42}})));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 314.9, P_E(start = 74559, displayUnit = "Pa")) annotation(
        Placement(transformation(extent = {{-102, -18}, {-82, 2}})));
      Components.NozzleExhaust nozzle(A = 0.6389, v_start = 518.3) annotation(
        Placement(visible = true, transformation(origin = {86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore) annotation(
        Placement(transformation(extent = {{-148, -40}, {-98, 10}})));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {-20, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(LPC.outlet, HPC.inlet) annotation(
        Line(points = {{-74, -28}, {-74, 10}, {-58, 10}}, color = {170, 223, 255}));
      connect(shaftLP.flange_b, LPT.shaft) annotation(
        Line(points = {{12, -36}, {54, -36}}, color = {0, 0, 0}));
      connect(HPT.shaft, shaftHP.flange_b) annotation(
        Line(points = {{30, 0}, {10, 0}}, color = {0, 0, 0}));
      connect(HPT.outlet, LPT.inlet) annotation(
        Line(points = {{42, 10}, {48, 10}, {48, -30}, {54, -30}}, color = {129, 170, 194}));
      connect(LPT.outlet, nozzle.inlet) annotation(
        Line(points = {{66, -26}, {80, -26}}, color = {129, 170, 194}));
      connect(fan.inlet, airIntake.outlet) annotation(
        Line(points = {{-132.375, -15}, {-136.188, -15}, {-136.188, -14}, {-142, -14}}, color = {170, 223, 255}));
      connect(fan.outlet_core, LPC.inlet) annotation(
        Line(points = {{-116.75, -21.25}, {-85.375, -21.25}, {-85.375, -24}, {-86, -24}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-97.8, -2.4}, {-105.9, -2.4}, {-105.9, -2.5}, {-116.75, -2.5}}, color = {170, 223, 255}));
      connect(HPC.outlet, combustor.airInlet) annotation(
        Line(points = {{-46, 6}, {-47, 6}, {-47, 24}, {-8, 24}}, color = {170, 223, 255}));
      connect(combustor.exhaust, HPT.inlet) annotation(
        Line(points = {{8, 24}, {30, 24}, {30, 6}}, color = {129, 170, 194}));
      connect(fuelFlow.y, combustor.fuelFlow) annotation(
        Line(points = {{-21, 54}, {0, 54}, {0, 32}}, color = {0, 0, 127}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-110.5, -33.75}, {-98.25, -33.75}, {-98.25, -34}, {-86, -34}}, color = {0, 0, 0}));
      connect(LPC.shaft_b, shaftLP.flange_a) annotation(
        Line(points = {{-74, -34}, {-42, -34}, {-42, -36}, {-8, -36}}, color = {0, 0, 0}));
      connect(HPC.shaft_b, shaftHP.flange_a) annotation(
        Line(points = {{-46, 0}, {-10, 0}}, color = {0, 0, 0}));
    end TestTurboFanTakeOff;

    model TestTurboFanTakeOffSteadyState
      extends Modelica.Icons.Example;
      inner Components.Environment environment(altitude = 0, airspeed = 0) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-148, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines HPC(data = GSP_HPC, useHomotopy = true) annotation(
        Placement(visible = true, transformation(origin = {-52, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CombustionChamberLHV combustor(P_start(displayUnit = "Pa") = 2.9943e+6, LHV = 43.031e6, V = 0.1, T_start = 1373.15, steadyStateInit = true, ZC = 1, ZH = 1.9167) annotation(
        Placement(visible = true, transformation(origin = {0, 34}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftHP(J = 74, phi(fixed = true, start = 0), w(start = 1078.6), a(start = 0, fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable fuelFlow(table = [0, 2.204; 20, 2.204; 20, 2.0; 100, 2.0]) annotation(
        Placement(visible = true, transformation(origin = {-32, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftLP(J = 380, phi(fixed = true, start = 0), w(start = 355), a(start = 0, fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {2, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = GSP_LPC, f_E(start = 120), useHomotopy = true) annotation(
        Placement(transformation(extent = {{-90, -44}, {-70, -24}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-130, -62}, {-110, -42}})));
      parameter BasicAeroEngines.Data.Compressors.GSP_HPC GSP_HPC annotation(
        Placement(visible = true, transformation(extent = {{-74, 34}, {-54, 54}}, rotation = 0)));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct(P_E_nom(displayUnit = "Pa"), T_E_nom(displayUnit = "K")) annotation(
        Placement(visible = true, transformation(origin = {-126, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-80, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.TurbineMapsBetaLines HPT(data = GSP_HPT) annotation(
        Placement(visible = true, transformation(extent = {{26, 0}, {46, 20}}, rotation = 0)));
      Components.TurbineMapsBetaLines LPT(data = GSP_LPT) annotation(
        Placement(transformation(extent = {{50, -46}, {70, -26}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_BetaLines GSP_LPT annotation(
        Placement(transformation(extent = {{52, -70}, {72, -50}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines GSP_HPT annotation(
        Placement(visible = true, transformation(extent = {{42, 32}, {62, 52}}, rotation = 0)));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 314.9, P_E(start = 74559, displayUnit = "Pa")) annotation(
        Placement(transformation(extent = {{-102, -18}, {-82, 2}})));
      Components.NozzleExhaust nozzle(A = 0.6389, v_start = 518.3) annotation(
        Placement(visible = true, transformation(origin = {86, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore, useHomotopy = true) annotation(
        Placement(transformation(extent = {{-148, -40}, {-98, 10}})));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {66, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.ShaftInitializer initShaftLP(W_nom = 10e6, useHomotopy = true, w_start = 355) annotation(
        Placement(visible = true, transformation(origin = {2, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.ShaftInitializer initShaftHP(W_nom = 10e6, useHomotopy = true, w_start = 1078) annotation(
        Placement(visible = true, transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(LPC.outlet, HPC.inlet) annotation(
        Line(points = {{-74, -28}, {-74, 20}, {-58, 20}}, color = {170, 223, 255}));
      connect(shaftLP.flange_b, LPT.shaft) annotation(
        Line(points = {{12, -36}, {54, -36}}, color = {0, 0, 0}));
      connect(HPT.shaft, shaftHP.flange_b) annotation(
        Line(points = {{30, 10}, {10, 10}}));
      connect(HPT.outlet, LPT.inlet) annotation(
        Line(points = {{42, 20}, {48, 20}, {48, -30}, {54, -30}}, color = {129, 170, 194}));
      connect(LPT.outlet, nozzle.inlet) annotation(
        Line(points = {{66, -26}, {80, -26}}, color = {129, 170, 194}));
      connect(fan.inlet, airIntake.outlet) annotation(
        Line(points = {{-132.375, -15}, {-136.188, -15}, {-136.188, -14}, {-142, -14}}, color = {170, 223, 255}));
      connect(fan.outlet_core, LPC.inlet) annotation(
        Line(points = {{-116.75, -21.25}, {-85.375, -21.25}, {-85.375, -24}, {-86, -24}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-97.8, -2.4}, {-105.9, -2.4}, {-105.9, -2.5}, {-116.75, -2.5}}, color = {170, 223, 255}));
      connect(HPC.outlet, combustor.airInlet) annotation(
        Line(points = {{-46, 16}, {-47, 16}, {-47, 34}, {-8, 34}}, color = {170, 223, 255}));
      connect(combustor.exhaust, HPT.inlet) annotation(
        Line(points = {{8, 34}, {30, 34}, {30, 16}}, color = {129, 170, 194}));
      connect(fuelFlow.y, combustor.fuelFlow) annotation(
        Line(points = {{-21, 64}, {0, 64}, {0, 42}}, color = {0, 0, 127}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-110.5, -33.75}, {-98.25, -33.75}, {-98.25, -34}, {-86, -34}}, color = {0, 0, 0}));
      connect(LPC.shaft_b, shaftLP.flange_a) annotation(
        Line(points = {{-74, -34}, {-42, -34}, {-42, -36}, {-8, -36}}, color = {0, 0, 0}));
      connect(HPC.shaft_b, shaftHP.flange_a) annotation(
        Line(points = {{-46, 10}, {-10, 10}}));
      connect(initShaftLP.flange, shaftLP.flange_b) annotation(
        Line(points = {{12, -56}, {24, -56}, {24, -36}, {12, -36}}));
      connect(initShaftHP.flange, shaftHP.flange_b) annotation(
        Line(points = {{10, -10}, {18, -10}, {18, 10}, {10, 10}}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html><head></head><body><p>Simple test model of a turbofan engine, using the GSP turbine and compressor data sets.&nbsp;</p><p>The simulation starts at steady state at take-off conditions, with a fuel flow of 2.204 kg/s. At time = 20 it is reduced to 2 kg/s.</p>
    </body></html>"),
        Diagram(coordinateSystem(extent = {{-160, -100}, {100, 100}})));
    end TestTurboFanTakeOffSteadyState;

    model TestTurboFanCruise "Test of full turbofan engine, cruise conditions"
      extends Modelica.Icons.Example;
      inner Components.Environment environment(altitude = 10000, Mach = 0.8, useMach = true, P(start = 30000)) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-148, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines HPC(data = GSP_HPC, Phi_n(start = 1.1, fixed = false), f_E(fixed = false, start = 65)) annotation(
        Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CombustionChamberLHV combustor(P_start(displayUnit = "Pa") = 1.5e+6, LHV = 43.031e6, V = 0.1, T_start = 1373.15, steadyStateInit = false, ZC = 1, ZH = 1.9167) annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftHP(J = 74, phi(fixed = true, start = 0), w(fixed = false, start = 1078.6)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable fuelFlow(table = [0, 0.77]) annotation(
        Placement(visible = true, transformation(origin = {-32, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftLP(J = 380, phi(fixed = true, start = 0), w(fixed = true, start = 355)) annotation(
        Placement(visible = true, transformation(origin = {2, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(data = GSP_LPC, f_E(fixed = true, start = 65), PR(fixed = false, start = 1.495)) annotation(
        Placement(transformation(extent = {{-92, -44}, {-72, -24}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-130, -62}, {-110, -42}})));
      parameter Data.Compressors.GSP_HPC GSP_HPC annotation(
        Placement(transformation(extent = {{-74, 24}, {-54, 44}})));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct annotation(
        Placement(visible = true, transformation(origin = {-126, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-80, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.TurbineMapsBetaLines HPT(data = GSP_HPT) annotation(
        Placement(transformation(extent = {{26, -10}, {46, 10}})));
      Components.TurbineMapsBetaLines LPT(data = GSP_LPT) annotation(
        Placement(transformation(extent = {{50, -44}, {70, -24}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_BetaLines GSP_LPT annotation(
        Placement(transformation(extent = {{52, -70}, {72, -50}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines GSP_HPT annotation(
        Placement(transformation(extent = {{42, 22}, {62, 42}})));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 314.9, P_E(start = 40000, displayUnit = "bar")) annotation(
        Placement(transformation(extent = {{-102, -18}, {-82, 2}})));
      Components.NozzleExhaust nozzle(A = 0.6389, v_start = 518.3) annotation(
        Placement(visible = true, transformation(origin = {86, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore) annotation(
        Placement(transformation(extent = {{-148, -40}, {-98, 10}})));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {-20, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression thermalEfficiency(y = (nozzle.W - airIntake.W) / combustor.Q) annotation(
        Placement(visible = true, transformation(origin = {-58, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression overallEfficiency(y = netThrust.y * environment.v / combustor.Q) annotation(
        Placement(visible = true, transformation(origin = {16, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(LPC.outlet, HPC.inlet) annotation(
        Line(points = {{-76, -28}, {-76, 10}, {-58, 10}}, color = {170, 223, 255}));
      connect(shaftLP.flange_b, LPT.shaft) annotation(
        Line(points = {{12, -34}, {54, -34}}, color = {0, 0, 0}));
      connect(HPT.shaft, shaftHP.flange_b) annotation(
        Line(points = {{30, 0}, {10, 0}}, color = {0, 0, 0}));
      connect(HPT.outlet, LPT.inlet) annotation(
        Line(points = {{42, 10}, {48, 10}, {48, -28}, {54, -28}}, color = {129, 170, 194}));
      connect(LPT.outlet, nozzle.inlet) annotation(
        Line(points = {{66, -24}, {80, -24}}, color = {129, 170, 194}));
      connect(fan.inlet, airIntake.outlet) annotation(
        Line(points = {{-132.375, -15}, {-136.188, -15}, {-136.188, -14}, {-142, -14}}, color = {170, 223, 255}));
      connect(fan.outlet_core, LPC.inlet) annotation(
        Line(points = {{-116.75, -21.25}, {-85.375, -21.25}, {-85.375, -24}, {-88, -24}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-97.8, -2.4}, {-105.9, -2.4}, {-105.9, -2.5}, {-116.75, -2.5}}, color = {170, 223, 255}));
      connect(HPC.outlet, combustor.airInlet) annotation(
        Line(points = {{-46, 6}, {-45, 6}, {-45, 24}, {-8, 24}}, color = {170, 223, 255}));
      connect(combustor.exhaust, HPT.inlet) annotation(
        Line(points = {{8, 24}, {30, 24}, {30, 6}}, color = {129, 170, 194}));
      connect(fuelFlow.y, combustor.fuelFlow) annotation(
        Line(points = {{-21, 54}, {0, 54}, {0, 32}}, color = {0, 0, 127}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-110.5, -33.75}, {-99.25, -33.75}, {-99.25, -34}, {-88, -34}}, color = {0, 0, 0}));
      connect(LPC.shaft_b, shaftLP.flange_a) annotation(
        Line(points = {{-76, -34}, {-8, -34}}, color = {0, 0, 0}));
      connect(HPC.shaft_b, shaftHP.flange_a) annotation(
        Line(points = {{-46, 0}, {-10, 0}}, color = {0, 0, 0}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html><head></head><body><p>Simple test model of a turbofan engine, using the GSP turbine and compressor data sets.&nbsp;</p>
<p>The simulation starts near steady state at cruise conditions, with a fuel flow of 0.77 kg/s.</p>
</body></html>"),
        Diagram(coordinateSystem(extent = {{-160, -100}, {100, 100}})),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end TestTurboFanCruise;

    model TestTurboFanCruiseSteadyState
      extends Modelica.Icons.Example;
      inner Components.Environment environment(altitude = 10000, Mach = 0.8, useMach = true, P(start = 30000)) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.AirIntake airIntake annotation(
        Placement(visible = true, transformation(origin = {-148, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines HPC(Phi_n(fixed = false, start = 1.1), data = GSP_HPC, f_E(fixed = false, start = 65), useHomotopy = true) annotation(
        Placement(visible = true, transformation(origin = {-52, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CombustionChamberLHV combustor(P_start(displayUnit = "Pa") = 1.5e+6, LHV = 43.031e6, V = 0.1, T_start = 1373.15, steadyStateInit = true, ZC = 1, ZH = 1.9167) annotation(
        Placement(visible = true, transformation(origin = {0, 32}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftHP(J = 74, phi(fixed = true, start = 0), w(start = 1078.6), a(start = 0, fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {0, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.TimeTable fuelFlow(table = [0, 0.77]) annotation(
        Placement(visible = true, transformation(origin = {-32, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia shaftLP(J = 380, phi(fixed = true, start = 0), w(start = 355), a(start = 0, fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {2, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.CompressorMapsBetaLines LPC(PR(fixed = false, start = 1.495), data = GSP_LPC, f_E(start = 65), useHomotopy = true) annotation(
        Placement(transformation(extent = {{-92, -44}, {-72, -24}})));
      parameter Data.Compressors.GSP_FanCore GSP_FanCore annotation(
        Placement(transformation(extent = {{-130, -62}, {-110, -42}})));
      parameter BasicAeroEngines.Data.Compressors.GSP_HPC GSP_HPC annotation(
        Placement(visible = true, transformation(extent = {{-74, 32}, {-54, 52}}, rotation = 0)));
      parameter Data.Compressors.GSP_FanDuct GSP_FanDuct annotation(
        Placement(visible = true, transformation(origin = {-126, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter BasicAeroEngines.Data.Compressors.GSP_LPC GSP_LPC annotation(
        Placement(visible = true, transformation(origin = {-80, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.TurbineMapsBetaLines HPT(data = GSP_HPT) annotation(
        Placement(visible = true, transformation(extent = {{26, -2}, {46, 18}}, rotation = 0)));
      Components.TurbineMapsBetaLines LPT(data = GSP_LPT) annotation(
        Placement(transformation(extent = {{50, -44}, {70, -24}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_LPT_BetaLines GSP_LPT annotation(
        Placement(transformation(extent = {{52, -70}, {72, -50}})));
      parameter BasicAeroEngines.Data.Turbines.GSP_HPT_BetaLines GSP_HPT annotation(
        Placement(visible = true, transformation(extent = {{42, 30}, {62, 50}}, rotation = 0)));
      Components.NozzleAir nozzleAir(A = 1.9092, v_start = 314.9, P_E(start = 40000, displayUnit = "bar")) annotation(
        Placement(transformation(extent = {{-102, -18}, {-82, 2}})));
      Components.NozzleExhaust nozzle(A = 0.6389, v_start = 518.3) annotation(
        Placement(visible = true, transformation(origin = {86, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.Fan fan(data_bypass = GSP_FanDuct, data_core = GSP_FanCore, useHomotopy = true) annotation(
        Placement(transformation(extent = {{-148, -40}, {-98, 10}})));
      Modelica.Blocks.Sources.RealExpression netThrust(y = nozzle.thrust - airIntake.drag + nozzleAir.thrustBypass) annotation(
        Placement(visible = true, transformation(origin = {-20, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression thermalEfficiency(y = (nozzle.W - airIntake.W) / combustor.Q) annotation(
        Placement(visible = true, transformation(origin = {-58, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression overallEfficiency(y = netThrust.y * environment.v / combustor.Q) annotation(
        Placement(visible = true, transformation(origin = {16, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      BasicAeroEngines.Components.ShaftInitializer initShaftLP(W_nom = 10e6, useHomotopy = true, w_start = 355) annotation(
        Placement(visible = true, transformation(origin = {0, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.ShaftInitializer initShaftHP(W_nom = 10e6, useHomotopy = true, w_start = 1078) annotation(
        Placement(visible = true, transformation(origin = {0, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(LPC.outlet, HPC.inlet) annotation(
        Line(points = {{-76, -28}, {-76, 18}, {-58, 18}}, color = {170, 223, 255}));
      connect(shaftLP.flange_b, LPT.shaft) annotation(
        Line(points = {{12, -34}, {54, -34}}, color = {0, 0, 0}));
      connect(HPT.shaft, shaftHP.flange_b) annotation(
        Line(points = {{30, 8}, {10, 8}}));
      connect(HPT.outlet, LPT.inlet) annotation(
        Line(points = {{42, 18}, {48, 18}, {48, -28}, {54, -28}}, color = {129, 170, 194}));
      connect(LPT.outlet, nozzle.inlet) annotation(
        Line(points = {{66, -24}, {80, -24}}, color = {129, 170, 194}));
      connect(fan.inlet, airIntake.outlet) annotation(
        Line(points = {{-132.375, -15}, {-136.188, -15}, {-136.188, -14}, {-142, -14}}, color = {170, 223, 255}));
      connect(fan.outlet_core, LPC.inlet) annotation(
        Line(points = {{-116.75, -21.25}, {-85.375, -21.25}, {-85.375, -24}, {-88, -24}}, color = {170, 223, 255}));
      connect(nozzleAir.inlet, fan.outlet_bypass) annotation(
        Line(points = {{-97.8, -2.4}, {-105.9, -2.4}, {-105.9, -2.5}, {-116.75, -2.5}}, color = {170, 223, 255}));
      connect(HPC.outlet, combustor.airInlet) annotation(
        Line(points = {{-46, 14}, {-45, 14}, {-45, 32}, {-8, 32}}, color = {170, 223, 255}));
      connect(combustor.exhaust, HPT.inlet) annotation(
        Line(points = {{8, 32}, {30, 32}, {30, 14}}, color = {129, 170, 194}));
      connect(fuelFlow.y, combustor.fuelFlow) annotation(
        Line(points = {{-21, 66}, {0, 66}, {0, 40}}, color = {0, 0, 127}));
      connect(fan.fan_mechanical_interface, LPC.shaft_a) annotation(
        Line(points = {{-110.5, -33.75}, {-99.25, -33.75}, {-99.25, -34}, {-88, -34}}, color = {0, 0, 0}));
      connect(LPC.shaft_b, shaftLP.flange_a) annotation(
        Line(points = {{-76, -34}, {-8, -34}}, color = {0, 0, 0}));
      connect(HPC.shaft_b, shaftHP.flange_a) annotation(
        Line(points = {{-46, 8}, {-10, 8}}));
      connect(initShaftLP.flange, shaftLP.flange_b) annotation(
        Line(points = {{10, -56}, {24, -56}, {24, -34}, {12, -34}}));
      connect(initShaftHP.flange, shaftHP.flange_b) annotation(
        Line(points = {{10, -12}, {18, -12}, {18, 8}, {10, 8}}));
      annotation(
        experiment(StartTime = 0, StopTime = 40, Tolerance = 1e-06, Interval = 0.08),
        Documentation(info = "<html><head></head><body><p>Simple test model of a turbofan engine, using the GSP turbine and compressor data sets.&nbsp;</p>
    <p>The simulation starts at steady state at cruise conditions, with a fuel flow of 0.77 kg/s.</p>
    </body></html>"),
        Diagram(coordinateSystem(extent = {{-160, -100}, {100, 100}})),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end TestTurboFanCruiseSteadyState;
  end TestSystems;
end Tests;
