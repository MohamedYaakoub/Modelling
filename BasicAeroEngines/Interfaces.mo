within BasicAeroEngines;
package Interfaces "Connectors"
  extends Modelica.Icons.InterfacesPackage;

  connector AirPort "Fluid port for air working medium"
    package Medium = Media.Air;
    Medium.AbsolutePressure P "Static pressure";
    flow Medium.MassFlowRate f "Mass flow rate (positive entering)";
    stream Medium.SpecificEnthalpy h_L "Specific enthalpy of leaving fluid";
    annotation (
      Icon(graphics={  Ellipse(lineColor = {170, 223, 255}, fillColor = {170, 223, 255},
              fillPattern =                                                                            FillPattern.Solid,extent = {{-100, 100}, {100, -100}}, endAngle = 360)}));
  end AirPort;

  connector ExhaustPort "Fluid port for exhaust gas working medium"
    package Medium = Media.ExhaustGas;
    Medium.AbsolutePressure P "Static pressure";
    flow Medium.MassFlowRate f "Mass flow rate (positive entering)";
    stream Medium.SpecificEnthalpy h_L "Specific enthalpy of leaving fluid";
    stream Medium.MassFraction X_L[Medium.nX] "Composition of leaving fluid";
    annotation (
      Icon(graphics={  Ellipse(lineColor = {129, 170, 194}, fillColor = {129, 170, 194},
              fillPattern =                                                                            FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
  end ExhaustPort;

  connector AirPortNozzle "Fluid port for air working medium"
    package Medium = Media.Air;
    Medium.AbsolutePressure P "Static pressure";
    flow Medium.MassFlowRate f "Mass flow rate (positive entering)";
    stream Medium.SpecificEnthalpy h_L "Specific enthalpy of leaving fluid";
    stream Medium.MassFraction X_L[Medium.nX]  "Composition of leaving fluid";

    annotation (
      Icon(graphics={  Ellipse(lineColor={170,223,255},     fillColor={85,255,
                255},
              fillPattern=FillPattern.Solid,                                                                             extent = {{-100, 100}, {100, -100}}, endAngle = 360)}));
  end AirPortNozzle;

  model Quality

    Interfaces.AirPort inlet
      annotation (Placement(transformation(extent={{-56,-8},{-36,12}})));
    Interfaces.AirPortNozzle outlet
      annotation (Placement(transformation(extent={{34,-8},{54,12}})));

  equation
    inlet.f -outlet.f = 0;
    outlet.P = inlet.P;
    outlet.h_L = inlet.h_L;
    outlet.X_L = {0.768,0.232};

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(extent=
               {{-46,42},{44,-44}}, lineColor={28,108,200})}));
  end Quality;
end Interfaces;
