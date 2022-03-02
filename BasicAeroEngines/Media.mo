within BasicAeroEngines;
package Media "Working medium models"
  extends Modelica.Icons.MaterialPropertiesPackage;

  package Air
    extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
      mediumName="SimpleAirN2O2",
      data={Modelica.Media.IdealGases.Common.SingleGasesData.N2,
            Modelica.Media.IdealGases.Common.SingleGasesData.O2},
      fluidConstants={Modelica.Media.IdealGases.Common.FluidData.N2,
                      Modelica.Media.IdealGases.Common.FluidData.O2},
      substanceNames = {"Nitrogen", "Oxygen"},
      reference_X={0.768,0.232},
      fixedX = true,
      referenceChoice=Modelica.Media.Interfaces.Choices.ReferenceEnthalpy.ZeroAt25C);
  end Air;

  package ExhaustGas
    extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
     mediumName="ExhaustGas",
     data={Modelica.Media.IdealGases.Common.SingleGasesData.N2,
           Modelica.Media.IdealGases.Common.SingleGasesData.O2,
           Modelica.Media.IdealGases.Common.SingleGasesData.H2O,
           Modelica.Media.IdealGases.Common.SingleGasesData.CO2},
     fluidConstants={Modelica.Media.IdealGases.Common.FluidData.N2,
                     Modelica.Media.IdealGases.Common.FluidData.O2,
                     Modelica.Media.IdealGases.Common.FluidData.H2O,
                     Modelica.Media.IdealGases.Common.FluidData.CO2},
     substanceNames = {"Nitrogen","Oxygen","Water", "Carbondioxide"},
     reference_X={0.768,0.232,0.0,0.0},
     referenceChoice=Modelica.Media.Interfaces.Choices.ReferenceEnthalpy.ZeroAt25C);
  end ExhaustGas;
end Media;
