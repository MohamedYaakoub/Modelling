within BasicAeroEngines;
package Types "Record definitions"
  extends Modelica.Icons.BasesPackage;
  record TurbomachineData
    extends Modelica.Icons.Record;
    parameter Modelica.SIunits.Pressure P_E_nom "Nominal/design inlet pressure";
    parameter Modelica.SIunits.Pressure P_L_nom "Nominal/design outlet pressure";
    parameter Modelica.SIunits.Temperature T_E_nom "Nominal inlet temperature";
    parameter Modelica.SIunits.PerUnit eta_nom "Nominal Isentropic efficiency";
    parameter Modelica.SIunits.MassFlowRate f_nom "Nominal/design mass flow rate";
    parameter Modelica.SIunits.AngularVelocity omega_nom "Nominal/design angular velocity";
  end TurbomachineData;

 record CompressorMapsBetaLinesData
   extends TurbomachineData;
   parameter Modelica.SIunits.PerUnit beta_surge "Beta value of the surge line";
   parameter Modelica.SIunits.PerUnit beta_choke "Beta value of the choke line";
   parameter Modelica.SIunits.PerUnit beta_nom "Nominal beta value (design point)";
   parameter Modelica.SIunits.PerUnit Phi_n_table[:,:] "Data points for Phi_n = Phi_n(N_n, beta)";
   parameter Modelica.SIunits.PerUnit PR_n_table[:,:] "Data points for PR_n = PR_n(N_n, beta)";
   parameter Modelica.SIunits.PerUnit eta_table[:,:] "Data points for eta = eta(N_n,beta)";
 annotation(defaultComponentName = "compressorData",
            defaultComponentPrefixes = "parameter");
 end CompressorMapsBetaLinesData;

 record TurbineStodolaData
   extends TurbomachineData;
   parameter Modelica.SIunits.PerUnit eta_table[:,:] "Data points for eta = eta(N_n,Phi_n)";
 annotation(defaultComponentName = "turbineData",
            defaultComponentPrefixes = "parameter");
 end TurbineStodolaData;

 record TurbineMapsBetaLinesData
   extends TurbomachineData;
   parameter Modelica.SIunits.PerUnit beta_min "Minimum beta value";
   parameter Modelica.SIunits.PerUnit beta_max "Maximum beta value";
   parameter Modelica.SIunits.PerUnit beta_nom "Nominal beta value (design point)";
   parameter Modelica.SIunits.PerUnit Phi_n_table[:,:] "Data points for Phi_n = Phi_n(N_n, beta)";
   parameter Modelica.SIunits.PerUnit PR_n_table[:,:] "Data points for PR_n = PR_n(N_n, beta)";
   parameter Modelica.SIunits.PerUnit eta_n_table[:,:] "Data points for eta = eta(N_n,beta)";
 annotation(defaultComponentName = "compressorData",
            defaultComponentPrefixes = "parameter");
 end TurbineMapsBetaLinesData;
 type Torque = Modelica.SIunits.Torque(nominal = 10000);
end Types;
