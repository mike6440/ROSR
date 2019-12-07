//!! See rmrtools/MakeRadTempTables.m and PlanckFiltered.m to compute these numbers.
//!! passbandfile='/Users/rmr/Dropbox/instruments/Heitronics_kt15/kt1585-13324_rosr6/kt1585-13324.txt';
// 1. edit PlanckFiltered by adding the correct passbandfile.
// 2. cd to rmrtools and run MakeRadTempTables.
// 3. fill in the coefficients below.
const double default_pt2b[4] = {
	-7.05571e-09,  6.95140e-05,  1.20961e-02,  6.54263e-01
};
const double default_pb2t[4] = {
	1.05200e+01,  -5.02525e+01,  1.33223e+02,  -6.84890e+01
};
