//!! See rmrtools/MakeRadTempTables.m and PlanckFiltered.m to compute these numbers.
//!! passbandfile='/Users/rmr/Dropbox/instruments/Heitronics_kt15/kt1585-13326_rosr8/kt1585-13326.txt';
// 1. edit PlanckFiltered by adding the correct passbandfile.
// 2. cd to rmrtools and run MakeRadTempTables.
// 3. fill in the coefficients below.
const double default_pt2b[4] = {
	-6.21851e-09,  6.97884e-05,  1.21077e-02,  6.53788e-01
};
const double default_pb2t[4] = {
	1.05158e+01,  -5.02353e+01,  1.33091e+02,  -6.83692e+01
};

//Result from MakeRadTempTables.m
// p3=[-6.21851e-09  6.97884e-05  1.21077e-02  6.53788e-01]
// q = [1.05158e+01  -5.02353e+01  1.33091e+02  -6.83692e+01]