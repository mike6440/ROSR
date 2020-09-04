//!! See rmrtools/MakeRadTempTables.m and PlanckFiltered.m to compute these numbers.
//!! passbandfile='/Users/rmr/Dropbox/instruments/Heitronics_kt15/kt1585-13326_rosr8/kt1585-13326.txt';
// 1. edit PlanckFiltered by adding the correct passbandfile.
// 2. cd to rmrtools and run MakeRadTempTables.
// 3. fill in the coefficients below.
const double default_pt2b[4] = {
	-4.68676e-09,  7.04110e-05,  1.21364e-02,  6.52658e-01
};
const double default_pb2t[4] = {
	1.04987e+01,  -5.01597e+01,  1.32734e+02,  -6.80710e+01
};

// >> MakeRadTempTables
// p3=č-4.68676e-09  7.04110e-05  1.21364e-02  6.52658e-01]
// q = [1.04987e+01  -5.01597e+01  1.32734e+02  -6.80710e+01]
