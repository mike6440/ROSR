package perltools::MRradiation;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&Ephem &Ephemx &gravity &IndexOfRefraction &ScatteringCrossSection 
	&aod_rayleigh &aod_rayleigh_m &AtmMass &ComputeLongwave &solflux &SunDistanceRatio 
	&aod_ozone &RelativeSolarVector &RotationTransform);

#USE IN A PROGRAM
use lib $ENV{MYLIB};
use perltools::MRtime;
use POSIX;
use Math::MatrixReal;

use constant PI => 3.14159265358979;
use constant D2R => PI / 180;
use constant R2D => 180 / PI;
use constant TWOPI => 2 * PI;
use constant MISSING => -999;


#=========================================================================
sub gravity
	# %function g = gravity(z, lat)
	# %====================================================
	# % COMPUTE GRAVITY ON THE EARTH
	# %
	# % Taken from Bodhaine et. al, "ON Rayleigh Depth Calculations"
	# % JTEC, 16, 1854-1861.
	# % They, in turn, reference List (1968) Smithsonian Meteorological Tables.
	# %
	# %input
	# %  z = altitude in m above s.l.
	# %  lat = latitude if f.p. degrees
	# %output
	# %  g = gravity in cm/s^2
	# %
	# % reynolds 010726
	# %======================================================
	# 
	# % test lat=0;  z = 5000;
{
	my ($z, $lat)=@_;
	
	my $d2r = acos(-1)/180;
	my $cos2phi = cos($d2r * $lat * 2);
	
		#GRAVITY AT SEA LEVEL
	my $g0 = 980.6160 * (1 - 0.0026373 * $cos2phi + 0.0000059 * $cos2phi*$cos2phi);
	
	my ($g, $zc);
	if( $z == 0 ) {$g = $g0}
	else {
		my $zc = (0.73737 * $z) + 5517.56;
		$g = $g0 
			-(3.085462e-4 + 2.27e-7 * $cos2phi) * $zc 
			+(7.254e-11 + 1e-13 * $cos2phi) * $zc * $zc
			- (1.517e-17 + 6e-20 * $cos2phi) * pow($zc,3);
	}
	return $g;  									# convert to m/s^2
}


#=========================================================================
sub ScatteringCrossSection
	# 
	# #function sigma = ScatteringCrossSection(w, tair, co2) 
	# %===========================================
	# % COMPUTE SIGMA, THE SCATTERING CROSS SECTION
	# %
	# % Taken from Bodhaine et. al, "ON Rayleigh Depth Calculations"
	# % JTEC, 16, 1854-1861.
	# %
	# %input
	# %  ref_index = refractive index (see function RefractionIndex.m)
	# %  Tair = temperature of the air in degrees C
	# %  w = wavelength in nm
	# %  co2 = co2 concentration in ppm
	# %
	# %output
	# %  sigma 
	# %
	# % reynolds 010726
	# %======================================================
{
# %TEST  ScatteringCrossSection(415,15,360) = 1.435769e-26
	my ($w, $tair, $co2) = @_;
	my $pi = acos(-1);  #3.14159265359;
	# REFRACTION INDEX
	my $n = IndexOfRefraction($w,$co2);
	my $n2=$n*$n+2;
	my $n1=$n*$n-1;
		# MOLECULAR DENSITY -- molecules per cm^-3
	my $Ns = 6.0221367e23 * 273.15 / ($tair+273.15) / 22.4141 /1000;
		# Depolarization term
	my $wmu=$w/1000;           # convert wavelength in nanometers to micrometers
	my $FN2 = 1.034 + (3.17e-4/($wmu*$wmu));
	my $FO2 = 1.096 + (1.385e-3/($wmu*$wmu)) + (1.448e-4/pow($wmu,4));
	my $FAr = 1.0;
	my $concenCO2= $co2 / 10000;        # converts ppm to parts per volume by percent of CO2
	my $Fair = (78.084*$FN2 + 20.946*$FO2 + 0.934*$FAr + $concenCO2*1.15)/(78.084 + 20.946 + 0.934 + $concenCO2);
	my $wcm=$w/1e7;  # converts wavelength in nanometers to centimeters
	my $sigma_term1 = 24 * pow($pi,3) * pow($n1,2);
	my $sigma_term2 = pow($wcm,4) * pow($Ns,2) * pow($n2,2);
	my $sigma = $sigma_term1 / $sigma_term2 * $Fair;
	return $sigma;
}


#==============================================================================
sub IndexOfRefraction
	# %function [n] = IndexOfRefraction(w,co2ppm)
	# %====================================================
	# % COMPUTE ATMOSPHERIC INDEX OF REFRACTION ON THE EARTH
	# %
	# % Taken from Bodhaine et. al, "ON Rayleigh Depth Calculations"
	# % JTEC, 16, 1854-1861.
	# %
	# %input
	# %  w = wavelength in nanometers
	# %  co2 = co2 concentration in parts per million by volumn
	# %  lat = latitude if f.p. degrees
	# %output
	# %  g = gravity in m/s^2
	# %
	# % reynolds 010726
	# %======================================================
{
	my ($w, $co2ppm)=@_;
	
	my $wu = $w / 1000;  # convert from nm to microns
	my $co2 = co2ppm / 1e6;  #  (e.g. 360 ppm => 0.00036)
		#INDEX FOR DRY AIR AT 300 PPM CO2
	my $n1300 = 8060.51 + 2480990 / (132.274 - pow($wu,-2)) + 17455.7 / (39.32957 - pow($wu,-2)); 

		# Scale for desired co2
	my $r = 1 + 0.54 * ($co2 - 0.0003);
	my $n1 = $n1300 * $r;
	
	my $n = $n1 / 1e8 + 1;
	return $n;
}

#=====================================================
sub aod_rayleigh
#CALLING:
#  ($ar) = aod_rayleigh ( $chan );
#INPUT
#  $chan = 2-7.  If chan=1, return 0
#
#  Computes the Rayleigh Optical Thickness for 
#  any frsr detector number
#  Based on the paper "bodhaine99"
# v101 060808 rmr -- from soarmatlab getrayleigh()
{
	my $det=shift;
	
	# RAYLEIGH AOD FOR CHANNELS 2-7
	my @a = (0.309, 0.14336, 0.061586, 0.040963, 0.001513, 0.001108);
	#my @a = (0.321, 0.151, 0.064, 0.044, 0.016, 0.012);  # for lambda head 351, Magic
	if ( $det < 2 || $det > 7 ) { return MISSING }
	
	return $a[$det-2];
}


#=====================================================
sub aod_rayleigh_m 
#	tr = aod_rayleigh_m(z, lat, w, co2, Tair, p)
# % COMPUTE RAYLEIGH AOT ON THE EARTH
# %
# % Taken from Bodhaine et. al, "ON Rayleigh Depth Calculations"
# % JTEC, 16, 1854-1861.
# %
# %input
# %  z = altitude in m above s.l.
# %  lat = latitude if f.p. degrees
# %  w = wavelength in nm
# %  co2 = co2 concentration in ppm
# %  Tair = temperature of the air degrees C
# %  pressure in mb
# %  gravity m/sec^2
# %output
# %  tr = Rayleigh optical thickness
# % Matlab functions
# % TEST  MLO
# %clear;  z = 3400;  lat = 19.533;  w = 500;  co2 = 360; Tair = 15; p=680.;
# 
# %======================================================
{
	my $tr=0;
	my ($z, $lat, $w, $co2, $tair, $p) = @_;
	#printf"z=%.1f, lat=%.6f, w=%.3f, co2=%.3f, tair=%.1f, p=%.1f\n",$z,$lat,$w,$co2,$tair,$p;

	my $A=6.0221367e23;     # Avogadro's number

		#COMPUTE THE INDEX OF REFRACTION
	my $n = IndexOfRefraction($w,$co2);
	#printf"n = %.10e\n", $n;

		#Scattering Cross Section
	my $sigma=ScatteringCrossSection($w,$tair,$co2);
	#printf"sigma = %.10e\n", $sigma;

		# mean nolecular weight of dry air
	my $concenCO2=$co2/1e6;        						# converts ppm to parts per volume
	my $ma = (15.0556 * $concenCO2) + 28.9595;          # no. molecules per mole
	#printf"concenCO2=%.10e,  ma=%.10e\n",$concenCO2,$ma;

		# gravitational constant
	my $g = gravity($z, $lat);
	
		# calculate Rayleigh optical thickness
	$p=$p*1e3;        			# convert mb to gm cm/sec^2 /cm^2
	my $tr=$sigma * $p * $A / $ma / $g; 

	return $tr;
}

# ================================================
sub Ephem
# CALL: (az, ze, ze0) = Ephem(lat, lon, dt); 
# function [az, ze, ze0] = Ephem(lat, lon, dt);
# pro sunae1,year,day,hour,lat,long,az,el
#       implicit real(a-z)
# Purpose:
# Calculates azimuth and elevation of sun
# 
# References:
# (a) Michalsky, J. J., 1988, The Astronomical Almanac's algorithm for
# approximate solar position (1950-2050), Solar Energy, 227---235, 1988
# 
# (b) Spencer, J. W., 1989, Comments on The Astronomical
# Almanac's algorithm for approximate solar position (1950-2050)
# Solar Energy, 42, 353
# 
# Input:
# year - the year number (e.g. 1977)
# day  - the day number of the year starting with 1 for
#        January 1
# time - decimal time. E.g. 22.89 (8.30am eastern daylight time is
#        equal to 8.5+5(hours west of Greenwich) -1 (for daylight savings
#        time correction
# lat -  local latitude in degrees (north is positive)
# lon -  local longitude (east of Greenwich is positive.
#                         i.e. Honolulu is 15.3, -157.8)
# Output:
# az - azimuth angle of the sun (measured east from north 0 to 360)
# el - elevation of the sun not currently returned
# ze - zenith angle at Earth's surface
# ze0 - zenith angle at top of atmosphere
# Spencer correction introduced and 3 lines of Michalsky code
# commented out (after calculation of az)
# 
# Based on codes of Michalsky and Spencer, converted to IDL by P. J.  Flatau

# 2001 rmr conversion from IDL to Matlab.
#  Reynolds 010318 -- remove singularity at line 142.
# 060629 rmr -- Converted from IDL to Matlab years before and today
#  converted from matlab to PERL.  This perl version was checked against
#  the matlab function and exactly matches.
#Test Ephem LAT: -11, LON: 130, DATE: 2006-02-01 (032) 03:00:00
#                 AZ=24.143,  ZE=13.227, ZE0=13.239

# REQUIRED PERL SUBROUTINES:
#  datevec() --

#EPHEMERIS CHECKING:
#First I got the matlab/ephim.m subroutine working::
#NOAA: http://www.esrl.noaa.gov/gmd/grad/solcalc/  		NREL: http://www.nrel.gov/midc/srrl%5Fbms/
#Gan I. (-0.6906, 73.15)     NOAA CALCULATOR				 MATLAB ephem*			NREL CALCULATOR
#	2011-12-21 	060000		146.36	62.3	27.7		146.352		27.679		146.3708	27.692
#		"      	070000      176.9	67.23	22.8		176.909		22.755		176.919		22.772
#		"		080000		209.3	63.67	26.33		209.317		26.318		209.311		26.335
#		"		120000		245.86	15.29	74.71		245.872		74.709		245.862		74.716
#Around Zero                              NOAA CALCULATOR 	MATLAB ephem 	  		PERL				NREL SPA CALCULATOR
#	0.0  0.0  2011-09-21	12000000	293.26	1.85   		293.219 	1.85		293.22	1.86 	√	
#	2.0  0.0      "				"		233.18	2.12		233.364		2.11		233.36	2.12 	√
#   -2.0  0.0  		"			"		328.12	3.21		328.012		3.21		211.89	3.22 	√
#   40.0	 0.0		"			"		182.68	39.29		182.692		39.282		182.69	39.29 	√
#  -40.0  0.0   		"			"		357.4	49.26		357.381		40.74		357.39	40.74 	*	357.38	40.75
#* 120315--There is a discrepancy in the computed ze between rmr and NOAA. I think NOAA is in error.

#v101 060629 rmr -- convert from matlab to perl
#================================================
{
# 	use constant PI => 3.14159265358979;
# 	use constant D2R => PI / 180;
# 	use constant R2D => 180 / PI;
# 	use constant TWOPI => 2 * PI;
	my $lat = shift();  my $lon = shift();  my $dt = shift();
	#printf "test BEGIN SUBROUTINE Ephem( %.5f, %.5f, %s )\n", $lat, $lon, dtstr($dt,'short');
	
	my ($yy, $MM, $dd, $hh, $mm, $ss);
	my ($hour, $jd, $jdf);
	
	($yy, $MM, $dd, $hh, $mm, $ss, $jdf) = datevec($dt); # day components
	$hour=$hh + $mm/60 + $ss/3600;
	#printf "test YEAR: %d, MONTH: %d, DAY: %d, HOUR: %d, MIN: %d, SEC: %s, JD: %d\n",
	# $yy, $MM, $dd, $hour, $mm, $ss, $dd;

	# get the current Julian date
	my $delta = $yy - 1949.;
	my $leap = int( $delta / 4 );
	$jd = 32916.5 + $delta * 365 + $leap + int($jdf) + $hour / 24;
	#printf "test YY: $yy, DAY: $dd, DELTA: $delta, LEAP: $leap, JDAY: %.6f\n", $jd;
	
	# calculate ecliptic coordinates
	my $time = $jd - 51545.0;
	#printf"test TIME: %.6f\n", $time;
	
	# force mean longitude between 0 and 360 degs
	my $mnlong = 280.460 + 0.9856474 * $time;
	##printf"MNLONG = %.6f\n", $mnlong;
	$mnlong -= 360 * int($mnlong/360);
	if ( $mnlong < 0 ) { $mnlong += 360 }
	if ( $mnlong > 360 ) { $mnlong -= 360 }
	#printf"MNLONG = %.6f\n", $mnlong;
	
	# mean anomaly in radians between 0, 2*pi
	my $mnanom = 357.528 + 0.9856003 * $time;
	#$mnanom = $mnanom%360;
	$mnanom -= 360 * int($mnanom/360);
	if ( $mnanom < 0 ) { $mnanom += 360 }
	$mnanom *= D2R;
	#printf"MNANOM = %.6f\n", $mnanom;
	
	# compute ecliptic longitude and obliquity of ecliptic
	#eclong=mnlong+1.915*(mnanom)+0.20*sin(2.*mnanom);
	my $eclong = $mnlong + 1.915 * sin($mnanom) + 0.020 * sin(2 * $mnanom);
	$eclong -= 360 * int($eclong/360);
	if ( $eclong < 0 ) { $eclong += 360 }
	$eclong *= D2R;
	#printf"ECLONG: %.6f\n", $eclong;
	
	my $oblqec = 23.429 - 0.0000004 * $time;
	$oblqec *= D2R;
	#printf"OBLQEC: %.6f\n", $oblqec;
	
	# calculate right ascention and declination
	my $num = cos($oblqec) * sin($eclong);
	my $den = cos($eclong);
	my $ra = atan($num / $den);
	#print"NUM: $num, DEN: $den,  RA: $ra\n";
	
	# force ra between 0 and 2*pi
	if ( $den < 0 ) { $ra += PI }
	elsif ( $den >= 0 ) { $ra += TWOPI }
	
	# dec in radians
	my $dec = asin( sin($oblqec) * sin($eclong) );
	#printf"RIGHT ASCENSION: %.6f, DECLINATION: %.6f\n",$ra, $dec;
	
	# calculate Greenwich mean sidereal time in hours
	my $gmst = 6.697375 + 0.0657098242 * $time + $hour;
	#print"GMST: $gmst\n";
	
	# hour not changed to sidereal sine "time" includes the fractional day
	$gmst -= 24 * int($gmst/24);
	#printf"GMST: %.6f\n", $gmst;
	if ( $gmst < 0 ) { $gmst += 24 }
	#printf"GMST: %.6f\n", $gmst;
	
	# calculate local mean sidereal time in radians
	my $lmst = $gmst + $lon / 15;
	$lmst -= 24 * int($lmst/24);
	if ( $lmst < 0 ) { $lmst -= 24}
	$lmst = $lmst * 15 * D2R;
	#printf"LMST: %.6f\n", $lmst;
	
	# calculate hour angle in radians between -pi, pi
	my $ha = $lmst - $ra;
	if ( $ha < -(PI) ) { $ha += 2*PI }
	if ( $ha > PI ) { $ha -= TWOPI }
	#printf"HA: %.6f\n", $ha;
	
	# calculate azimuth and elevation
	$lat *= D2R;
	my $el = asin( sin($dec) * sin($lat) + cos($dec) * cos($lat) * cos($ha) );
		#!! special for MLO
	my $az = asin( cos($dec) * sin($ha) / cos($el) ) + PI;
	#printf"test234: el=%.2f  ze=%.2f  az=%.2f\n",$el*R2D, (PI/2-$el)*R2D, $az*R2D;
	
	#========================
	# add J. W. Spencer code
	#========================
	#if ( $lat == 0 ) { $lat += 1e-5 } # move awat from a singularity	
	#my $elc = asin( sin($dec) / sin($lat) );
	#if ( $el >= $elc ) { $az = PI - $az }
	#if ( $el <= $elc && $ha > 0 ) { $az = 2*PI + $az }
	#printf"EL: %.6f, AZ: %.6f\n", $el, $az;
	
	#=================
	# REFRACTION
	#=================
	my $refrac;
	# this puts azimuth between 0 and 2*pi radians
	# calculate refraction correction for US stand. atm.
	$el *= R2D;
	if ( $el > -0.56 ) {
		$refrac = 3.51561 * (0.1594 + 0.0196 * $el + 0.00002 * $el*$el) /
	     (1 + 0.505 * $el + 0.0845 * $el*$el)
	} else { $refrac = 0.56 }
# 	printf"REFRAC: %.6f\n", $refrac;
	#$refrac = 0.56;
	
	my $ze0 = 90 - $el;
	my $ze = 90 - ($el + $refrac);
	$az *= R2D;
	if ( $az < 0 ) { $az += 360 }
	#printf"ZE0: %.6f,  ZE: %.6f, AZ: %.6f\n", $ze0, $ze, $az;
	return($az, $ze, $ze0);
}

#======================================================
sub AtmMass
# function m = AtmMass(z,p)
# %ATMMASS - compute atmospheric mass for zenith angle
# %=========================================================
# %	m = AtmMass(z)
# %
# % From Schwindling et al. (1998) JGR, 103, 24919-24935
# %and
# %Kasten and Young (1989), Appl Optics, 28, 4735-4738
# %
# % Absolute airmass (Wikepedia)  http://en.wikipedia.org/wiki/Air_mass_(solar_energy)
# %The relative air mass is only a function of the sun's zenith angle, 
# %and therefore does not change with local elevation. 
# %Conversely, the absolute air mass, equal to the relative air mass 
# %multiplied by the local atmospheric pressure and divided by the standard 
# %(sea-level) pressure, decreases with elevation above sea level. 
# %For solar panels installed at high altitudes, e.g. in an Altiplano region, 
# %it is possible to use a lower absolute AM numbers than for the corresponding 
# %latitude at sea level: AM numbers less than 1 towards the equator, 
# %and correspondingly lower numbers than listed above for other latitudes. 
# %However, this approach is approximate and not recommended. 
# %It is best to simulate the actual spectrum based on the relative air mass 
# %(e.g., 1.5) and the actual atmospheric conditions for the specific elevation of 
# %the site under scrutiny.
# %input: 
# % z = zenith angle
# % p = (1x1) local pressure in mbar (optional)
# %      
# %output: m = atmospheric mass
# % if narg = 2, use p and output absolute am.
# % if narg = 1, ignore p and use simply 1/cos ze
# %
# %reynolds 981105, 141211
# %=========================================================
## v101 060805 rmr -- adapted from Matlab
{
# 	use constant D2R => 0.017453292;
# 	use constant MISSING => -999;
	
	my $a = 0.50572;
	my $b = 6.07995;
	my $c = -1.6364;
	my $z = $_[0];
	if ($#_ == 1 ){$p=$_[1]}else{$p=1013.25}
	my $m = MISSING;
	
	
	if ( $z > 89 || $z < 0 ) { return MISSING }
	else {
		$m = cos( $z * D2R ) + $a * ( $z + $b)**$c;
		$m = 1.0 / $m;
		$m = $m * $p / 1013;
	}
	return $m;
}

#======================================================================
sub ComputeLongwave
# Uses the Albrecht method, the classic style of computation
# of longwave flux from the Eppley PIR.
# See papers by Fairall, Albrecht, and Paine on LW computations.
# also see matlab routine "PirTcTd2LW" in RMRTOOLS
# CALL:
#     $lw=ComputeLongWave $e,$tc,$td,$k,$sigma,$epsilon,$missing;
#
#INPUT: e, tc, td, K, sigma, epsilon
# where
# 0. e = thermopile computed flux in W/m^2
# 1. tc = case degC
# 2. td = dome degC
# 3. k = calib coef, usually = 4.
# 4. sigma = Boltzman's constant = 5.67e-8;
# 5. epsilon = instrument emissivity, usually 1.0 (Fairall98 and personal communication)
# 6. missing value, typically -999
#OUTPUT
# lw = longwave flux, (W/m^2)
# C_c = case correction (W/m^2)
# C_d = dome correction, (W/m^2)
#EXAMPLE
# -20,25,26 => 403.88 W/m^2
# -20,26,25 => 409.68
#  Note 1degC error gives 5 W/m^2 error
#
# 051218 -- rmr
#v101 060629 rmr -- start config control
{
	my ($lw, $C_c, $C_d, $tabs, $c, $d, $tc, $td);
	#print "Test ComputeLongwave ($_[0], $_[1], $_[2], $_[3], $_[4], $_[5], $_[6])\n";
	if ( $_[0] == $_[6] || $_[1] == $_[6] || $_[2] == $_[6] ) {
		$lw = $C_c = $C_d = $_[6] }
	else {
		$tabs = 273.15;
		$tc = $_[1] + $tabs;
		$td = $_[2] + $tabs;
		$c = $tc * $tc * $tc * $tc;
		$d = $td * $td * $td * $td ;
		$C_c = $_[5] * $_[4] * $c;
		$C_d = -$_[3] * $_[4] * ($d - $c);
		$lw = $_[0] + $C_c + $C_d;
	}
# 	print "  C_c = $C_c\n";
# 	print "  C_d = $C_d\n";
# 	print "  lw = $lw\n";
	return ($lw, $C_c, $C_d);
}

#==================================================================
sub solflux
# function (In,Id, Tr, To, Tg, Tw, Ta) = solflux(zdeg,w, p, k1, k2, l)
# SOLFLUX - COMPUTES GLOBAL SOLAR FLUX, CLEAR SKY
#   [In, Id, Tr, To, Tg, Tw, Ta] = solflux(zdeg, w, p, k1, k2, l)
# ==================================================================
#  Calculate direct solar irradiance using a parameterization technique
#  from M. Iqbal in "Physical Climatology for Solar and Wind Energy",
#  1988, World Scientific, pp. 196-242. 
# 
# Input:
#  zdeg = zenith angle in degrees
#  w = integrated water vapor (g/cm^2) typically 5 for TWP
#  p == surface pressure in hPa (optional, default = 1013)
#  k1 == aerosol optical thickness at 380 nm (optional, default = 0.0)
#  k2 == aerosol optical thickness at 500 nm (optional, default = 0.0)
#  l == ozone-layer thickness in cm(NTP) (optional, default = 0.3)
# Output:
#  In == Direct-Normal flux at surface
#  Id == Diffuse flux at surface
#  Tr == Transmittance by Rayleigh scattering
#  To == Transmittance by ozone
#  Tg == Transmittance by uniformly mixed gases
#  Tw == Transmittance by water vapor
#  Ta == Transmittance by aerosol
# Internal constants:
#  Isc == Solar constant = 1360 W/m^2
#  mu = cos(zenith);
#  Ma == Relative optical air mass
# 
# reynolds 981029
# v101 060630 rmr -- translate to PERL and check against matlab.
# =================================================================
{	
	# =================
	#  CONSTANTS
	# =================
# 	use constant PI => 3.14159265358979;
# 	use constant R2D => 180 / PI;
	use constant ISC => 1360.;
	use constant W0AER => 0.0;
	use constant FC => 0.84;
	use constant AG => 0.2;

	 my $zdeg = shift;  my $w = shift;  my $p = shift;  
	 my $k1 = shift;  my $k2 = shift;  my $l = shift;
	 
	 
	#==========================
	# CLIP ALL ZENITH ANGLES TO <= 89.99 DEG.
	#==========================
	if ( $zdeg > 89.99 ) { 
		$zdeg = 89.99;
	}
	my $mu = cos( $zdeg / R2D );
		 
	#=============================
	# COMPUTE AIR MASS AND OTHER PROPERTIES
	#=============================
	my $Mr = 1.0 / ($mu + 0.15 * (93.885 - $zdeg ) ** (-1.253) );
	my $Ma = $Mr * ($p / 1013.25);
	my $U1 = $w * $Mr;
	my $U3 = $l * $Mr;
	my $ka = 0.2758 * $k1 + 0.35 * $k2;
	 
	#===================
	# First calculate the transmittances due to various species
	# RAYLEIGH SCATTERING
	#=================
	my $Tr;
	if ( $zdeg <= 85 ) {
		$Tr = exp( -0.0903 * ( $Ma ** 0.84 * ( 1.0 + $Ma - ( $Ma ** 1.01) ) ) )
	} else {
		$Tr = exp( -( 0.0738 + ( $zdeg - 85) * .03) * ($Ma) ** 0.84);
	}
	 
	#===============
	# OZONE SCATTERING
	#================
	my $To = 1 - (0.1611 * $U3 * (1+139.48 * $U3) ** (-0.3035) - 0.002715 * $U3 / 
		(1+.044 * $U3 + 0.0003 * $U3 * $U3));
	 
	#================
	# UNIFORM GASES SCATTERING
	#================
	my $Tg = exp(-0.0127 * ($Ma) ** 0.26);
	 
	#===============
	# WATERVAPOR SCATTERING
	#===============
	my $Tw = 1 - 2.459 * $U1 / ((1+79.034 * $U1) ** 0.6828 + 6.385 * $U1);
	 
	#==============
	# AEROSOL SCATTERING
	#==============
	my $Ta = exp( -$ka ** 0.873 * (1 + $ka - $ka ** 0.7088) * $Ma ** 0.9108);
	 
	#=============
	# direct normal flux
	#=============
	my $In = ISC * $Tr * $To * $Tg * $Tw * $Ta - 0.95; #v101 a small .95 correction to give 0 at night.
	 
	#=============
	# diffuse component
	#===============
	my $Taa = 1 - (1 - W0AER) * (1 - $Ma + $Ma ** 1.06) * (1-$Ta);
	 
	my $Idr = 0.79 * ISC * $mu * $To * $Tg * $Tw * $Taa * ( 0.5 * (1-$Tr)) /
		(1 - $Ma + $Ma ** 1.02);
		
	my $Tas = $Ta / $Taa;
	 
	my $Ida = 0.79 * ISC * $mu * $To * $Tg * $Taa * (FC * (1 - $Tas)) / (1 - $Ma + $Ma ** 1.02);
	 
	my $aa = 0.0685 + ( 1 - FC ) * ( 1 - $Tas);
	 
	my $Idm = ( $In * $mu + $Idr + $Ida) * ( AG * $aa / (1 - AG * $aa));
	 
	my $Id = $Idr + $Ida + $Idm;
	 
	#=================
	# we get negative diffuse components??
	#  This is a quick fix
	#=================
	if ( $Id < 0 ) { $Id = 0 }
	 
	return ( $In, $Id, $Tr, $To, $Tg, $Tw, $Ta );
}

#=======================================================
sub SunDistanceRatio
# function  [r, d,  dmean] = SunDistance(dt)
# %SUNDISTANCE - compute the sun earth distance ratio
# %	[r, d, dmean] = SunDistance(dt)
# %====================================================
# % The sun-earth distance ratio is used to correct the solar
# %constant.
# %
# % Taken from Schwindling et al.,1998, JGR, 103(C11),24919-24935.
# % who  attribute this to Paltridge and Platt, 1977, Radiative
# %Processes in Meteorology and Climatology, in "Developments
# %in Atmos. Science 5," Elsevier Sci., New york.
# %
# % ALTERNATIVE FROM JOE MICHALSKY, EMAIL 02030`
# % calculates r2 = (1/r)^2
# % g = (2 * pi * (jdf-1)) / 365;
# % r2 = 1.00011 + 0.034221 * cos(g) + 0.00128 * sin(g) + ...
# %   0.000719 * cos(2 * g) + 7.7e-5 * sin(2 * g);
# %input:
# % dt = datenum
# %output:
# % r = the ratio of the distance to the sun.
# % d = actual distance (km)
# % dmean = the mean distance, a constant. (km)
# %
# %reynolds 020311
# %==========================================================
# 
# %TEST
# FOR 060805 --
# D = 151,825,492 km see http://www.galaxies.com/calendars.aspx
# gives r = 1.0148907.
# Schwindling method gives 1.01444.
# Michalsky method is way off.
## v101 060805 rmr -- PERL adapted from Matlab
{
# 	use constant PI => 3.14159265358979;

	my $dmean = 149597870.691;  # km ;  see http://neo.jpl.nasa.gov/glossary/au.html
	my $dt=shift();
	# =====================
	# COMPUTE THE JULIAN DAY
	# =====================
	my ( $y, $jdf) = dt2jdf( $dt );
	#printf"test dt=%s, y=$y, jdf=$jdf\n", dtstr($dt);
	##Schwindling
	
	# ======================
	# SUN-EARTH DISTANCE IN AU
	# ======================
	my $r = 1 - 0.01673 * cos(0.017201 * ( $jdf - 4));  ## Schmindling eq 9 method
	## Michalsky method (computes (1/r)^2)
	my $g = (2 * PI * ( $jdf - 1)) / 365;
	my $r2 = 1.00011 + 0.034221 * cos($g) + 0.00128 * sin($g) + 
	  0.000719 * cos(2 * $g) + 7.7e-5 * sin(2 * $g);  ## Michalsky method
	
	#printf"test SOLAR DISTANCE: %.5f, %.1f, %.1f\n",$r, $dmean*$r, $dmean;
	return ($r, $dmean * $r, $dmean);
}

#==============================================================
sub aod_ozone
# (tau, dob) = aod_ozone( dt, lat, det)
# ==========================================================================
# This function computes the climatological ozone concentration in dobson 
# units for a specified latitude on a specified julian day.  
# The climatological ozone concentrations is included in this function.
# 
# input
#  dt = datesec
#  lat = latitude nx1 vector
#  det = [1,...,7] is the channel number for the frsr
#  
# output
#  tau = optical thickness for ozone
#  dob = dobson units for this time and latitude
# =========================================================================
{
	my ($dt, $lat, $det) = @_;
	
	if ( $det < 2 || $det > 7 ) { return ( MISSING, MISSING ) }
	
	#=======================
	# CLIMATOLOGICAL OZONE
	# organized by 10 deg latitude bands and month
	#=======================
	my @oz = (
	315,330,338,330,315,290,264,245,240,240,249,265,284,305,325,346,352,348,340,
	360,376,380,372,350,316,278,252,240,240,242,257,276,291,307,316,318,315,307,
	420,428,422,405,380,340,295,260,242,240,240,252,268,285,296,302,300,300,300,
	440,440,440,423,394,347,304,272,253,240,242,252,262,280,290,293,300,300,300,
	430,430,428,415,380,342,305,275,255,240,244,252,260,279,288,292,300,300,300,
	400,395,390,377,353,330,295,272,255,240,246,254,265,286,296,295,300,300,300,
	350,350,350,340,330,310,281,265,252,240,248,258,273,295,307,305,300,300,300,
	315,315,317,314,310,290,273,258,244,240,250,262,280,307,316,314,304,300,300,
	287,292,294,297,293,278,263,250,240,240,252,268,291,318,327,324,313,300,300,
	280,280,288,291,284,270,257,244,240,240,256,276,300,327,335,335,322,308,300,
	285,290,294,293,284,268,255,240,240,240,259,278,300,331,344,352,338,323,312,
	295,300,310,308,295,275,256,240,240,240,256,272,292,320,340,360,360,360,355,
	315,330,338,330,315,290,264,245,240,240,249,265,284,305,325,346,352,348,340,
	360,376,380,372,350,316,278,252,240,240,242,257,276,291,307,316,318,315,307);
	
	# coefficients to convert dobson units to optical depth
	# for different bands.
	# Note broadband channel is set to zero for now
	my @ozcoef = (0, 0, 0.0328, 0.1221, 0.04976, 0.0036);
	
	if ( $lat < -90 ) { return (MISSING, MISSING) }
	
	#  THE MONTH DETERMINES THE ROW
	my ( $y, $m, $d );
	($y,$m,$d) = datevec( $dt );
	#print"month = $m\n";
	
	# THE LATITUDE BAND DETERMINES THE COLUMN
	my $ix = ($m-1)*19 + int( ( $lat + 90 ) / 10);
	#print"INDEX = $ix\n";
	
	# READ DOBSON FROM THE ARRAY
	my $dob = $oz[$ix];
	
	# COMPUTE THE OPTICAL DEPTH USING COEFFICIENTS
	my $aod = $dob * $ozcoef[$det-2] / 1000;
	
	#printf"ozone aod=%.6f, dob=%.0f\n", $aod, $dob;
	return ($aod, $dob);
}

#======================================================
sub RelativeSolarVector
# CALLING:
# (sz_rel, saz_rel) = RelativeSolarVector( saz, sz, az, pitch, roll)
#  
#   calculates the angle between the solar ray and sensor normal
# INPUT
#    sz  solar zenith angle, deg
#    saz solar azimuth angle, deg
#    az  ship heading (compass), deg
#    pitch ship pitch (positive bow up), deg
#    roll  ship roll  (positive port up), deg
#OUTPUT (references (pointers))
# sz_rel = solar zenith angle relative to the sensor normal, deg
# saz_rel = solar azimuth relative to the sensor reference mark.
# HISTORY
#  copied from Matlab: reynolds 990710
# v101 060729 rmr -- converted to PERL. 
#  ======================================================================
# USE MODULE:  Math-MatrixReal-2.01 > Math::MatrixReal
# This can be down loaded from CPAN and installed in the usual way.
# The matrix transpose is used to determine the relative solar
# azimuth and zenith angle from the ship pitch-roll-heading.
# See MATLAB routine.
# use Math::MatrixReal
# use constant PI => 3.14159265358979;
# use constant D2R => PI / 180;
# use constant R2D => 180 / PI;
# use constant TWOPI => 2 * PI;
# use constant MISSING => -999;
{
	my $sz = shift();
	my $saz = shift();
	my $az = shift();
	my $p = shift();
	my $r = shift();
# 	printf "RelativeSolarVector: %.1f, %.1f, %.1f, %.1f, %.1f\n",$saz, $sz, $az, $p, $r;
	if ( $saz == MISSING || $sz == MISSING || $az == MISSING || $p == MISSING || $r == MISSING ) {
		return (MISSING, MISSING);
	}
	else {
		# ==== SOLAR UNIT VECTOR IN TRUE EARTH COORDINATES ============
		my ($Ssz, $Csz, $Ssaz, $Csaz, $a1, $a2, $a3, $amag, $SolarVector);
		$Ssz = sin ( $sz * D2R);
		$Ssaz = sin ( $saz * D2R);
		$Csz = cos ( $sz * D2R);
		$Csaz = cos ( $saz * D2R);
		$a1 = $Ssz * $Ssaz;
		$a2 = $Ssz * $Csaz;
		$a3 = $Csz;
		$amag = sqrt ($a1*$a1 + $a2*$a2 + $a3*$a3 );
		$SolarVector = Math::MatrixReal->new_from_cols( [ [$a1, $a2, $a3] ] );
# 		printf "Direct Normal Unit Vector: (%.3f,%.3f,%.3f) = %.3f\n",$a1,$a2,$a3,$amag;
# 		print"Solar Unit Vector:\n";
# 		print $SolarVector;
		
		# ==== TRANSFORMATION MATRIX ==============
		$upvec = Math::MatrixReal->new_from_cols([ [0,0,1] ] );
		my ($x_r, $T);
		($x_r, $T) = RotationTransform( $upvec, $p, $r, $az );
# 		print "TRANSFER MATRIX: \n";
# 		print $T;
# 		print "x_r,  UNIT VECTOR RELATIVE TO INSTRUMENT:\n";
# 		print $x_r;
		
		# ===== INVERSE TRANSFORM TO GET THE SOLAR BEAM RELATIVE TO THE INSTRUMENT ========
		my $Tinverse = $T->inverse;
		my $aplat = $Tinverse * $SolarVector;
# 		print"Solar Vec Rel to Instrument:\n";
# 		print $aplat;
		
		# ===== FINALLY THE RELATIVE VECTOR COMPONENTS ===========
		
		my $sz_rel = acos ( $aplat->element(3,1) ) * R2D;
		my $saz_rel = atan2 ( $aplat->element(1,1), $aplat->element(2,1) ) * R2D;
		if ( $saz_rel < 0 ) { $saz_rel += 360 }
		
		return($sz_rel, $saz_rel);
	}
}


#======================================================
sub RotationTransform
#  ROTATION TRANSFORM
# CALLING:
# ($xtrue, $T) = RotationTransform($xrel, $pitch, $roll, $az);
#  
#  Computes the components of a vector in earth coordinates for any given
#  input
#   xrel is a 3x1 vector
#   pitch roll and azimuth are scalars
#  output
#   x = [3x1] vector
#  T = the rotation matrix to convert a vector in the platform frame
#     to a vector in the earth fram of reference.
#   x = Ta .* Tr .* Tp .* xrel = T .* xrel
#  
#  angles in degrees.
#   pitch positive for bow up
#  roll positive for port up
#  azimuth in standard copass coordinates
#  ===================================================================
# Matlab routine: reynolds 990710
# PERL version: 
# v101 060726 rmr -- adapt from MATLAB

# USE MODULE:  Math-MatrixReal-2.01 > Math::MatrixReal
# This can be down loaded from CPAN and installed in the usual way.
# The matrix transpose is used to determine the relative solar
# azimuth and zenith angle from the ship pitch-roll-heading.
# See MATLAB routine.
# use Math::MatrixReal
# use constant PI => 3.14159265358979;
# use constant D2R => PI / 180;
# use constant R2D => 180 / PI;
# use constant TWOPI => 2 * PI;
{
	my $xrel = shift();  #reference
	my $p = shift();
	my $r = shift();
	my $a = shift();
	my $x = shift();  #reference
	my $T = shift();  #reference
# 	printf"RotationTransform: pitch = %.2f, roll = %.2f, az = %.2f\n", $p, $r, $a;
#   
#   	print"XREL:\n";
# 	print $xrel;
	
	my ($Cp, $Cr, $Ca, $Sp, $Sr, $Sa);
	my ($Tp, $Tr, $Ta);
	
	$Cp = cos($p * D2R);   $Sp = sin($p * D2R);
	$Cr = cos($r * D2R);   $Sr = sin($r * D2R);
	$Ca = cos($a * D2R);   $Sa = sin($a * D2R);
	
	$Tp = Math::MatrixReal->new_from_rows([ [1,0,0], [0, $Cp, -$Sp], [0, $Sp, $Cp] ]);
	$Tr = Math::MatrixReal->new_from_rows([ [$Cr, 0, $Sr], [0,1,0], [-$Sr, 0, $Cr] ]);
	$Ta = Math::MatrixReal->new_from_rows([ [$Ca, $Sa, 0], [-$Sa, $Ca, 0], [0,0,1] ]);
# 	print "Tp\n";
# 	print $Tp;
# 	print "Tr\n";
# 	print $Tr;
# 	print "Ta\n";
# 	print $Ta;
	
	$T = $Ta * $Tr * $Tp;
# 	print "T:\n";
# 	print $T;
	
	$x = $T * $xrel;
#   	print"X:\n";
# 	print $x;
	return ($x, $T);
}



1;
