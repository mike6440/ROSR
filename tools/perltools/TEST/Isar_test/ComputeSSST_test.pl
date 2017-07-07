#!/usr/bin/perl -w

use lib "/opt/local/lib/perl5/site_perl/5.12.3";
use lib "/Users/rmr/swmain/perl";
use perltools::Isar;
use perltools::MRtime;
use IO::File;


my $ktfile= "/Users/rmr/swmain/apps/isardaq4/kt15/kt15_filter_15855339.dat";
#my $ktfile= "/Users/rmr/swmain/apps/isardaq4/kt15/kt15_filter_15854832.dat";
## TEST
#    DATE                        LAT        LON      SSST     CORR     BB1      BB2      KT1     KT2    KTSEA  KTSKY E-SEA    PITCH   PITCH_STD  ROLL ROLL_STD AZ   VAR   ORG  ORG_MIN ORG_MAX  SW1_ST END CHG   SW2_ST END CHG
#                                deg        deg      degC     DEGC     degC     degC     mv      mv      mv     mv             deg       deg     deg     deg   deg  deg   mV     mV     mV 
#2011-03-31 (090) 23:20:00,  20.76258, 144.18943,  27.344,   0.651,  23.464,  41.137,  679.9,  778.1,  698.6,  630.7,0.98667,    0.1,    0.5,    0.7,    1.1, 157,   1,  53.1,  51.6,  56.0,    1, 1, 0,    1, 1, 0
#$T1, $T2, $kt1, $kt2, $ktsky, $ktsea, $pointangle, $pitch, $roll, $e0, $e_bb, $Acorr, $CalOffset, $kv


$tmpfile = "tmp.txt";
$TMP = new IO::File(">$tmpfile");
printf $TMP "OPEN %s\n", dtstr(now());
print "Open verbal file: $tmpfile\n";


#================================
# MAKE THE PLANCK TABLES
#================================
print"Make Planck tables\n";
my ($ttr, $rtr) = MakeRadTable_planck($ktfile, 1, $TMP);
my $kv=1;

		# $T1 = black body 1, ambient, temperature, degC
		# $T2 = heated BB temp, degC
		# $kt1, $kt2, $ktsea, $ktsky = kt15 readings for the different pointing angles, adc counts or mV
		# $pointangle = the pointing angle, relative to the isar, for the sea view, deg. Typ 125 or 135 deg.
		# $pitch = nose up tilt angle. (connectors facing the bow).
		# $roll = port side: port up tilt angle.  stbd side: port down tilt angle.
		# $e0 = nominal emissivity value
		# $e_bb = estimated emissivity of the black bodies, usually set to 1.
		# $Acorr = calibration parameter, multiplier of the interpolation slope. Typ = 1 +/- 0.01
		# $CalOffset = final sst adjustment, deg C. Typ +/-0.1.
		# $kv = 0 or 1 for nonverbal or verbal. Set to zero during operation.
		# $missing = value for bad data, usually = -999
		# $ttr = reference to the planck table temperature, from the MakeRadTable_planck() function.
		# $rtr = ditto for radiance.
		# $fhTMP = IO::File handle for the TMP file.
	
# ksky 323.5
# HIGH LATITUDES
#2011-04-23 (113) 14:00:00,  47.65575,-125.20896,   9.480,  -0.320,   8.789,  28.131,  530.6,  588.3,  529.6,  323.5,0.98677,   -0.0,    0.2,    0.5,    0.5, 274, -19,  47.9,  47.8,  48.1,    1, 1, 0,    1, 1, 0
#
@xx1=(8.789, 28.131, 530.6, 588.3, 529.6, 500,   135, 0.0, 0.5, 0.98667, 1.0, 1.0175, 0.060, $kv, -999, $ttr, $rtr, $TMP);
#
# RESULTS:
# Open verbal file: tmp.txt
# Make Planck tables
# test ComputeSSST: 8.789 28.131 530.6 588.3 529.6 500 135 0 0.5 0.98667 1 1.0175 0.06 1 -999 ARRAY(0x898690) ARRAY(0x8986d0) IO::File=GLOB(0x8aca60)
# test GetEmis() input angle = 134.50
# Tskin = 8.622, Tcorrection = 0.555, Tuncorrected = 9.177


# MID LATITUDES
#2011-03-12 (071) 01:50:00,  22.37686,-171.30313,  22.103,   0.111,  33.637,  51.910,  797.6,  897.9,  739.4,  514.2,0.98739,    0.1,    0.6,   -0.5,    1.3, 293,  -9,  51.1,  50.5,  51.7,    1, 1, 0,    1, 1, 0
#
@xx2=(33.637, 51.910, 797.6, 897.9, 739.4, 514.2, 135, 0.1,-0.5, 0.98667, 1.0, 1.0175, 0.060, $kv, -999, $ttr, $rtr, $TMP);
#
# RESULTS
# Open verbal file: tmp.txt
# Make Planck tables
# test ComputeSSST: 33.637 51.91 797.6 897.9 739.4 514.2 135 0.1 -0.5 0.98667 1 1.0175 0.06 1 -999 ARRAY(0x898690) ARRAY(0x8986d0) IO::File=GLOB(0x8acb80)
# test GetEmis() input angle = 135.50
# Tskin = 22.084, Tcorrection = 0.098, Tuncorrected = 22.182


	my ($tsk, $tcor, $tuncor) = ComputeSSST(@xx2);
	printf"Tskin = %.3f, Tcorrection = %.3f, Tuncorrected = %.3f\n", $tsk, $tcor, $tuncor;
	die;


exit;

