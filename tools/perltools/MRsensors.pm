package perltools::MRsensors;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&RL1005_TempCal &ysi44006 &ysi44006_circuit );

#USE IN A PROGRAM
# use lib "/Users/rmr/swmain/perl";
# use perltools::MRsensors;

use POSIX;

#*************************************************************/
sub ysi44006_circuit
# function [t, Rt, vt] = ysi44006_temperature(c, R, Vt, Vr, $missing)
# YSI44006 thermistor in a standard resistor divider circuit.
# The circuit is assumed to be a 12-bit ADC where the maximum
# count size is 4095.
#	THERMISTOR IS TO GROUND
# input
#  0 c = 2550; #  counts from the adc circuit
#  1 R = 10000; #  ohms reference resistor
#  2 Vt = 5.099; #  volts on the thermistor circuit 
#  3 Vr = 4.0928; #  volts for adc reference
#  4 $missing typ -999
# output
# vt = 2.549
# Rt = 9993.178
# t = 25.017
#
# edit 051219 rmr
#v101 060629 rmr -- start config control
#v02 110524 rmr -- streamline and check
#========================
{
	my ($c, $R, $Vt, $Vr, $missing) = @_; 
	#print "ysi44006_circuit: @_\n";
	if ( $c == $missing || $c < 1000 || $c >= 3500) { return ( $missing ) }
	
	# COMPUTE VOLTAGE FROM ADC COUNTS
	my $vt = $Vr * ($c / 4095);
	#printf(" vt = %.3f\n", $vt);
	
	# COMPUTE THERMISTOR RESISTANCE
	my $Rt = $R * $vt / ($Vt - $vt);
	#printf("Rt = %.3f\n", $Rt);
	
	my $t = ysi44006( $Rt );
	#printf("temp = %.3f\n", $t );
	return ( $t );
}


#*************************************************************/
sub ysi44006
#  input R in ohms,
# output
#   T = temperature in degC
#   Cal is the calibration table nx2
#v101 060629 rmr -- start config control
#************
{
	
	my $r = $_[0];
	my ($i, $i1, $i2);
	my ($r1, $r2);
	my $t;

	#  YSY THERMISTOR MODEL 44006 CALIBRATION CURVE 
	#  CALIBRATION VALUES
	@Cal44006 = (
	-60, 845900,
	-50, 441300,
	-40, 239800,
	-30, 135200,
	-20, 78910,
	-10, 47540,
	-9,	45270,
	-8,	43110,
	-7,	41070,
	-6,	39140,
	-5,	37310,
	-4,	35570,
	-3,	33930,
	-2,	32370,
	-1,	30890,
	0,	29490,
	1,	28150,
	2,	26890,
	3,	25690,
	4,	24550,
	5,  23460,
	6,	22430,
	7,	21450,
	8,	20520,
	9,	19630,
	10, 18790,	
	11, 17980,
	12, 17220,
	13, 16490,
	14, 15790,
	15, 15130,
	16, 14500,
	17, 13900,
	18, 13330,
	19, 12790,
	20, 12260,
	21, 11770,
	22, 11290,
	23, 10840,
	24, 10410,
	25, 10000,
	26, 9605,
	27, 9227,
	28, 8867,
	29, 8523,
	30, 8194,
	31, 7880,
	32, 7579,
	33, 7291,
	34, 7016,
	35, 6752,
	36, 6500,
	37, 6258,
	38, 6026,
	39, 5805,
	40, 5592,
	41, 5389,
	42,	5193,
	43,	5006,
	44,	4827,
	45, 4655,
	46,	4489,
	47,	4331,
	48,	4179,
	49,	4033,
	50, 3893,
	55, 3270,
	60, 2760,
	65, 2339,
	70, 1990,
	75, 1700,
	80, 1458,
	85, 1255,
	90, 1084,
	95, 939.3,
	100, 816.8,
	110, 623.5,
	120, 481.8	);
	#print "Define Cal44006 thermistor array.\n";
	
	for ( $i=0; $i <= $#Cal44006; $i+=2 ) 
	{
		push ( @Tcal44006, $Cal44006[$i] );
		push ( @Rcal44006, $Cal44006[$i+1] );
	}
	
	#===========================
	# INTERPOLATE FOR TEMPERATURE BASED ON RESISTANCE
	# ==========================
	$i1 = 0; 
	$i2 = $#Rcal44006;
	$r1 = $Rcal44006[$i1];
	$r2 = $Rcal44006[$i2];
	# SEARCH FOR THE INTERPOLATION POINTS
	while ( 1 )
	{
		$i = int ( ($i1 + $i2 ) / 2 );
		if ( $r < $Rcal44006[$i] && $r < $Rcal44006[$i+1] ) { $i1 = $i; }
		elsif ( $r > $Rcal44006[$i] && $r > $Rcal44006[$i+1] ) { $i2 = $i; }
		else { last; }
	}	
	$t = $Tcal44006[$i] + ( $r - $Rcal44006[$i] ) * ( $Tcal44006[$i+1] - $Tcal44006[$i] ) / ( $Rcal44006[$i+1] - $Rcal44006[$i] );
	return ( $t );
}


#*************************************************************/
sub RL1005_TempCal
#function [t, Rt, vt] = RL1005_temperature(c, R, Vt, Vr, $type, $missing)
#Thermometrics RL1005 thermistor in a resistor divider circuit.
#input
# 0  c = 2550; % counts from the adc circuit
# 1  R = 10000; % ohms reference resistor
# 2  Vt = 5.099; % volts on the thermistor circuit 
# 3  Vr = 4.0928; % volts for adc reference
# 4  type = 0; #  type=0 therm to Vref.  =1, therm to ground
# 5  missing is NaN value, typ -999
#output
# 0 Temp in degC
#	EXAMPLE
#vt = 2.5486
#Rt = 10007
#t = 24.9847
#
#v101 060629 rmr -- start config control
#***************
{
	#print "RL1005_TempCal: @_\n";
	my $missing = $_[5];
	if ( $_[0] < 0 || $_[0] > 4000) {return ($missing) }
	
	my $vt = $_[3] * ($_[0] / 4095);
	my $Rt;

	# COMPUTE THERMISTOR RESISTANCE
	if ( $_[4] == 0 ) { $Rt = $_[1] * ($_[2] - $vt) / $vt; }
	else {	$Rt = $_[1] * $vt / ($_[2] - $vt); }	
	
	# RL1005 STEINHART-HART COEFS FROM THERMOMETRICS DATA SHEET
	#    MATERIAL TYPE: D10.3
	# 	ratio Rt / R25
	my $r = $Rt / 10000;
	# log of the value
	my $lnr = log($r);
	# coefficients
	my $a = 3.3540172e-3;
	my $b=2.5027462e-4;
	my $c=2.4300527e-6;
	my $d=-7.2909526e-8;
	# compute temperature
	$t = $a + $b * $lnr + $c * $lnr * $lnr +  $d * $lnr * $lnr * $lnr;
	$t = 1 / $t - 273.15;
	
	return ($t );
}

#==================================================================
sub gprmc
#  ($dtgps, $lat, $lon, $sog, $cog, $var) = gprmc($sentence, $missing);
# edit 100221
#
# $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
#         1    2    3    4    5     6    7    8      9     10  11 12
# 
# 
#       1   220516     Time Stamp
#       2   A          validity - A-ok, V-invalid
#       3   5133.82    current Latitude
#       4   N          North/South
#       5   00042.24   current Longitude
#       6   W          East/West
#       7   173.8      Speed in knots
#       8   231.8      True course
#       9   130694     Date Stamp
#       10  004.2      Variation
#       11  W          East/West
#       12  *70        checksum
# 
# 
# eg4. for NMEA 0183 version 3.00 active the Mode indicator field is added
#      $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,m*hh
# Field #
# 1    = UTC time of fix
# 2    = Data status (A=Valid position, V=navigation receiver warning)
# 3    = Latitude of fix
# 4    = N or S of longitude
# 5    = Longitude of fix
# 6    = E or W of longitude
# 7    = sog, Speed over ground in knots
# 8    = cog, Track made good in degrees True
# 9    = UTC date of fix
# 10   = Magnetic variation degrees (Easterly var. subtracts from true course)
# 11   = E or W of magnetic variation
# 12   = Mode indicator, (A=Autonomous, D=Differential, E=Estimated, N=Data not valid)
# 13   = Checksum

#$GPRMC,040302.663,A,3939.7,N,10506.6,W,0.27,358.86,200804,018.1,E*1A
#   0     1        2   3    4   5     6   7   8      9      10   11
#0. Header, 
#1. Satellite-derived time.
# GPS devices are able to calculate the current date and time using GPS 
# satellites (and not the computer's own clock, making it useful for 
# synchronization). This word stores the current time, in UTC, in a compressed 
# form "HHMMSS.XXX," where HH represents hours, MM represents minutes, SS 
# represents seconds, and XXX represents milliseconds. The above value 
# represents 04:03:02.663 AM UTC.
#2.Satellite fix status.
# When the signals of at least three GPS satellites become stable, the device 
# can use the signals to calculate the current location. The device is said to be 
# "fixed" when calculations of the current location are taking place. Similarly, 
# the phrases "obtaining a fix" or "losing a fix" speak of situations where three 
# signals become stable or obscured, respectively.
# 
# A value of "A" (for "active") indicates that a fix is currently obtained, 
# whereas a value of "V" (for "inValid") indicates that a fix is not obtained.
#3. Latitude Decimal Degrees
# The latitude represents the current distance north or south of the equator. 
# This word is in the format "HHMM.M" where HH represents hours and MM.M represents 
# minutes. A comma is implied after the second character. This value is used in 
# conjunction with the longitude to mark a specific point on Earth's surface. 
# This sentence says that the current latitude is "39°39.7'N".
#4. Latitude Hemisphere
# This word indicates if the latitude is measuring a distance north or south 
# of the equator. A value of "N" indicates north and "S" indicates south. This 
# sentence says that the current latitude is "39°39.7'N".
#5. Longitude Decimal Degrees
# The longitude represents the current distance east or west of the Prime 
# Meridian. This word is in the format "HHHMM.M" where HHH represents hours and 
# MM.M represents minutes. A comma is implied after the third character. This value 
# is used in conjunction with the latitude to mark a specific point on Earth's 
# surface. This sentence says that the current longitude is "105°06.6'W".
#6. Longitude Hemisphere
# This word indicates if the longitude is measuring a distance east or west 
# of the Prime Meridian. A value of "E" indicates east and "W" indicates west. 
# This sentence says that the current longitude is "105°06.6'W"
#7. SOG, Speed
# This word indicates the current rate of travel over land, measured in knots.
#8. COG, Bearing.
# This word indicates the current direction of travel over, measured as 
# an "azimuth." An azimuth is a horizontal angle around the horizon measure in degrees 
# between 0 and 360, where 0 represents north, 90 represents east, 180 represents south, 
# and 270 represents west. This word indicates that the direction of travel is 
# 358.86°, or close to north. 
#9. UTC Date
# GPS devices maintain their own date and time calculated from GPS satellite signals. 
# This makes GPS devices useful for clock synchronization since the date and time are 
# independent of the local machine's internal clock. This word contains two-digit 
# numbers for days, followed by months and years. In the example above, the date is 
# August (08) 20th (20), 2004 (04). The two-digit year is added to 2000 to make a 
# full year value.
#10. variation, Magnetic variation
#11. sign variation, E or W  magnetic variation
{
	my ($dtgps, $lat, $lon, $sog, $cog, $var);
	my $str = shift();
	my $nan = shift();
	my ($d1, $d2);
	my @dat;
	my @d;
	my ($yy,$MM,$dd,$hh,$mm,$ss);
	
	#print "input sentence = $str\n";
	@dat = split(/[,*]/, $str);							# parse the data record
	#$i=0; for (@dat) { printf "%d %s\n",$i++, $_  } #test
	#==============================
	# 0 $GPRMC			header
	# 1 190824			hhmmss
	# 2 A				A=good, V=bad
	# 3 4736.2032		ddmm.mmmm
	# 4 N				N+, S-
	# 5 12217.2883		dddmm.mmmm
	# 6 W				E+, W-
	# 7 000.1			sog kts
	# 8 209.1			cog degT
	# 9 210210			ddMMyy
	# 10 018.1			var
	# 11 E				E+, W- degT = degM + var.
	# 12 62				checksum (ignore)
	# ============================
	$dtgps = $lat = $lon = $sog = $cog = $var = $nan; # start with all missing.
	
	# CHECK VALIDITY
	if ( $dat[2] eq 'A' ) {
		# GMT
		$hh = substr($dat[1],0,2);
		$mm = substr($dat[1],2,2);
		$ss = substr($dat[1],4,2);
		$dd = substr($dat[9],0,2);
		$MM = substr($dat[9],2,2);
		$yy = 2000 + substr($dat[9],4,2);
		$dtgps = datesec($yy,$MM,$dd,$hh,$mm,$ss);
		#printf "gps time = %s\n", dtstr($dtgps);
		
		# latitude
		@d = split(/[.]/,$dat[3]);
		$d1 = substr($d[0], 0, length($d[0]) - 2);
		$d2 = substr($d[0],-2,2);
		$d2 = $d2.'.'.$d[1];
		$lat = $d1 + $d2/60;
		if ($dat[4] =~ /S/i) {$lat = -$lat}
		# longitude
		@d = split(/[.]/,$dat[5]);
		$d1 = substr($d[0], 0, length($d[0]) - 2);
		$d2 = substr($d[0],-2,2);
		$d2 = $d2.'.'.$d[1];
		$lon = $d1 + $d2/60;
		if ($dat[6] =~ /W/i) {$lon = -$lon}
		
		# speed over ground  CONVERT TO M/S
		$sog = $dat[7] * 0.51444445;
		
		# COURSE
		$cog = $dat[8];
		
		# variation
		$var = $dat[10];
		if ( $dat[11] =~ /W/i) { $var = -$var }
		
		#print "lat, lon, sog, cog, var = $lat, $lon, $sog, $cog, $var\n";
		
		return ($dtgps, $lat, $lon, $sog, $cog, $var);
	}
}


1;
