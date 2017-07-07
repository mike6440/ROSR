package perltools::Rosr;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&GetRadFromTemp &GetTempFromRad &GetEmis &MakeRadTable_planck &trapz &ComputeSSST &ComputeTarget);

#USE IN A PROGRAM
# use lib "/Users/rmr/swmain/perl";
# use perltools::Isar;
# my $dt = perltools::MRtime::now();
# printf"%s\n", dtstr($dt);

use lib $ENV{MYLIB};
use perltools::MRstatistics;
use constant Tabs => 273.15;
use constant PI => 3.14159265359;
use POSIX;

#=================================================================#
sub GetEmis
# COMPUTE EMISSIVITY OF THE SEA SURFACE FROM A GIVEN 
# VIEWING ANGLE.  See email from Donlon 040313
# Call: emis = GetEmis($viewangle, $missing)
#
#INPUT
# $viewangle = the isar drum angle toward the sea. From 90-180 deg.
#OUTPUT
# emissivity: viewangle from 90-120 emis=$missing, from 120-140 deg, emai is calculated, from 140-180 deg, emis = 0.99
#
#v2 110328 rmr -- include angles < 40 deg
#v3 110506 rmr -- moved to perl module Isar.pm
#v4 140329 rmr -- added safeguards to ComputeSSST

#=================================================================#
{
	my ($vsea, $missing) = @_;
	#printf "test GetEmis() input angle = %.2f\n", $vsea;
	
	my ($i);
	my @va= [];
	my @esea = [];
	my $e_sea;
	my ($e1, $e2, $a1, $a2);
	
	## NEAR VERTICAL POINTING
	if ( $vsea > 140 ) { $e_sea = 0.99 }
	## NEAR HORIZONTAL, EMIS IS NOT DEFINED
	elsif ( $vsea <= 120 ) { $e_sea = $missing }	
	## IN THE RANGE 40-60 DEG FROM NADIR
	else {
		# SEA EMISSIVITY BASED ON VIEW ANGLE
		# donlon email 040317
		my @a = (40,  0.9893371,
			41,  0.9889863,
			42,  0.9885924,
			43,  0.9881502,
			44,  0.9876541,
			45,  0.9870975,
			46,  0.9864734,
			47,  0.9857735,
			48,  0.9849886,
			49,  0.9841085,
			50,  0.9831214,
			51,  0.9820145,
			52,  0.9807728,
			53,  0.9793796,
			54,  0.9778162,
			55,  0.9760611,
			56,  0.9740904,
			57,  0.9718768,
			58,  0.9693894,
			59,  0.9665933,
			60,  0.9634488
		);
		# FILL THE TABLE ARRAYS
		for ( $i=0; $i <= $#a; $i+=2 )
		{
			push ( @va, 180 - $a[$i]);   # $va is the isar pointing angle is (180 - nadirangle) =[120-140] deg.
			push ( @esea, $a[$i+1]);
		}
		
		# --- INTERPOLATE TO SEA ANGLE ---
		for ( $i=0; $i<$#va; $i++ ) 
		{
			#printf"test %d: %.1f,  %.1f\n", $i, $va[$i], $vsea;
			if ( $va[$i] <= $vsea ) { 
				#$e_sea = $esea[$i];
				$a2 = $va[$i];    $a1 = $va[$i-1];
				$e2 = $esea[$i];  $e1 = $esea[$i-1];
				$e_sea = $e1 + ($vsea - $a1) * ($e2 - $e1) / ($a2 - $a1);
				#print "test e1=$e1, e2=$e2, a1=$a1, a2=$a2, vsea=$vsea\n";
				last;
			}
		}
	}
	return($e_sea);
}


#=========================================================================
sub GetRadFromTemp
# GET RADIANCE FROM THE TABLE GIVEN T
#
#CALL::		$rad = GetRadFromTemp(\@Ttable, \@Rtable, $temp);
#where
# \@Ttable is a reference to the temperature, degC, tables from MakeRadTable_planck
# \@Rtable ditto for radiation.
# $temp is the input temperature, degC
#
#v3 110506 rmr -- moved to perl module Isar.pm
{
	my @t = @{$_[0]};
	my @r = @{$_[1]};
	
	my $i1 = 0; 
	my $i2 = $#t;
	my $x = $_[2];
	my ($y);
	
	# SEARCH FOR THE INTERPOLATION POINTS
	while ( 1 )
	{
		$i = int ( ($i1 + $i2 ) / 2 );
		if ( $x < $t[$i] && $x < $t[$i+1] ) { $i2 = $i; }
		elsif ( $x > $t[$i] && $x > $t[$i+1] ) { $i1 = $i; }
		else { last; }
	}
	$y = $r[$i] + ( $x - $t[$i] ) * ( $r[$i+1] - $r[$i] ) / ( $t[$i+1] - $t[$i] );
	return ( $y );
}

#=========================================================================
sub GetTempFromRad
# GET T FROM THE TABLE GIVEN RADIANCE
#
# CALL:   $temp = GetTempFromRad(\@t, \@r, $rad);
#where
# \@Ttable is a reference to the temperature, degC, tables from MakeRadTable_planck
# \@Rtable ditto for radiation.
# $is the input radiance, normalized.
#
#v3 110506 rmr -- moved to perl module Isar.pm
{

	my @t = @{$_[0]};
	my @r = @{$_[1]};
	
	my $i1 = 0; 
	my $i2 = $#r;
	my $x = $_[2];
	my	$y;
	
	
	# SEARCH FOR THE INTERPOLATION POINTS
	while ( 1 )
	{
		$i = int ( ($i1 + $i2 ) / 2 );
		#printf("%d, %.3f, %.3f, %.3f     %.4f,  %.4f\n", $i, $x, $t[$i], $t[$i+1], $r[$i], $r[$i+1]);
		
		if ( $x < $r[$i] && $x < $r[$i+1] ) { $i2 = $i; }
		elsif ( $x > $r[$i] && $x > $r[$i+1] ) { $i1 = $i; }
		else { last; }
	}
	$y = $t[$i] + ( $x - $r[$i] ) * ( $t[$i+1] - $t[$i] ) / ( $r[$i+1] - $r[$i] );
	return ( $y );
}

#=========================================================================
sub MakeRadTable_planck

# CALL::   ($ttr, $rtr) = MakeRadTable_planck( $kt15file )

# MakeRadTable_planck($filterfile )
#INPUT
#   $filterfile points to the file.
#
# OUTPUT:
# $ttr = table temperature array.
# $rtr = table radiance array
#
#  globals ==> @Ttable, @Rtable
# Globals: Tabs
#
# Parameters: $kt15sn, $filterpath
# v101 060701 rmr – config control starts, filter file vector as input.
#v3 110506 rmr -- moved to perl module Isar.pm
{
	$fnm = shift();  # v101
	
	@Ttable = ();  # output temperature vector
	@Rtable = (); # output normalized radiance vector
	
	#==================
	# GET THE FILTER FUNCTION
	#=================
	my @lambda = ();
	my @resp = ();
	my $i;
	my $str;
	# -- OPEN THE FILTER FILE ---
	printf"Filter file = $fnm\n";
	open(FILTER, "< $fnm") or die "Open filter file fails\n";
	#printf "Filter file open: %s\n", $fnm;
	# --- READ EACH LINE, skip header lines ---
	# Make a table of @lambda and @response
	my $iline = 0;
	while ( <FILTER> )
	{
		chomp($str = $_);
		#if ( $iline < 2 ) { printf("%6d: %s\n", $iline, $str); }
		if ( $iline >= 7 )
		{
			@w = split;
			push ( @lambda, $w[0]);  # test
			push ( @resp, $w[1]  * $w[0]/10);
		}
		$iline++;
	}
	
	# --- ADD ZERO TERMS TO THE END -- IT HELPS ---
	push ( @resp, 0 );   # zeros at the end
	push ( @lambda, $lambda[$#lambda]+0.1 );
	unshift ( @resp, 0 );   # zeros at the beginning
	unshift ( @lambda, $lambda[0]-0.1 );
	#$ii = 0;
	#foreach(@lambda){print"$ii        $lambda[$ii]        $resp[$ii]\n"; $ii++}
	
	# --- NORMALIZE BY MAXIMUM VALUE ---
	my $respmax = -1e99;
	foreach (@resp) { if($_ > $respmax){ $respmax = $_ }}
	foreach  (0..$#resp) { $resp[$_] /= $respmax; }
	
	# --- COMPUTE THE INTEGRATED RADIANCE FOR EACH TEMPERATURE ---
	my @r;
	my $x;
	my $t = -80.0;
	my $R0;
	
	while ( $t <= 60.0 ) 
	{
		push ( @Ttable, $t );  # temperature column
		@r = ();				# clear r vector;
		#-- bb radiances for all lambda and temperature t ----
		foreach $i (0..$#lambda) 
		{ 
			# --- vector of filtered bb radiance ---
			push ( @r, $resp[$i] * planck( $t, $lambda[$i] ) );  # black body radiance for t,lambda 
		}
		# -- weighted area --
		$x = trapz(\@lambda, \@r, 0,0) / trapz(\@lambda, \@resp, 0,0);
		if ( $t > -0.00001 && $t < 0.00001 ) { $R0 = $x; }
		push ( @Rtable, $x );
		# -- increment temperature --
		$t+=0.1;
	}
	# --- Normalize ---
	foreach $i (0..$#Ttable)
	{
		$Rtable[$i] /= $R0;
	}
	return ( \@Ttable, \@Rtable );
}

#*************************************************************/
sub MakeRadTable 
# MakeRadTable;  uses Nightingale polynomial to create a 
#  table of T and rnormalized  @Ttable, @Rtable
# % LOAD THE BRIGHTNESS TEMP VS RADIANCE TABLE
#  0306 -- POLY FIT FROM TIM
# FUNCTION KT1585_4832_T2R, T
# ;
# ; VERSION
# ;     TJN  04-JUN-2003  Original
# ;
# ; DESCRIPTION
# ;     Converts KT1585 (SN3832) brightness temperatures in Kelvin to  <--- TYPO?  4832
# ;     radiances in units of fractional B(273.16K). Accurate to
# ;     approximately +/-0.5mK from 250K to 340K.
# ;
# ; MANDATORY PARAMETER
# ;     T       KT15 brightness temperature scalar or array in Kelvin.
# ;
# ; RETURNS
# ;     KT15 radiance scalar or array, in units of fractional B(273.16K).
# ;
# ; Code starts here --------------------------------------------------------
# ;
#   A = [-23.2332340,  66.124043d0, -82.426267d0,  57.659334d0, -21.432500d0,  3.3086280d0]
#   RETURN, EXP(POLY(T / 273.16d0, A))
# END
# 	@a = (-22.925646, 65.196703, -81.215855, 56.792568, -21.105313, 3.2575460);
{
	@Ttable = ();
	@Rtable = ();
	# kt15 sn: 4801
#	my @a = (-22.925646, 65.196703, -81.215855, 56.792568, -21.105313, 3.2575460);
	# kt15 sn: 4832
	my @a = (-23.2332340,  66.124043, -82.426267,  57.659334, -21.432500,  3.3086280);
	$header = $header. "Make T-R table using Nightingale polynomials: @a\n";
	my ($t, $r);
	$t = -80;
	while ( $t <= 60 )
	{ 
		$r = exp ( polyval(@a, ($t+$Tabs)/$Tabs) );
		push ( @Rtable, $r);
		push ( @Ttable, $t);
		$t += 0.1;
	}
	return ( \@Ttable, \@Rtable );
}

#*************************************************************/
sub trapz
# area = trapz( \@x, \@y, a, b);
# integrate under a given digital function
# input
#  \@x = reference to x vector 
#  \@y = reference to the y function.
#  $a = starting index, if = 0 start at the beginning
#  $b = ending index, if = 0 go to the end
# return the integral
# tested: 041025
#
#v3 110506 rmr -- moved to perl module Isar.pm
{
	my $a = 0;
	my $xr = $_[0];
	my $yr = $_[1];
	my $i;
# 	$i (0...$#x) { printf("%3d, %10.3f, %10.3f\n",$i, $$xr[$i], $$yr[$i]); }
	my $x1 = $_[2];   my $x2 = $_[3];
	if ( $x1 < 0 ) { $x1 = 0 }
	if ( $x2 > $#$xr || $x2 == 0 ) { $x2 = $#$xr }
# 	printf(" Integrate from index %d to %d\n", $x1, $x2);
	
	for ( $i = $x1; $i <= $x2-1; $i++ ) 
	{
		$a +=  ( $$xr[$i+1] - $$xr[$i] ) * ( $$yr[$i] + $$yr[$i+1] ) / 2;
# 		printf("%3d, x:  %.4f,  %.4f, y: %.4f,  %.4f, a =  %.4f\n", 
# 			$i, $$xr[$i], $$xr[$i+1], $$yr[$i], $$yr[$i+1], $a);
	}
	return($a);
}

#=====================================================
sub planck
# function B = plank(t,lambda),
# % where t is in degC or K
# % and lambda is in microns.
# global: Tabs
#constants  PI => 3.14159265359
# % Plank's law can be written:
# % \begin{equation}
# % B(\lambda; T)= \frac{c1} {\lambda^5(exp(c_2/\lambda T)-1)}
# % \end{equation}
# % \noindent where
# % B units are W/m^2 assuming an isotropic radiation where
# %  B = πL = pi * L
# % $T$ is expressed in \degK, 
# % $	c_1 = 3.74\times10^{-16}$ w\,m$^{-2}$, and 	$c_2 = 1.44\times 10^{-2}$ m\,K.
# % Actually, c1 = 2hc^2 where h = planck's constant and c = speed of light.
# % and c2 = hc/k where k = Boltzmann constant
# % c_0 = 2.99792458e+8 m/s,  h = 6.626076e-34 Js,  k = 1.380658e-23 J/K
# TEST
#  T      LAMBDA       Rplanck
# 20		10		27.7805
# 40		10		38.1179
# 60		10		50.3974
# -20		12		13.2758
# -20		10		12.7362
#
#v3 110506 rmr -- moved to perl module Isar.pm
{
	
	my $t = $_[0];
	my $lambda = $_[1];
	my $T = $t;
	
	# CHECK IF TEMPERATURE IS IN DEGC OR K
	if ( $t < 200 )	{ $T = $t + Tabs }
	
	# c = 2.99792458e+8;  h = 6.626076e-34; k = 1.380658e-23;
	my $c = 2.99793e+8;  
	my $h = 6.6262e-34; 
	my $k = 1.380e-23;  # Liou p.11 and Wallace & Hobbs, p287
	
	# CONSTANTS
	my $c1 = 2 * $h * $c * $c;
	my $c2 = $h * $c / $k;
	
	# B(\lambda; T)= \frac{c1} {\lambda^5 (exp(c_2 / \lambda T)-1)}
	my $lx = $lambda / 1e6;
	my $a1 = ($lx ** 5) * ( exp ( $c2 / ( $lx * $T ) ) -1 );
	my $L = 1e-6 *  $c1 / $a1;  # per micron wavelength
	
	my $B = $L * PI;
	return ($B);	
}


#*************************************************************/
sub ComputeSSST
#
#CALL:
#    ($T_skin, $T_corr, $T_uncorr, $e_sea) = ComputeSSST($T1, $T2, $kt1, $kt2, $ktsky, $ktsea, $pointangle, 
#		$pitch, $roll, $e0, $e_bb, $Acorr, $CalOffset, $kv, $missing, $ttr, $rtr);
#where
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
# example
#		use lib "/Users/rmr/swmain/perl";
#		use perltools::Isar;
#		my $ktfile= "/Users/rmr/swmain/apps/isardaq4/kt15/kt15_filter_15854832.dat";
#		my ($ttr, $rtr) = MakeRadTable_planck($ktfile);
# 		@xx1=(8.789, 28.131, 530.6, 588.3, 529.6, 323.5, 135, 0.0, 0.5, 0.98667, 1.0, 1.0175, 0.060, 1, -999, $ttr, $rtr, $);
# 		$ss = ComputeSSST(@xx1);
# gives Tskin=9.508, Tcorr=-0.3312, Tuncorr=9.1770  
#v3 110506 rmr -- moved to perl module Isar.pm
{
	my ($bb1t, $bb2t, $kt1, $kt2, $ktsea, $ktsky, $pointangle, $pitch, $roll, $e_sea_0, $e_bb, $Acorr, $CalOffset, $kv, $missing, $ttr, $rtr) = @_;
	my ($e_sea, $S_1, $S_2, $S_k, $S_1v, $S_2v, $S_upwelling, $Ad, $S_dv, $Au, $S_uv, $S_skin, $T_uncorr, $T_skin);
	#$kv=0;
	if($kv==1){print "test ComputeSSST: @_\n";}
	if ( $ktsky != $missing && $ktsea != $missing &&
	$kt1 != $missing && $kt2 != $missing && $kt2 >0 && $kt1 > 0 && $kt2 > $kt1 &&
	$bb1t != $missing && $bb2t != $missing && abs($bb2t-$bb1t) > 5 ) {
		# v4.0 check if we have pitch and roll data.  For this first effort we will use
		# only roll.  A positive roll decreases the isar view angle.  i.e. if the view angle is 
		# 125 deg and the roll is +2 deg then the corrected view angle is 123 deg.  And the nadir
		# angle is 57 deg.
		if ( $pitch == $missing || $roll == $missing ) {
			$e_sea = $e_sea_0;
		} else {
			$e_sea = GetEmis( $pointangle - $roll );
			if ($kv == 1 ) {
				printf  "pitch = $pitch,  roll = $roll, ActualViewAngle = %.3f\n", $pointangle - $roll;
				print   "e_sea = $e_sea, e_sea_0 = $e_sea_0\n";
			}
		}
		
		#===================
		# BB RADIANCES
		#
		$S_1 = GetRadFromTemp ($ttr, $rtr, $bb1t);
		if ($kv == 1 ) { printf  "bb1t = %.3f,  S_1 = %.4e\n", $bb1t, $S_1}
		$S_2 = GetRadFromTemp($ttr, $rtr, $bb2t);
		if ($kv == 1 ) { printf  "bb2t = %.3f,  S_2 = %.4e\n", $bb2t, $S_2}
		$S_k = $S_1;  # planck radiance from the kt15 lens
		
		# VIEW IRRADIANCES DEPEND ON THE EMISSIVITIES OF THE BB'S AND SOURCE
		$S_1v = $e_bb * $S_1 - ( 1 - $e_bb ) * $S_k;
		$S_2v = $e_bb * $S_2 - ( 1 - $e_bb) *  $S_k;
		if ($kv == 1 ) { printf  "S_1 = %.3f,  S_2 = %.3f\n", $S_1v, $S_2v}

		#-------------------------------
		# --- FIELD EXPERIMENT WITH SKY CORRECTION
		#--------------------------------
		# ---DOWN VIEW RADIANCE---
		# Ad = (kd-k1) ./ (k2-k1);
		if ($kv == 1 ) { printf  "kt1= %.4f, kt2=%.4f, ktsea=%.4f, ktsky=%.4f\n", $kt1, $kt2, $ktsea, $ktsky}
		
		$Ad = ( $ktsea - $kt1 ) / ( $kt2 - $kt1 );
		if ($kv == 1 ) { printf  "Ad = %.4f\n", $Ad}
		
		# Correct for the irt beam spread
		$Ad = $Acorr * $Ad;
		if ($kv == 1 ) { printf  "Ad = %.4f\n", $Ad}
		
		# DOWN VIEW INCOMING IRRADIANCE BY INTERPOLATION
		my $S_dv = $S_1v + ($S_2v - $S_1v) * $Ad;
		if ($kv == 1 ) { printf  "S_dv = %.4f\n", $S_dv}

		# --- UP VIEW RADIANCE ---
		# interpolation constant
		my $Au = ( $ktsky - $kt1 ) / ( $kt2 - $kt1 );
		if ($kv == 1 ) { printf  "slope Au = %.4f,   ", $Au}
		$Au = $Acorr * $Au;
		if ($kv == 1 ) { printf  "corrected Au = %.4f\n", $Au}
		
		my $S_uv = $S_1v + ( $S_2v - $S_1v) * $Au;
		if ($kv == 1 ) { printf  "sky view radiance S_uv = %.4f\n", $S_uv}
		
		#======================
		# UPWELLING SKY IRRADIANCE
		#=======================
		$S_upwelling = $S_uv * ( 1 - $e_sea );
		if ($kv == 1 ) { printf  "S_upwelling = %.4f\n", $S_upwelling}
		
		#======================
		# SEA SURFACE RADIANCE
		# view radiance minus the upwelling
		#=====================
		$S_skin = ( $S_dv - $S_upwelling) / $e_sea;
		if ($kv == 1 ) { printf  "S_skin = %.4f\n", $S_skin}
		
		# ===================
		# COMPUTE SSST FROM THE TABLE
		#====================
		if ($kv == 1){printf  "Rad2Temp: S_dv=%.3f, e_sea=%.5f\n", $S_dv, $e_sea}
		$T_uncorr = $CalOffset + GetTempFromRad( $ttr, $rtr, $S_dv / $e_sea);  # without sky correction
		$T_skin = $CalOffset + GetTempFromRad($ttr, $rtr, $S_skin);
		
		$T_corr = $T_uncorr - $T_skin;   # the correction for sky reflection
		if($kv == 1){printf  "T_uncorr = %.3f, T_skin = %.3f  T_corrdiff = %.3f\n", $T_uncorr, $T_skin,  $T_corr}
	}
	else {
		$T_uncorr = $T_skin = $T_corr = $missing;
		$e_sea = 0;
	}
	return ($T_skin, $T_corr, $T_uncorr, $e_sea);
}

#*************************************************************/
sub ComputeTarget
#
#CALL:
#    $T_cal = ComputeTarget($T1, $T2, $kt1, $kt2, $kttarget, $ecal, $e_bb, $Acorr, $CalOffset, $kv, $missing, $ttr, $rtr);
#where
	# $T1 = black body 1, ambient, temperature, degC
	# $T2 = heated BB temp, degC
	# $kt1, $kt2, $kttarget = kt15 readings for the different pointing angles, adc counts or mV
	# $ecal = emissivity of the calibration target, usually set to 1.
	# $e_bb = estimated emissivity of the black bodies, usually set to 1.
	# $Acorr = calibration parameter, multiplier of the interpolation slope. Typ = 1 +/- 0.01
	# $CalOffset = final sst adjustment, deg C. Typ +/-0.1.
	# $kv = 0 or 1 for nonverbal or verbal. Set to zero during operation.
	# $missing = value for bad data, usually = -999
	# $ttr = reference to the planck table temperature, from the MakeRadTable_planck() function.
	# $rtr = ditto for radiance.
# example
#		use lib "/Users/rmr/swmain/perl";
#		use perltools::Isar;
#		my $ktfile= "/Users/rmr/swmain/apps/isardaq4/kt15/kt15_filter_15854832.dat";
#		my ($ttr, $rtr) = MakeRadTable_planck($ktfile);
# 		@xx1=(8.789, 28.131, 530.6, 588.3, 529.6, 0.0,0.5, 0.98667, 1.0, 1.0175, 0.060, $kv, -999, $ktfile, $ttr, $rtr);
# 		$ss = perltools::Isar::ComputeSSST(@xx1);
# gives Tskin=9.508, Tcorr=-0.3312, Tuncorr=9.1770  
#v3 110506 rmr -- moved to perl module Isar.pm
{
	my ($bb1t, $bb2t, $kt1, $kt2, $kttarget, $e_cal, $e_bb, $Acorr, $CalOffset, $kv, $missing, $ttr, $rtr) = @_;
	my ($S_1, $S_2, $S_k, $S_1v, $S_2v, $S_upwelling, $Ad, $S_dv, $Au, $S_uv, $S_skin, $T_uncorr, $T_skin);
	#print "test ComputeTarget: @_\n";
	#=======================
	# COMPUTE ONLY IF WE HAVE ALL THE DATA
	#======================
	if ( $kttarget != $missing && $kt1 != $missing && $kt2 != $missing &&
		$bb1t != $missing && $bb2t != $missing) {
		# BB RADIANCES
		#
		$S_1 = GetRadFromTemp ($ttr, $rtr, $bb1t);
		if ($kv == 1 ) { printf  ("bb1t = %.3f,  S_1 = %.4e\n", $bb1t, $S_1); }
		$S_2 = GetRadFromTemp($ttr, $rtr, $bb2t);
		if ($kv == 1 ) { printf  ("bb2t = %.3f,  S_2 = %.4e\n", $bb2t, $S_2); }
		$S_k = $S_1;  # planck radiance from the kt15 lens
		# VIEW IRRADIANCES DEPEND ON THE EMISSIVITIES OF THE BB'S AND SOURCE
		$S_1v = $e_bb * $S_1 - ( 1 - $e_bb ) * $S_k;
		$S_2v = $e_bb * $S_2 - ( 1 - $e_bb) *  $S_k;
		if ($kv == 1 ) { printf  ("S_1 = %.3f,  S_2 = %.3f\n", $S_1v, $S_2v); }
		
		# ---CALIBRATION VIEW RADIANCE---
		# diagnostic print if kv == 1
		if ($kv == 1 ) { printf  ("kt %.4f, %.4f, %.4f\n", $kt1, $kt2, $kttarget ); }
			
		# Ad = (kd-k1) ./ (k2-k1);
		$Ad = ( $kttarget - $kt1 ) / ( $kt2 - $kt1 );
		
		# Correct for the irt beam spread
		$Ad = $Acorr * $Ad;
		if ($kv == 1 ) { printf  ("Ad = %.4f\n", $Ad); }
		
		# CAL VIEW INCOMING IRRADIANCE BY INTERPOLATION
		my $S_cal = $S_1v + ($S_2v - $S_1v) * $Ad;
		if ($kv == 1 ) { printf  ("S_cal = %.4f\n", $S_cal); }
		
		# --- BACKWELLING IRRADIANCE FROM CAL BATH -----------
		$S_upwelling = $S_cal * ( 1 - $e_cal );
		if ($kv == 1 ) { printf  ("S_upwelling = %.4f\n", $S_upwelling); }
		
		# ---- CAL RADIANCE ------------------
		# ---- view radiance minus the upwelling ------
		$S_cal = ( $S_cal - $S_upwelling) / $e_cal;
		if ($kv == 1 ) { printf  ("S_cal = %.4f\n", $S_cal); }
		
		# ---- COMPUTE CALIBRATION BATH FROM THE TABLE --------
		$T_target = GetTempFromRad($ttr, $rtr, $S_cal) + $CalOffset;  
	} else {
		$T_target = $missing;
	}
	if($kv==1){printf  "T_target = %.3f\n",$T_target}
	return $T_target;
}



1;
