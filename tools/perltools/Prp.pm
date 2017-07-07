package perltools::Prp;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&ZeError &EdgeAndShadow &Tcal);

use lib $ENV{MYLIB};
use perltools::MRutilities;
use perltools::MRstatistics;

# use perltools::mrtime;

use constant PI => 3.14159265358979;
use constant R2D => 180 / PI;
use constant D2R => PI / 180 ;
use constant YES => 1;
use constant NO => 0;
use constant MISSING => -999;

#*************************************************************/
sub EdgeAndShadow
# This routine uses the 2-min sweep stats to estimate the two 
# edge values and to determine the best edge value to use.  It also estimtes
# the error in the final edge value.
# CALL: ($edge,$shadow) = EstimateEdge($index1, $index2, $index0, $sweep); 
#  Where $sweep is the complete string for the channel
# Consider bins index1 and index2 are the correct bins.
{
	my ($i);
	my $i1 = shift();  my $i2 = shift(); my $i0 = shift();
	my $str = shift();
	$str =~ s/^[ ]+//;  # remove leading spaces
	my @d = split(/[ ]+/,$str);
	#print"i1=$i1, i2=$i2, d = @d\n";
	my $e1 = $d[$i1];
	my $e2 = $d[$i2];
	my $sh = $d[$i0];
	#print"edge1 = $e1,   edge2 = $e2\n";

	return (($e1+$e2)/2, $sh);	
}



#======================================================
sub ZeError
# ZEERROR - compute the zenith error calibration based on az and ze angle
#          corr = ZeError(sz, saz, det)
# =======================================================================
# 
# input: 
#	fcal = full filename to the MFR head ze calibration file (e.g. 469.SOL)
#   sz = solar zenith angle -- RELATIVE TO THE PLATFORM NORMAL
#   saz = solar azimuth -- RELATIVE TO THE PLATFORM NORTH MARK
#	det = the detector number. (2-7)  det#1 is the unfiltered detector.
# 
# output:
#  corr = the calibration value from the MFRSR head calibration.
# 
# The calibration SOL file is read.
# This function reads this file and uses the input information to compute
# the calibration value.  
# 	The 'north' mark of the head corresponds to the 
# bracket direction (also the direction the arm points to on an normal
# MFRSR.  Hence, normally the motor points to the equator (S).
# 	The correction array is listed in two columns.  There are 181 rows 
# in the matrix corresponding to angles from 0 to 180.  
# 	Column 1 is correction for zenith angles from 0 (south horizon) to 
# 180 (north horizon).  Column 2 corresponds to angles from 0 (west horizon)
# to 180 (east horizon);
# 
# 
# reynolds 980126
#  modified by MJB 2/24/99
# v200 2/13/2001 uses new path and info file
# v201 3/01/2001 "
# v020 4/26/2002 added global drive_matlab
## MODIFIED FROM THE MATLAB ROUTINE V020
## v 101 060807 rmr -- modified for a04_MaleParams.pl
## v102 061019 rmr -- if ze > 90, return missing
# 
# ========================================================================
{
	my ($fcal, $ze, $saz, $det) = @_;
	my ($corr, $quad, $iz1, $iz2);
	my ($sn1, $sn2, $we1, $we2);
	my @isn1 = (1,1); # row, col
	my @isn2 = (1,1); # row, col
	my @iwe1 = (1,1); # row, col
	my @iwe2 = (1,1); # row, col
	my @linesn;
	my @linewe;
	my $delaz;
	my @wds;
# use constant MISSING => -999;
	if ( $ze < 0 || $saz < 0 || $det < 2 || $det > 7 ) { return MISSING }
	
	if ( $ze > 89 ) { return MISSING }
	
	if ( $saz < 0 ) { $saz += 360 }
	if ( sprintf("%.2f",$saz) eq '360' ) { $saz = 0 } 
	## OPEN THE ZE CAL FILE
	@linesn = FindLines( $fcal, "SN$det", 19); # read the next 19 lines after the SNx line
	@linewe = FindLines( $fcal, "WE$det", 19);
	#foreach(@linesn){print"test104 $_\n"}  foreach(@linewe){print"test104 $_\n"}
	 		## CHECK FOR EACH QUADRANT
	$corr = MISSING;
	$quad = int($saz/90) + 1;
	$iz1 = int($ze);  
	($iz2) = min ( $iz1+1, 90);  # # 1,2,...,90
	
	#print"QUADRANT: $quad\n";
	## QUADRANT 1
	# arc a spans lower zeang from n to e
	# arc b spans ize + 1 deg
	if ( $quad == 1 ) {
		if ( $iz1 == 0 ) { 
			@isn1 = (10,0);  @isn2 = (11,0);
			@iwe1 = (10,0);  @iwe2 = (11,0);
		} else {
			@isn1 = ( 11 + int(($iz1-1)/10), ($iz1-1)%10 ); 
			@iwe1 = ( 11 + int(($iz1-1)/10), ($iz1-1)%10 );
			@isn2 = ( 11 + int(($iz2-1)/10), ($iz2-1)%10 );
			@iwe2 = ( 11 + int(($iz2-1)/10), ($iz2-1)%10 );
		}
		## ARC 1
		@wds = split( / /, $linesn[$isn1[0]] );
		$a1 = $wds[ $isn1[1] ];
		@wds = split( / /, $linewe[$iwe1[0]] );
		$a2 = $wds[ $iwe1[1] ];
		## ARC 2
		@wds = split( / /, $linesn[ $isn2[0] ] );
		$b1 = $wds[ $isn2[1] ];
		@wds = split( / /, $linewe[ $iwe2[0] ] );
		$b2 = $wds[ $iwe2[1] ];
		$delaz = $saz;
	}
	elsif ($quad == 2) {
		if ( $iz1 == 0 ) { 
			@iwe1 = (10,0);  @iwe2 = (11,0);
			@isn1 = (10,0);  @isn2 = (9,9);
		} else {
			@isn1 = ( 9 - int(($iz1-1)/10), 9-($iz1-1)%10 ); 
			@iwe1 = ( 11 + int(($iz1-1)/10), ($iz1-1)%10 );
			@isn2 = ( 9 - int(($iz2-1)/10), 9-($iz2-1)%10 );
			@iwe2 = ( 11 + int(($iz2-1)/10), ($iz2-1)%10 );
		}
		## ARC 1
 		#print"test148: isn1 = (@isn1), isn2 = (@isn2)\niwe1 = (@iwe1), iwe2 = (@iwe2)\n";
		@wds = split( / /, $linewe[$iwe1[0]] );
		$a1 = $wds[ $iwe1[1] ];
		#print"test151  linesn = $linesn[$isn1[0]]\n";
		@wds = split( / /, $linesn[$isn1[0]] );
		$a2 = $wds[ $isn1[1] ];
		### ARC 2
		@wds = split( / /, $linewe[ $iwe2[0] ] );
		$b1 = $wds[ $iwe2[1] ];
		@wds = split( / /, $linesn[ $isn2[0] ] );
		$b2 = $wds[ $isn2[1] ];
		$delaz = $saz - 90;
	}
	elsif ($quad == 3 ) {
		if ( $iz1 == 0 ) { 
			@isn1 = @iwe1 = ( 10, 0 );
			@isn2 = @iwe2 = (9,9);
		} else {
			@isn1 = ( 9 - int(($iz1-1)/10), 9-($iz1-1)%10 ); 
			@iwe1 = ( 9 - int(($iz1-1)/10), 9-($iz1-1)%10 );
			@isn2 = ( 9 - int(($iz2-1)/10), 9-($iz2-1)%10 );
			@iwe2 = ( 9 - int(($iz2-1)/10), 9-($iz2-1)%10 );
		}
		## ARC 1
		@wds = split( / /, $linesn[$isn1[0]] );
		$a1 = $wds[ $isn1[1] ];
		@wds = split( / /, $linewe[$iwe1[0]] );
		$a2 = $wds[ $iwe1[1] ];
		
		@wds = split( / /, $linesn[ $isn2[0] ] );
		$b1 = $wds[ $isn2[1] ];
		@wds = split( / /, $linewe[ $iwe2[0] ] );
		$b2 = $wds[ $iwe2[1] ];
		$delaz = $saz - 180;
	}
	else {  #quad == 4
		if ( $iz1 == 0 ) { 
			@isn1 = @iwe1 = ( 10, 0 );
			@isn2 = @iwe2 = (9,0);
		} else {
			@isn1 = ( 11 + int(($iz1-1)/10), ($iz1-1)%10 ); 
			@iwe1 = ( 9 - int(($iz1-1)/10), 9-($iz1-1)%10 );
			@isn2 = ( 11 + int(($iz2-1)/10), ($iz2-1)%10 );
			@iwe2 = ( 9 - int(($iz2-1)/10), 9-($iz2-1)%10 );
		}
		## ARC 1
		@wds = split( / /, $linewe[$iwe1[0]] );
		$a1 = $wds[ $iwe1[1] ];
		@wds = split( / /, $linesn[$isn1[0]] );
		$a2 = $wds[ $isn1[1] ];
		
		@wds = split( / /, $linewe[ $iwe2[0] ] );
		$b1 = $wds[ $iwe2[1] ];
		@wds = split( / /, $linesn[ $isn2[0] ] );
		$b2 = $wds[ $isn2[1] ];
		$delaz = $saz - 270;
	}
	
	
	## INTERPOLATE AROUND THE ANNULUS OVER THE SEGMENT LIMITS.
	if ( defined($a1) && defined($a2) && defined($b1) && defined($b2) && defined($delaz) ) {
		my $k1 = $a1 + ($a2-$a1) * $delaz / 90;
		my $k2 = $b1 + ($b2-$b1) * $delaz / 90;
		$corr = $k1 + ($k2 - $k1) * ($ze - $iz1);
		return $corr;
	} else {
		printf"ZeError undefined error: ze,saz,quad: %.4f, %.4f, %d\n", $ze, $saz, $quad;
		printf"a1,a2,b1,b2,delaz: %.3f, %.3f, %.3f, %.3f, %.3f\n", $a1,$a2,$b1,$b2,$delaz;
		if (!defined($delaz) ) {print"delaz is undefined\n"}
		printf"--- fcal, ze,saz,quad: %s, %.4f, %.4f, %d\n",$fcal, $ze, $saz, $quad;
		printf"--- a1,a2,b1,b2,delaz: %.3f, %.3f, %.3f, %.3f, %.3f\n", $a1,$a2,$b1,$b2,$delaz;
		printf"--- corr = %.4f\n",$corr;
		exit(1);
	}
}

#==========================================================================
sub Tcal
# PRP ROUTINE
# $tc = Tcal( @casecal, mv , MISSING);
# example from a03_da0_avg
# $tc = Tcal( @casecal, ( $dat[15] + $dat[26] ) /2 , MISSING);
#Description
# This uses measured mv on the thermistor and uses the calibration
# coefficients produced by the PrpCal routines.
# input
# p1, p2, p3, p4, mv, missing
# output
# temp in degC
#
# v101 060621 rmr -- added to rmr_toolbox_met.pl
# v102 060622 rmr -- added documentation and test prints
{
	my @cal = ($_[0],$_[1],$_[2],$_[3]);
	my $mv = $_[4];
	my $missing = $_[5];
	my ($a, $b);
	#print "cal = @cal\n";
	#print "mv = $mv\n";
	if ( $mv <= 0 ) { $a = $missing }
	else {
		$a = log($mv);
		$b = $a * $a * $a * $cal[0] + $a * $a * $cal[1] + $a * $cal[2] + $cal[3];
		$a = 1 / $b - 273.15;
	}
	return $a;	
}

1
