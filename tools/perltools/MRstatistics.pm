package perltools::MRstatistics;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&ComputeSampleEndPoints &chauvenet &mean &stats1 &stats &polyval
  &hunt &median &stdev &Yamartino &max &min
  &MetV2P &MetP2V &maxvalue &minvalue &VecV2P &VecP2V);

# IN THE PROGRAM
# use lib "/Users/rmr/swmain/perl";
# use perltools::MRstatistics;
# my $d = perltools::MRstatistics::chauvenet(...);

use lib $ENV{MYLIB};
use perltools::MRtime;
use POSIX;
#use Math::Trig;
use constant PI => 3.14159265358979;
use constant R2D => 180 / PI;
use constant D2R => PI / 180 ;

#==============================================================
sub chauvenet
# CHAUVENET'S CRITERION
#  yes/no (1/0) = chauvenet( $av, $stdev, $n, $suspect_point )
# see {taylor82} page 142
#v101 060713 rmr -- add to toolbox
{
	my ($avg, $stdev, $n, $suspect_point) = @_;
# 	printf("chauvenet->avg = %.5f, stdev = %.5f,  suspect_point = %.5f\n",$avg, $stdev, $suspect_point);
	my $x = 0; 
	if ( $stdev > 0 ) { $x = ( $suspect_point - $avg ) / $stdev};
# 	printf("chauvenet->t-sigma = %.5f\n", $x);
	$x = erfcc($x / 1.4142) / 2;
# 	printf("chauvenet->erfc(t-sigma) = %.5f\n", $x);
	$x = $n * $x;
# 	printf("Rejection criterion = %.5f, reject if < 0.5 \n", $x);
	if ( $x < 0.5 ) 
	{
# 		printf("chauvenet->Reject point\n");
		return (1);
	} else 
	{
# 		printf("chauvenet->No outlying points.\n");
		return (0);
	}
}

#*************************************************************/
sub ComputeSampleEndPoints
# ($dt_samp, $dt1,$dt2)=ComputeSampleEndPoints( $dtx, $avgsecs, $SampleFlag);
# Computes the time start and stop times for making an average.  Time is 
# expressed in seconds since 1970 by using the dtsecs() function.
#
#INPUT VARIABLES:
#  $dt0 (=$record{dt}) is the current record time
#  $avgsecs = averaging time
#  $SampleFlag = 0/1
# 		There are two optons: either divide the day into even sample 
# 		periods of $avgsecs long (e.g. 0, 5, 10, ... min) or begin 
# 		precisely with the input sample.  The sample start parameter
# 		$SampleFlag is set for this option.
# OUTPUT (GLOBAL)
#  $dt_samp = the mid point of the sample interval
#  $dt1, $dt2 = current sample end points
# REQUIRED SUBROUTINES
#	datevec();
#	datesec();
#
# v101 060622 rmr -- begin tracking this subroutine.
# v101 060628 -- begin toolbox_avg
{
	my ($y, $M, $d, $h, $m, $s, $dt0);
	my $dt_samp = shift();
	my $avgsecs = shift();
	my $SampleFlag=shift();
	
	#printf "ComputeSampleEndPoints dt_samp = %s, avgsecs = $avgsecs, SampleFlag = $SampleFlag\n",dtstr($dt_samp);
	
	#$dt_samp = $record{dt};				# this is the time of the first sample.
	if ( $SampleFlag == 0 )
	{
		#==================
		# COMPUTE THE dt CURRENT BLOCK
		#==================
		($y, $M, $d, $h, $m, $s) = datevec( $dt_samp );
		$dt0 = datesec($y, $M, $d, 0, 0, 0) - $avgsecs/2;  # epoch secs at midnight
		$dt1 = $dt0 + $avgsecs * floor( ($dt_samp - $dt0) / $avgsecs );	# prior sample block dtsec
	} 
	else { $dt1 = $dt_samp; }
	
	$dt2 = $dt1 + $avgsecs;			# next sample block dtsec
	$dt_samp = $dt1 + $avgsecs/2;  # the time of the current avg block
	return ($dt_samp, $dt1, $dt2);
}
#*************************************************************************#
sub hunt
# $jlo = hunt ( \@xx-1, $x, $jlo);
# input
#  \@xx is a pointer to the input array
#  $x is the input number
#  $jlo is the first guess
# output
#  $jlo is the index of the lower point bracketing $x.
# note: if $x < $xx[0], jlo=-1;  if $x >= $xx[$#xx] then $jlo=$#xx;
#
#=============================
# void hunt(float xx[0..n-1], unsigned long n, float x, unsigned long *jlo)
# From Numerical Recepies for C
# Given an array xx[1..n] and a given value of x, returns a value jlo such
# that x is between xx[jlo] and xx[jlo+1].  xx must be monotonic.
# jlo=0 or jlo = n is returned to indicate x is out of range.  jlo on input
# is the initial guess.
#=============================
# see test_hunt.pl
# v1 rmr 110307
{
	my ($jm, $jhi, $inc);
	my $ascnd;
	
	my $xx = $_[0];  # -- pointer to xx[]
	my $n = $#xx+1;  # -- number of points in the series.
	my $x = $_[1];   # -- search point
	my $jlo = $_[2]; # -- start index (if = -1 or n => straight bisection)
	
	if ($x < $xx[0]){return -1}
	if($x >= $xx[$#xx]){return $#xx}
	
	#printf("Start function hunt: %.2f, %d, %.2f, %d\n", $$xx[0], $n, $x, $jlo);  # test

	# TRUE FOR M=INCREASING,  FALSE FOR DECREASING
	$ascnd = ( $$xx[$n-1] >= $$xx[0] );
	

	# --- INITIAL RUN, jlo IS NOT SET
	if ( $jlo < 0 || $jlo > $n-1) {
		$jlo = -1;
		$jhi = $n;
	# -- jlo IS THE STARTING INDEX
	} else {
		$inc = 1;
		if ( $x >= $$xx[$jlo] == $ascnd) {
		# --- x IS BELOW RANGE ---
			if ($jlo == $n-1) { return($jlo) }
			$jhi = ( $jlo ) + 1;
			while ( $x >= $$xx[$jhi] == $ascnd) {
				$jlo = $jhi;
				$inc += $inc;
				$jhi = $jlo + $inc;
				if ( $jhi > $n - 1 ) {
					$jhi = $n - 1;
					last;
				}
			}
		#-- x is above range --
		} else {
			if ( $jlo == 0 ) { return( -1 ) }
			$jhi = $jlo--;
			$jlo -= 1;
			while ( $x < $$xx[$jlo] == $ascnd) {
				#printf("  %d   %d\n", $jlo, $jhi );  # test
				$jhi = $jlo;
				$inc <<= 1;   # double the size of inc
				if ( $inc >= $jhi ) {
					$jlo = -1;
					last;
				}
				else { $jlo = $jhi - $inc }
			}
		}
	}
	
	# --- BISECTION METHOD BEGINNING WITH jlo AND jhi ---
	#printf("bisection ...\n");  # test
	while ( $jhi - ($jlo) != 1) {
		#printf("  %d   %d\n", $jlo, $jhi );  # test
		$jm = ($jhi + ($jlo) ) >> 1;  # midpoint
		if ( $x >= $$xx[ $jm ] == $ascnd) { $jlo = $jm }
		else { $jhi = $jm }
		
	}
	if ( $x == $$xx[ $n-1 ] )  { return ( $n - 1 )}
	if ( $x == $$xx[0])    { return ( 0 ) }
	return ( $jlo );
}
#*************************************************************/
sub  max
# CALLING: ($mx, $imx) = max(@array)
# returns the maximum value and the index of the maximum.
# v102 060721 rmr
{
	# the input is an array of length $#_.
	my $mx = -1e20;
	my ($i, $imx);
	$i=0;
	foreach(@_) { 
		if ( $_ > $mx ) {
			$mx = $_;
			$imx = $i;
		}
		$i++;
	}
	return ( $mx, $imx );
}

#================================================
sub mean {
	  my(@data) = @_;
	  my $sum=0;
	  foreach(@data) {
		  $sum += $_;
	  }
	 return($sum / @data);
 }
#================================================
sub median {
	my(@data)=sort { $a <=> $b} @_;
	if (scalar(@data) % 2) {
		return($data[@data / 2]);
	} else {
		my($upper, $lower);
		$lower=$data[@data / 2];
		$upper=$data[@data / 2 - 1];
		return(mean($lower, $upper));
	}
}
#*************************************************************/
sub min
# CALLING: ($mn, $imn) = min(@array)
# returns the maximum value and the index of the maximum.
# v102 060721 rmr
{
	# the input is an array of length $#_.
	my $mn = 1e20;
	my ($i, $imn);
	$i=0;
	foreach(@_) { 
		if ( $_ < $mn ) {
			$mn = $_;
			$imn = $i;
		}
		$i++;
	}
	return ( $mn, $imn );
}

#*************************************************************/
sub stats1
# sub (mn, stdpcnt, n, min, max) = stats1(sum, sumsq, N, min, max, Nsamp_min, $missing);
{
	my ($sum, $sumsq, $N, $min0, $max0, $Nsamp_min, $missing) = @_;
	my ($mn, $std, $n, $min, $max);

	$mn = $std = $missing;
	$n = $N;  $min = $min0;  $max = $max0;
	
# 	print"Stats in: $sum, $sumsq, $N, $min0, $max0, $Nsamp_min\n";
	
	#=================
	# MUST HAVE >= Nsamp_min data points
	#=================
	if ( $N < $Nsamp_min || $N == $missing)  
	{
		$std = $missing;
		$mn = $min = $max = $missing;
	}
	elsif ( $N == 1 ) {
		$mn = $sum;
		$std = 0;
		$n = 1;
		$min = $max = $sum;
	}		
	else  ## N >= 2
	{
		$mn = $sum / $N ;			
		# -- stdev as a percent of the mean -------------
		# from b->ssig = sqrt((a->ssumsq - a->ssum * a->ssum / a->Ns) / (a->Ns-1));
		$x = ( ($sumsq - $sum * $sum / $N) / ($N - 1) );
		if ( $x > 0 ) { $std = sqrt ( $x ); }
		else { $std = 0; }
	}
# 	print"Stats out: $mn, $std, $n, $min, $max\n";
	
	return ( $mn, $std, $n, $min, $max ); 
}

#*************************************************************/
sub stats
# ($av, $stdev, $navg, $mn, $mx, $imn, $imx) = stats(\@data, $Nsamp_min);
#v101 060629 rmr -- start config control
#INPUT
#  \@data = series of data points
#  $Nsamp_min = the minimum number of data points for valid stats
#OUTPUT
#  $av = simple scalar mean
#  $stdev = scalar standard deviation
#  $navg = the number of points finally used
#  $mn, mx = minimum and maximum values
#  $imn, $imx = index of the minimum and maximum 
#
#If npts < $Nsamp_min return $avg = $stdev = $missing.  
#There must be at least 2 points for stdev.
#
# v2 051206 -- allow a single sample but set stdev = $missing
#  v1.2 060325 -- requires use constant $missing => -999
{
	my $missing = -999;
	my ($pdata, $nmin) = @_;	# parse the incoming array 
	my ($sum, $sumsq); # sum and sum of squares in the input;
	my ( $av, $std, $mn, $mx, $imn, $imx, $x, $n);  # misc and input variables
	my ($navg); # 
	$sum = $sumsq = 0;
	$mn = 1e20; $mx = -1e20;
	$av = $std = $missing;
	$n = $#$pdata + 1;  # total number of points in the input series
	#=====================
	# COMPUTE AVG AND STD, INCLUDE REJECTION OF AT LEAST TWO OUTLIERS
	#=====================
	if ( $n <= 0 )   # v2 rmr allow a single sample
	{
		$std = $missing;
		$av = $navg = $missing;
		$mn = $mx = $missing;
		$imn = $imx = 0;
	}
	else
	{
		$navg=0;
		foreach ( @$pdata ) 
		{
			# do not include bad data 
			if ( $_ != $missing )
			{
				$sum += $_;
				$sumsq += $_ * $_;
				if ( $_ < $mn ) { $mn = $_; $imn = $navg }
				if ( $_ > $mx ) { $mx = $_; $imx = $navg }
				$navg++;
			}
		}
 # 		printf("stats->navg = %d,  sum = %.2f,  sumsq = %.3f\n", $navg, $sum, $sumsq);
		# compute avg and stdev only if there are sufficient points
		# if there is one point stdev = 0;
		if ( $navg < $nmin ) { $av = $std = $missing }
		else
		{
			$av = $sum / $navg ;
			if ( $navg <= 1) {
				$std = 0;
			} else {
				$x = ( ($sumsq - $sum * $sum / $navg) / ($navg - 1 ) );
				if ( $x > 0 ) { $std = sqrt ( $x ) }
				else { $std = 0 }
			}
		}
	}
#  	printf("stats->av = %.2f, std = %.2f, min = %.2f, max = %.2f, imn = %d, imx = %d\n",
#  	  $av, $std, $mn, $mx, $imn, $imx);
	return ($av, $std, $navg, $mn, $mx, $imn, $imx);
}
#================================================
 sub stdev {
     my(@data)=@_;
     my($sq_dev_sum, $avg)=(0,0);
     $avg = mean(@data);
     foreach my $elem (@data) {
         $sq_dev_sum += ($avg - $elem) **2;
     }
     return(sqrt($sq_dev_sum / ( $#data )));
 }
#======================================================================
sub Yamartino
#sigtheta = Yamartino ( $x, $y, $missing );
# Computes sigma-theta by the Yamartino method
#  EPA Meteorological Monitoring Guidance for Regulatory Modeling Applications
# United States Office of Air Quality EPA-454/R-99-005 
# Environmental Protection Planning and Standards Agency Research 
# Triangle Park, NC 27711 February 2000 
# page 53
# INPUT
#    X = average of unit vector in the x axis, vector convention
#    Y = average in y axis
# OUTPUT
#  sigtheta is the stdev of direction
#  returns OK or NOTOK
# EXAMPLE
# example: d = [120 125 140 100 110]';
#   Xavg=    Yavg=
#    sigtheta = 13.56 deg
# example: d = [20    10   330    10     0]';
#    sigtheta = 17.16
#v101 060629 rmr -- start config control
{
# use constant PI => 3.14159265358979;
# use Math::Trig;
	if ( $_[0] == $_[2] || $_[1] == $_[2] ) { return ($_[2]) }
	
# 	use constant PI => 3.14159265358979;
	my $r2d = 180 / PI;
	my ($e, $st);
	$e = 1 - ($_[0] * $_[0] + $_[1] * $_[1]);
	if ( $e < 0 ) { return ( $_[2] ) }
	
	$e = sqrt ( $e );
# 	printf("test-->e = %.5f\n", $e);
	$st = asin ( $e ) * (1 + 0.1547 * $e*$e*$e ) * $r2d;
# 	printf("test--> sigtheta = %.2f\n", $st);
	return ( $st );
}

#======================================================================
sub MetP2V
# (x,y) = MetP2V(s,d, missing)
# METV2P = convert true vector components to met polar
# Wind vectors can be reported in meteorological convention or 
#   oceanographic convention which means the vector points in the direction the wind is coming from,
#  or in the direction the wind is going, respectively.
#  The convention we use here is that meteorological winds in POLAR coordinates are
# reported as speed and the direction the wind is coming from.
# The components of the wind, the wind vector, are the components of the
# vector and thus are in the direction the wind is going.  A true vector
# sense.
# INPUT
#  s = speed (magnitude)
#  d = meteorological direction, (from)
# OUTPUT
#  x,y are vector components (to direction)
# 
# adapted from C tools: rmrtools
{
# use constant PI => 3.14159265358979;
# use Math::Trig;
	if ( $_[0] == $_[2] || $_[1] == $_[2] ) { return ($_[2], $_[2]) }
	my $x = - $_[0] * sin ( D2R * $_[1]);
	my $y = - $_[0] * cos ( D2R * $_[1]);
	return ($x, $y);
}
#======================================================================
sub MetV2P
# (s,d) = MetP2V(u,v, missing)
# METV2P = convert true vector components to met polar
# Wind vectors can be reported in meteorological convention or 
#   oceanographic convention which means the vector points in the direction the wind is coming from,
#  or in the direction the wind is going, respectively.
#  The convention we use here is that meteorological winds in POLAR coordinates are
# reported as speed and the direction the wind is coming from.
# The components of the wind, the wind vector, are the components of the
# vector and thus are in the direction the wind is going.  A true vector
# sense.
# INPUT
#  x,y are vector components (to direction)
# OUTPUT
#  s = speed (magnitude)
#  d = meteorological direction, (from)
# 
# adapted from matlab tools: rmrtools
# ---------------------------------------------------------------------------
{
# use constant PI => 3.14159265358979;
# use Math::Trig;
	if ( $_[0] == $_[2] || $_[1] == $_[2] ) { return ($_[2], $_[2]) }
	my $x = $_[0];
	my $y = $_[1];
	my $s = sqrt( $x * $x + $y * $y);
	my $d = atan2(-$x, -$y) * R2D;
	if ( $d < 0 ) { $d += 360 }
	return ($s, $d);
}

#======================================================================
sub VecP2V
# (x,y) = VecP2V(s,d, missing)
# VECV2P = convert true vector components to VECTOR polar
# Wind vectors can be reported in meteorological convention or 
#   oceanographic convention which means the vector points in the direction the wind is coming from,
#  or in the direction the wind is going, respectively.
#  The convention we use here is that meteorological winds in POLAR coordinates are
# reported as speed and the direction the wind is coming from.
# The components of the wind, the wind vector, are the components of the
# vector and thus are in the direction the wind is going.  A true vector
# sense.
# INPUT
#  s = speed (magnitude)
#  d = VECTOR direction, (TO)
# OUTPUT
#  x,y are vector components (to direction)
# 
# adapted from C tools: rmrtools
{
# use constant PI => 3.14159265358979;
# use Math::Trig;
	my $d2r = PI / 180;
	if ( $_[0] == $_[2] || $_[1] == $_[2] ) { return ($_[2], $_[2]) }
	my $x = $_[0] * sin ( $d2r * $_[1]);
	my $y = $_[0] * cos ( $d2r * $_[1]);
	return ($x, $y);
}

#======================================================================
sub VecV2P
# (s,d) = VecP2V(u,v, missing)
# VECV2P = convert true vector components to VECTOR polar
# Wind vectors can be reported in meteorological convention or 
#   oceanographic convention which means the vector points in the direction the wind is coming from,
#  or in the direction the wind is going, respectively.
#  The convention we use here is that meteorological winds in POLAR coordinates are
# reported as speed and the direction the wind is coming from.
# The components of the wind, the wind vector, are the components of the
# vector and thus are in the direction the wind is going.  A true vector
# sense.
# INPUT
#  x,y are vector components (to direction)
# OUTPUT
#  s = speed (magnitude)
#  d = VECTOR direction, (TO)
# 
# adapted from matlab tools: rmrtools
# ---------------------------------------------------------------------------
{
# use constant PI => 3.14159265358979;
# use Math::Trig;
	my $r2d = 180 / PI;
	if ( $_[0] == $_[2] || $_[1] == $_[2] ) { return ($_[2], $_[2]) }
	my $x = $_[0];
	my $y = $_[1];
	my $s = sqrt( $x * $x + $y * $y);
	my $d = atan2($x, $y) * $r2d;
	if ( $d < 0 ) { $d += 360 }
	return ($s, $d);
}



#===================================================================
sub maxvalue {	
	if ( $_[0] >= $_[1] ) { return $_[0] }
	else { return $_[1] }
}

#===================================================================
sub minvalue {	
	if ( $_[0] <= $_[1] ) { return $_[0] }
	else { return $_[1] }
}

#*************************************************************/
sub polyval
# y = polyval (a, x);
#   y = ((( a_n * x + a_(n-1) ) * x + a_(n-2) )... * x + a0 ) ...
#example: a = (1,2,3)
#  y = 1 + 2 * x + 3 * x^2;
{
	my $i;
	my ($x, $y);
	my @a = @_;
	$x = pop(@a);
	$y = $a[$#a];  # start with the nth term
	$i = $#a-1;
	while ( $i >= 0 )
	{ 
		$y = $y * $x + $a[$i];
		$i--;
	}
	return ( $y );
}




1;
