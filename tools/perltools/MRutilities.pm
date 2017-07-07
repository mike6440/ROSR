package perltools::MRutilities;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw( &trim &ltrim &rtrim &FindLines &FindLineExact &FindInfo &isnumber &NotANumber &gprmc &NmeaChecksum);

# editdate = 130702;
# version = 4;

#USE IN A PROGRAM
use lib $ENV{MYLIB};
use perltools::MRtime;
use Scalar::Util qw(looks_like_number);



# Perl trim function to remove whitespace from the start and end of the string
sub trim($)
{
	my $string = shift;
	$string =~ s/^\s+//;
	$string =~ s/\s+$//;
	return $string;
}
# Left trim function to remove leading whitespace
sub ltrim($)
{
	my $string = shift;
	$string =~ s/^\s+//;
	return $string;
}
# Right trim function to remove trailing whitespace
sub rtrim($)
{
	my $string = shift;
	$string =~ s/\s+$//;
	return $string;
}

#*************************************************************/
sub FindLines
#
# @rtn = FindLines( $fullfilename, $strx, $nlines)
#
# SEARCHES FOR A LINE WITH A GIVEN STRING
# $fullfilename = file to search
# $strx = the search string, A RegEx string
# $nlines after the id string
#     If nlines = 0, return only the found line.
#     if nlines = 1, return the found line and the next line.
# $opts = optional features, leave blank for default
#   x = exact match
#
#  Stop at end of file or if 'END' is found at the beginning of a line
#
# ver 1.02 rmr 060612 
# ver 103 060807 rmr -- drops first char of out[0] line.
# v104 060902 rmr -- fixed problems with push
# v105 061017 rmr -- just a simple retrieve of lines.
# v106 130102 rmr -- add options
{
	my $fname = shift();
	my $sx = shift();
	my $nlines = shift();
	
	my $s;
	my $str2 = '';
	my @strout=('MISSING');
	my $i;
	
	open ( F, "<$fname" ) or die("FindLines(),  $fname FAILS\n");
	
	while ( <F> ) {
		chomp($s = $_);							# read each line
		if ( $s =~ /$sx/ ) {
			@strout= $s;						# v102, found string at [0]
			#print"Start Reading lines\n";
			# then read the next n lines
			if ($nlines > 0 ) {
				for ( $i=0; $i<$nlines; $i++ ) { 
					chomp($str2 = <F>);
					push(@strout, $str2);
					if ( $str2 =~ /^END/i || eof(F) ) { last }
				}
			}
			last;
		}
	}
	close(F);
	return @strout;						# return the line info and the second line
}
#*************************************************************/
sub FindLinesExact
#
# @rtn = FindLinesExact( $fullfilename, $strx, $nlines)
#
# SEARCHES FOR A LINE WITH AN EXACT MATCH TO THE GIVEN STRING
# $fullfilename = file to search
# $strx = the search string, A RegEx string
# $nlines after the id string
#     If nlines = 0, return only the found line.
#     if nlines = 1, return the found line and the next line.
# $opts = optional features, leave blank for default
#   x = exact match
#
#  Stop at end of file or if 'END' is found at the beginning of a line
#
# ver 1.02 rmr 060612 
# ver 103 060807 rmr -- drops first char of out[0] line.
# v104 060902 rmr -- fixed problems with push
# v105 061017 rmr -- just a simple retrieve of lines.
# v106 130102 rmr -- add options
{
	my $fname = shift();
	my $sx = shift();
	my $nlines = shift();
	
	my $s;
	my $str2 = '';
	my @strout='MISSING';
	my $i;
	
	open ( F, "<$fname" ) or die("FindLines(),  $fname FAILS\n");
	
	while ( <F> ) {
		chomp($s = $_);							# read each line
		if ( $s =~ /$sx/ ) {
			@strout= $s;						# v102, found string at [0]
			#print"Start Reading lines\n";
			# then read the next n lines
			if ($nlines > 0 ) {
				for ( $i=0; $i<$nlines; $i++ ) { 
					chomp($str2 = <F>);
					push(@strout, $str2);
					if ( $str2 =~ /^END/i || eof(F) ) { last }
				}
			}
			last;
		}
	}
	close(F);
	return @strout;						# return the line info and the second line
}
#===============================================================
sub FindInfo
# sub FindInfo;
# Search through a file line by line looking for a string
# When the string is found, remove $i characters after the string.
		# ==== CALLING ====
# $strout = FindInfo( $file, $string, [$splt,  [$ic, [exit_on_fail]]] )
		# ==== INPUT ===
# $file is the file name with full path
# $string is the search string (NOTE: THIS IS A REGEX STRING,
# $splt (optional) is the substring for the split of the line. (typically :)
# $ic (optional) is the number of characters to extract after the split.
#    If $ic is negative then characters before the string are extracted.
# $exit_on_fail is 1 for an exit if the string is not found, 0 to return "MISSING" 
#
		#EXAMPLE
#    $fn='path/name' contains a line "SEARCH STRING: the answer"
#  $str = FindInfo(filename_to_scan, 'SEARCH STRING');  print"$str\n";
#  returns 'the answer'
#
#    $fn='path/name' contains a line "SEARCH STRING - the answer"
#  $str = FindInfo(filename_to_scan, 'SEARCH STRING');  print"$str\n";
#  replies "CANNOT FIND SEARCH STRING" and exits. Program stops here.
#
#    $fn='path/name' contains a line "SEARCH STRING - the answer"
#  $str = FindInfo(filename_to_scan, 'SEARCH STRING','-');  print"$str\n";
#  returns 'the answer'
#
#    $fn='path/name' contains a line "SEARCH STRING - the answer"
#  $str = FindInfo(filename_to_scan, 'SEARCH STRING','-',4);  print"$str\n";
#  returns 'answer'
#
#    $fn='path/name' contains a line "SEARCH STRING - the answer"
#  $str = FindInfo(filename_to_scan, 'SEARCH STRING',':',0,0);  print"$str\n";
#  returns 'MISSING'

 
{
	my @v = @_;
		
	my @cwds;
	my ($fn, $strin, $splt, $strout, $ic, $str, $rec, $ix, $exit_on_fail);
	$fn = $v[0];  #0
			# SEARCH STRING, REMOVE HEAD AND TAIL BLANKS
	$strin = $v[1]; $strin =~ s/^\s+//;  $strin =~ s/\s+$//;
			# OPTIONS
	$ic=0;
	$exit_on_fail=1; 
	$splt=':';
	if ( $#v >= 2 ) { $splt = $v[2];
		if ( $#v >= 3 ) { $ic = $v[3];
			if ( $#v >= 4 ) { $exit_on_fail = $v[4] }
		}
	} 
	$strout = 'MISSING';
			# OPEN THE CAL FILE
	open(Finfo, "<$fn") or die("!!FindInfo OPEN FILE FAILS, $strin\n");
	$rec=0;
			# READ EACH LINE
	while ( <Finfo>) {
		$rec++;
				# LINE LIMIT
		if ( $rec >= 1000 ) { 			#v14
			close(Finfo);
			print"CANNOT FIND $strin.\n"; last;   #v17
		}
		else {
			#  SCAN THE LINE
			# clean the line (a.k.a. record)
			# find the first occurance of $splt.
			chomp($str=$_);
			if ( $str =~ /^$strin/ && $str =~ /$splt/) {
				$j = index($str,$splt);
				$c1 = substr($str,0,$j-1);
				$strout = substr($str,$j+1);
				$strout =~ s/^\s+//;  $strout =~ s/\s+$//;
				close(Finfo);
				return $strout;
			}
		}
	}
	# EOF AND NO STRING FOUND
	close(Finfo);
	print"CANNOT FIND $strin. exit_on_fail = $exit_on_fail\n"; 
	if ($exit_on_fail == 1) { print"STOP.\n"; exit 1 }
	return $strout;
}
#==============================================================
sub isnumber
#   $k = isnumber($x);
#INPUT
#  x is a variable, could be a string
#OUTPUT
#  k = 1 / 0 if true / false
{
	#return 0 if not a number, 1 if a legitimate number
	#v06 -- check each field to be sure it is a number
	# $x = '  -0.35a612';
	# if ( ($x = trim($x)) =~ /^\ *-?\d+\.?\d*e?\d*$/) { print "$x is a real number\n" }
	# else { print "$x is not a  number\n" }
	# die;
	my $x = shift();
	if ( ($x = trim($x)) =~ /^\ *-?\d+\.?\d*e?\d*$/) { return 1 }
	else { return 0 }
}

#=======================================================
sub NotANumber{
	local $_ = shift;
	if (looks_like_number($_)==1) {return 0} else {return 1};
#  my $val = trim(shift());
#  if($val !~ m/[^eE0-9+\-\.]+/) {return 0} else {return 1};
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
		$dtgps = perltools::MRtime::datesec($yy,$MM,$dd,$hh,$mm,$ss);
		#printf "gps time = %s\n", dtstr($dtgps);
		
		# latitude
		if (looks_like_number($dat[3])) {
			@d = split(/[.]/,$dat[3]);
			$d1 = substr($d[0], 0, length($d[0]) - 2);
			$d2 = substr($d[0],-2,2);
			$d2 = $d2.'.'.$d[1];
			$lat = $d1 + $d2/60;
			if ($dat[4] =~ /S/i) {$lat = -$lat}
		}
		# longitude
		if (looks_like_number($dat[5])) {
			@d = split(/[.]/,$dat[5]);
			$d1 = substr($d[0], 0, length($d[0]) - 2);
			$d2 = substr($d[0],-2,2);
			$d2 = $d2.'.'.$d[1];
			$lon = $d1 + $d2/60;
			if ($dat[6] =~ /W/i) {$lon = -$lon}
		}
		
		# speed over ground
		$sog = looks_like_number($dat[7]) ? $dat[7] * 0.51444445 : $nan;
		
		# COURSE
		$cog = looks_like_number($dat[8]) ? $dat[8] : $nan;
		
		# variation
		$var = looks_like_number($dat[10]) ? $dat[10] : $nan;
		if ( $dat[11] =~ /W/i) { $var = -$var }
		
		return ($dtgps, $lat, $lon, $sog, $cog, $var);
	}
}

#$GPRMC,190824,A,4737.0000,S,12300.0000,W,002.1,202.0,210210,019.0,W*62
sub NmeaChecksum
# $cc = NmeaChecksum($str) where $str is the NMEA string that starts with '$' and ends with '*'.
{
    my ($line) = @_;
    my $csum = 0;
    $csum ^= unpack("C",(substr($line,$_,1))) for(1..length($line)-2);
    return (sprintf("%2.2X",$csum));
}


1;
