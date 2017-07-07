#!/usr/bin/perl -w
#use strict;

#NECESSARY TO USE THESE ROUTINES
use lib "/Users/rmr/Dropbox/swmain/perl";
#use lib "/Users/rmr/Dropbox/swmain/apps/MET1/tools";
use perltools::MRutilities;
use perltools::MRtime;
my $missing = -999;

print"            GPRMC\n";
my $sentence='$GPRMC,154749,A,4736.2165,N,12217.2161,W,000.0,200.9,030912,018.1,E*61';
print"$sentence\n";
my ($dtgps, $lat, $lon, $sog, $cog, $var) = gprmc($sentence, $missing);
printf"lat=%.5f, lon=%.5f, sog=%.1f, cog=%.0f, var=%.1f\n", $lat, $lon, $sog, $cog, $var;

print"           NMEA CHECKSUM\n";
$sentence='$GPRMC,154749,A,4736.2165,N,12217.2161,W,000.0,200.9,030912,018.1,E*';
#$sentence='$A*';
print"NMEA string: $sentence\n";
my $chksum=NmeaChecksum $sentence;
print"checksum = $chksum\n";

print"           LOOKS_LIKE NUMBER\n";
$x='23.456'; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";
$x=23.456; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";
$x='23.23E23'; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";
$x='23.456 e 12'; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";
$x='23.456e12'; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";
$x='23.x456'; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";
$x='abcd'; $y=looks_like_number($x);  printf"number=$x --  ans=$y\n";

print"             NOTANUMBER\n";
$x='23.456'; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";
$x=23.456; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";
$x='23.23E23'; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";
$x='23.456 e 12'; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";
$x='23.456e12'; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";
$x='23.x456'; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";
$x='abcd'; $y=NotANumber($x);  printf"number=$x --  ans=$y\n";


exit(0);

