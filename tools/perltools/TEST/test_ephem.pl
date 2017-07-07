#!/usr/bin/perl -w
#use strict;

#NECESSARY TO USE THESE ROUTINES
use lib "/Users/rmr/swmain/perl";
use perltools::MRtime;
use perltools::MRradiation;

my $lat= -40;
my $lon= 0;
my $dt = datesec(2011,9,21,12,0,0);

printf"test_ephem: lat=%.6f   lon=%.6f   %s\n",$lat, $lon, dtstr($dt);
@x = Ephem($lat,$lon,$dt);
printf"az=%.2f,  ze=%.2f, ze0=%.2f\n", $x[0], $x[1], $x[2];


exit(0);

