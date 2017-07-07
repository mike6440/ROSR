#!/usr/bin/perl -w
#use strict;

#NECESSARY TO USE THESE ROUTINES
use lib "/Users/rmr/swmain/perl";
use perltools::MRtime;
use perltools::MRradiation;

# TEST AOD_OZONE
# (tau, dob) = aod_ozone( dt, lat, det)


my $dt = datesec(2011,8,13,2,10,0);
my $lat = 41.350;  my $lon = 141.233;  # MIRAI IN MUTSU
my $det = 2;
my ($tau, $dob) = aod_ozone($dt, $lat, $det);

printf"TAU = %.5f,  DOB = %.5f\n", $tau, $dob;



exit(0);

