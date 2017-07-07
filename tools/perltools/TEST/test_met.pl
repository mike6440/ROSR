#!/usr/bin/perl -w
#use strict;

#NECESSARY TO USE THESE ROUTINES
use constant PI => 3.14159265358979;
use Math::Trig;
sub MetV2P;
sub MetP2V;
sub Yamartino;
sub dewpoint;

# TEST MetP2V
# $spd = 1; 
# $dir = 135;
# ($x, $y) = MetP2V($spd, $dir, $missing);
# printf("spd/dir = %.2f at %.2f deg,  x/y = %.2f, %.2f \n", $spd, $dir, $x, $y);

# TEST YAMARTINO
# my $x = 0;
# my $y = 0;
# my $d2r = PI / 180;
# # my @d = (120, 125, 140, 100, 110);  # ==> 13.56
# my @d = (20,    10,   330,    10,     0);  # ==> 17.16
# # my @d = (90, 90, 90, 90, 90, 90);
# my $n = 0;
# foreach (@d) {
# 	$x += sin ( $d2r * $_ );
# 	$y += cos ( $d2r * $_ );
# 	$n++;
# }
# $x = $x / $n;  $y /= $n;
# printf(" mean x = %.2f,  y = %.2f nsamps = %d\n", $x, $y. $n);
# my $sigtheta = Yamartino ( $x, $y, $missing );
# printf(" sigma-theta = %.2f degrees\n", $sigtheta);

# TEST DEWPOINT
my $tair = 20;
my $rh = 82.9;
my $dpd = $tair - dewpoint(20,82.9);
print "Dep point depression = $dpd\n";


exit(0);

