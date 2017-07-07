#!/usr/bin/perl -w

		#====================
		# PRE-DECLARE SUBROUTINES
		#====================
use lib $ENV{DAQLIB};
use perltools::MRutilities;
		#use Scalar::Util qw(looks_like_number);

$str='2342o34';
print"Input number = $str\n";

$v = looks_like_number($str)  ? $str : -999;
print"\nanswer is $v\n";

exit;

