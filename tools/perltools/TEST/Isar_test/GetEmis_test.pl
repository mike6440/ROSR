#!/usr/bin/perl -w

use lib "/opt/local/lib/perl5/site_perl/5.12.3";
use lib "/Users/rmr/swmain/perl";
use perltools::Isar;

print"Enter a pointing angle value:";
chomp($x=<>);
print"You entered $x\n";

my $e = perltools::Isar::GetEmis($x, -999);
print"Emissivity = $e\n";
exit;


