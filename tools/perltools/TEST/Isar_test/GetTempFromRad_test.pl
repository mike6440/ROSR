#!/usr/bin/perl -w

use lib "/opt/local/lib/perl5/site_perl/5.12.3";
use lib "/Users/rmr/swmain/perl";
use perltools::Isar;

@tt = (1,2,3,4,5,6);
@rr = (10,20,30,40,50,60);

print"Ttable = @tt\n";
print"Rtable = @rr\n";
print"Enter a rad value:";
chomp($r=<>);
print"You entered $r\n";

my $temp = perltools::Isar::GetTempFromRad(\@tt, \@rr, $r);
print"temp = $temp\n";
exit;

