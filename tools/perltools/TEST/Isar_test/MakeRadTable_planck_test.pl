#!/usr/bin/perl -w

use lib "/opt/local/lib/perl5/site_perl/5.12.3";
use lib "/Users/rmr/swmain/perl";
use perltools::Isar;
use perltools::MRstatistics;


my $Tabs = 273.15;
my $ktfile= "/Users/rmr/swmain/apps/isardaq4/kt15/kt15_filter_15855339.dat";

my ($ttabler, $rtabler) = perltools::Isar::MakeRadTable_planck($ktfile, $Tabs);
@tt = @{$ttabler};
@rt = @{$rtabler};

$i=0;
foreach(@tt){ printf"%4d %.6e  %.6e\n", $i, $tt[$i], $rt[$i]; $i++}

exit;


