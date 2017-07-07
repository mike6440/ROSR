#!/usr/bin/perl -w
#120826,1357
use lib $ENV{MYLIB};
use perltools::MRtime;  

my ($dt, $dtx);
my ($julday,$jd0);
my @jdf;

print"This is a test of MRtime routines for Unix/Linux and Windows.\n";

print"--NOW--\n";
$dt=now();
print"dtz = $dt\n";

print"--GMTOFFSET--\n";
printf"gmtoffset = %d secs (%.0f hrs)\n", gmtoffset(), gmtoffset()/3600;

print"--DATEVEC DATESEC DATESEC_CHECK--\n";
@w=datevec($dt);
printf"time vector: @w -- datesec= %d -- string=%s\n", $dtx=datesec(@w), dtstr($dt);
$w[2]='xx';
printf"time vector: @w -- datesec_check=%d -- string=%s\n", $dtx=datesec_check(@w), dtstr($dt);

print"--DTSTR & JULIAN DAY--\n";
@jdf = dt2jdf($dt);
print"jdf=@jdf\n";
printf"Input time %s,  dt2jdf returns: %4d %.6f\n",dtstr($dt),$jdf[0], $jdf[1]; 
print"JDF->DT\n";
$dtx=jdf2dt($jdf[0],$jdf[1]);
printf"Input %d/%.5f,  jdf2dt = $dtx,  string=%s\n",$jdf[0],$jdf[1],dtstr($dtx);

@w=datevec($dt);
$julday = julday($w[0],$w[1],$w[2]);
print"julday = $julday\n";
$jd0 = julday($w[0],1,1);
print"julday0 = $jd0\n";
printf"yearday = %d\n", $julday-$jd0+1;

print"--DTSTR, DT TO STRING\n";
printf"csv:\t%s\n", dtstr($dt,'csv');
printf"def:\t%s\n", dtstr($dt);
printf"long:\t%s\n", dtstr($dt,'long');
printf"ssv:\t%s\n", dtstr($dt,'ssv');
printf"short:\t%s\n", dtstr($dt,'short');
printf"iso:\t%s\n", dtstr($dt,'iso');
printf"jday:\t%s\n", dtstr($dt,'jday');
printf"prp:\t%s\n", dtstr($dt,'prp');
printf"date:\t%s\n", dtstr($dt,'date');
printf"scs:\t%s\n", dtstr($dt,'scs');

print"--STRING TO DT\n";
printf"long:\t%d\n",dtstr2dt( dtstr($dt,'long') );
printf"csv:\t%d\n",dtstr2dt( dtstr($dt,'csv') );
printf"ssv:\t%d\n",dtstr2dt( dtstr($dt,'ssv') );
printf"short:\t%d\n",dtstr2dt( dtstr($dt,'short') );
printf"iso:\t%d\n",dtstr2dt( dtstr($dt,'iso') );
printf"scs:\t%d\n",dtstr2dt( dtstr($dt,'scs') );

print"WAIT 5 sec...\n";
WaitSec(5);
print"END\n";

exit 0;


