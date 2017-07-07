package perltools::MetFuncs;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&CompassCheck &ShipTrueWinds);

#USE IN A PROGRAM
# use lib "/Users/rmr/swmain/perl";
# use perltools::MetFuncs;


#=========================================================================
sub CompassCheck
{
	my $c=shift();
	while($c<0){$c+=360}
	while($c>=360){$c-=360}
	return $c;
}

#==========================================================================
sub ShipTrueWinds
{
	my($wsa,$wda,$sog,$cog,$hdg)=@_;
	print"ShipTrueWinds: wsa=$wsa, wda=$wda, sog=$sog, cog=$cog, hdg=$hdg\n";
	
	return 0;
}

1;
