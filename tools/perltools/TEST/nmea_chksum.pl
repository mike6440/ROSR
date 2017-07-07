#!/usr/bin/perl -w
#$WINAV,20120730,142334,45.34234,-122.34523,14.5,123.4,-2.34,1.66,-126.67*5B

$str='$WINAV,20120730,142334,45.34234,-122.34523,14.5,123.4,-2.34,1.66,-126.67*';
print"test string = <<$str>>\n";

$cc = NmeaChecksum($str);

print"checksum = $cc\n";

exit;

sub NmeaChecksum
# $cc = NmeaChecksum($str) where $str is the NMEA string that starts with '$' and ends with '*'.
{
    my ($line) = @_;
    my $csum = 0;
    $csum ^= unpack("C",(substr($line,$_,1))) for(1..length($line-1));

    return (sprintf("%2.2X",$csum));
}

