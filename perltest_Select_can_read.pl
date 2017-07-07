#!/usr/bin/perl -w
# Continues to loop until something comes from STDIN/

use IO::Select;
$s = IO::Select->new();
$s->add(\*STDIN);

while (++$i) {
  print "Hiya $i!\n";
  sleep(5);
  if ($s->can_read(.5)) {
    chomp($foo = <STDIN>);
    print "Got '$foo' from STDIN\n";
    if($foo =~ /^q/){last}
  }
}
exit 0;
