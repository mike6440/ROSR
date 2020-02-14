#!/usr/bin/expect --
# read the command line and boot accordingly.
$ans='n';
set setupfile  [lindex $argv 0]

if($#ARGV >= 0){
    $ans=shift();
    print"$ans was in the commannd line.\n";
} 
else {
    print"boot: WARNING  This process will turn power on or off
    to rosr. 
Call \"Archive\" to backup all collected data.
";    
    print"
    1. Turn ON
    2. Turn OFF
    3. Cycle OFF 5secwait ON
    Enter choice or <cr> do nothing: ";
    $ans=<>;
    chomp($ans);
    print"You entered $ans\n";
}

#edit 150425,1100L
#term_to_spare.ex (MAC)
#Note: the file .kermrc has the command "prompt k>>"
# KERMIT CONNECTION TO PDS752 COM ?
log_user 0
# IP number and port number
set ip [exec getsetupinfo "setup/su.txt" "IBOOT IP"]
if { $ip == 0 } {
	send_user "1";
	exit 1
}
send_user "Boot IP $ip\n";

# START PROCESS -- KERMIT FOR MODEM
spawn telnet $ip
set PDS $spawn_id
set timeout 4

expect {
	timeout {"TELNET FAILS TO OPEN\n"; exit 1}
	"User>"
}

send "admin\r\n";
expect {
	"Password>"
}

send "admin\r\n";
expect {
	"iBoot>"
}

send "set outlet on\r"
expect {
	"iBoot>"
}

send "get outlet\r"
expect {
	"On" {send_user "Power is ON.\n"}
	"Off" {send_user "Power is OFF.\n"}
}



send "logout\r"
send_user "0"
exit 0
