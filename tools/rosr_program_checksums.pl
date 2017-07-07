#!/usr/bin/perl -w


$swpath="$ENV{HOME}/swmain/apps/ROSR/sw";
print"swpath = $swpath\n";
$archivepath=shift();
#$archivepath="/Volumes/$archivepath/swmain/apps/ROSR/sw";
$archivepath="/media/$ENV{USER}/$archivepath/swmain/apps/ROSR/sw";
$archiveflag=1;
if(! -d $archivepath){
	print"archive path: $archivepath MISSING\n";
	$archiveflag=0;
}

$pgms='ArchiveRosr
avggps
avgrosr
bootoff
booton
bootstatus
Cleanuprosr
ClearRosrData
DaqUpdate
FindUSBPort
get_kt
getsetupinfo
gpsgetraw
help_advanced.txt
help.txt
kerm232
kermss
KillScreen
LastDataFolder
LastDataRecord
LastDataTime
PrepareForRun
RosrShutdown
sbd_transmit
SetDate
sockrxx
socktxx
term_to_gps
term_to_rosr
term_to_sbd
UpdateDaq
Z_gps
Z_rosr
setup/su_mossbay_rosr2.txt
../tools/bashrc_add_to_existing.txt
../tools/bashrc_rosr.txt
../tools/crontab_rosr.txt
../tools/kermrc_rosr.txt
../tools/screenrc_rosr.txt
../tools/rosr_program_checksums.pl
../sketchbook/rosr_main/rosr_main.ino';
#print"$pgms\n";
@p=split /\n/,$pgms;

print"  n     sw      archive  File\n";
# 0	29540	29540	ArchiveRosr

$i=0; foreach $f (@p){	
	$e=' ';
	# SW FOLDER
	$ff="$swpath/$f";
	if(! -f $ff){
		#print"file $ff missing\n";
		$s='-9999';
		$e='*';
	} else {
		@f=split / /,`sum $ff`;
		$s=$f[0];
	}
	# ARCHIVE FOLDER
	$ff="$archivepath/$f";
	if(! -f $ff){
		#print"archive file $ff missing\n";
		$e='*';
		$sa='-9999';
	} else {
		@f=split / /,`sum $ff`;
		$sa=$f[0];
	}
	if($sa ne $s){$e='*'}
	if($archiveflag==1){printf"%s%2d\t%d\t%d\t%s\n",$e,$i,$s,$sa,$f}
	else{printf"%d\t%d\t%s\n",$i,$s,$f}
	$i++;
}
