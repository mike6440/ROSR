#*************************************************************/
sub ReadNextRecord
# Scan FILEIN until end of until a good data record is found.
# Fills global hash variable %record(xx) where
# v101 060628 -- begin toolbox_avg
{
	my ($str);
	my (@dat);
	my $flag = 0;
	my @dt;
	#=========================
	# READ EACH RECORD
	#=========================
	while (<FILEIN>){
		chomp($str = $_);  
		# SKIP NON-DATA LINES
		@dat = split(/$RecordParsingString/, $str);	# parse the data record
		#$i=0; for (@dat) { printf ("ReadNextRecord: %d %s\n",$i++, $_ ) } 	#test
		
# @VARS = ('shadow', 'thead', 'pitch','roll','az','psp','piru','pir',
# 	'tcase','tdome','refmv','battmv');
		#======================
		# SKIP BAD STRINGS
		#======================
		if ( $#dat == $NumberOfFields) {
		
			#=============================
			# DATE AND TIME  jday  -> $dtrec
			#=============================
			my $dtrec = datesec($dat[3],$dat[4],$dat[5],$dat[6],$dat[7],$dat[8]);
			#test system("date -r$dtrec -u +\"%G/%m/%d (%j) %H:%M:%S, tz%z\"");
			
			#===========================
			# PARSE
			#===========================
			my ($tc, $td, $pir, $piru);
			$tc = Tcal( @casecal, ( $dat[15] + $dat[26] ) /2 , MISSING);
			$td = Tcal( @domecal, ( $dat[16] + $dat[27] ) /2, MISSING );
			#print "tc=$tc,  td=$td\n";
			$piru = ( $dat[14] + $dat[25] ) / 2 * $pircal[0] + $pircal[1];
			($pir) = ComputeLongwave($piru, $tc, $td, @pirparameters, MISSING);
			if ( $dtrec >= $DTSTART )
			{
			#	WE AVERAGE THE COMPONENTS OF EACH HORIZON
				%record = (
					dt => $dtrec,
					shadow => $dat[1],
					thead => $dat[9],
					pitch => ( $dat[10] + $dat[21] ) /2,
					roll => ( $dat[11] + $dat[22] ) /2,
					az => ( $dat[12] + $dat[23] ) / 2,
					psp => ( $dat[13] + $dat[24] ) / 2 * $pspcal[0] + $pspcal[1],
					piru => ( $dat[14] + $dat[25] ) / 2 * $pircal[0] + $pircal[1],
					tcase => $tc,
					tdome => $td,
					pir => $pir,
					refmv => ( $dat[19] + $dat[30] ) /2,
					battmv => ( $dat[20] + $dat[31] ) /2	* $volts2battery				
				);
				
				#======================
				# CHECK ALL VARIABLES FOR BAD VALUES
				#======================
				if ( $record{shadow} < 0 || $record{shadow} > 3000 ) { $record{shadow} = MISSING; }
				if ( $record{thead} < 0 || $record{thead} > 60 ) { $record{thead} = MISSING; }
				if ( $record{pitch} < -30 || $record{pitch} > 30 ) { $record{pitch} = MISSING; }
				if ( $record{roll} < -30 || $record{roll} > 30 ) { $record{roll} = MISSING; }
				if ( $record{az} < 0 || $record{az} > 360 ) { $record{az} = MISSING; }
				if ( $record{psp} < -100 || $record{psp} > 1500 ) { $record{psp} = MISSING; }
				if ( $record{piru} < -500 || $record{piru} > 500 ) { $record{piru} = MISSING; }
				if ( $record{pir} < 0 || $record{pir} > 700 ) { $record{pir} = MISSING; }
				if ( $record{tcase} < -20 || $record{tcase} > 50 ) { $record{tcase} = MISSING; }
				if ( $record{tdome} < -20 || $record{tdome} > 50 ) { $record{tdome} = MISSING; }
				if ( $record{refmv} < 0 || $record{refmv} > 500 ) { $record{refmv} = MISSING; }
				if ( $record{battmv} < 0 || $record{battmv} > 500 ) { $record{battmv} = MISSING; }
				
				return( $YES );  # means we like the data here.
			}
		}
	}
	return ( $NO );
}
