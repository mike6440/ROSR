#*************************************************************/
sub ReadNextRecord_soes
# CUSTOMIZED FOR SOES PC104 PROCESSOR FORMAT.
# 110201
# Scan FILEIN until end of until a good data record is found.
# Fills global hash variable %record(xx) where
#  xx = (dt, kt, bb2t3, bb2t2, bb2t1, bb1t3, bb1t2, bb1t1, Vref, bb1ap1, bb1bb2ap2, bb2ap3, kttempcase,
#	wintemp, tt8temp, Vpwr, sw1, sw2, pitch, roll, sog, cog, az, pnitemp, lat, lon, sog, var, kttemp )
# 0-7: 10/10/2004,06:54:06.678,$ISAR5,20041010T065414Z,
# 8-16: 55.05,0.1958,0.4216,1.8323,1.8562,1.8556,2.4406,2.4399,2.4420,
# 17-24: 2552,1970,1959,1963,1907,1958,2570,3149,
# 25-30: 1,0, -0.5,  0.4,226.4, 24.5,
# 31-35: 25.769817,-80.14400,-999.0,-999.0,-999.0,
# 36-40: 270.7,306.4,-63373268,   1/4832
{
	my ($str);
	my @dat;
	my $flag = 0;
	my @dt;
	
	while (<FILEIN>){
		chomp($str = $_);  					# remove end-of-line characters
		#printf("%s\n",$str); 				# test		
		#20040711-16:01:03.450,$ISAR5
		#
		if($str =~ /\$ISAR5/ )	{									# identifies a data string
			@dat = split(/[,\/:]/, $str);							# parse the data record
 			#$i=0; for (@dat) { printf "%d %s\n",$i++, $_  } #test
			# 0         20100609T170751Z    pc time
			# 1         $ISAR5    header
			# 2         19700101T000055Z    local time
			# 3         280.05    drum angle
			# 4         0.8756    org
			# 5         0.6215    kt15
			# 6         2.2485    bb23
			# 7         2.2454    bb22
			# 8         2.2448    bb21
			# 9         2.8257    bb13
			# 10        2.8242    bb12
			# 11        2.8243    bb11
			# 12        2495      adc7 vref
			# 13        2309      adc6
			# 14        2305      adc5
			# 15        2321      adc4
			# 16        2251      adc3
			# 17        2320      adc2
			# 18        2284      adc1
			# 19        3594      adc0
			# 20        1         sw1
			# 21        0         sw2
			# 22        1.8       pitch
			# 23        -4.5      roll
			# 24        68.1      pni az
			# 25        155000.0  pni temp
			# 26        -999.000000         lat
			# 27        -999.00000          lon
			# 28        -999.0    sog
			# 29        -999.0    cog
			# 30        -999.0    var
			# 31        292.2     kt15 temp
			# 32        296.7     kt15 ref temp
			# 33        544       
			# 34        3         
			# 35        4801      kt15 sn
					
			if ( $#dat == 35 ) {
				#=============================
				# DATE AND TIME  yy,MM,dd,hh,mm,ss
				#=============================
				my $dtrec;
				#print"Time string = $dat[0]\n";
				$dtrec = dtstr2dt($dat[0]);
				#printf "TIME MARK: %s\n", dtstr($dtrec);
				
				#================
				# SAVE THE PRIOR DRUM AND SWITCH POSITIONS
				#================
				$Lastdrum = $record{drum};
				$sw1_last = $record{sw1};
				$sw2_last = $record{sw2};
				
				%record = (
					dt => $dtrec,		# epoch seconds for this record
					drum => $dat[3],
					org => $dat[4],
					kt => $dat[5],
					bb2t3 => $dat[6],
					bb2t2 => $dat[7],
					bb2t1 => $dat[8],
					bb1t3 => $dat[9],
					bb1t2 => $dat[10],
					bb1t1 => $dat[11],
					Vref => $dat[12]*2/1000,  # v3.3 computing Vref from adc
					bb1ap1 => $dat[13],
					bb1bb2ap2 => $dat[14],
					bb2ap3 => $dat[15],
					kttempcase => $dat[16],
					wintemp => $dat[17],
					tt8temp => $dat[18],
					Vpwr => $dat[19], # v3.3 input power Vpwr
					sw1 => $dat[20],
					sw2 => $dat[21],
					pitch => $dat[22],
					roll => $dat[23],
					az => $dat[24],
					pnitemp => $dat[25],
					lat => $dat[26],
					lon => $dat[27],
					sog => $dat[28] *.51444,	# knots to m/s
					cog => $dat[29],
					var => $dat[30],
					kttemp => $dat[31],
					ktreftemp => $dat[32]
				);
				
				#======================
				# CHECK ALL VARIABLES FOR BAD VALUES
				#======================
				if ( $record{drum} < 0 || $record{drum} > 360 ) { $record{drum} = $missing; }
				if ( $record{org} < 0 || $record{org} > 5 ) { $record{org} = $missing; }
				if ( $record{kt} < 0 || $record{kt} > 5 ) { $record{kt} = $missing; }
				if ( $record{bb2t3} < 0 || $record{bb2t3} > 5 ) { $record{bb2t3} = $missing; }
				if ( $record{bb2t2} < 0 || $record{bb2t2} > 5 ) { $record{bb2t2} = $missing; }
				if ( $record{bb2t1} < 0 || $record{bb2t1} > 5 ) { $record{bb2t1} = $missing; }
				if ( $record{bb1t3} < 0 || $record{bb1t3} > 5 ) { $record{bb1t3} = $missing; }
				if ( $record{bb1t2} < 0 || $record{bb1t2} > 5 ) { $record{bb1t2} = $missing; }
				if ( $record{bb1t1} < 0 || $record{bb1t1} > 5 ) { $record{bb1t1} = $missing; }
				if ( $record{Vref} < 3.0 || $record{Vref} > 6.0 ) { $record{Vref} = $missing; }  #v3.3
				if ( $record{bb1ap1} < 0 || $record{bb1ap1} > 4096 ) { $record{bb1ap1} = $missing; }
					else{$record{bb1ap1} = ysi44006_circuit( $record{bb1ap1}, $Rref2[0], $Vref1, $V12adc, $missing); }
				if ( $record{bb1bb2ap2} < 0 || $record{bb1bb2ap2} > 4096 ) { $record{bb1bb2ap2} = $missing; }
					else{$record{bb1bb2ap2} = ysi44006_circuit( $record{bb1bb2ap2}, $Rref2[1], $Vref1, $V12adc, $missing); }
				if ( $record{bb2ap3} < 0 || $record{bb2ap3} > 4096 ) { $record{bb2ap3} = $missing; }
					else{$record{bb2ap3} = ysi44006_circuit( $record{bb2ap3}, $Rref2[2], $Vref1, $V12adc, $missing);}

				if ( $record{kttempcase} < 0 || $record{kttempcase} > 4096 ) { $record{kttempcase} = $missing; }
					else{$record{kttempcase} = ysi44006_circuit( $record{kttempcase}, $Rref2[3], $Vref1, $V12adc, $missing);}

				if ( $record{wintemp} < 0 || $record{wintemp} > 4096 ) { $record{wintemp} = $missing; }
				else{ $record{wintemp} = RL1005_TempCal( $record{wintemp}, $Rref1, $Vref1, $Vref1, 1,$missing); }

				if ( $record{tt8temp} < 0 || $record{tt8temp} > 4096 ) { $record{tt8temp} = $missing; }
				else{ $record{tt8temp} = RL1005_TempCal( $record{tt8temp}, $Rref1, $Vref1, $Vref1, 1, $missing); }
				if ( $record{Vpwr} < 0 || $record{Vpwr} > 4096 ) { $record{Vpwr} = $missing; }
				if ( $record{sw1} < 0 || $record{sw1} > 1 ) { $record{sw1} = $missing; }
				if ( $record{sw2} < 0 || $record{sw2} > 1 ) { $record{sw2} = $missing; }
				if ( $record{pitch} < -45 || $record{pitch} > 45 ) { $record{pitch} = $missing; }
				if ( $record{roll} < -45 || $record{roll} > 45 ) { $record{roll} = $missing; }
				if ( $record{az} < 0 || $record{az} > 360 ) { $record{az} = $missing; }
				if ( $record{pnitemp} < 0 || $record{pnitemp} > 5000 ) { $record{pnitemp} = $missing; }
				if ( $record{lat} < -90 || $record{lat} > 90 ) { $record{lat} = $missing; }
				if ( $record{lon} < -180 || $record{lon} > 360 ) { $record{lon} = $missing; }
				if ( $record{sog} < 0 || $record{sog} > 40 ) { $record{sog} = $missing; }
				if ( $record{cog} < 0 || $record{cog} > 360 ) { $record{cog} = $missing; }
				if ( $record{var} < -60 || $record{var} > 60 ) { $record{var} = $missing; }
				if ( $record{kttemp} < 250 || $record{kttemp} > 350 ) { $record{kttemp} = $missing; }
				else { $record{kttemp} -= 273.15; }
				
				my $tstr = dtstr($dtrec,'ssv');
				printf KTRAW "%s %8.1f %10.4f\n",$tstr ,$record{drum}, $record{kt};
				printf BBRAW "%s %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n",
					$tstr,$record{bb1t1}, $record{bb1t2}, $record{bb1t3}, 
					$record{bb2t1}, $record{bb2t2}, $record{bb2t3}, $record{Vref};
				printf NAVRAW "%s %10.5f %10.5f %03.0f %6.1f %6.1f %6.1f %6.1f %6.1f\n",
					$tstr, $record{lat}, $record{lon}, $record{cog}, $record{sog}, 
					$record{pitch}, $record{roll}, $record{az}, $record{var};
				printf TEMPRAW "%s %4.0f %4.0f %4.0f %4.0f %4.0f %6.2f %6.1f\n", 
					$tstr, $record{bb1ap1}, $record{bb1bb2ap2}, $record{bb2ap3}, 
					$record{kttempcase}, $record{kttemp}, $record{tt8temp}, $record{pnitemp};
				printf PPTRAW "%s %8.4f %2d %2d\n",
					$tstr, $record{org}, $record{sw1}, $record{sw2}; 
				return( YES );  # means we like the data here.
			}
		}
	}
	#print "Bad record\n";
	return ( NO );
}
