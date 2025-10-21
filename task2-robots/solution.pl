use v5.16;
use warnings;
use IO::Socket ();
use Math::Trig;
use Time::HiRes ('time');

sub connect_tele {
    my $telesrv = IO::Socket->new(
        Domain => IO::Socket::AF_INET,
        Type => IO::Socket::SOCK_STREAM,
        Proto => 'tcp',
        LocalHost => $ENV{'TEL_HOST'} || '0.0.0.0',
        LocalPort => $ENV{'TEL_PORT'} || '5600',
        ReusePort => 1,
        Listen => 1);
    if (!defined($telesrv)) {
        print "Error: $!\n" and die;
    }
    return $telesrv->accept();
}

sub connect_cmd {
    my $cmdskt = IO::Socket->new(
        Domain => IO::Socket::AF_INET,
        Type => IO::Socket::SOCK_DGRAM,
        Proto => 'udp',
        PeerHost => $ENV{'CMD_HOST'} || '127.0.0.1',
        PeerPort => $ENV{'CMD_PORT'} || '5555');
    if (!defined($cmdskt)) {
        print "Error: $!\n" and die;
    }
    return $cmdskt;
}

my $teleskt = connect_tele();
print "connected on telemetry socket, peer: " . $teleskt->peerhost() . ':' . $teleskt->peerport() . "\n";
my $cmdskt = connect_cmd();

sub command {
    my ($v, $da) = @_;
    $cmdskt->send(pack('ff', $v, $da));
}

my %tele;
my @lidar;

sub recvtele {
	my ($len) = @_;
	my $buf = '';
	while (length($buf) < $len) {
	    $teleskt->recv(my $b, $len - length($buf));
		$buf .= $b;
	}
	return $buf;
}

sub telemetry {
    my $buf = recvtele(48);
    my ($sz, $hdr, $x, $y, $th, $vx, $vy, $vth, $wx, $wy, $wz, $n) = unpack("LLf9L", $buf);
	%tele = ('x'=> $x, 'y'=> $y, 'th'=>$th, 'v'=>sqrt($vx**2+$vy**2), 'w'=>$vth, 't'=>time());
    my $bufl = recvtele($n * 4);
    @lidar = unpack("f$n", $bufl);
}

sub move_until {
    my ($v, $da, $cond) = @_;
	my $t0 = time();
	my $dt = 0;
    my $th = $tele{'th'};
    command($v, $da);
    do {
        telemetry();
        my $dth = $tele{'th'} - $th;
		$dt = time() - $t0;
    } until (eval($cond));
}

use constant lidar_step => pi / 180 / 4;

sub angles {
	my $d = 20;
	my @a = ();
	for (my $i = $d; $i < @lidar; $i++) {
		my ($al, $ar) = ($i*lidar_step - pi / 4, ($i-$d) * lidar_step - pi/4);
		my ($xl, $yl) = ($lidar[$i]*sin($al), $lidar[$i]*cos($al));
		my ($xr, $yr) = ($lidar[$i-$d]*sin($ar), $lidar[$i-$d]*cos($ar));
		my $a = atan2($yr-$yl, $xr-$xl);
		$a += pi*2 if $a < 0;
		push @a, $a;
	}
	my @dist = (0) x 128;
	$dist[int($_/pi*2*@dist) % @dist]++ for (@a);
	my $peak = 0;
	for (my $i = 0; $i < @dist; $i++) {
		$peak = $i if ($dist[$i] > $dist[$peak]);
	}
	my $shift = ($peak < @dist/2 ? @dist/2 : 3*@dist/2) - $peak;
	$shift = $shift * pi / 2 / @dist;
	my ($sum, $cnt) = (0, 0);
	for my $a (@a) {
		my $ad = $a/pi*2*@dist;
		my $d1 = ($ad + @dist - $peak) % @dist;
		my $d2 = ($peak + @dist - $ad) % @dist;
		next if ($d1 > 5 && $d2 > 5);
		$sum += (($a + $shift) * 2000000 / pi) % 1000000;
		$cnt++;
	}
	return ($sum / $cnt) * pi / 2000000 - $shift;
}

sub find_break {
	my $dir = shift;
	my $start = 80;
	$start = @lidar-$start-1 if $dir < 0;
	my $diff = angles() / lidar_step;
	$start -= $diff;
	my $cur = $lidar[$start];
	return 1 if ($cur > 0.8);
	for (my $i = 80; $i < 180; $i += abs($dir)) {
		if ($lidar[$start] - $cur > 0.35) {
           my $a = abs(180 - ($start+$diff)) * lidar_step;
		   print "   $a\n";
		   return int((0.25 / tan($a) - 0.15) * 2) + 1;
		}
		$cur = $lidar[$start];
		$start += $dir;
	}
	return 0;
}

sub right_break {
	return find_break 1;
}

sub left_break {
	return find_break -1;
}

sub front_dist {
	my ($sum, $cnt) = (0, 0);
	for (my $i = -7; $i < 7; $i++) {
		my $angle = (pi / 180 / 4) * ($i + 0.5);
		$sum += $lidar[$i + @lidar/2] * cos($angle);
		$cnt++;
	}
	return $sum / $cnt;
}

sub forth {
	my $towall = shift;
	my $left = front_dist();
	my $cells = int($left * 2) - $towall;
	$left -= 0.25 * ($towall * 2 + 1); 
	my $t0 = time();
	while (1) {
		my $sideway = -angles();
		$left = 0 if $left < 0;
		my $v = sqrt(0.3*$left) + 0.05;
		command($v, $sideway);
		last if ($left < 0.03);
		telemetry();
		$left = front_dist() - 0.25 * ($towall * 2 + 1);
	}
	command(0, 0);
	print "FWD$cells took " . (time() - $t0) . " sec\n";
}

sub turnl {
	my $t0 = time();
	move_until(0, 2, 'angles() < -0.5');
	move_until(0, 0.2, 'angles() > -0.1');
	move_until(0, 0.05, 'angles() > -0.03');
	command(0, 0);
	print "turnL took " . (time() - $t0) . " sec\n";
}

sub turnr {
	my $t0 = time();
	move_until(0, -2, 'angles() > 0.5');
	move_until(0, -0.2, 'angles() < 0.1');
	move_until(0, -0.05, 'angles() < 0.03');
	command(0, 0);
	print "turnR took " . (time() - $t0) . " sec\n";
}

$|=1;
telemetry();
while (1) {
	my ($rb, $lb) = (right_break(), left_break());
	print "breaks: R-$rb, L-$lb\n";
	if ($rb == 0 && $lb == 0) {
		turnl();
	} elsif ($rb > 0) {
		forth(int((front_dist() - 0.15) * 2) - $rb);
		turnr();
	} else {
		forth(int((front_dist() - 0.15) * 2) - $lb);
	}
}
sleep(3);
