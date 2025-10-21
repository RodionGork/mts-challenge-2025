use v5.16;
use warnings;
use IO::Socket ();
use Math::Trig;
use Time::HiRes ('time');

sub connect_tele {
    my $telesrv = IO::Socket->new(
        Domain => IO::Socket::AF_INET,
        Type => IO::Socket::SOCK_DGRAM,
        Proto => 'udp',
        LocalHost => $ENV{'TEL_HOST'} || '0.0.0.0',
        LocalPort => $ENV{'TEL_PORT'} || '5600',
        ReusePort => 1);
    if (!defined($telesrv)) {
        print "Error: $!\n" and die;
    }
    return $telesrv;
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
my $cmdskt = connect_cmd();

sub command {
    my ($v, $da) = @_;
    print "CMD: $v, $da\n";
    $cmdskt->send(pack('ff', $v, $da));
}

my ($camw, $camh);
my @cam;

sub telemetry {
    my $addr = $teleskt->recv(my $buf, 1000000);
    my ($magic, $w, $h, $ds, $min, $max) = unpack('A4SSl2f', $buf);
    ($camw, $camh) = ($w, $h);
    @cam = unpack('S' . ($w * $h), substr($buf, 20));
}

sub move_until {
    my ($v, $da, $cond) = @_;
    my $t0 = time();
    my $dt = 0;
    command($v, $da);
    do {
        telemetry();
        $dt = time() - $t0;
    } until (eval($cond));
}

sub view {
    my ($dx, $dy) = @_;
    return $cam[$camw * ($camh - $dy - 1) + $camw/2 + $dx];
}

$|=1;
command(.5, 0);
while (1) {
    telemetry();
    my ($lt, $rt, $fr, $fl, $ff);
    for ($ff = 0; $ff < $camh && view(0, $ff); $ff++) {};
    for ($fr = 0; $fr < $camh && view(int($fr)/3, $fr); $fr++) {};
    #for ($fl = 0; $fl < $camh && $fl < $camw/2 && view(-int($fl), $fl+10); $fl++) {};
    for ($rt = 0; $rt < $camw/2 && view($rt, 15); $rt++) {}
    print "R=$rt FR=$fr FF=$ff\n";
    if ($rt < 12) {
        command($rt < 10 ? 0 : 0.05, 0.3);
    } elsif ($fr < 22) {
        command(0.1, 0.3);
    } elsif ($fr > 25) {
        command(0.1, -0.3);
    } else {
        command($ff > 36 ? .6 : .25, 0);
    }
}
sleep(3);
