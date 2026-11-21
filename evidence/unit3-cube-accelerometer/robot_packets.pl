use strict;
use warnings;
alarm(85);
$|=1;
socket(my $sock,1,1,0) or die "socket: $!";
connect($sock,pack("S a108",1,"/data/misc/bluetooth/abtd.socket")) or die "connect: $!";
print STDERR "Daemon monitor connected\n";
sub exact {
 my($n)=@_; my $buf='';
 while(length($buf)<$n){my $got=sysread($sock,my $part,$n-length($buf)); die "socket ended" unless $got; $buf.=$part;}
 return $buf;
}
while (1) {
 my($magic,$v,$type,$len)=unpack("a4 V V V",exact(16));
 die "invalid header" unless $magic eq "ipcf" && $v==1 && $len<=4096;
 my $data=exact($len);
 next unless $type==5 && $len>=45;
 my($id,$uuid,$n)=unpack("V Z37 V",$data);
 next unless lc($uuid) eq "43ef14af-5fb1-7b81-3647-2a9477824cab" && $n==$len-45;
 open(my $up,'<','/proc/uptime') or die; my($clock)=split(/ /,<$up>); close($up);
 print "$clock ",unpack("H*",substr($data,45)),"\n";
}
