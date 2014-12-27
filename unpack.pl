#!/usr/bin/perl
use strict;
use 5.010;

if ($#ARGV == -1) {
    say "unpack_loader.pl FILENAME [write]";
    exit;
}

my $KEY = q/7C4E0304550509072D2C7B38170D1711/;
my $SCRAMBLE = qq/openssl rc4 -K $KEY/;
my $SCRAMBLE_PARTS = qq/split -b 512 --filter='$SCRAMBLE'/;

my $to_write = [];
my $fn = $ARGV[0];
my $WRITE = $ARGV[1];

open FILE, $fn;
binmode(FILE);

$/ = \0x64;
my ($magic, $len,
    $vers, $var2, $year, $mon, $day, $h, $m, $s, $var1, 
    $c1, $o1, $s1,
    $c2, $o2, $s2,
    $c3, $o3, $s3) = unpack(
        '@0L' .   # magic
        '@4S' .  # length
        '@6L' .   # version
        '@10L' .   # var2
        '@14S' .  # year
        '@16C5' . # date
        '@21L' .   # var1
        'CLC'x3  # parts off size
    , <FILE>);

printf "Version: %x.%02x\n", $vers >> 8, $vers & 0xFF; 
say '=' x 15;
printf "Date: %02d/%02d/%4d %02d:%02d:%02d\n", $day, $mon, $year, $h, $m, $s;; 
say '=' x 15;

printf "%3s %5s %5s %s\n", qw/no off size name/;  
sub getAllParts { my ($c, $o, $s) = @_;  map { getPart($o + $_*$s) } (0..$c-1)  }

getAllParts($c1, $o1, $s1);
getAllParts($c2, $o2, $s2);
getAllParts($c3, $o3, $s3);

sub getPart {
    my $s = shift;
    $/ = \0x39;
    seek(FILE, $s, 0);

    my ($len, $no, $name, $offset, $size) = unpack 
        '@0C' . # len
        '@1L' . # num
        '@5a40' . # name
        '@45LLL', <FILE>; # offset size unknown

    $name =~ s/\x00//g;

    printf "%3d %5x %5x %s\n", $no, $offset, $size, $name;

    push @$to_write, [ $name, $offset, $size ];
}
close FILE;

say '=' x 15;
say;
say 'Script for unpacking';
say '=' x 15;

for my $w (@$to_write) {
    my $str = qq/dd if='$fn' skip=$w->[1] count=$w->[2] bs=1 |\\\n     /;
    $str .= $w->[0] =~ /Flash/ ? $SCRAMBLE_PARTS : $SCRAMBLE;
    $str .= " > $w->[0].bin";
    say $str;

    qx{$str} if ($WRITE);
}

