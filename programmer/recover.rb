$: << File.realpath('..', __FILE__)
require 'swd-mchck-bitbang'

$s = MchckBitbangSwd.new(:dev=> ARGV[0])

def t0
  $s.write_swdio 0
  $s.flush!
end

def t1
  $s.write_swdio 1
  $s.flush!
end

def tck0
  $s.write_swdclk 0
  $s.flush!
end

def tck1
  $s.write_swdclk 1
  $s.flush!
end

t0
sleep 0.0007
tck1
sleep 0.0007
t1
sleep 0.0017
t0
sleep 0.0017
t1
sleep 0.0017
t0
sleep 0.0017
t1
sleep 0.0017
t0
sleep 0.0017
t1
sleep 0.0017
t0
sleep 0.0017
t1
sleep 0.0017
t0
sleep 0.0017
t1
sleep 0.0017
t0
sleep 0.0017
t1
sleep 0.0017
tck0
sleep 0.100
