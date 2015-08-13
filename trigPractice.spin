CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"

Var
  long a

PUB main

  a := 10000  'DCM radian = 10000*rad = 0.0169
  a := a*65536/10000   ' input = radian * 65536
  
  fds.quickstart

  repeat
    fds.clear
    fds.str(string("intput = "))
    fds.decln(a)
    fds.str(string("result = "))
    fds.decLn(tr.asin(-16384))
    waitcnt(cnt + clkfreq/10)



    