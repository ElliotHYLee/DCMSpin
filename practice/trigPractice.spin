CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"

Var
  long a

PUB main | flag
  flag :=0
  a := 10000  'DCM radian = 10000*rad = 0.0169
  a := a*65536/10000   ' input = radian * 65536
  a := 9814
  fds.quickstart

  repeat while (flag==0)
    fds.clear
    fds.str(string("intput = "))
    fds.dec(a)
    fds.str(string(" result = "))
    fds.decLn(tr.asin(a))
    'a+=1
    'if a> 65555
    '  flag :=1
    waitcnt(cnt + clkfreq/10)



    