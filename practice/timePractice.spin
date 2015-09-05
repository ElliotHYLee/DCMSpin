CON
_clkmode = xtal1 + pll16x                                                    
_xinfreq = 5_000_000

OBJ
  FDS    : "FullDuplexSerial.spin"
Var
  long dt, prev                               ' time variables
  long sss[128]
  long cNum



PUB main

  FDS.quickStart  
  cNum := 0

  if cNum
    cogstop(cNum ~ -1)
  
  cNum := cognew(foo1, @sss) + 1

  repeat
    fds.clear
    fds.str(String("dt = "))
    fds.decLn(dt)
    fds.str(String("freq = "))
    fds.dec(80_000_000/dt)
    fds.strLn(String(" Hz"))
    waitcnt(cnt + clkfreq/10)

PUB foo1

  repeat
    foo2 

PUB foo2 
   prev := cnt 
   waitcnt(cnt + 5000)  
   dt := cnt - prev - 536 
    
  
  
   
   