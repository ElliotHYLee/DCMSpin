CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"
  
VAR

  long DCM[9], euler[3],temp3x3[9]

PUB main
fds.quickStart
    
  DCM[0] := 0    
  DCM[3] := 0   
  DCM[6] := 0   
  
  DCM[1] := 0  
  DCM[4] := 0    
  DCM[7] := 0    
  
  DCM[2] := 0     
  DCM[5] := 0    
  DCM[8] := 0   

  euler[0] := -10
  euler[1] := 121
  euler[2] := 0    

  
  repeat
    fds.clear
    a2d(@DCM, @euler)
    printDCM
    
    fds.newline
    
    waitcnt(cnt + clkfreq/10)




{ ================================================= 
  a2d : getting DCM from euler angles
  @Rptr in mili radian
  @eulerPtr in degree*100 , theta, phi, psi <- always this order

  @updates Rptr in 10_000
=================================================== }
PUB a2d(RPtr, eulerPtr)| th, ph, ps, temp

  th := long[eulerPtr][0]
  ph := long[eulerPtr][1]
  ps := long[eulerPtr][2]

                  
  long[RPtr][0] := conv(tr.cosine(th)) * conv(tr.cosine(ps)) / 10000 

  temp := conv(tr.sine(ph))*conv(tr.sine(th))/10000 * conv(tr.cosine(ps))/10000
  long[RPtr][1] :=  temp - conv(tr.cosine(ph))*conv(tr.sine(ps))/10000
  
  temp := conv(tr.cosine(ph))*conv(tr.sine(th))/10000*conv(tr.cosine(ps))/10000
  long[RPtr][2] :=  temp + conv(tr.sine(ph))*conv(tr.sine(ps))/10000

  long[RPtr][3] := conv(tr.cosine(th))*conv(tr.sine(ps))/10000

  temp := conv(tr.sine(ph))*conv(tr.sine(th))/10000*conv(tr.sine(ps))/10000
  long[RPtr][4] := temp + conv(tr.cosine(ph))*conv(tr.cosine(ps))/10000   

  temp := conv(tr.cosine(ph))*conv(tr.sine(th))/10000* conv(tr.sine(ps))/10000
  long[RPtr][5] := temp - conv(tr.sine(ph))*conv(tr.cosine(ps))/10000

  long[RPtr][6] := -conv(tr.sine(th))
  long[RPtr][7] := conv(tr.sine(ph))*conv(tr.cosine(th))/10000
  long[RPtr][8] := conv(tr.cosine(ph))*conv(tr.cosine(th))/10000


{ ================================================= 
  conv : convert 65536 to 10_000 
  @value in 65536 : 1

  @returns value in 10_000
=================================================== }
PUB conv(value)

  result := (value*10_000+65536/2)/65536


























PRI printDCM | iter, digit, counter
  
  fds.str(String("R = (value*10_000) "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(DCM[iter])  
    counter := 0
    fds.dec(DCM[iter])
    repeat counter from 0 to (10-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  

PRI getDigit(input)| ans, flag

  flag := 0

  ans := 0
  if input < 0
    input := -input
    flag := 1

    
  if (input <10)
    ans := 1
  elseif (input <100)
    ans := 2
  elseif (input <1000)
    ans := 3
  elseif (input < 10000)
    ans := 4
  elseif (input < 100000)
    ans := 5
  elseif(input <1000000)
    ans:= 6

  if flag ==1
    ans += 1

  return ans