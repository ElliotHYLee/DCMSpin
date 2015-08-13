CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000
CMNSACLE =10_000
OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"
  
VAR

  long DCM[9], euler[3],temp3x3[9], eOut[3]

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

  eOut[0] :=0
  eOut[1] :=0
  eOut[2] :=0
  
  repeat
    fds.clear
    a2d(@DCM, @euler)
    printEuler
    fds.newline
    printDCM
    fds.newline
    d2a(@DCM, @eOut) 
    printEuler2
    
    waitcnt(cnt + clkfreq/10)


{ ================================================= 
  a2d : get first euler angle from accelerometer
  @Rptr in mili radian
  @eulerPtr in degree*100 , theta, phi, psi <- always this order

  @updates Rptr in 10_000
=================================================== }
PUB a2d(RPtr, eulerPtr)| th, ph, ps, temp

  th := long[eulerPtr][0]
  ph := long[eulerPtr][1]
  ps := long[eulerPtr][2]

                  
  long[RPtr][0] := (conv(tr.cosine(th)) * conv(tr.cosine(ps))+CMNSACLE/2) / CMNSACLE 

  temp := ((conv(tr.sine(ph))*conv(tr.sine(th)) +CMNSACLE/2)/10000 * conv(tr.cosine(ps))+CMNSACLE/2)/10000
  long[RPtr][1] :=  temp - (conv(tr.cosine(ph))*conv(tr.sine(ps))+CMNSACLE/2)/10000
  
  temp := ((conv(tr.cosine(ph))*conv(tr.sine(th))+CMNSACLE/2)/10000*conv(tr.cosine(ps))+CMNSACLE/2)/10000
  long[RPtr][2] :=  temp + (conv(tr.sine(ph))*conv(tr.sine(ps))+CMNSACLE/2)/10000

  long[RPtr][3] := (conv(tr.cosine(th))*conv(tr.sine(ps))+CMNSACLE/2)/10000

  temp := ((conv(tr.sine(ph))*conv(tr.sine(th))+CMNSACLE/2)/10000*conv(tr.sine(ps))+CMNSACLE/2)/10000
  long[RPtr][4] := temp + (conv(tr.cosine(ph))*conv(tr.cosine(ps))+CMNSACLE/2)/10000   

  temp :=((conv(tr.cosine(ph))*conv(tr.sine(th))+CMNSACLE/2)/10000* conv(tr.sine(ps))+CMNSACLE/2)/10000
  long[RPtr][5] := temp - (conv(tr.sine(ph))*conv(tr.cosine(ps))+CMNSACLE/2)/10000

  long[RPtr][6] := -conv(tr.sine(th))
  long[RPtr][7] := (conv(tr.sine(ph))*conv(tr.cosine(th))+CMNSACLE/2)/10000
  long[RPtr][8] := (conv(tr.cosine(ph))*conv(tr.cosine(th))+CMNSACLE/2)/10000


{ ================================================= 
  conv : convert 65536 to 10_000 
  @value in 65536 : 1

  @returns value in 10_000
=================================================== }
PUB conv(value)

  result := (value*10_000+65536/2)/65536

{
d2a  : direction cosince matrix to angle
@ad  : DCM pointer (in rad*10_000), matlab convention of data representation
@out : Euler angle pointers in degree*100
}
PUB d2a(RPtr,outPtr) | counter

  'convert centirad to rad*32768
  'origanl, destination, DCM factor, scale factor
  copy(RPtr, @temp3x3, 10_000, 32768) 

'  LONG[out][0] := -tr.asin(LONG[temp][2])                   ' p, roll, psi
'  LONG[out][1] := tr.atan2(LONG[temp][8], LONG[temp][5]) ' q, pitch, theta
'  LONG[out][2] := tr.atan2(LONG[temp][0], LONG[temp][1]) ' r, yaw, phi

                                                    
  LONG[outPtr][0] := -tr.asin(temp3x3[6])*2           ' q, pitch, theta
  LONG[outPtr][1] := tr.atan2(temp3x3[8], temp3x3[7]) ' p, roll, psi  
  LONG[outPtr][2] := tr.atan2(temp3x3[0], temp3x3[3]) ' r, yaw, phi   

PRI copy(oriPtr, desPtr, convention, scale) | counter

  repeat counter from 0 to 8
    long[desPtr][counter] := (long[oriPtr][counter] * scale + convention/2 ) / convention






















PRI printEuler
  fds.strln(String("unit = centi degree"))
  fds.str(String("pitch = "))
  fds.decln(euler[0])

  fds.str(String("roll = "))
  fds.decln(euler[1])

  fds.str(string("yaw = "))
  fds.decln(euler[2])
  
PRI printEuler2
  fds.strln(String("unit = centi degree"))
  fds.str(String("pitch = "))
  fds.decln(eOut[0])

  fds.str(String("roll = "))
  fds.decln(eOut[1])

  fds.str(string("yaw = "))
  fds.decln(eOut[2])
  
 
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