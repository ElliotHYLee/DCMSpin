CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000
CMNSACLE =10_000
OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"
  
VAR

  long DCM[9], euler[3],temp3x3[9], eOut[3], i, j, k
  long error[3], maxE[3]

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


  maxE[0] :=0
  maxE[1] :=0
  maxE[2] :=0
  
  euler[0] := 13000
  euler[1] := 13000
  euler[2] := 13000    

  eOut[0] :=0
  eOut[1] :=0
  eOut[2] :=0
  
    repeat i from -8900 to 8900
      repeat j from -8900 to 8900
        repeat k from -8900 to 8900
          euler[0] := i
          euler[1] := j
          euler[2] := k
          k+=1000
            
         
          fds.clear
          a2d(@DCM, @euler)
         ' printEuler
          'fds.newline
          'printDCM
          'fds.newline
          d2a(@DCM, @eOut) 
          'printEuler2

          'fds.newline
          
          error[0] := (euler[0] - eOut[0])'*100/euler[0]
          error[1] := (euler[1] - eOut[1])'*100/euler[1]
          error[2] := (euler[2] - eOut[2])'*100/euler[2]

          if error[0] > maxE[0]
            maxE[0] := error[0]
          if error[1] > maxE[1]
            maxE[1] := error[1]
          if error[2] > maxE[2]
            maxE[2] := error[2]

          printEuler
          fds.newline
          printMaxError 
          
          if (error[0] > 100) OR (error[1] > 100) OR (error[2] > 100)
            printEuler
            fds.newline
            printDCM
            fds.newline
            printEuler2
            fds.newline
            printError
            
            'waitcnt(cnt +clkfreq*5)
          
          
          waitcnt(cnt + clkfreq/10)
        j+=1000
      i+=1000

{ ================================================= 
  a2d : getting DCM from euler angles
  @Rptr in value * 10_000
  @eulerPtr in degree*100 , theta, phi, psi <- always this order

  @updates Rptr in 10_000
=================================================== }
PUB a2d(RPtr, eulerPtr)| th, ph, ps, temp

  th := long[eulerPtr][0]
  ph := long[eulerPtr][1]
  ps := long[eulerPtr][2]

                  
  long[RPtr][0] := (conv(tr.cosine(th)) * conv(tr.cosine(ps))+CMNSACLE/2) / CMNSACLE 

  temp := ((conv(tr.sine(ph))*conv(tr.sine(th)) +CMNSACLE/2)/10000 * conv(tr.cosine(ps))+CMNSACLE/2)/CMNSACLE
  long[RPtr][1] :=  temp - (conv(tr.cosine(ph))*conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE
  
  temp := ((conv(tr.cosine(ph))*conv(tr.sine(th))+CMNSACLE/2)/10000*conv(tr.cosine(ps))+CMNSACLE/2)/CMNSACLE
  long[RPtr][2] :=  temp + (conv(tr.sine(ph))*conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE

  long[RPtr][3] := (conv(tr.cosine(th))*conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE

  temp := ((conv(tr.sine(ph))*conv(tr.sine(th))+CMNSACLE/2)/CMNSACLE*conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE
  long[RPtr][4] := temp + (conv(tr.cosine(ph))*conv(tr.cosine(ps))+CMNSACLE/2)/CMNSACLE   

  temp :=((conv(tr.cosine(ph))*conv(tr.sine(th))+CMNSACLE/2)/CMNSACLE* conv(tr.sine(ps))+CMNSACLE/2)/CMNSACLE
  long[RPtr][5] := temp - (conv(tr.sine(ph))*conv(tr.cosine(ps))+CMNSACLE/2)/CMNSACLE

  long[RPtr][6] := -conv(tr.sine(th))
  long[RPtr][7] := (conv(tr.sine(ph))*conv(tr.cosine(th))+CMNSACLE/2)/CMNSACLE
  long[RPtr][8] := (conv(tr.cosine(ph))*conv(tr.cosine(th))+CMNSACLE/2)/CMNSACLE


{ ================================================= 
  conv : convert 65536 to 10_000 
  @value: it is in 65536 : 1

  @returns value in 10_000
=================================================== }
PUB conv(value)

  result := (value*10_000 + 65536/2)/65536

{
d2a  : direction cosince matrix to angle
@ad  : DCM pointer (in value*10_000), matlab convention of data representation
@out : Euler angle pointers in degree*100
}
PUB d2a(RPtr,outPtr) | counter

  'convert 10_000*value to rad*32768
  'origanl, destination, DCM factor, scale factor
  copy(RPtr, @temp3x3, 10_000, 32768) 

'  LONG[out][0] := -tr.asin(LONG[temp][2])                   ' p, roll, psi
'  LONG[out][1] := tr.atan2(LONG[temp][8], LONG[temp][5]) ' q, pitch, theta
'  LONG[out][2] := tr.atan2(LONG[temp][0], LONG[temp][1]) ' r, yaw, phi

                                                    
  LONG[outPtr][0] := -tr.asin(temp3x3[6]*2)           ' q, pitch, theta
  LONG[outPtr][1] := tr.atan2(temp3x3[8], temp3x3[7]) ' p, roll, psi  
  LONG[outPtr][2] := tr.atan2(temp3x3[0], temp3x3[3]) ' r, yaw, phi   

PRI copy(oriPtr, desPtr, convention, scale) | counter

  repeat counter from 0 to 8
    long[desPtr][counter] := (long[oriPtr][counter] * scale + convention/2 ) / convention
















PRI printMaxError
  fds.str(String("maxERR_pitch = "))
  fds.decln(maxE[0])
  fds.str(String("max ERR_roll = "))
  fds.decln(maxE[1])
  fds.str(String("max ERR_yaw = "))
  fds.decln(maxE[2])
PRI printError

  fds.str(String("ERR_pitch = "))
  fds.decln(error[0])
  fds.str(String("ERR_roll = "))
  fds.decln(error[1])
  fds.str(String("ERR_yaw = "))
  fds.decln(error[2])

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