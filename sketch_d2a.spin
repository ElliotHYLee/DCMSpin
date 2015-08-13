CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"
  
VAR

  long DCM[9], Euler[3],temp3x3[9]
  
PUB Main

  fds.quickStart
    
{
  DCM[0] := 11932*2    '0.7283    * 32768
  DCM[3] := 11225*2    '0.6851    * 32768
  DCM[6] := 277 *2    '0.0169     * 32768
  
  DCM[1] := -11220*2   '-0.6848   * 32768
  DCM[4] := 11905*2    '0.7266    * 32768
  DCM[7] := 922*2     '0.0563     * 32768
  
  DCM[2] := 431*2     '0.0263     * 32768
  DCM[5] := -860*2    '-0.0525    * 32768
  DCM[8] := 16356*2    '0.9983    * 32768  
}

  DCM[0] := 7283    '0.7283    * 32768
  DCM[3] := 6851    '0.6851    * 32768
  DCM[6] := 169    '0.0169     * 32768
  
  DCM[1] := -6848   '-0.6848   * 32768
  DCM[4] := 7266    '0.7266    * 32768
  DCM[7] := 563     '0.0563     * 32768
  
  DCM[2] := 263     '0.0263     * 32768
  DCM[5] := -525    '-0.0525    * 32768
  DCM[8] := 9983    '0.9983    * 32768 

 
  repeat
    fds.clear
    printDCM
    d2a(@DCM, @Euler)
    fds.newline
    fds.str(String("q = "))
    fds.decln(Euler[0]) ' q
    fds.str(String("p = "))    
    fds.decln(Euler[1]) ' p
    fds.str(String("r = "))    
    fds.decln(Euler[2]) ' r
    waitcnt(cnt + clkfreq/10)
    
{
d2a  : direction cosince matrix to angle
@ad  : DCM pointer (in rad*10_000), matlab convention of data representation
@out : Euler angle pointers in degree*100
}
PUB d2a(ad,out) | counter

  'convert centirad to rad*32768
  'origanl, destination, DCM factor, scale factor
  copy(ad, @temp3x3, 10_000, 32768) 

'  LONG[out][0] := -tr.asin(LONG[temp][2])                   ' p, roll, psi
'  LONG[out][1] := tr.atan2(LONG[temp][8], LONG[temp][5]) ' q, pitch, theta
'  LONG[out][2] := tr.atan2(LONG[temp][0], LONG[temp][1]) ' r, yaw, phi

                                                    
  LONG[out][0] := -tr.asin(temp3x3[6])*2              ' q, pitch, theta
  LONG[out][1] := tr.atan2(temp3x3[8], temp3x3[7]) ' p, roll, psi  
  LONG[out][2] := tr.atan2(temp3x3[0], temp3x3[3]) ' r, yaw, phi   

PRI copy(oriPtr, desPtr, convention, scale) | counter

  repeat counter from 0 to 8
    long[desPtr][counter] := (long[oriPtr][counter] * scale + convention/2 ) / convention



           
               
PRI printDCM | iter, digit, counter
  
  fds.str(String("R = (rad*10_000) "))
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