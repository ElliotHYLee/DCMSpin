CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

scale = 10_000

OBJ

  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG.spin"
  
VAR

  long DCM[9], euler[3],temp3by3[9] , acc[3],  accSI[3]
  
PUB Main

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

  acc[0] := -26
  acc[1] := -485
  acc[2] := -15884     

  
  repeat
    fds.clear
    printAcc
    
    acc2ang(@acc, @euler)
    printEuler
    
    fds.newline
    
    waitcnt(cnt + clkfreq/10)


{ ================================================= 
  acc2ang: calcualtes euler angle from accelerometer raw

  @accPtr: accelerometer pointer
  @eulerPtr: euler angle pointer

  @updates eulerPtr in degree*100
=================================================== }
PUB acc2ang(accPtr, eulerPtr) | x, y, temp

  temp := long[accPtr][2] * long[accPtr][2]+long[accPtr][1] * long[accPtr][1]
  x := math.sqrt(temp)
  y := long[accPtr][0] 

  long[eulerPtr][0] := tr.atan2(x, y)  ' theta
  long[eulerPtr][1] := tr.atan2(-long[accPtr][2], -long[accPtr][1]) 'phi
  long[eulerPtr][2] := 0

  
PRI printEuler

  fds.str(String("e[0] = theta, pitch [q] = "))
  fds.decLn(euler[0])
  fds.str(String("e[1] = phi, roll [p] = "))
  fds.decLn(euler[1])
  fds.str(String("e[2] = yaw, psi [r] = "))
  fds.decLn(euler[2])


PRI printAcc

  fds.str(String("accX = "))
  fds.decLn(acc[0])
  fds.str(String("accY = "))
  fds.decLn(acc[1])
  fds.str(String("accZ = "))
  fds.decLn(acc[2])

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