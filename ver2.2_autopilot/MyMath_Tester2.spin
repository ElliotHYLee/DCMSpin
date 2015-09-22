
CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000
  
ObJ

  fds : "FullDuplexSerial"
  math : "MyMath.spin"


VAR

  long R[9], testVec[3],testVec2[3], testVec3[3], testMat[9]


PUB main   | ans, R2[9], i ,R3[9]

  fds.quickStart
  
  R[0] := 7253
  R[1] := -354
  R[2] := -6868
  R[3] := 0
  R[4] := 9986
  R[5] := -514  
  R[6] := 6878
  R[7] := 373
  R[8] := 7243  

  R2[0] := 30
  R2[1] := 20
  R2[2] := -90
  R2[3] := 11
  R2[4] := -84
  R2[5] := 20  
  R2[6] := 13
  R2[7] := 553
  R2[8] := 4658
  
  
  R3[0] := R[0]*R2[0]+R[1]*R2[3]+R[2]*R2[6]
  R3[1] := R[0]*R2[1]+R[1]*R2[4]+R[2]*R2[7]
  R3[2] := R[0]*R2[2]+R[1]*R2[5]+R[2]*R2[8] 
  R3[3] := R[3]*R2[0]+R[4]*R2[3]+R[5]*R2[6]
  R3[4] := R[3]*R2[1]+R[4]*R2[4]+R[5]*R2[7]
  R3[5] := R[3]*R2[2]+R[4]*R2[5]+R[5]*R2[8]
  R3[6] := R[6]*R2[0]+R[7]*R2[3]+R[8]*R2[6]
  R3[7] := R[6]*R2[1]+R[7]*R2[4]+R[8]*R2[7]
  R3[8] := R[6]*R2[2]+R[7]*R2[5]+R[8]*R2[8]

  math.copy(@R3, @R)
  repeat
    fds.clear
    printVector
    printMatrix
    waitcnt(cnt + clkfreq/10)





PRI printVector | iter, digit, counter
  
  fds.newline
  
  repeat iter from 0 to 2
    digit := getDigit(testVec3[iter])  
    counter := 0
    fds.dec(testVec3[iter])
    repeat counter from 0 to (10-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))

  fds.newline   
    
PRI printMatrix | iter, digit, counter
  
  fds.str(String("R = (value*10_000) "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(R[iter])  
    counter := 0
    fds.dec(R[iter])
    repeat counter from 0 to (10-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  

PRI getDigit(input)| ans , flag


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
  elseif(input <10000000)
    ans:= 7
  elseif(input <100000000)
    ans:= 8    
  elseif(input <1000000000)
    ans:= 9
   

  
  if flag ==1
    ans += 1

  return ans