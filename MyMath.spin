{
Matrix representation follows Matlab's matrix repesentation

}

CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000
CMNSACLE = 10_000
ObJ

  fds : "FullDuplexSerial"
  tr     : "TRIG.spin"


VAR

  long R[9], testAcc[3], temp3x3[9]


PUB main

  fds.quickStart

  testAcc[0] := -15
  testAcc[1] := -290
  testAcc[2] := -9507
  
  repeat
    
    fds.clear
   ' fds.decLn(getFirstDCM(@R, @testAcc))
    waitcnt(cnt + clkfreq/10)

PUB sqrt(value)| x, i

  x := value

  repeat i from 0 to 20
    x := (value/x + x) /2

  return x


  
PUB skew(Dptr, a, b, c)

  long[Dptr][1] := a
  long[Dptr][2] := b
  long[Dptr][3] := -a
  long[Dptr][5] := c
  long[Dptr][6] := -b  
  long[Dptr][7] := -c

                                
PUB getIdentityMatrix(E) { 10^4 = unity to represent 1.xxxx}

  long[E][0] := 1000
  long[E][1] := 0
  long[E][2] := 0

  long[E][3] := 0
  long[E][4] := 1000
  long[E][5] := 0

  long[E][6] := 0
  long[E][7] := 0
  long[E][8] := 1000  
  
PUB getSign(value)

  if value >= 0
    result := 1
  else
    result := -1

PUB getAbs(value)
  if value > 0
    result := value
  else
    result := -value



PUB scalarMultOp(matPtr, k) | i

  repeat i from 0 to 9
    long[matPtr][i] *= k


' 3x3 Matrix Operations
PUB multOp(matAPtr, matBPtr, matCPtr)| i,j, rowCheck, colCheck

  rowCheck := 0
  colCheck := 0

  i:= 0
  j:= 0
  
  repeat 3
    repeat 3  
      repeat 3
        long[matCPtr][i] += long[matAPtr][rowCheck+j]*long[matBPtr][colCheck + 3*j] 
        j++
      i++
      j:=0        
      colCheck++  
    rowCheck+=3 
    colCheck :=0
            
  return
  
PUB addOp(matAPtr, matBPtr, matCPtr) | i

  i := 0
  repeat i from 0 to 8
    long[matCPtr][i] := long[matAPtr][i] + long[matBPtr][i]
  return


PUB subOp(matAPtr, matBPtr, matCPtr) | i

  i := 0
  repeat i from 0 to 8
    long[matCPtr][i] := long[matAPtr][i] - long[matBPtr][i]
  return

PUB transposeOp(matAPtr, matResultPtr) | i, j,k

  i :=0
  j :=0
  k :=0

  repeat i from 0 to 8
    long[matResultPtr][i] := long[matAPtr][3*k + j]
    k++
    if ((i+1)//3 ==0)
      j++
      k :=0


PUB detOp(matAPtr) : det

  det := long[matAPtr][0]*(long[matAPtr][4]*long[matAPtr][8]-long[matAPtr][7]*long[matAPtr][5])-long[matAPtr][1]*(long[matAPtr][3]*long[matAPtr][8]-long[matAPtr][6]*long[matAPtr][5]) + long[matAPtr][2]*(long[matAPtr][3]*long[matAPtr][7]-long[matAPtr][6]*long[matAPtr][4])

PUB invOp(matAPtr, matResultPtr)| det, i

  det := detOp(matAPtr)
  if (det ==0)
    return -1

  long[matResultPtr][0] := long[matAPtr][4]*long[matAPtr][8]-long[matAPtr][5]*long[matAPtr][7]
  long[matResultPtr][1] := long[matAPtr][2]*long[matAPtr][7]-long[matAPtr][1]*long[matAPtr][8] 
  long[matResultPtr][2] := long[matAPtr][1]*long[matAPtr][5]-long[matAPtr][2]*long[matAPtr][4] 
  
  long[matResultPtr][3] := long[matAPtr][5]*long[matAPtr][6]-long[matAPtr][3]*long[matAPtr][8]
  long[matResultPtr][4] := long[matAPtr][0]*long[matAPtr][8]-long[matAPtr][2]*long[matAPtr][6]
  long[matResultPtr][5] := long[matAPtr][2]*long[matAPtr][3]-long[matAPtr][0]*long[matAPtr][5]
  
  long[matResultPtr][6] := long[matAPtr][3]*long[matAPtr][7]-long[matAPtr][4]*long[matAPtr][6]
  long[matResultPtr][7] := long[matAPtr][1]*long[matAPtr][6]-long[matAPtr][0]*long[matAPtr][7]
  long[matResultPtr][8] := long[matAPtr][0]*long[matAPtr][4]-long[matAPtr][1]*long[matAPtr][3]

  repeat i from 0 to 8 'rounding up for final result
    if long[matResultPtr][i] > 0
      long[matResultPtr][i] := (long[matResultPtr][i] + getAbs(det)/2) / det
    else
      long[matResultPtr][i] := (long[matResultPtr][i] - getAbs(det)/2) /det



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



PUB acc2ang(accPtr, eulerPtr) | x, y, temp

  temp := long[accPtr][2] * long[accPtr][2]+long[accPtr][1] * long[accPtr][1]
  x := sqrt(temp)
  y := long[accPtr][0] 

  long[eulerPtr][0] := tr.atan2(x, y)  ' theta
  long[eulerPtr][1] := tr.atan2(-long[accPtr][2], -long[accPtr][1])
  long[eulerPtr][2] := 0
 