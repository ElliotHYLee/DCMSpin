{

  variables       units
  omega           10000 rad/s
  acc             10000 m/s

}



CON
_clkmode = xtal1 + pll16x                                                    
_xinfreq = 5_000_000

SCALE = 10_000

OBJ
  sensor    : "Tier2MPUMPL_Refinery.spin"
  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
Var
  long acc[3], gyro[3], mag[3], temperature   ' raw values
  long playID, runStack[128]                  ' cog variables
  long R[9], I[3], omega[3], eye[9]           ' DCM variables
  long accSI[3]
  long euler[3], eulerInput[3]               ' Euler angles
  long avgAcc[3], prevAccX[20], prevAccY[20], prevAccZ[20], avgAccInter[3]
  long flag                                  ' flags
  long dt, prev                               ' time variables

{
main is only to run this file
}  
PUB main

  FDS.quickStart  
  
  initSensor(15,14)
  setMpu(%000_00_000, %000_00_000) '250 deg/s, 2g
  startPlay
                  
  repeat
    FDS.clear
    fds.newline
    printDt
    printOmega

    printAcc
    fds.newline
    printEulerInput
    fds.newline
    
    printDCM
    fds.newline
    printEuler
    waitcnt(cnt+clkfreq/10)

PUB initSensor(scl, sda)
  sensor.initSensor(scl, sda)

PUB setMpu(gyroSet, accSet)
  sensor.setMpu(gyroSet, accSet)

PUB stopPlay
  if playID
    cogstop(playID ~ -1)
    
PUB startPlay
  stopPlay
  playID := cognew(playSensor, @runStack) + 1

  
{
playSensor: simulates the autopilot's sensor cog                 
}
PUB playSensor 'see the structure when implementing autopilot
  
  setUpDCM
  
  repeat
    dt := cnt - prev 
    run
    prev := cnt   

PUB getIdentity

  math.getIdentityMatrix(@R)

PUB setUpDCM | counter

  repeat counter from 0 to 2
    I[counter] := 0  
    euler[counter] := 0
    
  repeat 50
    preRun
    'accRaw_to_10000mps
    getAvgAcc

  math.acc2ang(@avgAcc, @eulerInput)
  math.a2d(@R,@eulerInput)
  math.d2a(@R, @euler) 
{
PRI accRaw_to_10000mps | counter

  repeat counter from 0 to 3
    accSI[counter] := ((acc[counter] * SCALE + 16384/2)/ 16384 *981+50)/100

}
PUB getAvgAcc | counter, avgCoef

  avgCoef:= 20

  repeat counter from 0 to (avgCoef-2)
    prevAccX[counter] := prevAccX[counter+1]
    prevAccY[counter] := prevAccY[counter+1]
    prevAccZ[counter] := prevAccZ[counter+1] 
  prevAccX[avgCoef-1] := acc[0]
  prevAccY[avgCoef-1] := acc[1]
  prevAccZ[avgCoef-1] := acc[2]
    
  avgAccInter[0] := 0
  avgAccInter[1] := 0
  avgAccInter[2] := 0
    
  repeat i from 0 to (avgCoef-1)
    avgAccInter[0] += prevAccX[i]/avgCoef 
    avgAccInter[1] += prevAccY[i]/avgCoef
    avgAccInter[2] += prevAccZ[i]/avgCoef

  avgAcc[0] := avgAccInter[0]
  avgAcc[1] := avgAccInter[1]
  avgAcc[2] := avgAccInter[2]

PUB preRun {put this function into a loop for autopilot}
    
  sensor.run
  sensor.getAcc(@acc)
  sensor.getGyro(@gyro)
  sensor.getHeading(@mag)

 
PUB run {put this function into a loop for autopilot}
    
  sensor.run
  sensor.getAcc(@acc)
  sensor.getGyro(@gyro)
  sensor.getHeading(@mag)
  getOmega
  'sensor.getTemp(@temperature)
  'accRaw_to_10000mps
  'calcDCM

  'math.d2a(@R, @euler)
  
{ calcDCM: updates eAngle}

PUB  calcDCM | temp[9]

  
  'math.skew(@temp , omega[0], omega[1] ,omega[2])   

   

PRI getOmega | counter

  repeat counter from 0 to 3
    omega[counter] := gyro[counter]*SCALE/131*31416/10_000/180   '10_000 rad/s
    omega[counter] += I[counter]

















PRI printAcc | counter

  fds.str(String("accX = ")) 
  fds.decln(acc[0])
  'fds.strln(String(" (10000^-1 m/s)"))

  fds.str(String("accY = "))  
  fds.decln(acc[1])
  'fds.strln(String(" (10000^-1 m/s)"))
  
  fds.str(String("accZ = ")) 
  fds.decln(acc[2])
  'fds.strln(String(" (10000^-1 m/s)"))


PRI printEulerInput

  fds.str(String("pitch = "))
  fds.decln(eulerInput[0])

  fds.str(String("roll = "))
  fds.decln(eulerInput[1])

  fds.str(string("yaw = "))
  fds.decln(eulerInput[2])


PRI printEuler

  fds.str(String("pitch = "))
  fds.decln(euler[0])

  fds.str(String("roll = "))
  fds.decln(euler[1])

  fds.str(string("yaw = "))
  fds.decln(euler[2])
  

PRI printAll | counter, j
  repeat counter from 0 to 2
    repeat j from 0 to 2
      if counter==0
        FDS.str(String("Acc["))
        FDS.dec(j)
        FDS.str(String("]=  "))      
        FDS.decLn(acc[j])
        
      if counter==1
        FDS.str(String("Gyro["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.decLn(gyro[j])

      if counter ==2
        FDS.str(String("Mag["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.decLn(mag[j])

        
    'fds.decLn(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2])
  FDS.Str(String("Tempearture = "))
  FDS.decLn(temperature)
  FDS.Str(String("% gForce = "))
'  FDS.decLn(gForce)
PRI printDt

  fds.str(String("dt = "))
  fds.decLn(dt)


  fds.str(String("freq = "))
  fds.dec(80_000_000/dt)
  fds.strLn(String(" Hz"))

   
PRI sendToMatlab

  fds.dec(acc[0])
  fds.str(String(" ")) 
  fds.dec(acc[1]) 
  fds.str(String(" "))
  fds.dec(acc[2])
  fds.str(String(" "))


PRI printBasicInfo

  fds.strLn(String("acc"))
  fds.str(String("X: "))
  fds.dec(acc[0])
  fds.str(String(" Y: "))
  fds.dec(acc[1])
  fds.str(String(" Z: "))
  fds.decLn(acc[2])
  fds.newline
  fds.strLn(String("gyro"))
  fds.str(String("X: "))
  fds.dec(gyro[0])
  fds.str(String(" Y: "))
  fds.dec(gyro[1])
  fds.str(String(" Z: "))
  fds.decLn(gyro[2])
  fds.newline
  fds.str(string("normal of gyro: "))
 ' fds.decLn(gyroNorm)

  fds.newline
  printOmega
  fds.newline
  printDCM
  
  fds.newline
  fds.newline      
  fds.strLn(String("Avg Magnetometer"))  
  fds.str(String("X: "))
  fds.dec(mag[0])
  fds.str(String(" Y: "))
  fds.dec(mag[1])
  fds.str(String(" Z: "))
  fds.decLn(mag[2])
  fds.newline
  fds.str(string("magnitude of magnetometer: "))
  fds.decLn(math.sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]))

  fds.newline


  
  fds.newline  
  fds.str(String("x/y (aTan)"))
  fds.decLn(mag[0]/mag[1])

PRI printOmega

  fds.newline
  fds.strLn(String("omega(10 mili rad/s)"))
  fds.str(String("X: "))
  fds.dec(omega[0])
  fds.str(String(" Y: "))
  fds.dec(omega[1])
  fds.str(String(" Z: "))
  fds.decLn(omega[2])
  fds.newline 


PRI printDCM | iter, digit, counter
  
  fds.str(String("R = (rad*10_000) "))
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

PRI getDigit(input)| ans


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

   