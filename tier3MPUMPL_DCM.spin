CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

PERCENT_CONST = 1000

OBJ
  sensor    : "Tier2MPUMPL_Refinery.spin"
  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
Var
  long acc[3], gyro[3], mag[3], temperature, gForce
  long playID, runStack[128]
  long gyroNorm, D[9], DCM[9], E[9], omega[3]
  long elapse, prev

PUB main

  FDS.quickStart  
  
  initSensor(15,14)
  setMpu(%000_00_000, %000_00_000) '250 deg/s, 2g
  startPlay

  repeat
    FDS.clear
    fds.newline
    printAll

    'sendToMatlab
    fds.newline
    fds.dec(elapse*1000000/clkfreq)
    fds.strLn(string(" micro sec ==> limit = 20000 ms (50Hz)"))
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
 
PUB playSensor

  math.getIdentityMatrix(@D)  
  repeat
    prev := cnt
    run
    'calcDCM
    elapse := cnt - prev

PUB run {put this function into a loop for autopilot}
  sensor.run 
  sensor.getAcc(@acc)
  sensor.getGyro(@gyro)
  sensor.getHeading(@mag)
  sensor.getTemp(@temperature)
  getGFroce

PUB getGFroce

  gForce := (math.sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2])* 100 + 8192 )/ 16384 


PUB  calcDCM | dt{updates eAngle}
  'gyro max = 2000 deg/s
  'long max = (2^32)/2 - 1 = 21_4748_3647 ( signed 32 bits ) 
  'long min = -(2^32)/2 = -21_4748_3648   ( signed 32 bits )
  'gyro norm max = sqrt(3*2000^2) = 1200_0000
  dt := 1
  gyroNorm := math.sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2])
  getOmega
  math.skew(@D, omega[0], omega[1] ,omega[2], 1, dt)   


  copyDCM 


PRI printAll | i, j
  repeat i from 0 to 2
    repeat j from 0 to 2
      if i==0
        FDS.str(String("Acc["))
        FDS.dec(j)
        FDS.str(String("]=  "))      
        FDS.decLn(acc[j])
        
        
      if i==1
        FDS.str(String("Gyro["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.decLn(gyro[j])

        
      if i ==2
        FDS.str(String("Mag["))
        FDS.dec(j)
        FDS.str(String("]= "))      
        FDS.decLn(mag[j])

        
    'fds.decLn(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2])
  FDS.Str(String("Tempearture = "))
  FDS.decLn(temperature)
  FDS.Str(String("% gForce = "))
  FDS.decLn(gForce)

PRI getOmega
  '1_0000_0000 > gyroNormMax = 1200_0000, also no overflow should occur
  if ((gyroNorm > 0) OR (gyroNorm < 0)) 
    omega[0] := 1_0000_0000/gyroNorm*gyro[0]/1000     'omega[0] = 0.xxxxx * 100000 <- 5 decimal points
    omega[1] := 1_0000_0000/gyroNorm*gyro[1]/1000     'omega[1] = 0.xxxxx * 100000  
    omega[2] := 1_0000_0000/gyroNorm*gyro[2]/1000     'omega[2] = 0.xxxxx * 100000
  else
    omega[0] := 0
    omega[1] := 0
    omega[2] := 0

PRI sendToMatlab

  fds.dec(acc[0])
  fds.str(String(" ")) 
  fds.dec(acc[1]) 
  fds.str(String(" "))
  fds.dec(acc[2])
  fds.str(String(" "))

PRI printBasicInfo| i, j

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
  fds.decLn(gyroNorm)

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
  fds.strLn(String("omega"))
  fds.str(String("X: "))
  fds.dec(omega[0])
  fds.str(String(" Y: "))
  fds.dec(omega[1])
  fds.str(String(" Z: "))
  fds.decLn(omega[2])
  fds.newline 


PRI printDCM | i

  
  fds.str(String("R = "))
  fds.newline
  repeat i from 0 to 8
    fds.dec(D[i])
    if ((i+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  

PRI copyDCM | i

  repeat i from 0 to 8
    DCM[i] := D[i]