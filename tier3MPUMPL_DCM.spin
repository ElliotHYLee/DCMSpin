CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

PERCENT_CONST = 1000

OBJ
  sensor    : "Tier2MPUMPL_CF.spin"
  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
Var
  long compFilter[3], gyro[3], mag[3]
  long playID, runStack[128]
  long gyroNorm, D[9], DCM[9], E[9], omega[3] 

PUB main

  FDS.quickStart  
  
  initSensor(15,14)
  setMpu(%000_11_000, %000_01_000) '2000 deg/s, 4g
  startPlay

  repeat
    FDS.clear
    fds.newline
    printBasicInfo

    'sendToMatlab
    fds.newline
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
    run
    calcDCM

PUB run {put this function into a loop for autopilot}
    sensor.run 
    sensor.getCompFilter(@compFilter)
    sensor.getGyro(@gyro)
    sensor.getHeading(@mag)
    calcDCM


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

  fds.dec(compFilter[0])
  fds.str(String(" ")) 
  fds.dec(compFilter[1]) 
  fds.str(String(" "))
  fds.dec(compFilter[2])
  fds.str(String(" "))

PRI printBasicInfo| i, j

  fds.strLn(String("compFilter"))
  fds.str(String("X: "))
  fds.dec(compFilter[0])
  fds.str(String(" Y: "))
  fds.dec(compFilter[1])
  fds.str(String(" Z: "))
  fds.decLn(compFilter[2])
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