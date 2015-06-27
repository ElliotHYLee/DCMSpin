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

PUB main

  FDS.quickStart  
  
  initSensor(15,14)
  setMpu(%000_11_000, %000_01_000) '2000 deg/s, 4g
  startPlay

  repeat
    FDS.clear
    fds.newline
    'printBasicInfo
    sendToMatlab
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
  
  repeat
    run

PUB run {put this function into a loop for autopilot}
    sensor.run 
    sensor.getCompFilter(@compFilter)
    sensor.getGyro(@gyro)
    sensor.getHeading(@mag)

PRI sendToMatlab

  fds.dec(compFilter[0])
  fds.str(String(" ")) 
  fds.dec(compFilter[1]) 
  fds.str(String(" "))
  fds.dec(compFilter[2])
  fds.str(String(" "))

PRI printBasicInfo| i, j

  fds.strLn(String("Euler Angle"))
  fds.str(String("X: "))
  fds.dec(compFilter[0])
  fds.str(String(" Y: "))
  fds.dec(compFilter[1])
  fds.str(String(" Z: "))
  fds.decLn(compFilter[2])
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