CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

  CMNSCALE = 10_000

OBJ
  sensor : "Tier3MPUMPL_DCM.spin"
  fds    : "FullDuplexSerial.spin"

VAR

  long stack[128]
  long acc[3], gyro[3], mag[3], euler[3]
  long dcmIsUpdated , dt, prev

PUB main


  fds.quickStart
  sensor.turnOnMPu
  sensor.startDcm

  cognew(runTest, @stack)

  repeat
    fds.clear
    printDt
    printEuler
    fds.newline
    printAcc
    fds.newline
    printGyro

    waitcnt(cnt + clkfreq/10)
  


PUB runTest

repeat
   if (sensor.getDcmStatus>0)
    sensor.getAcc(@acc)
    sensor.getGyro(@gyro)
    sensor.getMag(@mag)
    sensor.getEulerAngles(@euler)
    dt := cnt - prev
    prev := cnt 
    waitcnt(cnt + clkfreq/59)
  


PRI printEuler

  fds.strLn(String("Calculated Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(euler[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(euler[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(euler[2])
  fds.strLn(String("  centi degree"))


PRI printAcc 

  fds.str(String("accX = ")) 
  fds.decln(acc[0])

  fds.str(String("accY = "))  
  fds.decln(acc[1])

  fds.str(String("accZ = ")) 
  fds.decln(acc[2])


PRI printGyro 

  fds.str(String("gyroX = ")) 
  fds.decln(gyro[0])

  fds.str(String("gyroY = "))  
  fds.decln(gyro[1])

  fds.str(String("gyroZ = ")) 
  fds.decln(gyro[2])


PRI printDt
  
  fds.str(String("dt: "))
  fds.decLn(dt)
  fds.str(String("freq: ")) 
  fds.dec(clkfreq/dt)
  fds.strln(String(" Hz"))

  


  