Con

{
        *** Fast Update Rate Demo showing the most Basic Functions.

        Notes:  For some reason, when UpdateForceVals is ran in the driver's MPU-9150_Loop,
                it resets every value to = 0.  If it is called here the all works well, no problem.
}

  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
  
    ' // Accelerometer Settings
    mAFS0 = 0
    mAFS1 = 1
    mAFS2 = 2
    mAFS3 = 3
  
    ' // Gyroscope Settings
    mFS0 = 0
    mFS1 = 1
    mFS2 = 2
    mFS3 = 3
    
    ' // Digital Low Pass Filter Settings
    DLP0 = 0          ' Bandwidth = 260 Hz
    DLP1 = 1          ' Bandwidth = 184 Hz
    DLP2 = 2          ' Bandwidth = 94  Hz
    DLP3 = 3          ' Bandwidth = 44  Hz
    DLP4 = 4          ' Bandwidth = 21  Hz
    DLP5 = 5          ' Bandwidth = 10  Hz
    DLP6 = 6          ' Bandwidth = 5   Hz
    DLP7 = 7          ' Reserved
  
    ' // Current Settings (Passed to MPU-6050.spin)
    mAFS    = mAFS0
    mFS     = mFS0
    mDLP    = DLP3

    I2C_SDA = 14
    I2C_SCL = 15
  
    
Obj
  MPU  :  "MPU-9150_Spin_Basic"
  ser  :  "FullDuplexSerial"

var

  long dt, prev
Pub Start | Calibrate

  ser.start(31,30,0,115200)

  Calibrate := 1                ' 0 = No, 1 = Yes        
    
  'MPU.StartX( I2C_SCL, I2C_SDA, Calibrate )                     ' MPU-9150 Standard Settings   
  MPU.Start(I2C_SCL, I2C_SDA, mAFS, mFS, mDLP, Calibrate)       ' MPU-9150 Gyro & Accel Sensor Data  w/ User Init AFS, FS, & DLP

  repeat    
    ser.tx($00) 
    ' // Raw Values are Standard Decimal
    prev := cnt
    MPU.GetaX
    MPU.GetaY
    MPU.GetaZ
    MPU.GetgX
    MPU.GetgY
    MPU.GetgZ
    MPU.GetmX
    MPU.GetmY
    MPU.GetmZ
    dt := cnt - prev
    ser.str(string("dt: "))
    ser.dec(80_000_000/dt)
    ser.str(string(" Hz"))
    ser.tx(13)  
    ser.str(string("Accel X, Y, Z: "))
    ser.dec(MPU.GetaX)
    ser.str(string(", "))
    ser.dec(MPU.GetaY)
    ser.str(string(", "))
    ser.dec(MPU.GetaZ)
    ser.tx(13)

    ser.str(string("Gyro X, Y, Z: "))   
    ser.dec(MPU.GetgX)
    ser.str(string(", "))    
    ser.dec(MPU.GetgY)
    ser.str(string(", "))
    ser.dec(MPU.GetgZ)
    ser.tx(13)
    
    ser.str(string("Mag X, Y, Z: "))       
    ser.dec(MPU.GetmX)
    ser.str(string(", "))    
    ser.dec(MPU.GetmY)
    ser.str(string(", "))    
    ser.dec(MPU.GetmZ)                          
    ser.tx(13)
   { 
    MPU.UpdateForceVals    ' // Does not update in MPU-9150_Loop like it should  Why???
    
    ' // Rounded Floating Point G-Force, Degrees per Second, and uT
    ser.str(string("Accel X, Y, Z: "))
    ser.dec(MPU.GetAccelX)
    ser.str(string(", "))
    ser.dec(MPU.GetAccelY)
    ser.str(string(", "))
    ser.dec(MPU.GetAccelZ)
    ser.tx(13)

    ser.str(string("Gyro X, Y, Z: "))   
    ser.dec(MPU.GetGyroX)
    ser.str(string(", "))    
    ser.dec(MPU.GetGyroY)
    ser.str(string(", "))
    ser.dec(MPU.GetGyroZ)
    ser.tx(13)
    
    ser.str(string("Mag X, Y, Z: "))       
    ser.dec(MPU.GetMagX)
    ser.str(string(", "))    
    ser.dec(MPU.GetMagY)
    ser.str(string(", "))    
    ser.dec(MPU.GetMagZ)                          
    ser.tx(13)
   
    ser.tx(13)
    }     
    waitcnt(cnt + clkfreq/10)
    