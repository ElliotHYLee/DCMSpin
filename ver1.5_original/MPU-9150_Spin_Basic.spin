CON
{
  *******************************************
  * MPU-9150_Spin_Basic                     *
  * by: Zack Lantz                          *
  *******************************************

  7,198 Longs Free Basic,  5,605 Free Full,  8,185 Max  ("Pub Start" Only)

  5,605 / 8,185 = 0.68478924862553451435552840562004 = 65% of Total EEPROM Free = 35% Usage    <This File>
  7,189 / 8,185 = 0.87941356139279169211973121563836 = 88% of Total EEPROM Free = 12% Usage    <This File>
 
  Notes:   This is a Minimal MPU-9150 Driver.  No Angle or Heading Calculations. 
           If there is a feature here that you need, just copy it over.

           Returns:  Raw Values, G-Force / Degrees per Second / uT, and Zero-Offset Raw 

           It is also a work in progress.  Advanced features currently not supported
           will be added in future releases.

           Some features to be included:

           BMP-085 / BMP-180 Aux Driver via the MPU's i2c.
           MPU Master / Bypass Support.       <Currently only Bypass i2c is supported>
           Digital Motion Processor Support
           FIFO Control (Read / Write)
           Self Test Mode            
           
}
  
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  CLK_FREQ = ((_clkmode - xtal1) >> 6) * _xinfreq               ' system freq as a constant
  MS_001   = CLK_FREQ / 1_000                                   ' ticks in 1ms
  US_001   = CLK_FREQ / 1_000_000                               ' ticks in 1us

  
  ' // MPU-6050 Registers
  SMPLRT_Div      = $19  ' 25
  Config          = $1A  ' 26
  Gyro_Config     = $1B  ' 27
  Accel_Config    = $1C  ' 28

  INT_Pin_Cfg     = $37  ' 55
   
  Accel_XOut_H    = $3B  ' 59
  Accel_XOut_L    = $3C  ' 60
  Accel_YOut_H    = $3D  ' 61
  Accel_YOut_L    = $3E  ' 62
  Accel_ZOut_H    = $3F  ' 63
  Accel_ZOut_L    = $40  ' 64
   
  Temp_Out_H      = $41  ' 65
  Temp_Out_L      = $42  ' 66
   
  Gyro_XOut_H    = $43  ' 67
  Gyro_XOut_L    = $44  ' 68
  Gyro_YOut_H    = $45  ' 69
  Gyro_YOut_L    = $46  ' 70
  Gyro_ZOut_H    = $47  ' 71
  Gyro_ZOut_L    = $48  ' 72

  User_Ctrl         = $6A ' 106
   
  PWR_MGMT_1        = $6B ' 107

  WHO_AM_I          = $75 ' 117
   
   
  ' // *** Reset Value is 0x00 for all registers other than:
  ' //     Register 107: 0x40  (PWR_MGMT_1)
  ' //     Register 117: 0x68  (WHO_AM_I)


  ' // MPU-6050 Accelerometer Divisor
  AFS0 = 16384.0    '  +/- 2  G
  AFS1 = 8192.0     '  +/- 4  G
  AFS2 = 4096.0     '  +/- 8  G
  AFS3 = 2048.0     '  +/- 16 G

  mAFS0 = 0
  mAFS1 = 1
  mAFS2 = 2
  mAFS3 = 3

  ' // MPU-6050 Gyro Divisor
  FS0  = 16.0       '  +/- 250  °/S
  FS1  = 131.0      '  +/- 500  °/S
  FS2  = 32.8       '  +/- 1000 °/S
  FS3  = 16.4       '  +/- 2000 °/S

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

  ' // MPU-9150 Magnetometer Divisor   (uT / LSB)
  mMIN = 0.285
  mTYP = 0.3
  mMAX = 0.315
  
  ' // Current Settings:
  cAccelFS = AFS2
  cGyroFS  = FS0 'FS2
  cMagFS   = mMIN                

  mAFS    = mAFS2
  mFS     = mFS0 'mFS2
  mDLP    = DLP3

  ' // Comp Filter to 16-Bit Value
  Multiplier = 4.096

  ' // Pitch & Roll Calculations
  Alpha = 0.3       ' Low Pass Filter Sensitivity  
  

  I2C_SDA = 14
  I2C_SCL = 15
  
  AddressW = %1101_0000         ' MPU-6050/9150 Write Address
  AddressR = %1101_0001         ' MPU-6050/9150 Read Address
                  
  MagAddrW  = %00011000         ' AK8975 Write Address $0C, $0D, $0E
  MagAddrR  = %00011001         ' AK8975 Read Address  $0C, $0D, $0E  
    
   
Obj
  ser : "FullDuplexSerial" 
  f   : "FloatMath"        ' // Not as EEPROM Space Intensive as FloatString and FME
             

VAR
  ' // Cog Storage
  long Cog, Stack[32]

  ' // I2C Storage
  long i2csda, i2cscl, Ready

  ' // Zero Calibration, Measurement Data, and Status Storage
  long x0, y0, z0, a0, b0, c0, d0, e0, f0, t ', drift 
  long cID, Temp, aX, aY, aZ, gX, gY, gZ, mX, mY, mZ 
  long mID, mInfo, mStatus, Verify_Registers

  ' // Calculated Angles
  long aHdg, Gyro[3], Accel[3], Mag[3]

  ' // MPU-6050 User Set Init Settings
  long GyroFS, AccelFS, PowerMgmt, SampleRate, DLP              


  
Pub TestMPU | MPUcog    

  ser.start(31, 30, 0, 115200)

  ser.str(string("Starting..."))

  i2csda := I2C_SDA
  i2cscl := I2C_SCL

  MPUcog := StartX( I2C_SCL, I2C_SDA, 1 )
        
  repeat
    {
    ser.tx(13)
    ser.str(string("Chip ID: 0x"))
    ser.hex(cID, 2)
    ser.str(string("   "))       
    }
    ' // Temperature
    ser.dec(Temp)                                                                                          
    ser.str(string(", "))
    ser.dec(GetTempC)                                                                                          
    ser.str(string(", "))
    ser.dec(GetTempF)        
    ser.str(string("   "))
   
    ' // Update Accel[#], Gyro[#], and Mag[#]
    UpdateForceVals             

    ' // G-Force, Degrees per Second, and Mag uT
    ser.dec(GetAccelX)                                                                                          
    ser.str(string(", "))
    ser.dec(GetAccelY)                                                                                          
    ser.str(string(", "))
    ser.dec(GetAccelZ)
    ser.str(string("   "))
        
    ser.dec(GetGyroX)                                                                                          
    ser.str(string(", "))
    ser.dec(GetGyroY)                                                                                          
    ser.str(string(", "))
    ser.dec(GetGyroZ)
    ser.str(string("   "))
    
    ser.dec(GetMagX)                                                                                          
    ser.str(string(", "))
    ser.dec(GetMagY)                                                                                          
    ser.str(string(", "))
    ser.dec(GetMagZ)
    ser.str(string("   "))
    
    ser.tx(13)
                           


' // Basic Start with User Accel AFS & Gyro FS & DLP   
PUB Start( SCL, SDA, aFS, gFS, fDLP, doCal ) : Status 

  if aFS == 0
    AccelFS := %00000000  ' ±  2 g
  elseif aFS == 1          
    AccelFS := %00001000  ' ±  4 g
  elseif aFS == 2
    AccelFS := %00010000  ' ±  8 g
  elseif aFS == 3
    AccelFS := %00011000  ' ± 16 g

  if gFS == 0
    GyroFS := %00000000   ' ±  250 ° /s
  elseif gFS == 1
    GyroFS := %00001000   ' ±  500 ° /s
  elseif gFS == 2
    GyroFS := %00010000   ' ± 1000 ° /s
  elseif gFS == 3
    GyroFS := %00011000   ' ± 2000 ° /s

                      '| DLPF_CFG |   Accelerometer    |          Gyroscope          |       
  if fDLP == 0        '             Bw (Hz)  Delay (ms)  Bw (Hz)  Delay (ms)  FS (Khz)
    DLP := %00000000  '      0        260        0         256       0.98        8
  elseif fDLP == 1
    DLP := %00000001  '      1        184       2.0        188       1.9         1
  elseif fDLP == 2
    DLP := %00000010  '      2         94       3.0         98       2.8         1
  elseif fDLP == 3
    DLP := %00000011  '      3         44       4.9         42       4.8         1
  elseif fDLP == 4
    DLP := %00000100  '      4         21       8.5         20       8.3         1
  elseif fDLP == 5
    DLP := %00000101  '      5         10      13.8         10      13.4         1
  elseif fDLP == 6    
    DLP := %00000110  '      6          5      19.0          5      18.6         1
  elseif fDLP == 7
    DLP := %00000111  '      7           RESERVED             RESERVED           8

  PowerMgmt  := %00000001  ' X gyro as clock source
  SampleRate := %00000001  ' 500 Hz
    
  i2csda := SDA
  i2cscl := SCL

  Ready := 0
  Status := Cog := cognew(MPU9150_Loop, @stack) + 1

  if doCal == 1
    CalibrateAccel
    CalibrateGyro
    CalibrateMag

' // Start w/ Full User Init Settings
PUB StartA( SCL, SDA, aFS, gFS, fDLP, PM, doCal ) : Status 

  if aFS == 0
    AccelFS := %00000000
  elseif aFS == 1
    AccelFS := %00001000
  elseif aFS == 2
    AccelFS := %00010000
  elseif aFS == 3
    AccelFS := %00011000

  if gFS == 0
    GyroFS := %00000000
  elseif gFS == 1
    GyroFS := %00001000
  elseif gFS == 2
    GyroFS := %00010000
  elseif gFS == 3
    GyroFS := %00011000

                      '| DLPF_CFG |   Accelerometer    |          Gyroscope          |       
  if fDLP == 0        '             Bw (Hz)  Delay (ms)  Bw (Hz)  Delay (ms)  FS (Khz)
    DLP := %00000000  '      0        260        0         256       0.98        8
  elseif fDLP == 1
    DLP := %00000001  '      1        184       2.0        188       1.9         1
  elseif fDLP == 2
    DLP := %00000010  '      2         94       3.0         98       2.8         1
  elseif fDLP == 3
    DLP := %00000011  '      3         44       4.9         42       4.8         1
  elseif fDLP == 4
    DLP := %00000100  '      4         21       8.5         20       8.3         1
  elseif fDLP == 5
    DLP := %00000101  '      5         10      13.8         10      13.4         1
  elseif fDLP == 6    
    DLP := %00000110  '      6          5      19.0          5      18.6         1
  elseif fDLP == 7
    DLP := %00000111  '      7           RESERVED             RESERVED           8
                      
  if PM == 0
    PowerMgmt := %00000000  ' Internal 8MHz oscillator
  elseif PM == 1
    PowerMgmt := %00000001  ' PLL with X axis gyroscope reference
  elseif PM == 2
    PowerMgmt := %00000010  ' PLL with Y axis gyroscope reference
  elseif PM == 3
    PowerMgmt := %00000011  ' PLL with Z axis gyroscope reference
  elseif PM == 4
    PowerMgmt := %00000100  ' PLL with external 32.768kHz reference
  elseif PM == 5
    PowerMgmt := %00000101  ' PLL with external 19.2MHz referenc
  elseif PM == 6
    PowerMgmt := %00000110  ' Reserved
  elseif PM == 7
    PowerMgmt := %00000111  ' Stops the clock and keeps the timing generator in reset
          
  ' *** Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)  
  ' // 0 threw 255
  SampleRate := %00000001   ' 500 Hz Sample Rate,   %00000000 = 1000 Hz Sample rate

  i2csda := SDA
  i2cscl := SCL

  Ready := 0
  Status := Cog := cognew(MPU9150_Loop, @stack) + 1

  if doCal == 1
    CalibrateAccel
    CalibrateGyro
    CalibrateMag


' // Start Basic, No User Init
PUB StartX( SCL, SDA, doCal ) : Status 

  AccelFS := %000_00_000  ' AFS2
  GyroFS  := %000_00_000  ' FS3

  DLP        := %00000011  ' 40 Hz
  PowerMgmt  := %00000001  ' X gyro as clock source
  SampleRate := %00000001  ' 500 Hz

  i2csda := SDA
  i2cscl := SCL
  
  Status := Cog := cognew(MPU9150_Loop, @stack) + 1

  if doCal == 1
    CalibrateAccel
    CalibrateGyro
    CalibrateMag

  
PUB Stop
''Stops the Cog and the PID controller
  cogstop(cog)

  

'**********************
'   Accessors
'**********************
     
' // Rounded Floating Point G-force, Degrees/sec, uT
PUB GetAccelX          ' Accel X
  return f.fRound(Accel[0])

PUB GetAccelY          ' Accel Y
  return f.fRound(Accel[1])

PUB GetAccelZ          ' Accel Z
  return f.fRound(Accel[2])
            
PUB GetGyroX           ' Gyro X
  return f.fRound(Gyro[0])

PUB GetGyroY           ' Gyro Y
  return f.fRound(Gyro[1])

PUB GetGyroZ           ' Gyro Z
  return f.fRound(Gyro[2])

PUB GetMagX            ' Mag X 
  return f.fRound(Mag[0])

PUB GetMagY            ' Mag Y 
  return f.fRound(Mag[1])

PUB GetMagZ            ' Mag Z 
  return f.fRound(Mag[2])


' // Raw  -  Zero Offset
PUB GetAccelXZero          ' Raw Accel X - Zero Offset
  return aX - x0
 
PUB GetAccelYZero          ' Raw Accel Y - Zero Offset
  return aY - y0

PUB GetAccelZZero          ' Raw Accel Z - Zero Offset
  return aZ - z0
            
PUB GetGyroXZero           ' Raw Gyro X - Zero Offset
  return gX - a0

PUB GetGyroYZero           ' Raw Gyro Y - Zero Offset
  return gY - b0

PUB GetGyroZZero           ' Raw Gyro Z - Zero Offset
  return gZ - c0

PUB GetMagXZero            ' Raw Mag X - Zero Offset
  return mX - d0

PUB GetMagYZero            ' Raw Mag Y - Zero Offset
  return mY - e0

PUB GetMagZZero            ' Raw Mag Z - Zero Offset
  return mZ - f0
                          
  
' // Raw Values
Pub GetaX              ' Accel X Raw
  return aX

Pub GetaY              ' Accel Y Raw
  return aY

Pub GetaZ              ' Accel Z Raw
  return aZ

Pub GetgX              ' Gyro X Raw
  return gX

Pub GetgY              ' Gyro Y Raw
  return gY

Pub GetgZ              ' Gyro Z Raw
  return gZ

Pub GetmX              ' Mag X Raw
  return mX

Pub GetmY              ' Mag Y Raw
  return mY

Pub GetmZ              ' Mag Z Raw
  return mZ

PUB GetTemp            ' Raw Temperature
  return Temp

Pub GetTempF           ' Temp Deg F
  ' // Celsius to Fahrenheit = (°C × 9/5) + 32 = °F
  return (GetTempC * (9 / 5)) + 32

Pub GetTempC | tTmp           ' Temp Deg C
  ' // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 35
  tTmp := 65535 / 125         ' Temperatrue Range is from -40 to +85
  return Temp / tTmp - 35     ' (TempRaw / (65535 / (85 + 40))) - Temp Offset



  
' // Update G-Force, Degrees per Second, and Mag Angle       
Pub UpdateForceVals | faX, faY, faZ, fgX, fgY, fgZ, fmX, fmY, fmZ                   

    ' // Accelerometer
    faX := f.fSub(f.fFloat(aX), 32768.0)  
    faY := f.fSub(f.fFloat(aY), 32768.0)
    faZ := f.fSub(f.fFloat(aZ), 32768.0)  

    ' // Convert Accel Value to G-Force
    ' G-Force = Accel[x] / 8,192  (AFS-1)
    Accel[0] := f.fAdd(f.fDiv(faX, cAccelFS), 8.0)                          
    Accel[1] := f.fAdd(f.fDiv(faY, cAccelFS), 8.0)
    Accel[2] := f.fAdd(f.fDiv(faZ, cAccelFS), 8.0)


    ' // Gyroscope
    fgX := f.fSub(f.fFloat(gX), 32768.0)  
    fgY := f.fSub(f.fFloat(gY), 32768.0)
    fgZ := f.fSub(f.fFloat(gZ), 32768.0)    

    ' // Convert Gyro Value to Degrees per Second
    ' Gyro = Gyro[x] / 16.4       (FS-3)    
    Gyro[0] := f.fAdd(f.fDiv(fgX, cGyroFS), 2048.0)                          
    Gyro[1] := f.fAdd(f.fDiv(fgY, cGyroFS), 2048.0)
    Gyro[2] := f.fAdd(f.fDiv(fgZ, cGyroFS), 2048.0)


    ' // Magnetometer   (uT)   (Full Scale Range = +/- 1200 uT)  (Raw Value / 0.3 uT/LSB) 
    Mag[0] := f.fDiv(f.fFloat(mX), cMagFS)                       
    Mag[1] := f.fDiv(f.fFloat(mY), cMagFS) 
    Mag[2] := f.fDiv(f.fFloat(mZ), cMagFS)

    
' // Main Loop  
PUB MPU9150_Loop  

  SetConfig     

  repeat

    MPUReadValues

    'UpdateForceVals  '*** Makes All Values = 0 ***   Why???  

    'drift := (Temp + 15000) / 100      

  
' // Calibrations Copied from MPU-6050 PASM  
PUB CalibrateAccel | tc, xc, yc, zc, dr

  x0 := 0         ' Initialize offsets
  y0 := 0
  z0 := 0
  
  'wait 1/2 second for the body to stop moving
  waitcnt( constant(80_000_000 / 2) + cnt )

  'Find the zero points of the 3 axis by reading for ~1 sec and averaging the results
  xc := 0
  yc := 0
  zc := 0

  repeat 256
    xc += aX
    yc += aY
    zc += aZ

    waitcnt( constant(80_000_000/192) + cnt )

  'Perform rounding
  if( xc > 0 )
    xc += 128
  elseif( xc < 0 )
    xc -= 128

  if( yc > 0 )
    yc += 128
  elseif( yc < 0 )
    yc -= 128

  if( zc > 0 )
    zc += 128
  elseif( zc < 0 )
    zc -= 128
    
  x0 := xc / 256
  y0 := yc / 256
  z0 := zc / 256
      
PUB CalibrateGyro | tc, xc, yc, zc, dr

  a0 := 0         ' Initialize offsets
  b0 := 0
  c0 := 0
  
  'wait 1/2 second for the body to stop moving
  waitcnt( constant(80_000_000 / 2) + cnt )

  'Find the zero points of the 3 axis by reading for ~1 sec and averaging the results
  xc := 0
  yc := 0
  zc := 0

  repeat 256
    xc += gX
    yc += gY
    zc += gZ

    waitcnt( constant(80_000_000/192) + cnt )

  'Perform rounding
  if( xc > 0 )
    xc += 128
  elseif( xc < 0 )
    xc -= 128

  if( yc > 0 )
    yc += 128
  elseif( yc < 0 )
    yc -= 128

  if( zc > 0 )
    zc += 128
  elseif( zc < 0 )
    zc -= 128
    
  a0 := xc / 256
  b0 := yc / 256
  c0 := zc / 256

PUB CalibrateMag | tc, xc, yc, zc, dr

  d0 := 0         ' Initialize offsets
  e0 := 0
  f0 := 0
  
  'wait 1/2 second for the body to stop moving
  waitcnt( constant(80_000_000 / 2) + cnt )

  'Find the zero points of the 3 axis by reading for ~1 sec and averaging the results
  xc := 0
  yc := 0
  zc := 0

  repeat 256
    xc += mX
    yc += mY
    zc += mZ

    waitcnt( constant(80_000_000/192) + cnt )

  'Perform rounding
  if( xc > 0 )
    xc += 128
  elseif( xc < 0 )
    xc -= 128

  if( yc > 0 )
    yc += 128
  elseif( yc < 0 )
    yc -= 128

  if( zc > 0 )
    zc += 128
  elseif( zc < 0 )
    zc -= 128
    
  d0 := xc / 256
  e0 := yc / 256
  f0 := zc / 256


' // MPU-6050 / MPU-9150 Config
PUB SetConfig
                   
    i2cstart                         ' #i2cReset

    Write_Register(PWR_MGMT_1,   PowerMgmt)   ' 107 - PWR_MGMT_1    
    Write_Register(SMPLRT_DIV,  SampleRate)   ' 25  - SMPLRT_DIV = 1 => 1khz/(1+1) = 500hz sample rate
    Write_Register(CONFIG,             DLP)   ' 26  - Set DLPF_CONFIG to 4 for 20Hz bandwidth     
    Write_Register(Gyro_Config,     GyroFS)   ' 27  - Gyro_Config   
    Write_Register(Accel_Config,   AccelFS)   ' 28  - Accel_Config

    Write_Register(User_Ctrl,    %00000000)   ' 106 - Disable Master Mode 
    Write_Register(INT_Pin_Cfg,  %00000010)   ' 55  - INT_Pin_Cfg  i2c Bypass Enabled  *** Required to Read Mag Via Aux Bus    
                                                                         
    
' // MPU-6050 / MPU-9150 Get Values
PUB MPUReadValues | dr, cFactor
    ' // Start at Register $3B  (Accel, Temp, Gyro) 
    i2cStart                   ' start the transaction  
    i2cWrite(AddressW)         ' 0 for write
    i2cWrite(Accel_XOut_H)     ' send the register to start the read at
    i2cStart
    i2cWrite(AddressR)         ' start the read 
    
    aX  := i2cRead(0)   << 8      
    aX  |= i2cRead(0)    
    aY  := i2cRead(0)   << 8      
    aY  |= i2cRead(0)   
    aZ  := i2cRead(0)   << 8
    aZ  |= i2cRead(0)   
    Temp := i2cRead(0)  << 8  
    Temp |= i2cRead(0)    
    gX  := i2cRead(0)   << 8      
    gX  |= i2cRead(0)   
    gY  := i2cRead(0)   << 8      
    gY  |= i2cRead(0)    
    gZ  := i2cRead(0)   << 8       
    gZ  |= i2cRead(1)
    
    ~~aX
    ~~aY
    ~~aZ
    ~~gX
    ~~gY
    ~~gZ

    ' // Change Register to $75  (Chip ID)
    i2cStart            '\         
    i2cWrite(AddressW)  ' > Address Register Select  
    i2cWrite(WHO_AM_I)  '/   
    i2cStop             '\  StartRead
    i2cStart            '/
    i2cWrite(AddressR)  '\  i2cRead       
    cID := i2cRead(1)   '/       

    ' // Change Address to MagAddr    (Magnetometer)

    ' // Magnetometer (AK8975)  - Set CTRL ($0A) = %00000001 = Single Measurement Mode
    i2cStart            
    i2cWrite(MagAddrW)     ' AK8975 Write Address
    i2cWrite($0A)          ' CTRL Register
    i2cWrite(%00000001)    ' Set config $0A to %00000001 to turn on the device.    
    i2cStop
    
    i2cStart            
    i2cWrite(MagAddrW)     ' AK8975 Write Address
    i2cWrite($00)          ' Address to start reading from
    i2cStop
    i2cStart
    i2cWrite(MagAddrR)     ' AK8975 Read Address
    mID     := i2cRead(1)

    i2cStart            
    i2cWrite(MagAddrW)     ' AK8975 Write Address
    i2cWrite($01)          ' Address to start reading from
    i2cStop
    i2cStart
    i2cWrite(MagAddrR)     ' AK8975 Read Address    
    mInfo   := i2cRead(1)

    i2cStart            
    i2cWrite(MagAddrW)     ' AK8975 Write Address
    i2cWrite($02)          ' Address to start reading from
    i2cStop
    i2cStart
    i2cWrite(MagAddrR)     ' AK8975 Read Address    
    mStatus := i2cRead(1)

    i2cStart            
    i2cWrite(MagAddrW)     ' AK8975 Write Address
    i2cWrite($03)          ' Address to start reading from
    i2cStop
    i2cStart
    i2cWrite(MagAddrR)     ' AK8975 Read Address

    ' // is this right: ????
    mX := i2cRead(0)       ' Mag_X_L
    mX |= i2cRead(0) << 8  ' Mag_X_H
    mY := i2cRead(0)       ' Mag_Y_L
    mY |= i2cRead(0) << 8  ' Mag_Y_H
    mZ := i2cRead(0)       ' Mag_Z_L
    mZ |= i2cRead(1) << 8  ' Mag_Z_H       

    ~~mX
    ~~mY
    ~~mZ

    i2cStop
      
                                               

' // Read MPU Register    
Pri Read_Register(rReg) : rVal | key  
    i2cStart                    ' start the transaction  
    i2cWrite(AddressW)
    i2cWrite(rReg)
    i2cStop
    i2cStart
    i2cWrite(AddressR)          ' start the read 
    rVal := i2cRead(1)          ' read first bit field 0 - 7
    i2cStop 

    key := >| rVal              'encode >| bitfield tonumber
    ' // Method to debug with piezo (from example code)
    if rVal == $FFFF
     {use this to insure that if the Address fails or is unplugged
     that the program does not lock since I2C will be showing $FFFF}    
      return 0
      
    return rVal    
 
' // Write MPU Register    
Pri Write_Register(wReg, wVal)
    i2cStart
    i2cWrite(AddressW)
    i2cWrite(wReg)
    i2cWrite(wVal)
    i2cStop  
      
   

' // Minimal I2C Driver:
con

   i2cack    = 0                                        
   i2cnak    = 1                                           
   i2cxmit   = 0                                               
   i2crecv   = 1                                              
   i2cboot   = 28                                               
   i2ceeprom = $a0                                           

'Var
'  long i2csda, i2cscl
      
Pri i2cstart                                        
                     
   outa[i2cscl]~~                                         
   dira[i2cscl]~~
   outa[i2csda]~~                                         
   dira[i2csda]~~
   outa[i2csda]~                                   
   outa[i2cscl] ~                              

Pri i2cstop

   outa[i2cscl] ~~                              
   outa[i2csda] ~~                              
   dira[i2cscl] ~                                   
   dira[i2csda] ~                                                      

Pri i2cwrite(i2cdata) : ackbit

   ackbit := 0 
   i2cdata <<= 24
   repeat 8                                          
      outa[i2csda] := (i2cdata <-= 1) & 1
      outa[i2cscl] ~~                                                
      outa[i2cscl] ~
   dira[i2csda] ~                                              
   outa[i2cscl] ~~
   ackbit := ina[i2csda]                                      
   outa[i2cscl] ~
   outa[i2csda] ~                                      
   dira[i2csda] ~~

Pri i2cread(ackbit): i2cdata

   i2cdata := 0
   dira[i2csda]~                                    
   repeat 8                                             
      outa[i2cscl] ~~                                         
      i2cdata := (i2cdata << 1) | ina[i2csda]
      outa[i2cscl] ~
   outa[i2csda] := ackbit                               
   dira[i2csda] ~~
   outa[i2cscl] ~~                                                   
   outa[i2cscl] ~
   outa[i2csda] ~                                      

Pri i2creadpage(i2caddr, addrreg, dataptr, count) : ackbit
                                                                              
   i2caddr |= addrreg >> 15 & %1110
   i2cstart
   ackbit := i2cwrite(i2caddr | i2cxmit)
   ackbit := (ackbit << 1) | i2cwrite(addrreg >> 8 & $ff)
   ackbit := (ackbit << 1) | i2cwrite(addrreg & $ff)          
   i2cstart
   ackbit := (ackbit << 1) | i2cwrite(i2caddr | i2crecv)
   repeat count - 1
      byte[dataptr++] := i2cread(i2cack)
   byte[dataptr++] := i2cread(i2cnak)
   i2cstop
   return ackbit

Pri i2cwritepage(i2caddr, addrreg, dataptr, count) : ackbit
                                                                           
   i2caddr |= addrreg >> 15 & %1110
   i2cstart
   ackbit := i2cwrite(i2caddr | i2cxmit)
   ackbit := (ackbit << 1) | i2cwrite(addrreg >> 8 & $ff)
   ackbit := (ackbit << 1) | i2cwrite(addrreg & $ff)          
   repeat count
      ackbit := ackbit << 1 | ackbit & $80000000                             
      ackbit |= i2cwrite(byte[dataptr++])
   i2cstop
   return ackbit