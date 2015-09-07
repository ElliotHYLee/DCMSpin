'===============================================================================
'MyMath.Spin :Matrix representation follows Matlab's matrix repesentation
'Made by : Elliot Lee
'Date    : Sep/5/2015
'===============================================================================

{

  variables       units
  omega           10000 rad/s
  DCM             10000 rad      DCM is in radian because DCM carries the information about rotation

}



CON
_clkmode = xtal1 + pll16x                                                    
_xinfreq = 5_000_000

CMNSCALE = 10_000

OBJ
  sensor    : "Tier2MPUMPL_Refinery.spin"
  FDS    : "FullDuplexSerial.spin"
  math   : "MyMath.spin"  'no cog
  tr     : "TRIG"
Var
  long acc[3], gyro[3], mag[3], temperature   ' raw values
  long playID, runStack[128]                  ' cog variables
  long DCM[9], I[3], omega[3], eye[9]          ' DCM variables
  long euler[3], eulerInput[3]               ' Euler angles
  long avgAcc[3], prevAccX[20], prevAccY[20], prevAccZ[20], avgAccInter[3]
  long dtMPU, prevMPU, dtDCM, prevDCM             ' time variables

  byte attIsUpdated, dcmIsUpdated
  
  long dcmCalcID, dcmCalcStack[256]
  long iterStopper, targetMatrix[9],targetMatrix2[9], firstDCM[9], firstMag[3], global_matrix1[9]






{
main is only to run this file
}  
PUB main

  dcmIsUpdated :=0
  
  FDS.quickStart  
  
  turnOnMPU
  startDCMCalc
  
repeat
  if dcmIsUpdated >0
    
    FDS.clear
    fds.newline
    printDt
    printOmega

    printAcc
    fds.newline
    printMag
    fds.newline

    printFirstMag
    fds.newline
    printAvgAcc
    fds.newline 
    printFirstEulerInput
    fds.newline

    printFirstDCM
    fds.newline
    printDCM
    fds.newline
    printTargetMatrix
    fds.newline

        
    printEulerOutput
    fds.newline
    dcmIsUpdated := 0
    waitcnt(cnt+clkfreq/10)

PUB turnOnMPU

  initSensor(15,14)
  setMpu(%000_00_000, %000_00_000) '250 deg/s, 2g
  startPlay


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
  
  repeat
    prevMPU := cnt 
    run   ' run mpu
    dtMPU := cnt - prevMPU     
{======================================================
run : - runs the primary DCM calculation
      - must be called at autopilot main
======================================================}

PUB run

   
  sensor.run
  sensor.getAcc(@acc)
  sensor.getGyro(@gyro)
  sensor.getHeading(@mag)
  attIsUpdated := 1 
  
  



'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
  
{======================================================
setUpDCM : It is called from DCM cog
           - prepares first Euler angles
           - prepares first DCM
           - prepares I, euler valriables

           - must be called at autopilot main
======================================================}
PUB setUpDCM | counter

     
  repeat 50
    'preRun
    getAvgAcc

  repeat counter from 0 to 2
    I[counter] := 0  
    euler[counter] := 0
    firstMag[counter] := mag[counter]

  
  
  math.acc2ang(@avgAcc, @eulerInput)
  math.a2d(@DCM,@eulerInput)
  d2a 
  
{
{======================================================
preRun : run and accelerometer values to calcualte first Euler angles
======================================================} 
PUB preRun 
    
  sensor.run
  sensor.getAcc(@acc)

  }
{======================================================
getAvgAcc : calculates average acceleromete values
======================================================}
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

  
PUB stopDCMCalc
  if dcmCalcID
    cogstop(dcmCalcID ~ -1)
    
PUB startDCMCalc
  stopDCMCalc

  dcmCalcID := cognew(runDCM, @dcmCalcStack) + 1

PUB runDCM  | il 
  iterStopper :=0
  setUpDCM   'this includes preRun

  repeat il from 0 to 8
    firstDCM[il] := DCM[il]
    
  repeat
    if attIsUpdated > 0
      prevDCM := cnt    
      calcDCM
      dtDCM := cnt - prevDCM
      attIsUpdated := 0
      

     
{=====================================================================
getOmega: - get omega and converts to CMNSCALE
          - compensates omega with cumulatative error
=====================================================================}    
PRI getOmega | counter

  repeat counter from 0 to 2
    omega[counter] := gyro[counter]*CMNSCALE/131*314/100/180   '10_000 rad/s
    if (omega[counter] < 70 AND omega[counter] > -70)  ' for now eliminate gyro drift
      omega[counter] := 0 
  'omega[counter] += I[counter]
    
PRI getEye

  eye[0] := 10000
  eye[1] := 0
  eye[2] := 0

  eye[3] := 0
  eye[4] := 10000
  eye[5] := 0

  eye[6] := 0
  eye[7] := 0
  eye[8] := 10000         
    
'DCM primary interation codes start from here

{=====================================================================
calcDCM: updates eAngle
   step 1: get omega
   step 2: make new DCM
   step 3: orthogonalize
   temp 4: compensation
=====================================================================}
PUB calcDCM | err_earth[3]  

  err_earth[0] :=0
  err_earth[1] :=0
  err_earth[2] :=0   

  DCMstep1 'R = R*(eye(3) + skew(omega(i,:))*dt(i));     
  DCMstep2
  DCMstep3_4(@err_earth)
  DCMstep5_6(@err_earth)
  DCMstep7
  DCMstep8
  d2a

  dcmIsUpdated := 1

PRI DCMstep1 | il, temp1[9], temp2[9], temp3[9], temp4[9],temp5[9],  freq 

 'R = R*(eye(3) + skew(omega(i,:))*dt(i));
  repeat il from 0 to 8
    temp1[il] := 0
    temp2[il] := 0
    temp3[il] := 0
    temp4[il] := 0
    temp5[il] := 0

    
  getOmega
  math.skew(@temp1 , omega[0], omega[1] ,omega[2])

  freq := clkfreq / dtMPU
  getEye
  repeat il from 0 to 8
    temp2[il] := temp1[il] / freq
    temp3[il] := eye[il] + temp2[il]
  
  'math.multOp33(@DCM, @temp3, @temp4) ' result = temp3
  
  temp4[0] := DCM[0]*temp3[0]+DCM[1]*temp3[3]+DCM[2]*temp3[6]
  temp4[1] := DCM[0]*temp3[1]+DCM[1]*temp3[4]+DCM[2]*temp3[7]
  temp4[2] := DCM[0]*temp3[2]+DCM[1]*temp3[5]+DCM[2]*temp3[8] 
  temp4[3] := DCM[3]*temp3[0]+DCM[4]*temp3[3]+DCM[5]*temp3[6]
  temp4[4] := DCM[3]*temp3[1]+DCM[4]*temp3[4]+DCM[5]*temp3[7]
  temp4[5] := DCM[3]*temp3[2]+DCM[4]*temp3[5]+DCM[5]*temp3[8]
  temp4[6] := DCM[6]*temp3[0]+DCM[7]*temp3[3]+DCM[8]*temp3[6]
  temp4[7] := DCM[6]*temp3[1]+DCM[7]*temp3[4]+DCM[8]*temp3[7]
  temp4[8] := DCM[6]*temp3[2]+DCM[7]*temp3[5]+DCM[8]*temp3[8]
  
  
  repeat il from 0 to 8
    temp5[il] := temp4[il]/CMNSCALE   
    DCM[il] := temp5[il]

   
PRI DCMstep2  | il , temp1[9],  col1[3], col2[3], col3[3], err_orth, x_orth[3], y_orth[3], z_orth[3], x_norm[3], y_norm[3], z_norm[3], magnitude[3]   

  repeat il from 0 to 2
    col1[il] := DCM[3*il]
    col2[il] := DCM[3*il+1]
    col3[il] := DCM[3*il+2]

  '% calc error (numerical error of orthogonality)
  'err_orth = dot(R(:,1)', R(:,2));
  err_orth := (math.dot31(@col1, @col2))/CMNSCALE
      
  '% calc x_orth
  'x_orth = R(:,1) - err_orth/2*R(:,2);
  '% calc y_orth
  'y_orth = R(:,2) - err_orth/2*R(:,1);

  repeat il from 0 to 2
    x_orth[il] := col1[il] - col2[il]*err_orth/2/CMNSCALE
    y_orth[il] := col2[il] - col1[il]*err_orth/2/CMNSCALE

  '% get z_orth
  'z_orth = cross(x_orth, y_orth);

  z_orth[0] := (x_orth[1]*y_orth[2] - x_orth[2]*y_orth[1])/CMNSCALE
  z_orth[1] := (x_orth[2]*y_orth[0] - x_orth[0]*y_orth[2])/CMNSCALE
  z_orth[2] := (x_orth[0]*y_orth[1] - x_orth[1]*y_orth[0])/CMNSCALE   
  
  '% renormalize
  'x_norm = 0.5*(3-dot(x_orth, x_orth))*x_orth;  
  'y_norm = 0.5*(3-dot(y_orth, y_orth))*y_orth;
  'z_norm = 0.5*(3-dot(z_orth, z_orth))*z_orth;

  magnitude[0] := math.dot31(@x_orth, @x_orth)/CMNSCALE
  magnitude[1] := math.dot31(@y_orth, @y_orth)/CMNSCALE
  magnitude[2] := math.dot31(@z_orth, @z_orth)/CMNSCALE

  repeat il from 0 to 2
    x_norm[il] := (3*x_orth[il] - magnitude[0]*x_orth[il]/CMNSCALE)/2  
    y_norm[il] := (3*y_orth[il] - magnitude[1]*y_orth[il]/CMNSCALE)/2
    z_norm[il] := (3*z_orth[il] - magnitude[2]*z_orth[il]/CMNSCALE)/2

    
  '% update R
  'R = [x_norm y_norm z_norm];   

  repeat il from 0 to 2
'    targetMatrix[il*3] := x_norm[il]
'    targetMatrix[il*3+1] := y_norm[il]
'    targetMatrix[il*3+2] := z_norm[il]
    DCM[il*3] := x_norm[il]
    DCM[il*3+1] := y_norm[il]
    DCM[il*3+2] := z_norm[il]



PUB DCMstep3_4(err_earthPtr)  | il, acc_body[3], acc_earth[3], mag_earth[3], err_acc_earth[3], err_mag_earth[3], g[3], magSize[2] , norm_first_mag_earth[3], norm_mag_earth[3]

  'STEP 3  
  'Acc_earth(i,:) = R*Acc_body(i,:)';
  acc_body[0] := acc[0] * CMNSCALE /100 * 981  /16384
  acc_body[1] := acc[1] * CMNSCALE /100 * 981  /16384
  acc_body[2] := acc[2] * CMNSCALE /100 * 981 /16384

  acc_earth[0] := (DCM[0]*acc_body[0] + DCM[1]*acc_body[1] + DCM[2]*acc_body[2])/CMNSCALE
  acc_earth[1] := (DCM[3]*acc_body[0] + DCM[4]*acc_body[1] + DCM[5]*acc_body[2])/CMNSCALE
  acc_earth[2] := (DCM[6]*acc_body[0] + DCM[7]*acc_body[1] + DCM[8]*acc_body[2])/CMNSCALE  

  'Mag_earth(i,:) = R*Mag_body(i,:)';
  mag_earth[0] := (DCM[0]*mag[0] + DCM[1]*mag[1] + DCM[2]*mag[2])/CMNSCALE 
  mag_earth[1] := (DCM[3]*mag[0] + DCM[4]*mag[1] + DCM[5]*mag[2])/CMNSCALE 
  mag_earth[2] := 0 '(DCM[6]*mag[0] + DCM[7]*mag[1] + DCM[8]*mag[2])'/CMNSCALE 


  'STEP 4
  'err_acc_earth = cross(Acc_earth(i,:), [0 0 g]);  % z axis acc is positive
  g[0] := 0
  g[1] := 0
  g[2] := -98100                        
  err_acc_earth[0] := acc_earth[1]*g[2] /CMNSCALE '- acc_earth[2]*g[1])/CMNSCALE
  err_acc_earth[1] := -acc_earth[0]*g[2]/CMNSCALE   'acc_earth[2]*g[0]/CMNSCALE - acc_earth[0]*g[2]/CMNSCALE
  err_acc_earth[2] := 0'acc_earth[0]*g[1]/CMNSCALE - acc_earth[1]*g[0]/CMNSCALE  

  'Mag_earth(i,:) = [Mag_earth(i,1) Mag_earth(i,2) 0] /norm(Mag_earth(i,:)); 
  magSize[0] := math.sqrt(mag_earth[0]*mag_earth[0] + mag_earth[1]*mag_earth[1] )'+mag_earth[2]*mag_earth[2])
  norm_mag_earth[0] := mag_earth[1] * CMNSCALE / magSize[0]
  norm_mag_earth[1] := mag_earth[2] * CMNSCALE / magSize[0]
  norm_mag_earth[2] := 0

  'err_mag_earth = cross(Mag_earth(i,:),  Mag_earth(1,:)); 
  magSize[1] := math.sqrt(firstMag[0]*firstMag[0] +firstMag[1]*firstMag[1]) ' +first_mag_earth[2]*first_mag_earth[2]/CMNSCALE)
  norm_first_mag_earth[0] := firstMag[0] * CMNSCALE / magSize[1]
  norm_first_mag_earth[1] := firstMag[1] * CMNSCALE / magSize[1]
  norm_first_mag_earth[2] := 0'first_mag[2] * CMNSCALE / magSize[1] 
  
  err_mag_earth[0] := 0'(norm_mag_earth[1]*norm_first_mag_earth[2] - norm_mag_earth[2]*norm_first_mag_earth[1])/CMNSCALE
  err_mag_earth[1] := 0'(norm_mag_earth[2]*norm_first_mag_earth[0] - norm_mag_earth[0]*norm_first_mag_earth[2])/CMNSCALE
  err_mag_earth[2] := (norm_mag_earth[0]*norm_first_mag_earth[1] - norm_mag_earth[1]*norm_first_mag_earth[0])/CMNSCALE 

  
  'Err_earth(i,:) = err_acc_earth + err_mag_earth;
  long[err_earthPtr][0] := err_acc_earth[0]
  long[err_earthPtr][1] := err_acc_earth[1]
  long[err_earthPtr][2] := err_mag_earth[2]

{
  targetMatrix[0] := err_earth[0]
  targetMatrix[3] := err_earth[1]
  targetMatrix[6] := err_earth[2]

  targetMatrix[1] := acc_earth[0]
  targetMatrix[4] := acc_earth[1]
  targetMatrix[7] := acc_earth[2]

  targetMatrix[2] := err_acc_earth[0]
  targetMatrix[5] := err_acc_earth[1]
  targetMatrix[8] := err_acc_earth[2]  

  targetMatrix2[0] := mag[0]
  targetMatrix2[3] := mag[1]
  targetMatrix2[6] := mag[2]

  targetMatrix2[1] := mag_earth[0]
  targetMatrix2[4] := mag_earth[1]
  targetMatrix2[7] := mag_earth[2]

  targetMatrix2[2] := err_mag_earth[0]
  targetMatrix2[5] := err_mag_earth[1]
  targetMatrix2[8] := err_mag_earth[2] 
}

PUB DCMstep5_6(err_earthPtr)  | il, DCMtrans[9], a,b,c , err_body[3], temp1[9]

  a := long[err_earthPtr][0]
  b := long[err_earthPtr][1]
  c := long[err_earthPtr][2]

  DCMTrans[0] := DCM[0]
  DCMTrans[1] := DCM[3]
  DCMTrans[2] := DCM[6]
  DCMTrans[3] := DCM[1]
  DCMTrans[4] := DCM[4]
  DCMTrans[5] := DCM[7]
  DCMTrans[6] := DCM[2]
  DCMTrans[7] := DCM[5]
  DCMTrans[8] := DCM[8]

  'Err_body(i,:) = (R'*Err_earth(i,:)')';
  'err_earth[3], err_body[3] 

'  err_body[0] := (DCMTrans[0]*err_earth[0] + DCMTrans[1]*err_earth[1] + DCMTrans[2]*err_earth[2])/CMNSCALE
'  err_body[1] := (DCMTrans[3]*err_earth[0] + DCMTrans[4]*err_earth[1] + DCMTrans[5]*err_earth[2])/CMNSCALE
'  err_body[2] := (DCMTrans[6]*err_earth[0] + DCMTrans[7]*err_earth[1] + DCMTrans[8]*err_earth[2])/CMNSCALE  

  err_body[0] := (DCMTrans[0]*a + DCMTrans[1]*b + DCMTrans[2]*c)/CMNSCALE
  err_body[1] := (DCMTrans[3]*a + DCMTrans[4]*b + DCMTrans[5]*c)/CMNSCALE
  err_body[2] := (DCMTrans[6]*a + DCMTrans[7]*b + DCMTrans[8]*c)/CMNSCALE  


  global_matrix1[0] := 0  
  global_matrix1[1] := -err_body[2]
  global_matrix1[2] := err_body[1]
  global_matrix1[3] := err_body[2]
  global_matrix1[4] := 0
'  temp1[5] := -err_body[0]
 ' matrix1[6] := -err_body[1]
 ' matrix1[7] := err_body[0]
 ' matrix1[8] := 0


  targetMatrix[0] := a' err_earth[0]
  targetMatrix[3] := b'err_earth[1]
  targetMatrix[6] := c'err_earth[2]

  targetMatrix[1] := err_body[0]
  targetMatrix[4] := err_body[1]
  targetMatrix[7] := err_body[2]  


  targetMatrix[2] := err_body[0]
  targetMatrix[5] := err_body[1]
  targetMatrix[8] := err_body[2] 
  
  repeat il from 0 to 8
    targetmatrix2[il] := global_matrix1[il]     
 {
  freq := clkfreq / dtMPU
  getEye
  repeat il from 0 to 8
    temp2[il] := temp1[il] / freq
    temp3[il] := eye[il] + temp2[il]
  
  'math.multOp33(@DCM, @temp3, @temp4) ' result = temp3
  
  temp4[0] := DCM[0]*temp3[0]+DCM[1]*temp3[3]+DCM[2]*temp3[6]
  temp4[1] := DCM[0]*temp3[1]+DCM[1]*temp3[4]+DCM[2]*temp3[7]
  temp4[2] := DCM[0]*temp3[2]+DCM[1]*temp3[5]+DCM[2]*temp3[8] 
  temp4[3] := DCM[3]*temp3[0]+DCM[4]*temp3[3]+DCM[5]*temp3[6]
  temp4[4] := DCM[3]*temp3[1]+DCM[4]*temp3[4]+DCM[5]*temp3[7]
  temp4[5] := DCM[3]*temp3[2]+DCM[4]*temp3[5]+DCM[5]*temp3[8]
  temp4[6] := DCM[6]*temp3[0]+DCM[7]*temp3[3]+DCM[8]*temp3[6]
  temp4[7] := DCM[6]*temp3[1]+DCM[7]*temp3[4]+DCM[8]*temp3[7]
  temp4[8] := DCM[6]*temp3[2]+DCM[7]*temp3[5]+DCM[8]*temp3[8]
  
  
  repeat il from 0 to 8
    temp5[il] := temp4[il]/CMNSCALE   
    DCM[il] := temp5[il]
}

PUB DCMstep7


PUB DCMstep8



PUB d2a | counter, temp1[9]

  'convert 10_000*value to rad*32768
  'origanl, destination, DCM factor, scale factor
  'copy_scale(RPtr, @temp3x3, 10_000, 32768) 
  repeat counter from 0 to 8
    temp1[counter] := DCM[counter] * 32768 /CMNSCALE
  

                                                    
  euler[0] := -tr.asin(temp1[6]*2)           ' q, pitch, theta
  euler[1] := tr.atan2(temp1[8], temp1[7]) ' p, roll, psi  
  euler[2] := tr.atan2(temp1[0], temp1[3]) ' r, yaw, phi


PRI copy_scale(oriPtr, desPtr, convention, scale) | counter, reg

  repeat counter from 0 to 8
    'reg := long[oriPtr][counter]
    long[desPtr][counter] := (long[oriPtr][counter] * scale / convention)' + getSign(reg)*convention/2 ) / convention



'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================  
'==============================================================================================================
'==============================================================================================================
'==============================================================================================================
' print code start from here

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


PRI printAvgAcc | counter
  fds.strLn(String("First average Accel"))
  fds.str(String("accX = ")) 
  fds.decln(avgAcc[0])
  'fds.strln(String(" (10000^-1 m/s)"))

  fds.str(String("accY = "))  
  fds.decln(avgAcc[1])
  'fds.strln(String(" (10000^-1 m/s)"))
  
  fds.str(String("accZ = ")) 
  fds.decln(avgAcc[2])
  'fds.strln(String(" (10000^-1 m/s)"))

PRI printFirstEulerInput

  fds.strLn(String("First Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(eulerInput[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(eulerInput[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(eulerInput[2])
  fds.strLn(String("  centi degree"))


PRI printEulerOutput


  fds.strLn(String("Calcualted Euler Angles"))

  fds.str(String("pitch = "))
  fds.dec(euler[0])
  fds.strLn(String("  centi degree"))

  
  fds.str(String("roll = "))
  fds.dec(euler[1])
  fds.strLn(String("  centi degree"))

  fds.str(string("yaw = "))
  fds.dec(euler[2])
  fds.strLn(String("  centi degree"))
  

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

  fds.strln(String("MPU time"))
  fds.str(String("dt = "))
  fds.decLn(dtMPU)
  fds.str(String("freq = "))
  fds.dec(80_000_000/dtMPU)
  fds.strLn(String(" Hz"))

  fds.strln(String("DCM time"))   
  fds.str(String("dt = "))
  fds.decLn(dtDCM)
  fds.str(String("freq = "))
  fds.dec(80_000_000/dtDCM)
  fds.strLn(String(" Hz"))  

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


PRI printMag


  fds.str(String("magX = ")) 
  fds.decln(mag[0])
  'fds.strln(String(" (10000^-1 m/s)"))

  fds.str(String("magY = "))  
  fds.decln(mag[1])
  'fds.strln(String(" (10000^-1 m/s)"))
  
  fds.str(String("magZ = ")) 
  fds.decln(mag[2])
  'fds.strln(String(" (10000^-1 m/s)"))

PRI printFirstDCM | iter, digit, counter
  
  fds.str(String("firstDCM "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(firstDCM[iter])  
    counter := 0
    fds.dec(firstDCM[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  


PRI printFirstMag

  fds.strLn(String("First Magnetometer "))
  fds.str(String("magX = ")) 
  fds.decln(firstMag[0])
  'fds.strln(String(" (10000^-1 m/s)"))

  fds.str(String("magY = "))  
  fds.decln(firstMag[1])
  'fds.strln(String(" (10000^-1 m/s)"))
  
  fds.str(String("magZ = ")) 
  fds.decln(firstMag[2])
  'fds.strln(String(" (10000^-1 m/s)"))

PRI printTargetMatrix | iter, digit, counter
  
  fds.str(String("targetMatrix1 "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(targetMatrix[iter])  
    counter := 0
    fds.dec(targetMatrix[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  


  fds.newline
  fds.newline
  fds.str(String("targetMatrix2 "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(targetMatrix2[iter])  
    counter := 0
    fds.dec(targetMatrix2[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  


PRI printDCM | iter, digit, counter
  
  fds.str(String("R = (value*10_000) "))
  fds.newline
  
  repeat iter from 0 to 8
    digit := getDigit(DCM[iter])  
    counter := 0
    fds.dec(DCM[iter])
    repeat counter from 0 to (12-digit)
      fds.str(String(" "))
    if ((iter+1)//3 == 0)
      fds.newline
    else
      fds.str(string(" "))  

PRI getDigit(input)| ans, flag 


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
  else
    ans :=10 

  
  if flag ==1
    ans += 1

  return ans
   