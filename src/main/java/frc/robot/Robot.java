// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Epic libraries
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
/* import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel; */
import com.revrobotics.spark.config.*;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import java.util.Arrays;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  
  /* CREATING VARIABLES FOR DIFFERENT THINGS */
  // Creating 'drive'
  private final DifferentialDrive drive;

  // Creating the Controller called 'controller'
  private final XboxController controller;

  // Creating a timer for the autonomous stage
  private final Timer autoTimer = new Timer();

  // Creating all the different SparkMaxes
  private final SparkMax rightMotorA = new SparkMax(1, SparkMax.MotorType.kBrushed);
  private final SparkMax rightMotorB = new SparkMax(2, SparkMax.MotorType.kBrushed);
  private final SparkMax leftMotorA = new SparkMax(3, SparkMax.MotorType.kBrushed);
  private final SparkMax leftMotorB = new SparkMax(4, SparkMax.MotorType.kBrushed);
  
  // Creating different SparkMaxes but for Climber
  /*private final SparkMax CMotor1 = new SparkMax(6, SparkMax.MotorType.kBrushed);
  private final SparkMax CMotor2 = new SparkMax(7, SparkMax.MotorType.kBrushed); */
  AHRS ahrs; 
 
  /** Called once at the beginning of the robot program. */
  public Robot() {
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.inverted(true);
    rightMotorA.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /* CONFIGURING THE FOLLOWER MOTORS */
    // On right side, motor B follows motor A
    SparkMaxConfig followerConfig1 = new SparkMaxConfig();
    followerConfig1.follow(rightMotorA.getDeviceId(), false);
    rightMotorB.configure(followerConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // Motor 4 follows Motor 2
    SparkMaxConfig followerConfig2 = new SparkMaxConfig();
    followerConfig2.follow(leftMotorA.getDeviceId(), false); 
    leftMotorB.configure(followerConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // Setting up what motors are going to be used in 'drive' control system
    drive = new DifferentialDrive(leftMotorA::set, rightMotorA::set); 

    /* SparkMaxConfig followerConfig3 = new SparkMaxConfig();
    followerConfig3.follow(CMotor1, false);
    CMotor2.configure(followerConfig3, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); */

    // Setting up what port the controller is goining to be in
    controller = new XboxController(0);

    // Sending info to dashboard I think
    SendableRegistry.addChild(drive, rightMotorA);
    SendableRegistry.addChild(drive, rightMotorB);
    // SendableRegistry.addChild(null, CMotor1); 
   
  }
 
  @Override
  public void robotPeriodic() {

    // Gyro stuff
    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
   
  }
  
  @Override
  public void robotInit() {
    // Start the camera stream
    UsbCamera camera = CameraServer.startAutomaticCapture();
    
    // Configuring the camera options for better performance and less lag
    camera.setFPS(15);
    camera.setBrightness(50);
    camera.setResolution(640, 48);
    //DriverStation.reportWarning("The camera has successfully been configured!", false);

    // Initialize the gyroscope
    try {
      ahrs = new AHRS(NavXComType.kMXP_SPI);
      //DriverStation.reportWarning("Gyro initialized successfully!", false);
  } catch (RuntimeException ex) {
      //DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), false);
     
      ahrs = null; 
  }
}

  public void autonomousInit() {
    autoTimer.reset();
    autoTimer.start();
    //DriverStation.reportWarning("The autoTimer has started!", false);

  }
 
  public void autonomousPeriodic() {
    double yaw = ahrs.getYaw();
    double distance = 3;
    double speed = -0.4;
    double time = (distance + 0.66) / 0.565;
    double kp = 1; //Proportional constant for the yaw correction.
    double correction =  kp * -yaw / 180;
    int[] yawList = {85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95};

    // Check if yaw is in the list
    boolean yawInLimit = Arrays.stream(yawList).anyMatch(y -> y == (int) yaw);

    if (!yawInLimit && autoTimer.get() <= time)   {
      drive.arcadeDrive(speed, correction);
      //DriverStation.reportWarning("The turn is being corrected automatically!", false);
    
    } else if (yawInLimit && autoTimer.get() <= time) {
      drive.arcadeDrive(speed, 0);

    } else {
      drive.arcadeDrive(0, 0);
      //DriverStation.reportError("The autonomous code has completed successfully!", false);

    }

  } 
   
  @Override
  public void teleopPeriodic() {
    // Actually running the 'drive' control system    
    double speedDivisor = 1;
    double turnDivisor = 0.75; 
    drive.arcadeDrive(controller.getLeftY() * speedDivisor, controller.getRightX() * turnDivisor);
   
    /*  Running the Climber system
    if (controller.getAButton() == true) {
      CMotor1.set(0.1);
    
    } else if (controller.getBButton() == true) {
      CMotor1.set(-0.1);

    } else {
      CMotor1.set(0);

    } */

  }

}