// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
 
 
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
 
  ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private Field2d m_field = new Field2d();

  
 
  TalonSRX rearRightShoot = new TalonSRX(1);
  VictorSPX frontRightShoot = new VictorSPX(4);
  TalonSRX rearLeftShoot = new TalonSRX(2);
  VictorSPX frontLeftShoot = new VictorSPX(5);
 //spark orginal
  // Spark frontLeft = new Spark(4);
  // Spark rearLeft = new Spark(3);
  // Spark frontRight = new Spark(1);
  // Spark rearRight = new Spark(2);
  
  
  //field
  // private final Field2d m_field = new Field2d();

  //spark max

  private CANSparkMax frontLeftMotor1;
  private CANSparkMax frontLeftMotor2;
  private CANSparkMax frontLeftMotor3;
  private CANSparkMax frontLeftMotor4;
  private CANSparkMax frontLeftMotor5;
  private CANSparkMax frontLeftMotor6;
  private CANSparkMax frontLeftMotor7;
  private CANSparkMax frontLeftMotor8;



  private static final int M_frontLeftID = 1;
  private static final int R_frontLeftID = 2;

  private static final int M_frontReftID = 3;
  private static final int R_frontReftID = 4;

  private static final int M_backReftID = 5;
  private static final int R_backReftID = 6;

  private static final int M_backLeftID = 7;
  private static final int R_backLeftID = 8;



  

 

 
  Joystick stick = new Joystick(0);
 
  public boolean shooting = false;
  public double chargeTime = 1.5;
  public double speed = 0.8;
  Timer timer = new Timer();
  
  
  
  
  
  // private SpeedControllerGroup left, right;
  // private DifferentialDrive drive;
  String trajectoryJSON = "paths/C:\\Users\\chuih\\Desktop\\2024 FRC ROBOT\\SR 2024-Imported\\PathWeaver\\Paths.wpilib.json";
  Trajectory trajectory = new Trajectory();


  
  NetworkTable limelightTable;
  // private Field2d field2d;
 
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    rearRightShoot.setInverted(true);
    rearLeftShoot.setInverted(false);
    frontRightShoot.setInverted(true);
    frontLeftShoot.setInverted(false);
    

    //gyro code
    m_gyro.reset();



    //sparkmax swever
    frontLeftMotor1 = new CANSparkMax(M_frontLeftID, MotorType.kBrushless); 
    frontLeftMotor2 = new CANSparkMax(R_frontLeftID, MotorType.kBrushless);
    
    frontLeftMotor3 = new CANSparkMax(M_frontReftID, MotorType.kBrushless); 
    frontLeftMotor4 = new CANSparkMax(R_frontReftID, MotorType.kBrushless);

    frontLeftMotor5 = new CANSparkMax(M_backReftID, MotorType.kBrushless); 
    frontLeftMotor6 = new CANSparkMax(R_backReftID, MotorType.kBrushless);

    frontLeftMotor7 = new CANSparkMax(M_backLeftID, MotorType.kBrushless); 
    frontLeftMotor8 = new CANSparkMax(R_backLeftID, MotorType.kBrushless);
    
    
    
    
    
    
    // 예를 들어, 2번 CAN ID에 연결된 Spark MAX 모터 (브러시리스)

    //limelight code 
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
   
    SmartDashboard.putData("Field", m_field);

    //do not touch this
    // edu.wpi.first.math.trajectory.Trajectory m_trajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
    //         new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    Field2d field2d;
    // Create and push Field2d to SmartDashboard.
   

    // Push the trajectory to Field2d.
    // m_field.getObject("traj").setTrajectory(m_trajectory);

    

    


    
    
  }
 
  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }
 
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
 
  @Override
  public void disabledPeriodic() {}
 
  
   @Override
   public void autonomousInit() {
  //   m_autonomousCommand = m_robotContainer.getAutonomousCommand();
 
  //   // schedule the autonomous command (example)
  //   if (m_autonomousCommand != null) {
  //     m_autonomousCommand.schedule();
  //   }
  //   //frontRightShoot.set(ControlMode.PercentOutput, -1);
  //   //frontLeftShoot.set(ControlMode.PercentOutput, -0.5);
  //   //rearRightShoot.set(ControlMode.PercentOutput, -0.5);  
  //   //rearLeftShoot.set(ControlMode.PercentOutput, -0.5);

  //    // Reset timer and start rotating
  //    timer.reset();
  //    timer.start();
  //    drive.arcadeDrive(0.5, 1.0); // Adjust rotation speed as needed
   }
 
  // /** This function is called periodically during autonomous. */
  // @Override
  // public void autonomousPeriodic() {
  
  //       if (limelightTable.getEntry("tv").getDouble(0) == 1 && limelightTable.getEntry("tlong").getDouble(0) < 3000) {
  //           // 회전을 멈춥니다
  //           drive.stopMotor();
            
  //           // AprilTag와의 거리를 가져옵니다
  //           double distance = limelightTable.getEntry("tlong").getDouble(0);
    
  //           // 만약 AprilTag와의 거리가 2m보다 작으면 로봇을 멈춥니다
  //           // 이후에 필요한 경우 움직임을 추가하실 수 있습니다
  //           if (distance < 2000) {
  //               drive.stopMotor();
  //               // 여기에 추가 동작을 입력하세요
  //           } else {
  //               // AprilTag와의 거리가 2m 이상이면 로봇을 전진합니다 (예시: 속도 0.5)
  //               drive.arcadeDrive(0.5, 0); // 전진 속도를 필요에 따라 조절하세요
  //           }
  //       } else {
  //           // AprilTag가 감지되지 않은 경우 계속 회전합니다
  //           drive.arcadeDrive(0.5, 1.0); // 회전 속도를 필요에 따라 조절하세요
  //       }
    
 
  // }
 
  /** This function is called periodically during operator control. */
 
  // public void shoot(){
  //   if(stick.getRawButton(8) && shooting == false){
  //     timer.reset();
  //     timer.start();
  //     frontRight.set(1);
  //     frontLeft.set(1);
  //     SmartDashboard.putNumber("frontRight", frontRightShoot.getMotorOutputPercent());
  //     SmartDashboard.putNumber("frontLeft", frontLeftShoot.getMotorOutputPercent());
  //     if(timer.get() > chargeTime && timer.get() < chargeTime + 2){
  //       rearRight.set(1);
  //       rearLeft.set(1);
  //       SmartDashboard.putNumber("rearRight", rearRightShoot.getMotorOutputPercent());
  //       SmartDashboard.putNumber("rearLeft", rearLeftShoot.getMotorOutputPercent());
  //     }
  //   }
  //   else if(stick.getRawButton(7)){
  //     rearRightShoot.set(ControlMode.PercentOutput, -0.4);
  //     rearLeftShoot.set(ControlMode.PercentOutput, -0.4);
  //     frontRightShoot.set(ControlMode.PercentOutput, -0.4);
  //     frontLeftShoot.set(ControlMode.PercentOutput, -0.4);
  //   }
  //   else{
  //     rearRightShoot.set(ControlMode.PercentOutput, 0);
  //     rearLeftShoot.set(ControlMode.PercentOutput, 0);
  //     frontRightShoot.set(ControlMode.PercentOutput, 0);
  //     frontLeftShoot.set(ControlMode.PercentOutput, 0);
  //   }
  // }
  public void teleopPeriodic() {
    
    double yAxis = stick.getRawAxis(0); // X축 값 읽기

    double forwardSpeed = yAxis;

    frontLeftMotor1.set(forwardSpeed);
    frontLeftMotor2.set(forwardSpeed);
    frontLeftMotor3.set(forwardSpeed);
    frontLeftMotor4.set(forwardSpeed);
    frontLeftMotor5.set(forwardSpeed);
    frontLeftMotor6.set(forwardSpeed);
    frontLeftMotor7.set(forwardSpeed);
    frontLeftMotor8.set(forwardSpeed);


  }
   
 
   
    
    
  

    
    // shoot();
   
    // if (stick.getRawButton(8)) {
    //   // If button 8 is pressed, execute the following block of code
 
    //   // Set frontRightShoot and frontLeftShoot motors to 100% output
    //   frontRightShoot.set(ControlMode.PercentOutput, 1);
    //   frontLeftShoot.set(ControlMode.PercentOutput, 1);
 
    //   // Continue running the front motors until 2 seconds have passed
    //   Timer.delay(1.5);
 
    //   // Set rearRightShoot and rearLeftShoot motors to 0.5 for 2 seconds
    //   rearRightShoot.set(ControlMode.PercentOutput, 1);
    //   rearLeftShoot.set(ControlMode.PercentOutput, 1);
 
    //   // Continue running all motors until the button is released
    //   long startTime = System.currentTimeMillis();
 
    //   // Continue running all motors until the button is released
    //   while (stick.getRawButton(8)) {
    //     // Check if 2 seconds have passed since the rear motors started
    //     if (System.currentTimeMillis() - startTime >= 1500) {
    //         // Stop all motors
    //         rearRightShoot.set(ControlMode.PercentOutput, 0);
    //         rearLeftShoot.set(ControlMode.PercentOutput, 0);
    //         frontRightShoot.set(ControlMode.PercentOutput, 0);
    //         frontLeftShoot.set(ControlMode.PercentOutput, 0);
    //         break;  // Exit the loop when the button is released
    //     }
    //   }
    // }
    // else if(stick.getRawButton(7))
    // {
    //   rearRightShoot.set(ControlMode.PercentOutput, -0.5);
    //   rearLeftShoot.set(ControlMode.PercentOutput, -0.5);
    //   frontRightShoot.set(ControlMode.PercentOutput, -0.5);
    //   frontLeftShoot.set(ControlMode.PercentOutput, -0.5);
    // }
    // else
    // {
    //   rearRightShoot.set(ControlMode.PercentOutput, 0);
    //   rearLeftShoot.set(ControlMode.PercentOutput, 0);
    //   frontRightShoot.set(ControlMode.PercentOutput, 0);
    //   frontLeftShoot.set(ControlMode.PercentOutput, 0);
    // }
 
 
 
  //}
   
     
   
 
 
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
 
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
   
  }
 
}