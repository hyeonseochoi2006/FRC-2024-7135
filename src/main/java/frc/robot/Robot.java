// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
 
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
 
  TalonSRX rearRightShoot = new TalonSRX(1);
  VictorSPX frontRightShoot = new VictorSPX(4);
  TalonSRX rearLeftShoot = new TalonSRX(2);
  VictorSPX frontLeftShoot = new VictorSPX(5);
 //spark orginal
  // Spark frontLeft = new Spark(4);
  // Spark rearLeft = new Spark(3);
  // Spark frontRight = new Spark(1);
  // Spark rearRight = new Spark(2);
  
  
  

 //spark max




 
 private static final int frontLeftID = 4; // CAN ID 설정 해야 될수 도 있음 REV hardware client 에서
 private static final int rearLeftID = 3;
 private static final int frontRightID = 1;
 private static final int rearRightID = 2;

 private static final int frontLeftROID = 5; // CAN ID 설정 해야 될수 도 있음 REV hardware client 에서
 private static final int rearLeftROID = 6;
 private static final int frontRightROID = 7;
 private static final int rearRightROID = 8;
  
  CANSparkMax frontLeft = new CANSparkMax(frontLeftID, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax rearLeft = new CANSparkMax(rearLeftID, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax frontRight = new CANSparkMax(frontRightID, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax rearRight = new CANSparkMax(rearRightID, CANSparkLowLevel.MotorType.kBrushed);

  CANSparkMax frontLeftRO = new CANSparkMax(frontLeftROID, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax rearLeftRO = new CANSparkMax(rearLeftROID, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax frontRightRO = new CANSparkMax(frontRightROID, CANSparkLowLevel.MotorType.kBrushed);
  CANSparkMax rearRightRO = new CANSparkMax(rearRightROID, CANSparkLowLevel.MotorType.kBrushed);
  




 
  DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);
  DifferentialDrive _drive = new DifferentialDrive(rearLeft, rearRight);
 
  Joystick stick = new Joystick(0);
 
  public boolean shooting = false;
  public double chargeTime = 1.5;
  public double speed = 0.8;
  Timer timer = new Timer();
  // private SpeedControllerGroup left, right;
  // private DifferentialDrive drive;



  
  NetworkTable limelightTable;
 
 
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
    gyro.reset();

    //limelight code 
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
   
   
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }
 
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
 
  @Override
  public void disabledPeriodic() {}
 
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
 
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //frontRightShoot.set(ControlMode.PercentOutput, -1);
    //frontLeftShoot.set(ControlMode.PercentOutput, -0.5);
    //rearRightShoot.set(ControlMode.PercentOutput, -0.5);  
    //rearLeftShoot.set(ControlMode.PercentOutput, -0.5);

     // Reset timer and start rotating
     timer.reset();
     timer.start();
     drive.arcadeDrive(0.5, 1.0); // Adjust rotation speed as needed
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
        if (limelightTable.getEntry("tv").getDouble(0) == 1 && limelightTable.getEntry("tlong").getDouble(0) < 3000) {
            // 회전을 멈춥니다
            drive.stopMotor();
            
            // AprilTag와의 거리를 가져옵니다
            double distance = limelightTable.getEntry("tlong").getDouble(0);
    
            // 만약 AprilTag와의 거리가 2m보다 작으면 로봇을 멈춥니다
            // 이후에 필요한 경우 움직임을 추가하실 수 있습니다
            if (distance < 2000) {
                drive.stopMotor();
                // 여기에 추가 동작을 입력하세요
            } else {
                // AprilTag와의 거리가 2m 이상이면 로봇을 전진합니다 (예시: 속도 0.5)
                drive.arcadeDrive(0.5, 0); // 전진 속도를 필요에 따라 조절하세요
            }
        } else {
            // AprilTag가 감지되지 않은 경우 계속 회전합니다
            drive.arcadeDrive(0.5, 1.0); // 회전 속도를 필요에 따라 조절하세요
        }
    
 
  }
 
  /** This function is called periodically during operator control. */
 
  public void shoot(){
    if(stick.getRawButton(8) && shooting == false){
      timer.reset();
      timer.start();
      frontRight.set(1);
      frontLeft.set(1);
      SmartDashboard.putNumber("frontRight", frontRightShoot.getMotorOutputPercent());
      SmartDashboard.putNumber("frontLeft", frontLeftShoot.getMotorOutputPercent());
      if(timer.get() > chargeTime && timer.get() < chargeTime + 2){
        rearRight.set(1);
        rearLeft.set(1);
        SmartDashboard.putNumber("rearRight", rearRightShoot.getMotorOutputPercent());
        SmartDashboard.putNumber("rearLeft", rearLeftShoot.getMotorOutputPercent());
      }
    }
    else if(stick.getRawButton(7)){
      rearRightShoot.set(ControlMode.PercentOutput, -0.4);
      rearLeftShoot.set(ControlMode.PercentOutput, -0.4);
      frontRightShoot.set(ControlMode.PercentOutput, -0.4);
      frontLeftShoot.set(ControlMode.PercentOutput, -0.4);
    }
    else{
      rearRightShoot.set(ControlMode.PercentOutput, 0);
      rearLeftShoot.set(ControlMode.PercentOutput, 0);
      frontRightShoot.set(ControlMode.PercentOutput, 0);
      frontLeftShoot.set(ControlMode.PercentOutput, 0);
    }
  }
  public void teleopPeriodic() {
    double forward = -stick.getRawAxis(1); // 오른쪽 조이스틱의 Y축 값을 가져옵니다
    double turn = stick.getRawAxis(0); // 왼쪽 조이스틱의 X축 값을 가져옵니다


    
    shoot();
   
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
 
 
 
  }
   
     
   
 
 
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
 
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if(stick.getRawButton(8)){
      rearRightShoot.set(ControlMode.PercentOutput, 1);
      rearLeftShoot.set(ControlMode.PercentOutput, 1);
      frontRightShoot.set(ControlMode.PercentOutput, 1);
      frontLeftShoot.set(ControlMode.PercentOutput, 1);
    }
    else if(stick.getRawButton(7))
    {
      rearRightShoot.set(ControlMode.PercentOutput, -0.5);
      rearLeftShoot.set(ControlMode.PercentOutput, -0.5);
      frontRightShoot.set(ControlMode.PercentOutput, -0.5);
      frontLeftShoot.set(ControlMode.PercentOutput, -0.5);
    }
    else
    {
      rearRightShoot.set(ControlMode.PercentOutput, 0);
      rearLeftShoot.set(ControlMode.PercentOutput, 0);
      frontRightShoot.set(ControlMode.PercentOutput, 0);
      frontLeftShoot.set(ControlMode.PercentOutput, 0);
    }
  }
 
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
 
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}