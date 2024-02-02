// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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
  
  Spark frontLeft = new Spark(4);
  Spark rearLeft = new Spark(3);
  Spark frontRight = new Spark(1);
  Spark rearRight = new Spark(2);
  DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);
  DifferentialDrive _drive = new DifferentialDrive(rearLeft, rearRight);
  
  Joystick stick = new Joystick(0);
  
  
  // private SpeedControllerGroup left, right;
  // private DifferentialDrive drive;
  
  
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
    gyro.reset();
    //limelight code
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    double targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0.0);
    double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0);
    double targetArea = table.getEntry("ta").getDouble(0.0);
    double targetSkew = table.getEntry("ts").getDouble(0.0);
    
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double angle = gyro.getAngle();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }  
    double forwardSpeed = 0.5;  // Adjust the speed as needed
    drive.arcadeDrive(forwardSpeed, 0);
    _drive.arcadeDrive(forwardSpeed, 0);
    
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double angle = gyro.getAngle();
    double speed = 0.8;
    drive.arcadeDrive(stick.getRawAxis(2) * speed, stick.getRawAxis(1) * speed);
    _drive.arcadeDrive(stick.getRawAxis(2) * speed, stick.getRawAxis(1) * speed);
    if (stick.getRawButton(8)) {
      // If button 8 is pressed, execute the following block of code
  
      // Set frontRightShoot and frontLeftShoot motors to 100% output
      frontRightShoot.set(ControlMode.PercentOutput, 1);
      frontLeftShoot.set(ControlMode.PercentOutput, 1);
  
      // Continue running the front motors until 2 seconds have passed
      Timer.delay(4);
  
      // Set rearRightShoot and rearLeftShoot motors to 0.5 for 2 seconds
      rearRightShoot.set(ControlMode.PercentOutput, 1);
      rearLeftShoot.set(ControlMode.PercentOutput, 1);
  
      // Continue running all motors until the button is released
      long startTime = System.currentTimeMillis();

      // Continue running all motors until the button is released
      while (stick.getRawButton(8)) {
        // Check if 2 seconds have passed since the rear motors started
        if (System.currentTimeMillis() - startTime >= 1500) {
            // Stop all motors
            rearRightShoot.set(ControlMode.PercentOutput, 0);
            rearLeftShoot.set(ControlMode.PercentOutput, 0);
            frontRightShoot.set(ControlMode.PercentOutput, 0);
            frontLeftShoot.set(ControlMode.PercentOutput, 0);
            break;  // Exit the loop when the button is released
        }
      }
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



    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");
    
    
    inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
    
    // NetworkTableEntry TeamEntry = table.getEntry("tx");
    NetworkTableEntry xEntry = table.getEntry("tx");
    NetworkTableEntry yEntry = table.getEntry("ty");
    NetworkTableEntry aEntry = table.getEntry("ta");
    NetworkTableEntry lEntry = table.getEntry("tl");
    NetworkTableEntry vEntry = table.getEntry("tv");
    NetworkTableEntry sEntry = table.getEntry("ts");
    
    NetworkTableEntry tshortEntry = table.getEntry("tshort");
    NetworkTableEntry tlongEntry = table.getEntry("tlong");
    NetworkTableEntry thorEntry = table.getEntry("thor");
    NetworkTableEntry tvertEntry = table.getEntry("tvert");
    NetworkTableEntry getpipeEntry = table.getEntry("getpipe");
    NetworkTableEntry camtranEntry = table.getEntry("camtran");
    NetworkTableEntry ledModeEntry = table.getEntry("ledMode");
    
    // double tx = xEntry.getDouble(0.0);
    double tx = xEntry.getDouble(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    double ty = yEntry.getDouble(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    double ta = aEntry.getDouble(0.0); // Target Area (0% of image to 100% of image)
    double tl = lEntry.getDouble(0.0); // The pipeline’s latency contribution (ms) Add at least 11ms for image capture
                                      // latency.
    double tv = vEntry.getDouble(0.0); // Whether the limelight has any valid targets (0 or 1)
    double ts = sEntry.getDouble(0.0); // Skew or rotation (-90 degrees to 0 degrees)
    
    // double tshort = tshortEntry.getString(); // Sidelength of shortest side of
    // the fitted bounding box (pixels)
    // double tlong = tlong // Sidelength of longest side of the fitted bounding box
    // (pixels)
    // double thor = thor // Horizontal sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double tvert = tvert // Vertical sidelength of the rough bounding box (0 -
    // 320 pixels)
    // double getpipe = getpipe // True active pipeline index of the camera (0 .. 9)
    // double camtran = camtran // Results of a 3D position solution, 6 numbers:
    // Translation (x,y,y) Rotation(pitch,yaw,roll)
    
    //ledModeEntry.setNumber(0); // use the LED Mode set in the current pipeline
    ledModeEntry.setNumber(1); // force off
    //ledModeEntry.setNumber(2); // force blink
    //ledModeEntry.setNumber(3); // force on
    
    //System.out.println("X: " + tx);
    //System.out.println("Y: " + ty);
    //System.out.println("A: " + ta);
    //System.out.println("L: " + tl);
    //System.out.println("V: " + tv);
    //System.out.println("S: " + tv);
    
    // post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight X", tx);
    SmartDashboard.putNumber("Limelight Y", ty);
    SmartDashboard.putNumber("Limelight Area", ta);
    SmartDashboard.putNumber("Limelight Latency", tl);
    SmartDashboard.putNumber("Limelight Valid Target", tv);
    SmartDashboard.putString("Alex", "Monkey");
    
    // Limelight Data End

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
