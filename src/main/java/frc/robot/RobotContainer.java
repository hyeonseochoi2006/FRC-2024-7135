// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MAXSwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);

  // 발사 메커니즘 모터
    TalonSRX rearRightShoot = new TalonSRX(1);
    VictorSPX frontRightShoot = new VictorSPX(4);
    TalonSRX rearLeftShoot = new TalonSRX(2);
    VictorSPX frontLeftShoot = new VictorSPX(5);
    private boolean shooting = false;
    private final Timer timer = new Timer();
    public double chargeTime = 1.5;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

        // 설정 추가
        rearRightShoot.setInverted(false);
        rearLeftShoot.setInverted(true);
        frontRightShoot.setInverted(false);
        frontLeftShoot.setInverted(true);
        
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  


  

// ------------------------------------------------------------------------------------




public void teleopPeriodic() {
  Wheelinitial();
    
  // double maxForwardSpeed = 0.1; // 최대 전진 속도를 50%로 설정
  // double maxRotationSpeed = 0.1;
  
  
  
  // // 조이스틱 입력을 가져옴
  // double forward = m_driverController.getRawAxis(1); // Forward는 조이스틱을 아래로 향할수록 양수
  // double strafe = m_driverController.getRawAxis(0);   // 오른쪽으로 향할수록 양수
  
  // double rotation = m_driverController.getRawAxis(2); // 시계 방향으로 회전할수록 양수
  
  // // 최대 전진 속도 적용
  // forward = forward * maxForwardSpeed;
  // strafe = strafe * maxForwardSpeed;
  // rotation = rotation * maxRotationSpeed;
// 조이스틱 입력에 따라 바퀴를 회전시키거나 직진/후진하도록 모터를 제어(2번째)
// if (Math.abs(strafe) > 0.1) {
//   // 좌우 이동 값이 일정 값 이상일 때는 바퀴를 회전시킴
//   // 예를 들어, 오른쪽으로 이동하는 경우
//   m_robotDrive.drive(0, 0, strafe, true, true);
// } else {
//   // 좌우 이동 값이 일정 값 미만일 때는 바로 해당 방향으로 이동
//   // 전진
//   if (forward > 0.1) {
//       m_robotDrive.drive(forward, 0, 0, true, true);
//   }
//   // 후진
//   else if (forward < -0.1) {
//       m_robotDrive.drive(forward, 0, 0, true, true);
//   }
//   // 정지
//   else {
//       m_robotDrive.drive(0, 0, 0, true, true);
//   }
// }

  // 조이스틱 입력에 deadbend apply
  // forward = applyDeadband(forward, OIConstants.kDriveDeadband);
  // strafe = applyDeadband(strafe, OIConstants.kDriveDeadband);
  // rotation = applyDeadband(rotation, OIConstants.kDriveDeadband);


    //  조이스틱 입력의 크기가 0 또는 매우 작을 때 바퀴를 정면으로 설정
// if (Math.abs(forward) < 0.1 && Math.abs(strafe) < 0.1 && Math.abs(rotation) < 0.1) {
//     // 정면으로 설정하는 코드 추가
//     // m_robotDrive.drive(0, 0, 0, true, true); // 모든 바퀴를 멈추도록 설정
//     Wheelinitial();
// } else {
//     // 조이스틱 입력을 사용하여 로봇 구동
//     m_robotDrive.drive(forward, strafe, rotation, true, true);
//     m_robotDrive.drive(-forward, strafe, rotation, true, true);
// }
  shoot();
}




private void Wheelinitial() {
  MAXSwerveModule frontRightModule = m_robotDrive.getFrontRightModule();
  MAXSwerveModule frontLeftModule = m_robotDrive.getFrontLeftModule();
  MAXSwerveModule rearLeftModule = m_robotDrive.getRearLeftModule();
  MAXSwerveModule rearRighttModule = m_robotDrive.getRearRightModule();

  SwerveModuleState zeroState = new SwerveModuleState(0, frontRightModule.getState().angle);

  frontRightModule.setDesiredState(zeroState);
  frontLeftModule.setDesiredState(zeroState);
  rearLeftModule.setDesiredState(zeroState);
  rearRighttModule.setDesiredState(zeroState);
}




private double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
        return 0.0;
    } else {
        return input;
    }
}

public void shoot(){
    if(m_driverController.getRawButton(8) && shooting == false || shooting == true){
      timer.reset();
      timer.start();
      shooting = true;
      frontRightShoot.set(ControlMode.PercentOutput, 1);
      frontLeftShoot.set(ControlMode.PercentOutput, 1);
      
    
      SmartDashboard.putNumber("frontRight", frontRightShoot.getMotorOutputPercent());
      SmartDashboard.putNumber("frontLeft", frontLeftShoot.getMotorOutputPercent());
      if(timer.get() > chargeTime && timer.get() < chargeTime + 2){
        rearRightShoot.set(ControlMode.PercentOutput, 1);
        rearLeftShoot.set(ControlMode.PercentOutput, 1);
        shooting = false;
        SmartDashboard.putNumber("rearRight", rearRightShoot.getMotorOutputPercent());
        SmartDashboard.putNumber("rearLeft", rearLeftShoot.getMotorOutputPercent());
      }
    }
    else if(m_driverController.getRawButton(7)){
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
}
