// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  private final WPI_TalonFX m_leftShooter = new WPI_TalonFX(Constants.ShooterControllers.leftShooter);
  private final WPI_TalonFX m_rightShooter = new WPI_TalonFX(Constants.ShooterControllers.rightShooter);

  private final DifferentialDrive m_shooter = new DifferentialDrive(m_leftShooter, m_rightShooter);

  private final WPI_TalonSRX m_elevator = new WPI_TalonSRX(Constants.ElevatorControllers.elevator);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    if(m_controller.getBButton()){
      m_shooter.arcadeDrive(Constants.ShooterConstants.shooterSpeed, 0.0, false);
    } else{
      m_shooter.arcadeDrive(0.0, 0.0, false);
    }

    m_elevator.set(m_controller.getTriggerAxis(Hand.kRight) * Constants.ElevatorConstants.elevatorSpeed
                    - m_controller.getTriggerAxis(Hand.kLeft) * Constants.ElevatorConstants.elevatorSpeed);
    
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft))
            * frc.robot.Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(m_controller.getX(GenericHID.Hand.kLeft))
            * frc.robot.Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(m_controller.getX(GenericHID.Hand.kRight))
            * frc.robot.Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
