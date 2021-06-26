// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = Constants.Chassis.wheelRadius;
  private static final int kEncoderResolution = Constants.EncoderRes.turnRes;
  private static final int kEncoderResolutionDrive = Constants.EncoderRes.driveRes;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final WPI_VictorSPX m_driveMotor;
  private final WPI_TalonSRX m_turningMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_turningEncoder;

  public enum PinType { DigitalIO, PWM, AnalogIn, AnalogOut };

  public final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
  public final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
  public final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
  public final int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
  public final int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
  public final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;
  
  /* getChannelFromPin( PinType, int ) - converts from a navX-MXP */
  /* Pin type and number to the corresponding RoboRIO Channel     */
  /* Number, which is used by the WPI Library functions.          */
  
  public int getChannelFromPin( PinType type, int io_pin_number ) 
             throws IllegalArgumentException {
      int roborio_channel = 0;
      if ( io_pin_number < 0 ) {
          throw new IllegalArgumentException("Error:  navX-MXP I/O Pin #");
      }
      switch ( type ) {
      case DigitalIO:
          if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");
          }
          roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS + 
                            (io_pin_number > 3 ? 4 : 0);
          break;
      case PWM:
          if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");
          }
          roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
          break;
      case AnalogIn:
          if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Input Pin #");
          }
          roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
          break;
      case AnalogOut:
          if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {
              throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Output Pin #");
          }
          roborio_channel = io_pin_number;            
          break;
      }
      return roborio_channel;
  }
  //private final Encoder m_turningEncoder = new Encoder(2, 3);
  //private double m_turningEncoderAngle;


  private final PIDController m_drivePIDController = new PIDController(0.001, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.001,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, 
            int driveEncoderChannelA, int driveEncoderChannelB,
            int turnEncoderChannelA, int turnEncoderChannelB) {
    m_driveMotor = new WPI_VictorSPX(driveMotorChannel);
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
    m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_turningEncoder = new Encoder(getChannelFromPin(PinType.DigitalIO,turnEncoderChannelA), getChannelFromPin(PinType.DigitalIO, turnEncoderChannelB));

    //double encoderAngle = m_turningMotor.getSelectedSensorPosition();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolutionDrive);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.setDistancePerPulse(2.3 * 2 * Math.PI / kEncoderResolution);
    //m_turningEncoderAngle = 
    //  (((encoderAngle % kEncoderResolution) + kEncoderResolution) % kEncoderResolution - 
    //    (kEncoderResolution/2)) * 45 / (kEncoderResolution/8);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));
    
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    
  }
}
