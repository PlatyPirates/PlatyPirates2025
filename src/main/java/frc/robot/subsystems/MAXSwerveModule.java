// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private final SparkMaxConfig m_drivingConfig;
  private final SparkMaxConfig m_turningConfig;

  private final EncoderConfig m_drivingEncoderConfig;
  private final EncoderConfig m_turningEncoderConfig;

  private final ClosedLoopConfig m_drivingClosedLoopConfig;
  private final ClosedLoopConfig m_turningClosedLoopConfig;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingConfig = new SparkMaxConfig();
    m_turningConfig = new SparkMaxConfig();
    m_drivingEncoderConfig = new EncoderConfig();
    m_drivingClosedLoopConfig = new ClosedLoopConfig();
    m_turningEncoderConfig = new EncoderConfig();
    m_turningClosedLoopConfig = new ClosedLoopConfig();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    m_drivingClosedLoopConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_turningClosedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoderConfig.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoderConfig.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoderConfig.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoderConfig.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoderConfig.inverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningClosedLoopConfig.positionWrappingEnabled(true);
    m_turningClosedLoopConfig.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningClosedLoopConfig.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingClosedLoopConfig.p(ModuleConstants.kDrivingP);
    m_drivingClosedLoopConfig.i(ModuleConstants.kDrivingI);
    m_drivingClosedLoopConfig.d(ModuleConstants.kDrivingD);
    m_drivingClosedLoopConfig.velocityFF(ModuleConstants.kDrivingFF);
    m_drivingClosedLoopConfig.outputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningClosedLoopConfig.p(ModuleConstants.kTurningP);
    m_turningClosedLoopConfig.i(ModuleConstants.kTurningI);
    m_turningClosedLoopConfig.d(ModuleConstants.kTurningD);
    m_turningClosedLoopConfig.velocityFF(ModuleConstants.kTurningFF);
    m_turningClosedLoopConfig.outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingConfig.apply(m_drivingEncoderConfig);
    m_drivingConfig.apply(m_drivingClosedLoopConfig);
    m_turningConfig.apply(m_turningEncoderConfig);
    m_turningConfig.apply(m_turningClosedLoopConfig);
    m_drivingSparkMax.configure(m_drivingConfig, ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_turningSparkMax.configure(m_turningConfig, ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
