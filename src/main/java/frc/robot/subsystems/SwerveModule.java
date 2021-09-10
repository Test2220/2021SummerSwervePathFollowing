// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController, 0, 0,
      new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, boolean driveEncoderReversed,
      boolean turningEncoderReversed) {

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_driveMotor.setInverted(driveEncoderReversed);
    m_turningMotor.setInverted(turningEncoderReversed);
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
    return new SwerveModuleState(getDriveMotorEncoder(), new Rotation2d(getTurningMotorEncoder()));
  }

  private double getDriveMotorEncoder() {
    return m_driveMotor.getSelectedSensorVelocity() * 10 / ModuleConstants.kEncoderResolution * Math.PI
        * ModuleConstants.kWheelDiameterMeters;
  }

  private double getTurningMotorEncoder() {
    return m_turningMotor.getSelectedSensorVelocity() * 10 / ModuleConstants.kEncoderResolution * 2 * Math.PI;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    final var driveOutput = m_drivePIDController.calculate(getDriveMotorEncoder(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(getTurningMotorEncoder(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(TalonFXControlMode.Current, driveOutput);
    m_turningMotor.set(TalonFXControlMode.Current, turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }
}
