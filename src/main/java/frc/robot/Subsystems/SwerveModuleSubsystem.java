// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModuleSubsystem extends SubsystemBase {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 2048;
  private static final double kSensorGearRatio = 1;

  //private static final double kModuleMaxAngularVelocity = DrivetrainSubsytem.kMaxAngularSpeed;
  //private static final double kModuleMaxAngularAcceleration =
      //2 * Math.PI; // radians per second squared
  private SwerveModuleState targetState;

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final CANCoder m_cCoder;

  private final TalonFXSimCollection m_driveMotorSim;
  private final TalonFXSimCollection m_turningMotorSim;
  private double m_driveVoltage;
  private double m_turningVoltage;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModuleSubsystem(
      int driveMotorChannel,
      int turningMotorChannel,
      int cCoderChannel) {
    this.targetState = new SwerveModuleState();

    //Create Motor Objects
    this.m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    this.m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    this.m_cCoder = new CANCoder(cCoderChannel);

    //Configure Motors
    this.m_driveMotor.configFactoryDefault();
    this.m_driveMotor.setInverted(false);
    this.m_driveMotor.setNeutralMode(NeutralMode.Brake);
    this.m_turningMotor.configFactoryDefault();
    this.m_turningMotor.setInverted(false);
    this.m_turningMotor.setNeutralMode(NeutralMode.Brake);

    //Configure Rotational Encoder
    this.m_turningMotor.getSensorCollection().setIntegratedSensorPosition(
      360 * this.m_cCoder.getPosition() / (2 * Math.PI), 
      0);

    //Configure Simulation
    this.m_driveMotorSim = m_driveMotor.getSimCollection();
    this.m_turningMotorSim = m_turningMotor.getSimCollection();
    this.m_driveVoltage = 0; //Tracking for simulation
    this.m_turningVoltage = 0; //Tracking for simulation
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(m_cCoder.getAbsolutePosition()));//getRotationDistance() % kEncoderResolution));
  }
  
  public void zeroModule() {
    setDesiredState(new SwerveModuleState(0d, new Rotation2d(0)));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

    if (
      Math.min(
        Math.abs(m_cCoder.getAbsolutePosition() - desiredState.angle.getDegrees()%360),
        Math.abs(360 + desiredState.angle.getDegrees()%360 - m_cCoder.getAbsolutePosition())
      ) > 90
    ) {
      state = new SwerveModuleState(-desiredState.speedMetersPerSecond, new Rotation2d(2*Math.PI*(desiredState.angle.getDegrees() + 180)/360d));
    }

    this.targetState = state;
  }

  @Override
  public void periodic() {
    double targetAngle = this.targetState.angle.getDegrees();
    targetAngle %= 360;
    if (targetAngle < 0) {
      targetAngle = 360 + targetAngle;
    }

    /*double linearControl = 0.85*(
      (m_cCoder.getAbsolutePosition() - targetAngle%360 < 180 ? 
      m_cCoder.getAbsolutePosition() - targetAngle%360 : 
      targetAngle%360 - m_cCoder.getAbsolutePosition())
    )/360d;*/
    double linearControl;
    if (
      Math.abs(m_cCoder.getAbsolutePosition() - targetAngle%360) < 
      Math.abs(360 + targetAngle%360 - m_cCoder.getAbsolutePosition())
    ) {
      linearControl = 0.85*(m_cCoder.getAbsolutePosition() - targetAngle%360)/360d;
    } else {
      linearControl = -0.85*(360 - (m_cCoder.getAbsolutePosition() - targetAngle%360))/360d;
    }

    final double turnOutput = Math.signum(linearControl) * Math.pow(
      Math.abs(linearControl),
      1d/1.5
    );

    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);

    SmartDashboard.putNumber("Motor Power" + m_turningMotor.getBaseID(), turnOutput);
    SmartDashboard.putNumber("Motor Rotation" + m_turningMotor.getBaseID(), m_cCoder.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
    //6380 is max motor rpm
    m_driveMotorSim.setIntegratedSensorRawPosition(
      (int)(
        m_driveMotor.getSensorCollection().getIntegratedSensorPosition() +
        (m_driveVoltage / 12) * (kEncoderResolution*6380/60) * 0.02
      )
    );
    m_driveMotorSim.setIntegratedSensorVelocity((int)((m_driveVoltage / 12) * (kEncoderResolution*6380/60) * 0.02));

    m_turningMotorSim.setIntegratedSensorRawPosition(
      (int)(
        m_turningMotor.getSensorCollection().getIntegratedSensorPosition() +
        (m_turningVoltage / 12) * (kEncoderResolution*6380/60) * 0.02
      )
    );
    m_turningMotorSim.setIntegratedSensorVelocity((int)((m_turningVoltage / 12) * (kEncoderResolution*6380/60) * 0.02));
  }

  /**
   * Gets the encoder value from the drive motor in real world distance.
   * 
   * @return Drive motor position.
   */
  private double getDriveDistance() {
    // Multiplies encoder ticks by distance per encoder tick.
    return m_driveMotor.getSensorCollection().getIntegratedSensorPosition() * 2 * Math.PI * kWheelRadius / kEncoderResolution;
  }

  /**
   * Gets the encoder velocity from the drive motor in real world velocity.
   * 
   * @return Drive motor velocity.
   */
  private double getDriveVelocity() {
    // Multiplies encoder ticks per 100ms by distance per encoder tick.
    return m_driveMotor.getSensorCollection().getIntegratedSensorVelocity() * 2 * Math.PI * kWheelRadius / kEncoderResolution;
  }

  /**
   * Gets the encoder value from the drive motor in radians.
   * 
   * @return Turning motor position.
   */
  private double getRotationDistance() {
    // Multiplies encoder ticks by radians per encoder tick.
    return m_turningMotor.getSensorCollection().getIntegratedSensorPosition() * 2 * Math.PI / kEncoderResolution;
  }

  /**
   * Gets the encoder velocity from the drive motor in radians per second.
   * 
   * @return Turning motor velocity.
   */
  private double getRotationVelocity() {
    // Multiplies encoder ticks per 100ms by radians per encoder tick.
    return m_turningMotor.getSensorCollection().getIntegratedSensorVelocity() * 2 * Math.PI / kEncoderResolution;
  }

  /**
   * Converts meters to encoder counts.
   * @param positionMeters
   * @return Encoder counts.
   */
  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * kWheelRadius);
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kEncoderResolution);
    return sensorCounts;
  }

  /**
   * Converts Mps to encoder counts per 100 ms.
   * @param velocityMetersPerSecond
   * @return Encoder counts per 100ms.
   */
  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * kWheelRadius);
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kEncoderResolution);
    return sensorCountsPer100ms;
  }

  /**
   * Converts encoder counts to meters.
   * @param sensorCounts
   * @return Meters.
   */
  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kEncoderResolution;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * kWheelRadius);
    return positionMeters;
  }
}
