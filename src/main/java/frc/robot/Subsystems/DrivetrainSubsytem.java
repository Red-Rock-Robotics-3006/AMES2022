// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsytem extends SubsystemBase {
  public static final double kMaxSpeed = 5.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModuleSubsystem m_frontLeft = new SwerveModuleSubsystem(1, 2);
  private final SwerveModuleSubsystem m_frontRight = new SwerveModuleSubsystem(3, 4);
  private final SwerveModuleSubsystem m_backLeft = new SwerveModuleSubsystem(5, 6);
  private final SwerveModuleSubsystem m_backRight = new SwerveModuleSubsystem(7, 8);

  private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private double m_expectedRotVelocity = 0; //Tracking for simulation

  private final Field2d m_fieldSim;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public DrivetrainSubsytem() {
    m_gyro.reset();

    // the Field2d class lets us visualize our robot in the simulation GUI.
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  //@SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_expectedRotVelocity = rot;
    
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(10*xSpeed, 10*ySpeed, 5*rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(10*xSpeed, 10*ySpeed, 5*rot));
    double idealModule1Velocity = swerveModuleStates[0].speedMetersPerSecond;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    if (Math.abs(swerveModuleStates[0].speedMetersPerSecond) > 0.001 && Math.abs(idealModule1Velocity) > 0.001) {
      m_expectedRotVelocity *= swerveModuleStates[0].speedMetersPerSecond/idealModule1Velocity;
    }
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override 
  public void simulationPeriodic() {
    m_gyroSim.setAngle(m_gyroSim.getAngle()+0.02*360*m_expectedRotVelocity/(2*Math.PI));
  }
}
