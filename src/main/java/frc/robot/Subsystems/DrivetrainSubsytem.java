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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.Pigeon2;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsytem extends SubsystemBase {
  public static final double kMaxSpeed = 5.0; // 3 meters per second
  //public static final double kMaxAngularSpeed = 2*Math.PI; // 1/2 rotation per second //---

  private final Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(-0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(0.381, -0.381);

  private final SwerveModuleSubsystem m_frontLeft = new SwerveModuleSubsystem(20, 6, 0, false);
  private final SwerveModuleSubsystem m_frontRight = new SwerveModuleSubsystem(11, 8, 41, true);
  private final SwerveModuleSubsystem m_backLeft = new SwerveModuleSubsystem(4, 5, 44, false);
  private final SwerveModuleSubsystem m_backRight = new SwerveModuleSubsystem(50, 3, 43, true);

  private final Pigeon2 m_gyro = new Pigeon2(45);
  //private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private double m_expectedRotVelocity = 0; //Tracking for simulation

  private final Field2d m_fieldSim;

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360d));//getRotation2d());

  public DrivetrainSubsytem() {
    m_gyro.setYaw(0);//.reset();

    // the Field2d class lets us visualize our robot in the simulation GUI.
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot in degrees.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  //@SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (Math.abs(xSpeed) + Math.abs(ySpeed) + Math.abs(rot) > 0.1) { //---
      //m_expectedRotVelocity = rot;
      
      var swerveModuleStates =
          m_kinematics.toSwerveModuleStates(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 2*Math.PI*(rot/360), new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360d)) //High Risk Change!
                  : new ChassisSpeeds(xSpeed, ySpeed, 2*Math.PI*(rot/360))); //High Risk Change!
      //double idealModule1Velocity = swerveModuleStates[0].speedMetersPerSecond;
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

      /*if (Math.abs(swerveModuleStates[0].speedMetersPerSecond) > 0.001 && Math.abs(idealModule1Velocity) > 0.001) {
        m_expectedRotVelocity *= swerveModuleStates[0].speedMetersPerSecond/idealModule1Velocity;
      }*/
      
      m_frontLeft.setDesiredState(swerveModuleStates[0]); //High Risk Change!
      m_frontRight.setDesiredState(swerveModuleStates[1]); //High Risk Change!
      m_backLeft.setDesiredState(swerveModuleStates[2]); //High Risk Change!
      m_backRight.setDesiredState(swerveModuleStates[3]); //High Risk Change!
    } else {
      m_frontLeft.zeroPower();
      m_frontRight.zeroPower();
      m_backLeft.zeroPower();
      m_backRight.zeroPower();
    }
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{ m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState() };
  }

  public void setUniformDirection(Rotation2d rot) {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, rot));
    m_frontRight.setDesiredState(new SwerveModuleState(0, rot));
    m_backLeft.setDesiredState(new SwerveModuleState(0, rot));
    m_backRight.setDesiredState(new SwerveModuleState(0, rot));
  }

  public void zeroWheels() {
    m_frontLeft.zeroModule();
    m_frontRight.zeroModule();
    m_backLeft.zeroModule();
    m_backRight.zeroModule();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360),//getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }
  
  @Override
  public void periodic() {
    //m_gyroSim.setAngle(m_gyroSim.getAngle()+0.02*360*m_expectedRotVelocity/(2*Math.PI)); //Should really go in simulation periodic but is needed for rough angle approximation
  }

  @Override 
  public void simulationPeriodic() {
    //m_gyroSim.setAngle(m_gyroSim.getAngle()+0.02*360*m_expectedRotVelocity/(2*Math.PI));
  }
}
