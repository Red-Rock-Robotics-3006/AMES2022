package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsytem;

public class ZeroWheelsCommand extends CommandBase {
  public DrivetrainSubsytem m_swerve;

  public ZeroWheelsCommand(DrivetrainSubsytem swerve) {
    this.m_swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    m_swerve.zeroWheels();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    SwerveModuleState[] drivetrainStates = m_swerve.getModuleStates();

    return 
      drivetrainStates[0].angle.equals(new Rotation2d(0)) &&
      drivetrainStates[1].angle.equals(new Rotation2d(0)) &&
      drivetrainStates[2].angle.equals(new Rotation2d(0)) &&
      drivetrainStates[3].angle.equals(new Rotation2d(0));
  }

}