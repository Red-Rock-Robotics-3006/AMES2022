package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.DrivetrainSubsytem;

public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final DrivetrainSubsytem m_swerve = new DrivetrainSubsytem();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Track autonomous command
  private CommandBase m_autonomousCommand = null;

  public RobotContainer() {
    disableControllers();
  }

  public void enableControllers() {
    if (m_swerve.getDefaultCommand() != null) {
      m_swerve.getDefaultCommand().cancel();
    }

    RunCommand dc = new RunCommand(
        () -> m_swerve.drive(m_xspeedLimiter.calculate(m_controller.getRawAxis(0)), -m_yspeedLimiter.calculate(m_controller.getRawAxis(1)), m_rotLimiter.calculate(m_controller.getRawAxis(2)), true),
        m_swerve
      );
    dc.setName("Controller");

    m_swerve.setDefaultCommand(dc);
  }

  public void disableControllers() {
    /*zeroAllOutputs();
    if (m_swerve.getDefaultCommand() != null) {
      m_swerve.getDefaultCommand().cancel();
    }
    System.out.println("Deafult: " + m_swerve.getDefaultCommand());*/
    RunCommand dc = new RunCommand(
      () -> m_swerve.drive(0, 0, 0, true),
      m_swerve
    );
    dc.setName("Stay Still");

    m_swerve.setDefaultCommand(dc);
  }

  public void zeroAllOutputs() {
    m_swerve.drive(0, 0, 0, true);
  }

  public Command getAutonomousCommand() {
    if (m_autonomousCommand == null) {
      m_autonomousCommand = new ParallelDeadlineGroup(
          new WaitCommand(15),
          new RunCommand(() -> m_swerve.drive(1, 0, -10, true), m_swerve)
        )
        .andThen(() -> zeroAllOutputs());
      m_autonomousCommand.setName("AutoTimedForward");
    }

    return m_autonomousCommand;
  }

  public void periodic() {
    m_swerve.updateOdometry();
  }
}
