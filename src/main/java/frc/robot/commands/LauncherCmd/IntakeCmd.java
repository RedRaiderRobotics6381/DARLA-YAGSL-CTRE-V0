// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LauncherCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.Constants.LauncherConstants;

public class IntakeCmd extends Command {
  
  private final Launcher m_Launcher;

  public IntakeCmd(Launcher m_Launcher) {
    this.m_Launcher = m_Launcher;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Launcher.setLaunchWheel(LauncherConstants.kIntakeFeederSpeed);
    m_Launcher.setFeedWheel(LauncherConstants.kIntakeFeederSpeed);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Launcher.setLaunchWheel(LauncherConstants.kIntakeFeederSpeed);
    m_Launcher.setFeedWheel(LauncherConstants.kIntakeFeederSpeed);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Launcher.setLaunchWheel(0);
    m_Launcher.setFeedWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
