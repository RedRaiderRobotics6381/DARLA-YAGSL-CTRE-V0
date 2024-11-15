// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  CANSparkMax m_launchWheelFollowerL;
  CANSparkMax m_launchWheelLeaderR;
  CANSparkMax m_feedWheelRight;

  // PWM ports/CAN IDs for motor controllers
  public static final int kFeederRightID = 22;
  public static final int kLauncherRightID = 20;
  public static final int kLauncherLeftID = 21;

  // Current limit for launcher and feed wheels
  public static final int kLauncherCurrentLimit = 80;
  public static final int kFeedCurrentLimit = 80;

  // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kLauncherSpeed = 1;
  public static final double kLaunchFeederSpeed = 1;
  public static final double kIntakeLauncherSpeed = -.2;
  public static final double kIntakeFeederSpeed = -.2;

  public static final double kLauncherDelay = 1;

  /** Creates a new Launcher. */
  public Launcher() {
    m_launchWheelFollowerL = new CANSparkMax(kLauncherLeftID, MotorType.kBrushless);
    m_launchWheelLeaderR = new CANSparkMax(kLauncherRightID, MotorType.kBrushless);
    m_feedWheelRight = new CANSparkMax(kFeederRightID, MotorType.kBrushless);

    m_launchWheelFollowerL.restoreFactoryDefaults();
    m_launchWheelLeaderR.restoreFactoryDefaults();
    m_feedWheelRight.restoreFactoryDefaults();
    m_launchWheelFollowerL.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_launchWheelLeaderR.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_feedWheelRight.setSmartCurrentLimit(kFeedCurrentLimit);
    m_launchWheelFollowerL.setInverted(true);
    m_launchWheelFollowerL.follow(m_launchWheelLeaderR);
    m_launchWheelFollowerL.setIdleMode(IdleMode.kCoast);
    m_launchWheelLeaderR.setIdleMode(IdleMode.kCoast);
    m_feedWheelRight.setIdleMode(IdleMode.kCoast);
    m_launchWheelFollowerL.burnFlash();
    m_launchWheelLeaderR.burnFlash();
    m_feedWheelRight.burnFlash();
    
  }

  /**
   * The code above sets up the motors used in the feeder and launcher subsystems. The motors are
   * initialized and restored to factory defaults. The current limits are set for the launcher and
   * feed wheels. The follower motor is inverted and set to follow the leader motor. The motors are
   * set to coast mode and the flash is burned to the motor controllers.
   * 
   * The code below contains commands to run the launcher and feeder motors at specified speeds, these
   * commands are then used in more complex sequences to perform actions such as intaking and launching.
   */
  
  /**
   * Runs the launcher motor at the specified speed. This will run both launcher motors at the same
   * speed because the follower motor is set to follow the leader motor.
   * 
   * @param speed the speed at which to run the launcher motor
   * @return the Command object representing the run launcher motor command
   */
  public Command RunLauncherMotorCommand(double speed) {
    return this.run(() -> {m_launchWheelLeaderR.set(speed);}); // Set the launch wheels to the intake speed value
  }

  /**
   * Runs the feeder motor at the specified speed.
   * 
   * @param speed the speed at which to run the feeder motor
   * @return the Command object representing the action of running the feeder motor
   */
  public Command RunFeederMotorCommand(double speed) {
    return this.run(() -> {m_feedWheelRight.set(speed);}); // Set the feeder wheel to the intake speed value
  }

  public Command IntakeNoteCommand() {
    return this.runEnd(
        () -> {
                RunFeederMotorCommand(kIntakeFeederSpeed)
                .alongWith(RunLauncherMotorCommand(kIntakeLauncherSpeed))
                .handleInterrupt(() -> {StopMotorsCommand();});
              },
        () -> {
                StopMotorsCommand();
              });
  }

  public Command OpenNoteHatch() {
    return this.runEnd(
        () -> {
                RunFeederMotorCommand(-kIntakeFeederSpeed)
                .alongWith(RunLauncherMotorCommand(-kIntakeLauncherSpeed))
                .withTimeout(2.0)
                .andThen( RunFeederMotorCommand(kIntakeFeederSpeed)
                .alongWith(RunLauncherMotorCommand(kIntakeLauncherSpeed)))
                .handleInterrupt(() -> {StopMotorsCommand();});
              },
        () -> {
          StopMotorsCommand();
        });
  }

  public Command ScoreSpeakerCommand() {
    return this.runEnd(
      () -> {
        RunLauncherMotorCommand(1.0)
        .until(() -> Math.abs( m_launchWheelLeaderR.getEncoder().getVelocity()) >= 5500)
        .andThen(() -> setLaunchWheel(1.0))
        .andThen(RunFeederMotorCommand(1.0))
        .withTimeout(2.0)
        .andThen(StopMotorsCommand())
        .handleInterrupt(() -> {StopMotorsCommand();});
      },
      () -> {
        StopMotorsCommand();
      });
  }

  public Command ScoreAmpCommand() {
    return this.runEnd(
      () -> {
        RunLauncherMotorCommand(0.2)
        .until(() -> Math.abs( m_launchWheelLeaderR.getEncoder().getVelocity()) >= 1000)
        .andThen(() -> setLaunchWheel(0.2))
        .andThen(RunFeederMotorCommand(0.2))
        .withTimeout(2.0)
        .andThen(StopMotorsCommand())
        .handleInterrupt(() -> {StopMotorsCommand();});
      },
      () -> {
        StopMotorsCommand();
      });
  }

  /**
   * Returns a Command object that stops the motors.
   *
   * @return the Command object that stops the motors
   */
  public Command StopMotorsCommand() {
    return this.runOnce(() -> {
                                m_launchWheelLeaderR.set(0);
                                m_feedWheelRight.set(0);
                              });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheelLeaderR.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheelRight.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheelLeaderR.set(0);
    m_feedWheelRight.set(0);
  }
}
