// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmAuto extends Command {

  private ArmSubsystem armSub;
  private double armSpeed;
  private Timer timer;
  private double timeout;
  
  /** Creates a new ArmAuto. */
  public ArmAuto(
  ArmSubsystem armSub,
  double speed, double timeout
  ) {

    this.armSub = armSub;
    this.armSpeed = speed;
    this.timeout = timeout;
   
    addRequirements(armSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    if (timer.get() < timeout) {
      speed = armSpeed;
    } else{
      speed = 0.0;
    }

    armSub.setSpeed(speed);
     SmartDashboard.putNumber("Motor Input", armSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((timer.get() >= timeout));
  }
}
