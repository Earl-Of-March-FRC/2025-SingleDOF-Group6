// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmAuto;
import frc.robot.commands.ArmRotate;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem armSub = new ArmSubsystem();
  private final XboxController xboxController = new XboxController(OperatorConstants.driverControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    armSub.setDefaultCommand(
      new ArmRotate(
        armSub,
        () -> MathUtil.applyDeadband(xboxController.getLeftY(), OperatorConstants.joystickDeadband)
      )
    );
    
    /*
    autonomous not completed / tested 
    DON'T TRY IT
     */

    //autoChooser.setDefaultOption("Auto Phase 1", new ArmAuto(armSub, MotorConstants.autoSpeed, MotorConstants.autoTimeout));
   
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
