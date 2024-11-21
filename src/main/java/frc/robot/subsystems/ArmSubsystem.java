// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

  Encoder encoder = new Encoder(0, 1);

  private final WPI_VictorSPX motor = new WPI_VictorSPX(Constants.MotorConstants.motorPort);

  /** Creates a new Arm. */
  public ArmSubsystem() {
    motor.setNeutralMode(NeutralMode.Brake);
    encoder.reset();
  }

  @Override
  public void periodic() {
   SmartDashboard.putNumber("Encoder output", getDistance());
  }

  public double getDistance(){
    return encoder.getDistance() * EncoderConstants.encoderCountsToMeters;
  }

  public void setSpeed(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, -MotorConstants.maxSpeed, MotorConstants.maxSpeed);
    motor.set(clampedSpeed);
  }
}