// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final MotorController m_leftIntake = new PWMTalonSRX(5);
  private final MotorController m_rightIntake = new PWMTalonSRX(6);
  private final MotorControllerGroup m_intakeGroup =
  new MotorControllerGroup(m_leftIntake, m_rightIntake);

  public IntakeSubsystem() {
    m_leftIntake.setInverted(false);
    m_rightIntake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed){
    m_intakeGroup.set(speed);
  }

  public void stopIntake(){
    m_intakeGroup.set(0);
  }
}
