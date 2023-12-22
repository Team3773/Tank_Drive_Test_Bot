// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BallIntakeCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class RobotContainer {

    public final DriveSubsystem m_drive = new DriveSubsystem();
    public final IntakeSubsystem m_feeder = new IntakeSubsystem();

    private final XboxController controller = new XboxController(0);    

    public RobotContainer() {
        m_drive.setDefaultCommand(new TeleopDrive(
            m_drive,
            () -> controller.getLeftX(),
            () -> controller.getLeftY()            
        ));
        m_feeder.setDefaultCommand(new BallIntakeCommand(
            m_feeder,
            ()-> controller.getLeftBumper(),
            ()->controller.getRightBumper()
        ));
    }; 
}
