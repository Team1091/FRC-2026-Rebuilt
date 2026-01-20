// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public final class Autos {

  // Spin wildly for a second.
  public static Command spinAuto(Drive drive) {
    return  Commands.race(
            DriveCommand.joystickDrive(drive, ()->0.0, ()->0.0, ()->1.0),
            new TimerCommand(2000)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
