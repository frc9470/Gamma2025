// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import com.team9470.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer
{
    CommandXboxController xbox = new CommandXboxController(0);

    // ---------------- SUBSYSTEMS --------------------
    private final Elevator elevator = new Elevator();

    public RobotContainer()
    {
        configureBindings();
    }
    
    
    private void configureBindings() {
        xbox.getButtonA().whenPressed(elevator::home);
        xbox.getButtonB().whenPressed(elevator.L1());
        xbox.getButtonX().whenPressed(elevator.L3());
        xbox.getButtonY().whenPressed(elevator.L4());
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
