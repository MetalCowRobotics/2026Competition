package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private Spark lightController;
    private String color;

    public LEDSubsystem(int portNum) {
        lightController = new Spark(portNum);
    }

    public void setWhite() {
        lightController.set(0.93);  // Fixed palette white
        color = "White";
    }

    public void setGreen() {
        // Options for darker/more vibrant green:
        lightController.set(0.77);  // Solid Dark Green
        color = "Strobe Green";
    }

    public void setRed() {
        lightController.set(0.61);  // Fixed palette red
        color = "Fixed Red";
    }
    
    public String getColor() {
        return color;
    }
} 