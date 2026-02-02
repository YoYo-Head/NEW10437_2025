package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Auto {
    private boolean found = false;

    public void searchForGyro() {
        while (!found){
            try {
                Robot robot = new Robot();

                float pitch = robot.ahrs.getPitch();

                

                
                DriverStation.reportWarning(pitch, false);
                
            }
                }
                    }

    
}


