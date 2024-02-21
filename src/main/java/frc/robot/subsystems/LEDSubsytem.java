package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsytem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(150);

    public LEDSubsytem() {
        led.setLength(ledBuffer.getLength());

        // Set the data
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        int index = 0;
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            // Cool Blue
            // ledBuffer.setRGB(i, 11, 76, 181);
            
            //american flag
            // index = (index + 1) % 3;
            // if (index == 0)
            //     ledBuffer.setRGB(i, 255, 255, 255);
            // if (index == 1)
            //     ledBuffer.setRGB(i, 255, 0, 0);
            // if (index == 2)
            //     ledBuffer.setRGB(i, 11, 76, 181);


            ledBuffer.setRGB(i, 117, 13, 158);

        }
        led.setData(ledBuffer);
    }

}
