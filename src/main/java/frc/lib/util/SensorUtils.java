// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public final class SensorUtils {
    // HIGH when not pressed, so negate
    public static boolean isBumperSwitchDown(DigitalInput bumperSwitch) {
        return !bumperSwitch.get();
    }

    // TODO: double check allen bradley sensor state
    public static boolean isAllenBradleyDown(DigitalInput allenBradley) {
        return !allenBradley.get();
    }
}
