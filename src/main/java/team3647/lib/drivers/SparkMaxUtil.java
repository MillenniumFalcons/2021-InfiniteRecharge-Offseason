package team3647.lib.drivers;

import com.revrobotics.CANError;
import team3647.lib.wpi.HALMethods;
/**
 * (254)
 */
public class SparkMaxUtil {
    // Checks the specified error code for issues.
    public static void checkError(CANError errorCode, String message) {
        if (errorCode != CANError.kOk) {
            HALMethods.sendDSError(message + errorCode);
        }
    }
}
