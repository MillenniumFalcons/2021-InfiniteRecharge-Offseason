package team3647.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import team3647.lib.wpi.HALMethods;

public class TalonSRXUtil {
    /**
     * (254)
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            HALMethods.sendDSError(message + errorCode);
        }
    }
}
