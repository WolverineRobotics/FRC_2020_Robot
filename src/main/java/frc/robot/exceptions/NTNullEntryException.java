package frc.robot.exceptions;

/**
 * Thown when getting an entry from NetworkTables returns the default value.
 */
public class NTNullEntryException extends Exception {
    public NTNullEntryException(String message) {
        super(message);
    }
}