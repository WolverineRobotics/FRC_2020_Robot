package frc.robot.exceptions;

/**
 * Thown when getting an entry from NetworkTables returns the default value.
 */
public class NTEntryNullException extends Exception {
    public NTEntryNullException(String message) {
        super(message);
    }
}