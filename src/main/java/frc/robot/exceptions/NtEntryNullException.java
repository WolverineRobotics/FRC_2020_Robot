package frc.robot.exceptions;

/**
 * Thown when getting an entry from NetworkTables returns the default value.
 */
public class NtEntryNullException extends Exception {
    public NtEntryNullException() {
    }

    public NtEntryNullException(String message) {
        super(message);
    }
}