package frc.robot.constants;

import java.util.HashMap;

public class JoystickMap {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    //********************************************************************************** 
    // Analog inputs
    //**********************************************************************************

    public enum AxisMap{
        LEFT_STICK_X(0),
        LEFT_STICK_Y(1),
        LEFT_TRIGGER(2),
        RIGHT_TRIGGER(3),
        RIGHT_STICK_X(4),
        RIGHT_STICK_Y(5),
        ;

        private int port;

        AxisMap(int port){
            this.port = port;
        }

        public int getPort(){
            return this.port;
        }
    }

    
    //********************************************************************************** 
    // Boolean inputs
    //**********************************************************************************

    public enum ButtonMap{
        BUTTON_A(1),
        BUTTON_B(2),
        BUTTON_X(3),
        BUTTON_Y(4),
        BUTTON_LEFT_BUMPER(5),
        BUTTON_RIGHT_BUMPER(6),
        BUTTON_SELECT(7),
        BUTTON_START(8),
        ;

        private int portNum;

        ButtonMap(int portNum){
            this.portNum = portNum;
        }

        public int getPortNum(){
            return this.portNum;
        }
    }

    //********************************************************************************** 
    // POV inputs
    //**********************************************************************************

    public enum POVMap{
        POV_OFF(-1),
        POV_NORTH(0),
        POV_EAST(90),
        POV_SOUTH(180),
        POV_WEST(270),
        POV_NORTH_EAST(45),
        POV_SOUTH_EAST(135),
        POV_SOUTH_WEST(225),
        POV_NORTH_WEST(315),
        ;

        private int portNum;
        private static HashMap<Integer, POVMap> map = new HashMap<>();

        static{
            for(final POVMap pov: POVMap.values()){
                map.put(pov.portNum, pov);
            }
        }

        POVMap(int portNum){
            this.portNum = portNum;
        }

        public int getPortNum(){
            return this.portNum;
        }

        public static POVMap getPOV(int port){
            return map.get(port);
        }

    }

}