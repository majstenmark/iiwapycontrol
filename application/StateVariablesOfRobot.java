package application;

//Copyright: Mohammad SAFEEA, 18th-April-2018
//Support iiwa 14 R 820
//Flange functions are disabled, no flange is used in the script


import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

public class StateVariablesOfRobot {
    
	private LBR _lbr; 	
    
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));

    StateVariablesOfRobot(LBR _lbr)
	{
		this._lbr=_lbr;
	}
	
    public void sendJointsMeasuredTorquesToClient() {
    	
    	TorqueSensorData measuredData= _lbr.getMeasuredTorque();
    	double[] vals=
    			measuredData.getTorqueValues();
    	String s="";
    	for(int i=0;i<vals.length;i++)
    	{
    		s=s+Double.toString(vals[i])+"_";
    	}
		s=s+stopCharacter;
		while(!BG_Server.sendCommand(s))
		{
			BG_Server.sendCommand(s);
		}
		MyTask.daCommand=""; // clear the command
    	
    }
    
    public void sendJointsExternalTorquesToClient() {
    	
    	TorqueSensorData measuredData= _lbr.getExternalTorque();
    	double[] vals=
    			measuredData.getTorqueValues();
    	String s="";
    	for(int i=0;i<vals.length;i++)
    	{
    		s=s+Double.toString(vals[i])+"_";
    	}
		s=s+stopCharacter;
		while(!BG_Server.sendCommand(s))
		{
			BG_Server.sendCommand(s);
		}
		MyTask.daCommand=""; // clear the command
    	
    }
    
    public void sendEEFforcesToClient() {
    	
		ForceSensorData cforce = _lbr.getExternalForceTorque(_lbr.getFlange());
		
		String 	cmdforce=Double.toString(cforce.getForce().getX())
				+"_"+Double.toString(cforce.getForce().getY())
				+"_"+Double.toString(cforce.getForce().getZ())
				+"_"+stopCharacter;
		while(!BG_Server.sendCommand(cmdforce))
		{
			BG_Server.sendCommand(cmdforce);
		}
		MyTask.daCommand=""; // clear the command
    }
    
    public void sendEEFMomentsToClient() {
    	
		ForceSensorData cforce = _lbr.getExternalForceTorque(_lbr.getFlange());
		
		String 	cmdforce=Double.toString(cforce.getTorque().getX())
				+"_"+Double.toString(cforce.getTorque().getY())
				+"_"+Double.toString(cforce.getTorque().getZ())
				+"_"+stopCharacter;
		while(!BG_Server.sendCommand(cmdforce))
		{
			BG_Server.sendCommand(cmdforce);
		}
		MyTask.daCommand=""; // clear the command
    }

    public void sendJointsPositionsToClient() {
		// This functions sends the joints positions to the client
    	String s="";
		JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
		for(int i=0;i<7;i++)
		{
	        s=s+Double.toString(initialPosition.get(i))+"_";       	        
		}
		s=s+stopCharacter;
		while(!BG_Server.sendCommand(s))
		{
			BG_Server.sendCommand(s);
		}
		MyTask.daCommand=""; // clear the command


		
	}
    
    public void sendEEfPositionToClient() {
		// This functions sends the end effector position to the client
    	String cmdPos="";
		// Read Cartesian position data
		Frame daframe= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		cmdPos= 
		Double.toString(daframe.getX())
				+"_"+Double.toString(daframe.getY())
				+"_"+Double.toString(daframe.getZ())
				+"_"+Double.toString(daframe.getAlphaRad())
				+"_"+Double.toString(daframe.getBetaRad())
				+"_"+Double.toString(daframe.getGammaRad())
				+"_"+stopCharacter;
		//Send the data back to the client
		while(!BG_Server.sendCommand(cmdPos))
		{
			BG_Server.sendCommand(cmdPos);
		}
		MyTask.daCommand=""; // clear the command	
	}
    


}
