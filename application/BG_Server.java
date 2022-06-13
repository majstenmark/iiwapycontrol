package application;


//Copyright: Mohammad SAFEEA, 18th-April-2018
//Support iiwa 14 R 820
//Flange functions are disabled, no flange is used in the script

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

import sun.security.action.GetLongAction;


//import com.kuka.connectivity.motionModel.directServo.DirectServo;
//import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class BG_Server extends RoboticsAPIBackgroundTask {

//////////////////////////For testing
    private static String msg = "";
    private static String echo = "";
    ////////////////////////
    
	//private MediaFlangeIOGroup daIO;
	
	private static boolean appRunning;
	private int _port;
	private int _timeOut;
	private static ServerSocket ss;
	private static Socket soc;
	
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));
    private static final String ack="done"+stopCharacter;

   
    public void initialize() {
    	appRunning = true;
    	_port=30001;
		_timeOut=60*1000;  // milli seconds
		
   }
    

public void run(){
	// TODO Auto-generated method stub
	//if(appRunning){
	while(true){
	try {
		ss= new ServerSocket(_port);
		try
		{
		ss.setSoTimeout(_timeOut);
		soc= ss.accept();
		appRunning = true;
		}
		catch (Exception e) {
			// TODO: handle exception
			ss.close();
			PythonCommands.terminateFlag=true;
			//return;
		}
		Scanner scan= new Scanner(soc.getInputStream());
		// In this loop you shall check the input signal
		while((soc.isConnected() && appRunning))
		{
			if(scan.hasNextLine())
			{			
				PythonCommands.daCommand=scan.nextLine();
				if(!PythonCommands.daCommand.equals("")) System.out.println("BG: got command " + PythonCommands.daCommand);
				
				 
				if(PythonCommands.daCommand.startsWith("jf_"))
	        	{
	        		boolean tempBool=getTheJointsf(PythonCommands.daCommand);
	        		PythonCommands.daCommand="";
	        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
	        		if(tempBool==false)
	        		{
	        			PythonCommands.directSmart_ServoMotionFlag=false;
	        		}
	        		// this.sendCommand(ack); no acknowledgement in fast execution mode
	        	}
				// If the signal is equal to end, you shall turn off the server.
				else if(PythonCommands.daCommand.startsWith("end"))
				{
					/* Close all existing loops:
					/  1- The BackgroundTask loop.
					 * 2- The main class, MatlabToolboxServer loops:
					 * 		a- The while loop in run, using the flag: MatlabToolboxServer.terminateFlag.
					 * 		b- The direct servo loop, using the flag: MatlabToolboxServer.directServoMotionFlag.
					*/
					PythonCommands.directSmart_ServoMotionFlag=false;
					PythonCommands.terminateFlag=true;
					break;						
				}
				// Put the direct_servo joint angles command in the joint variable
				else if(PythonCommands.daCommand.startsWith("jp"))
	        	{
	        		updateJointsPositionArray();
	        	}
				else if(PythonCommands.daCommand.startsWith("vel"))
	        	{
	        		updateVelocityArrays();
	        	}
				else if(PythonCommands.daCommand.startsWith("cArtixanPosition"))
	        	{
					if(PythonCommands.daCommand.startsWith("cArtixanPositionCirc1"))
					{
		        		boolean tempBool=getEEFposCirc1(PythonCommands.daCommand);
		        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		        		if(tempBool==false)
		        		{
		        			//MatlabToolboxServer.directServoMotionFlag=false;
		        		}
		        		this.sendCommand(ack);
		        		PythonCommands.daCommand="";
					}
					else if(PythonCommands.daCommand.startsWith("cArtixanPositionCirc2"))
					{
		        		boolean tempBool=getEEFposCirc2(PythonCommands.daCommand);
		        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		        		if(tempBool==false)
		        		{
		        			//MatlabToolboxServer.directServoMotionFlag=false;
		        		}
		        		this.sendCommand(ack);
		        		PythonCommands.daCommand="";
					}
					else
					{
		        		boolean tempBool=getEEFpos(PythonCommands.daCommand);
		        		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		        		if(tempBool==false)
		        		{
		        			//MatlabToolboxServer.directServoMotionFlag=false;
		        		}
		        		this.sendCommand(ack);
		        		PythonCommands.daCommand="";
					}
	        	}
				
				// This insturction is used to turn_off the direct_servo controller
	        	else if(PythonCommands.daCommand.startsWith("stopDirectServoJoints"))
	        	{
	        		PythonCommands.directSmart_ServoMotionFlag=false;
	        		this.sendCommand(ack);
	        		PythonCommands.daCommand="";
	        	}
				else if(PythonCommands.daCommand.startsWith("DcSe"))
	        	{
	        		updateEEFPositionArray();
	        	}
	        	else
	        	{
	        		// inquiring data from server
	        		dataAqcuisitionRequest();
	        	}
				
			}				
		}
	} catch (IOException e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
		//PythonCommands.terminateFlag=true;
		appRunning = false; // soc not connected
	}
	
	//Close the socket!!!
	try {
		PythonCommands.terminateFlag=true;
		soc.close();
		ss.close();
		appRunning = false;
	} catch (IOException e) {
		e.printStackTrace();
	}
	}
	
	
	
	
}

private void updateVelocityArrays()
{
	if(PythonCommands.daCommand.startsWith("velJDC_"))
	{
		boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		this.sendCommand(ack);
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("velJDCExT_"))
	{
		boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendJointsExternalTorquesToClient();
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("velJDCMT_"))
	{
		boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendJointsMeasuredTorquesToClient();
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("velJDCEEfP_"))
	{
		boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendEEFforcesToClient();
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("velJDCJP_"))
	{
		boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendJointsPositionsToClient();
		PythonCommands.daCommand="";
	}
}

private void updateEEFPositionArray()
{
	//////////////////////////////////////////////////
	//Start of server update functions
	/////////////////////////////////////////////////////						
	
	if(PythonCommands.daCommand.startsWith("DcSeCar_"))
	{
		boolean tempBool=getThePositions(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		// this.sendCommand(ack);
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("DcSeCarExT_"))
	{
		boolean tempBool=getTheJoints(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendJointsExternalTorquesToClient();
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("DcSeCarMT_"))
	{
		boolean tempBool=getTheJoints(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendJointsMeasuredTorquesToClient();
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("DcSeCarEEfP_"))
	{
		boolean tempBool=getTheJoints(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendEEFforcesToClient();
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("DcSeCarJP_"))
	{
		boolean tempBool=getTheJoints(PythonCommands.daCommand);
		// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
		if(tempBool==false)
		{
		PythonCommands.directSmart_ServoMotionFlag=false;
		}
		PythonCommands.svr.sendJointsPositionsToClient();
		PythonCommands.daCommand="";
	}
	
	//////////////////////////////////////////////////
	//End of Servo joints update functions
	//////////////////////////////////////////////////////
}

private boolean getThePositions(String thestring) {
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
			int j=0;
			while(st.hasMoreTokens())
			{
				if(j<6)
				{
					//getLogger().warn(jointString);
					try
					{
						PythonCommands.EEfServoPos[j]=Double.parseDouble(st.nextToken());
					}
					catch(Exception e)
					{
						return false;
					}						
				}					
				j++;
			}
			PythonCommands.daCommand="";
			return true;
			
		}
		else
		{
			return false;
		}
}

private boolean getJointsVelocitiesForVelocityContrtolMode(String thestring) {
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
			int j=0;
			while(st.hasMoreTokens())
			{
				if(j<7)
				{
					//getLogger().warn(jointString);
					try
					{
						PythonCommands.jvel[j]=
						Double.parseDouble(st.nextToken());
					}
					catch(Exception e)
					{
						return false;
					}						
				}					
				j++;
			}
			PythonCommands.daCommand="";
			return true;
			
		}
		else
		{
			return false;
		}
}


private void updateJointsPositionArray()
{
	//////////////////////////////////////////////////
	//Start of server update functions
	/////////////////////////////////////////////////////						
	
	if(PythonCommands.daCommand.startsWith("jp_"))
	{
	boolean tempBool=getTheJoints(PythonCommands.daCommand);
	// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
	if(tempBool==false)
	{
	PythonCommands.directSmart_ServoMotionFlag=false;
	}
	this.sendCommand(ack);
	PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("jpExT_"))
	{
	boolean tempBool=getTheJoints(PythonCommands.daCommand);
	// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
	if(tempBool==false)
	{
	PythonCommands.directSmart_ServoMotionFlag=false;
	}
	PythonCommands.svr.sendJointsExternalTorquesToClient();
	PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("jpMT_"))
	{
	boolean tempBool=getTheJoints(PythonCommands.daCommand);
	// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
	if(tempBool==false)
	{
	PythonCommands.directSmart_ServoMotionFlag=false;
	}
	PythonCommands.svr.sendJointsMeasuredTorquesToClient();
	PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("jpEEfP_"))
	{
	boolean tempBool=getTheJoints(PythonCommands.daCommand);
	// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
	if(tempBool==false)
	{
	PythonCommands.directSmart_ServoMotionFlag=false;
	}
	PythonCommands.svr.sendEEFforcesToClient();
	PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("jpJP_"))
	{
	boolean tempBool=getTheJoints(PythonCommands.daCommand);
	// MatlabToolboxServer.printMessage(MatlabToolboxServer.daCommand);
	if(tempBool==false)
	{
	PythonCommands.directSmart_ServoMotionFlag=false;
	}
	PythonCommands.svr.sendJointsPositionsToClient();
	PythonCommands.daCommand="";
	}
	
	//////////////////////////////////////////////////
	//End of Servo joints update functions
	//////////////////////////////////////////////////////
}

// respond to a data Acquisition Request
private void dataAqcuisitionRequest()
{
	// Inquiring data from server
	if(PythonCommands.daCommand.startsWith("getJointsPositions"))
	{
		PythonCommands.svr.sendJointsPositionsToClient();
		//sendCommand(ack); //this was added!
		
		PythonCommands.daCommand="";
	}        	
	// Write output of Mediaflange
	else if(PythonCommands.daCommand.startsWith("blueOn"))
	{
		PythonCommands.mff.blueOn();
		sendCommand(ack);
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("blueOff"))
	{
		PythonCommands.mff.blueOff();
		sendCommand(ack);
		PythonCommands.daCommand="";
	}
	else if(PythonCommands.daCommand.startsWith("pin"))
	{
    	if(PythonCommands.daCommand.startsWith("pin1on"))
		{
    		PythonCommands.mff.pin1On();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin1off"))
		{
			PythonCommands.mff.pin1Off();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin11on"))
		{
			PythonCommands.mff.pin11On();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin11off"))
		{
			PythonCommands.mff.pin11Off();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin2on"))
		{
			PythonCommands.mff.pin2On();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin2off"))
		{
			PythonCommands.mff.pin2Off();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin12on"))
		{
			PythonCommands.mff.pin12On();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("pin12off"))
		{
			PythonCommands.mff.pin12Off();
			sendCommand(ack);
			PythonCommands.daCommand="";
		}
	}
	// Read input of Mediaflange
	if(PythonCommands.daCommand.startsWith("getPin"))
	{
		if(PythonCommands.daCommand.startsWith("getPin10"))
		{
			PythonCommands.mff.getPin10();
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("getPin16"))
		{
			PythonCommands.mff.getPin16();
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("getPin13"))
		{
			PythonCommands.mff.getPin13();
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("getPin3"))
		{
			PythonCommands.mff.getPin3();
			PythonCommands.daCommand="";
		}
		else if(PythonCommands.daCommand.startsWith("getPin4"))
		{
			PythonCommands.mff.getPin4();
			PythonCommands.daCommand="";
		}
	}
}


/* The following function is used to extract the 
 joint angles from the command
 */
private boolean getTheJoints(String thestring)
{
	
	
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
		if(temp.startsWith("jp"))
		{
			int j=0;
			while(st.hasMoreTokens())
			{
				String jointString=st.nextToken();
				if(j<7)
				{
					//getLogger().warn(jointString);
					PythonCommands.jpos[j]=Double.parseDouble(jointString);
				}
				
				j++;
			}
			PythonCommands.daCommand="";
			return true;
			
		}
		else
		{
			return false;
		}
	}

	return false;
}

/* The following function is used to extract the 
 joint angles from the command
 */
private boolean getTheJointsf(String thestring)
{
	
	
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
		if(temp.startsWith("jf"))
		{
			int j=0;
			while(st.hasMoreTokens())
			{
				String jointString=st.nextToken();
				if(j<7)
				{
					//getLogger().warn(jointString);
					PythonCommands.jpos[j]=Double.parseDouble(jointString);
				}
				
				j++;
			}
			return true;
			
		}
		else
		{
			return false;
		}
	}

	return false;
}

private boolean getEEFpos(String thestring)
{
	
	
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
		if(temp.startsWith("cArtixanPosition"))
		{
			int j=0;
			while(st.hasMoreTokens())
			{
				String jointString=st.nextToken();
				if(j<6)
				{
					//getLogger().warn(jointString);
					PythonCommands.EEFpos[j]=Double.parseDouble(jointString);
				}
				
				j++;
			}
			PythonCommands.daCommand="";
			return true;
			
		}
		else
		{
			return false;
		}
	}

	return false;
}


private boolean getEEFposCirc2(String thestring)
{
	
	
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
		if(temp.startsWith("cArtixanPosition"))
		{
			int j=0;
			while(st.hasMoreTokens())
			{
				String jointString=st.nextToken();
				if(j<6)
				{
					//getLogger().warn(jointString);
					PythonCommands.EEFposCirc2[j]=Double.parseDouble(jointString);
				}
				
				j++;
			}
			PythonCommands.daCommand="";
			return true;
			
		}
		else
		{
			return false;
		}
	}

	return false;
}


private boolean getEEFposCirc1(String thestring)
{
	
	
	StringTokenizer st= new StringTokenizer(thestring,"_");
	if(st.hasMoreTokens())
	{
		String temp=st.nextToken();
		if(temp.startsWith("cArtixanPosition"))
		{
			int j=0;
			while(st.hasMoreTokens())
			{
				String jointString=st.nextToken();
				if(j<6)
				{
					//getLogger().warn(jointString);
					PythonCommands.EEFposCirc1[j]=Double.parseDouble(jointString);
				}
				
				j++;
			}
			PythonCommands.daCommand="";
			return true;
			
		}
		else
		{
			return false;
		}
	}

	return false;
}
/* The following function is used to sent a string message
 * to the server
 */
public static boolean sendCommand(String s)
{
	if(ss==null)return false;
	if(soc==null)return false;
	if(soc.isClosed())return false;
	
	try {
		soc.getOutputStream().write(s.getBytes("UTF-8"));
		soc.getOutputStream().flush();
		return true;
		/*
		OutputStream output = soc.getOutputStream();
		int len = s.length() * 100;
		ByteBuffer b = ByteBuffer.allocate(4);
		b.putInt(len);
		output.write(b.array());
		output.write(s.getBytes("UTF-8"));
		output.flush();
		System.out.println("Sent ack");
		*/
	} catch (IOException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	
	return false;
	
}

	public static void stop(){
		appRunning = false;
		try {
			soc.close();
			ss.close();
		} catch (IOException e) {
			e.printStackTrace();
		}catch(Exception e){
			//Nullpointer
		}
	}
	
	public void dispose(){
		appRunning = false;
		try {
			soc.close();
			ss.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	} 
}
