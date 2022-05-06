package application;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.med.controllerModel.MedController;
import com.kuka.med.robotStateEvent.IRobotStateListener;
import com.kuka.med.robotStateEvent.RobotStateEvent;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.*;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.IControllerStateListener;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

public class Test extends RoboticsAPIApplication   {
	
	public static String msg = "";

    public static String add = "";
	private boolean running = true;
	

	@Inject private LBR _lbr;
	
	private final static String informationText=
         "This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot moves to the transportation position.";


	@Override
	public void initialize(){
		

	}
		
	@Override
	public void run() {
		
        
        
	}
	@Override
	public void dispose(){
		BG_Server.stop();
		super.dispose();
	}
	
	@Override
	public void onApplicationStateChanged(RoboticsAPIApplicationState state){
		getLogger().info("Test: state chagned " + state);
		running = state != RoboticsAPIApplicationState.MOTIONPAUSING;
		if(!running) BG_Server.stop();
	} 
	
	

}
