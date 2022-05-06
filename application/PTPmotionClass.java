package application;

//Copyright: Mohammad SAFEEA, 18th-April-2018
//Support iiwa 14 R 820
//Flange functions are disabled, no flange is used in the script

import static com.kuka.roboticsAPI.motionModel.BasicMotions.circ;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.StringTokenizer;


//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;

public class PTPmotionClass {
	private static LBR _lbr; 
	private Controller kuka_Sunrise_Cabinet_1;
	//private MediaFlangeIOGroup daIO;
	
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));

	
	PTPmotionClass(LBR _lbr,Controller kuka_Sunrise_Cabinet_1)
	{
		this._lbr=_lbr;
		this.kuka_Sunrise_Cabinet_1=kuka_Sunrise_Cabinet_1;
		//daIO=new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
	}

	
	/*
	 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 */
	public static void PTPmotionJointSpaceTorquesConditional(int n,double[] indices,double[] minTorque, double[] maxTorque)
	{
		
		int num=n;
		if(num==0)
		{
			// Do not perform motion
			String tempString="done"+stopCharacter;
			BG_Server.sendCommand(tempString);
			return;
		}
		
		ICondition comb=generateTorqueCondition( n, indices, minTorque, maxTorque);
				
		double[] distPos=new double [7];
		for(int i=0;i<7;i++)
		{
			distPos[i]=MyTask.jpos[i];
		}
		
		_lbr.move(
        		ptp(distPos[0],distPos[1],
        				distPos[2],
        				distPos[3],distPos[4],
        				distPos[5],
        				distPos[6]).setJointVelocityRel(MyTask.jRelVel).breakWhen(comb));
		
		boolean interruptionFlag=false;
		for(int i=0;i<7;i++)
		{
			double x1=_lbr.getCurrentJointPosition().get(i);
			double x2=distPos[i];
			double clearance=0.1*Math.PI/180;
			if(Math.abs(x1-x2)>clearance)
			{
				interruptionFlag=true;
				break;
			}
		}
		// Return back the acknowledgment message
		String tempString;
		if(interruptionFlag==true)
		{
			tempString="interrupted"+stopCharacter;
		}
		else
		{
			tempString="done"+stopCharacter;
		}
		
		BG_Server.sendCommand(tempString);
		
	}
	
	public static void PTPmotionLineCartizianSpaceTorquesConditional(int n,double[] indices,double[] minTorque, double[] maxTorque)
	{
		
		int num=n;
		if(num==0)
		{
			// Do not perform motion
			String tempString="done"+stopCharacter;
			BG_Server.sendCommand(tempString);
			return;
		}
		
		ICondition comb=generateTorqueCondition( n, indices, minTorque, maxTorque);
				
		double[] distPos=new double [6];
		for(int i=0;i<6;i++)
		{
			distPos[i]=MyTask.EEFpos[i];
		}

		
		Frame daframe= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		int kkk=0;
		daframe.setX(distPos[kkk]);
		kkk=kkk+1;
		daframe.setY(distPos[kkk]);
		kkk=kkk+1;
		daframe.setZ(distPos[kkk]);
		kkk=kkk+1;
		daframe.setAlphaRad(distPos[kkk]);
		kkk=kkk+1;
		daframe.setBetaRad(distPos[kkk]);		
		kkk=kkk+1;
		daframe.setGammaRad(distPos[kkk]);
		
		IMotionContainer x=_lbr.move(lin(daframe).setCartVelocity(MyTask.jRelVel).breakWhen(comb));
		//System.out.println("After move");
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Bloco catch gerado automaticamente
			e.printStackTrace();
		}
		double[] reachedPos=new double[6];
		Frame reachedFrame= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		kkk=0;
		reachedPos[kkk]=reachedFrame.getX();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getY();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getZ();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getAlphaRad();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getBetaRad();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getGammaRad();
		
		boolean interruptionFlag=false;
		if(x.isFinished()==false)
		{
			interruptionFlag=true;
		}
		for(int i=0;i<3;i++)
		{
			double x1=reachedPos[i];
			double x2=distPos[i];
			double clearance=5*Math.PI/180; // angular clearance (rad)
			if(i<3)
			{
				clearance=0.2; // linear clearance (mm)
			}
			
			if(Math.abs(x1-x2)>clearance)
			{
				interruptionFlag=true;
				break;
			}
		}
		
		// Return back the acknowledgment message
		String tempString;
		if(interruptionFlag==true)
		{
			tempString="interrupted"+stopCharacter;
		}
		else
		{
			tempString="done"+stopCharacter;
		}
		
		BG_Server.sendCommand(tempString);

		
	}
	
	public static void PTPmotionCartizianSpaceCircleTorquesConditional(int n,double[] indices,double[] minTorque, double[] maxTorque)
	{
		
		int num=n;
		if(num==0)
		{
			// Do not perform motion
			String tempString="done"+stopCharacter;
			BG_Server.sendCommand(tempString);
			return;
		}
		
		ICondition comb=generateTorqueCondition( n, indices, minTorque, maxTorque);
					
		Frame daframe1= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		Frame daframe2= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		int kkk=0;
		daframe1.setX(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setY(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setZ(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setAlphaRad(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setBetaRad(MyTask.EEFposCirc1[kkk]);		
		kkk=kkk+1;
		daframe1.setGammaRad(MyTask.EEFposCirc1[kkk]);
		
		kkk=0;
		daframe2.setX(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setY(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setZ(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setAlphaRad(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setBetaRad(MyTask.EEFposCirc2[kkk]);		
		kkk=kkk+1;
		daframe2.setGammaRad(MyTask.EEFposCirc2[kkk]);
		
		double[] distPos=new double [6];
		for(int i=0;i<6;i++)
		{
			distPos[i]=MyTask.EEFposCirc2[i];
		}
		
		_lbr.move(circ(daframe1,daframe2).setCartVelocity(MyTask.jRelVel).breakWhen(comb));
		
		double[] reachedPos=new double[6];
		Frame reachedFrame= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		kkk=0;
		reachedPos[kkk]=reachedFrame.getX();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getY();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getZ();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getAlphaRad();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getBetaRad();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getGammaRad();
		
		boolean interruptionFlag=false;
		for(int i=0;i<6;i++)
		{
			double x1=reachedPos[i];
			double x2=distPos[i];
			double clearance=0.1*Math.PI/180; // angular clearance (rad)
			if(i>2)
			{
				clearance=0.05; // linear clearance (mm)
			}
			
			if(Math.abs(x1-x2)>clearance)
			{
				interruptionFlag=true;
				break;
			}
		}
		// Return back the acknowledgment message
		String tempString;
		if(interruptionFlag==true)
		{
			tempString="interrupted"+stopCharacter;
		}
		else
		{
			tempString="done"+stopCharacter;
		}
		
		BG_Server.sendCommand(tempString);

		
	}
	
	public static void PTPmotionCartizianSpaceRelWorldTorquesConditional(int n,double[] indices,double[] minTorque, double[] maxTorque)
	{
		
		int num=n;
		if(num==0)
		{
			// Do not perform motion
			String tempString="done"+stopCharacter;
			BG_Server.sendCommand(tempString);
			return;
		}
		
		ICondition comb=generateTorqueCondition( n, indices, minTorque, maxTorque);
		
		Frame daframe= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		

		double[] distPos=new double [3];
		distPos[0]=MyTask.EEFpos[0]+daframe.getX();
		distPos[1]=MyTask.EEFpos[1]+daframe.getY();
		distPos[2]=MyTask.EEFpos[2]+daframe.getZ();
		
		daframe.setX(distPos[0]);

		daframe.setY(distPos[1]);

		daframe.setZ(distPos[2]);

		
		
		_lbr.move(lin(daframe).setCartVelocity(MyTask.jRelVel).breakWhen(comb));
		
		double[] reachedPos=new double[3];
		Frame reachedFrame= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		int kkk=0;
		reachedPos[kkk]=reachedFrame.getX();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getY();
		kkk=kkk+1;
		reachedPos[kkk]=reachedFrame.getZ();

		
		boolean interruptionFlag=false;
		for(int i=0;i<3;i++)
		{
			double x1=reachedPos[i];
			double x2=distPos[i];
			double clearance=0.05; // linear clearance (mm)
			if(Math.abs(x1-x2)>clearance)
			{
				interruptionFlag=true;
				break;
			}
		}
		// Return back the acknowledgment message
		String tempString;
		if(interruptionFlag==true)
		{
			tempString="interrupted"+stopCharacter;
		}
		else
		{
			tempString="done"+stopCharacter;
		}
		
		BG_Server.sendCommand(tempString);
		
	}
	
	
	public static void PTPmotionCartizianSpaceRelEEfTorquesConditional(int n,double[] indices,double[] minTorque, double[] maxTorque)
	{
		
		double x,y,z;
		x=MyTask.EEFpos[0];

		y=MyTask.EEFpos[1];

		z=MyTask.EEFpos[2];

		
		_lbr.getFlange().move(linRel(x,y,z).setCartVelocity(MyTask.jRelVel));
		
		String tempString="done"+stopCharacter;
		BG_Server.sendCommand(tempString);
		
	}

	/*
	 * ************************************
	 *  Get condition functions
	 *  ***********************************
	 */
	
	private static ICondition generateTorqueCondition(int n,double[] indices,double[] minTorque, double[] maxTorque)
	{
		int num=n;
		
		JointTorqueCondition[] con= new JointTorqueCondition [7];
		JointEnum[] jointsNum=new JointEnum[7];
		jointsNum[0]=JointEnum.J1;
		jointsNum[1]=JointEnum.J2;
		jointsNum[2]=JointEnum.J3;
		jointsNum[3]=JointEnum.J4;
		jointsNum[4]=JointEnum.J5;
		jointsNum[5]=JointEnum.J6;
		jointsNum[6]=JointEnum.J7;
				
		for(int i=0;i<num;i++)
		{
			int index=(int)indices[i];
			con[i]=new JointTorqueCondition(jointsNum[index],minTorque[i],maxTorque[i]);
		}
		
		ICondition comb=con[0];
		
		for(int i=1;i<num;i++)
		{
			comb=comb.or(con[i]);
		}
		
		return comb;

	}
	/*
	 * ********************************************************************************************************************************************
	 */
	public static void PTPmotionJointSpace()
	{
		System.out.println("In the Ptp JS function");
		_lbr.move(
        		ptp(MyTask.jpos[0],MyTask.jpos[1],
        				MyTask.jpos[2],
        				MyTask.jpos[3],MyTask.jpos[4],
        				MyTask.jpos[5],
        				MyTask.jpos[6]).setJointVelocityRel(MyTask.jRelVel));
		
		String tempString="done"+stopCharacter;
		BG_Server.sendCommand(tempString);
		
	}
	
	public static void PTPmotionCartizianSpace()
	{
		Frame daframe= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		int kkk=0;
		daframe.setX(MyTask.EEFpos[kkk]);
		kkk=kkk+1;
		daframe.setY(MyTask.EEFpos[kkk]);
		kkk=kkk+1;
		daframe.setZ(MyTask.EEFpos[kkk]);
		kkk=kkk+1;
		daframe.setAlphaRad(MyTask.EEFpos[kkk]);
		kkk=kkk+1;
		daframe.setBetaRad(MyTask.EEFpos[kkk]);		
		kkk=kkk+1;
		daframe.setGammaRad(MyTask.EEFpos[kkk]);

		//System.out.println("B4 move" + MatlabToolboxServer.jRelVel);
		LIN L1 = lin(daframe).setCartVelocity(MyTask.jRelVel);
		L1.setBlendingCart(1);
		//System.out.println("cartisian acceleration " + L1.getCartAcceleration());
		//L1.setCartAcceleration(0.1);
		_lbr.move(L1);

		//System.out.println("After move");
		
		String tempString="done"+stopCharacter;
		BG_Server.sendCommand(tempString);
		
	}
	
	public static void PTPmotionCartizianSpaceCircle()
	{
		Frame daframe1= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		Frame daframe2= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		int kkk=0;
		daframe1.setX(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setY(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setZ(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setAlphaRad(MyTask.EEFposCirc1[kkk]);
		kkk=kkk+1;
		daframe1.setBetaRad(MyTask.EEFposCirc1[kkk]);		
		kkk=kkk+1;
		daframe1.setGammaRad(MyTask.EEFposCirc1[kkk]);
		
		kkk=0;
		daframe2.setX(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setY(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setZ(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setAlphaRad(MyTask.EEFposCirc2[kkk]);
		kkk=kkk+1;
		daframe2.setBetaRad(MyTask.EEFposCirc2[kkk]);		
		kkk=kkk+1;
		daframe2.setGammaRad(MyTask.EEFposCirc2[kkk]);
		
		
		_lbr.move(circ(daframe1,daframe2).setCartVelocity(MyTask.jRelVel));
		
		String tempString="done"+stopCharacter;
		BG_Server.sendCommand(tempString);
		
	}
	
	public static void PTPmotionCartizianSpaceRelWorld()
	{
		
		Frame daframe= _lbr.getCurrentCartesianPosition(_lbr.getFlange());
		

		daframe.setX(MyTask.EEFpos[0]+daframe.getX());

		daframe.setY(MyTask.EEFpos[1]+daframe.getY());

		daframe.setZ(MyTask.EEFpos[2]+daframe.getZ());

		
		
		_lbr.move(lin(daframe).setCartVelocity(MyTask.jRelVel));
		
		String tempString="done"+stopCharacter;
		BG_Server.sendCommand(tempString);
		
	}
	
	
	public static void PTPmotionCartizianSpaceRelEEf()
	{
		
		double x,y,z;
		x=MyTask.EEFpos[0];

		y=MyTask.EEFpos[1];

		z=MyTask.EEFpos[2];

		
		_lbr.getFlange().move(linRel(x,y,z).setCartVelocity(MyTask.jRelVel));
		
		String tempString="done"+stopCharacter;
		BG_Server.sendCommand(tempString);
		
	}
	
}
