package application;

//Copyright: Mohammad SAFEEA, 18th-April-2018
//Support iiwa 14 R 820
//Flange functions are disabled, no flange is used in the script

import java.util.StringTokenizer;

public class StringManipulationFunctions {
	
	public static double jointRelativeVelocity(String daCommand)
	{
		StringTokenizer st= new StringTokenizer(daCommand,"_");
		String temp="0";
		if(st.hasMoreElements())temp=st.nextToken();
		if(st.hasMoreElements())temp=st.nextToken();
		return Double.parseDouble(temp);
	}

	public static int get_Indexes_ValBoundaries(String daCommand,double[] indices,double[] minTorque,double[] maxTorque)
	{
		StringTokenizer st=new StringTokenizer(daCommand,"_");
		String temp="0";
		if(st.hasMoreElements())temp=st.nextToken();
		int i=0;
		while(st.hasMoreTokens())
		{
			temp=st.nextToken();
			indices[i]=Double.parseDouble(temp);
			temp=st.nextToken();
			minTorque[i]=Double.parseDouble(temp);
			temp=st.nextToken();
			maxTorque[i]=Double.parseDouble(temp);
			i=i+1;
		}
		return i;
	}
}
