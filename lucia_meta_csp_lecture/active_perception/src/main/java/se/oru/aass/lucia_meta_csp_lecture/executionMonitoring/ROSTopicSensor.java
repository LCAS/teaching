package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;

import geometry_msgs.PoseWithCovariance;

import java.util.Arrays;
import java.util.Vector;

import services.getLocation;
import services.getLocationRequest;
import services.getLocationResponse;
import services.getPanel;
import services.getPanelRequest;
import services.getPanelResponse;
import services.getQR;
import services.getQRRequest;
import services.getQRResponse;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.Activity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.Sensor;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import actionlib_msgs.GoalStatus;

import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.PanelFactory;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

public class ROSTopicSensor extends Sensor {

	private static final long serialVersionUID = -8096347089406131305L;

	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver solver;
	private float[] pose = null;
	private ConnectedNode rosNode = null;
	private ServiceClient<getQRRequest, getQRResponse> serviceClientQR = null;
	private int seenQR = -2;
	private String robot;
	private String robotCurrentPose = "";
	
	public ROSTopicSensor(String rob, ConstraintNetworkAnimator animator, LuciaMetaConstraintSolver metaSolver, final ConnectedNode rosNode) {
		super(rob, animator);
		this.metaSolver = metaSolver;
		this.solver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		this.rosNode = rosNode;
		this.robot = rob;


		monitorRobotPose();
		
		//Subscribe to active topic
		Subscriber<std_msgs.String> poseFeedback = rosNode.newSubscriber("active_sensing", std_msgs.String._TYPE);
		poseFeedback.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				System.out.println(message.getData());
				if(message.getData().equals("active")){
					long timeNow = rosNode.getCurrentTime().totalNsecs()/1000000;
					postSensorValue(robotCurrentPose, timeNow);
				}
			}
		}, 10);
	}


	
	public String getRobotCurrentPoseString(){
		return robotCurrentPose;
	}
	
	public Vec2 getRobotCurrentPose(){
	    float[] newPose = new float[4];
	    String[] poseS = robotCurrentPose.split(",");
	    for (int i = 0; i < poseS.length; i++) newPose[i] = Float.parseFloat(poseS[i]);        
		return new Vec2(newPose[0], newPose[1]);
	}
	
	public String getRobotName(){
		return this.robot;
	}
	
	private void monitorRobotPose(){
		//Subscribe to location topic
		Subscriber<geometry_msgs.PoseWithCovarianceStamped> poseFeedback = rosNode.newSubscriber("/" + robot + "/amcl_pose", geometry_msgs.PoseWithCovarianceStamped._TYPE);
		poseFeedback.addMessageListener(new MessageListener<geometry_msgs.PoseWithCovarianceStamped>() {
			@Override
			public void onNewMessage(geometry_msgs.PoseWithCovarianceStamped message) {
				//currentPose = message.getPose();
				float x = (float)message.getPose().getPose().getPosition().getX();
				float y = (float)message.getPose().getPose().getPosition().getY();
				float oZ = (float)message.getPose().getPose().getOrientation().getZ();
				float oW = (float)message.getPose().getPose().getOrientation().getW();
				robotCurrentPose = x + "," + y + "," + oZ + "," + oW;
				if (pose == null) {
					pose = new float[4];
					pose[0] = x;
					pose[1] = y;
					pose[2] = oZ;
					pose[3] = oW;
				}
			}
		}, 10);
	}
	
	private void getObservedPanel() {
		seenQR = -2;
		try { serviceClientQR = rosNode.newServiceClient(robot+"/getQR", getQR._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		getQRRequest request = serviceClientQR.newMessage();

		request.setRead((byte) 0);
		serviceClientQR.call(request, new ServiceResponseListener<getQRResponse>() {
			@Override
			public void onSuccess(getQRResponse arg0) {
				seenQR = (int)arg0.getQrcode();
			}

			@Override
			public void onFailure(RemoteException arg0) {
				throw new RosRuntimeException(arg0);				
			}
		});
	}

	private SpatioTemporalSet createPanelObservation(String panel, String positionString) {
		SpatioTemporalSet act = (SpatioTemporalSet)RobotFactory.createSpatioTemporalSetVariable(this.name, new Vec2(0.0f,0.0f), 0.0f, solver);
		act.setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
		((SpatioTemporalSet)act).setTask("Observe");

		SymbolicValueConstraint observedPanelConstraint = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
		observedPanelConstraint.setValue(panel);
		observedPanelConstraint.setFrom(act);
		observedPanelConstraint.setTo(act);
		solver.addConstraint(observedPanelConstraint);


		//set the position of the polygon and (no) orientation
        float[] newPose = new float[4];
        String[] poseS = positionString.split(",");
        for (int i = 0; i < poseS.length; i++) newPose[i] = Float.parseFloat(poseS[i]);        
		Polygon p = act.getPolygon();
		p.setDomain(RobotFactory.getVerticesByCenter(new Vec2(newPose[0], newPose[1])));

 
		
		System.out.println("%%%%%%%%%%%%%%%%%%%%%% MODELING (" + robot + " sees " + panel + ") " + act);

		return act;
	}
	
	//value is the string representing the position of the robot
	protected Activity createNewActivity(String value) {	
		
		//Get currently observed panel
		long timeStart = rosNode.getCurrentTime().totalNsecs()/1000000;
		long timeNow = rosNode.getCurrentTime().totalNsecs()/1000000;
		while (true) {
			getObservedPanel();
			try { Thread.sleep(100); } catch (InterruptedException e) { e.printStackTrace(); }
			if (seenQR != -2) timeNow = rosNode.getCurrentTime().totalNsecs()/1000000;
			if ((seenQR != -2) && (timeNow-timeStart < 2000)) break;
		}

		//See if there is an expectation
		Variable[] vars = this.metaSolver.getFocused();
		SpatioTemporalSet expectation = null;

		if (vars != null) {
			for (Variable var : vars) {
				if (var.getComponent().equals(this.robot)) {
					expectation = (SpatioTemporalSet)var;
					break;
				}
			}
		}

		//If we have no expectation, model a new sensor reading (initial condition)
		if (expectation == null) {
			if (seenQR < 0) return createPanelObservation("None", value);
			else return createPanelObservation("P"+seenQR, value);
		}

		//If we are here, we have finished executing, thus
		//expectation is certainly != null
		//String expectedPanel = expectation.getSet().getSymbols()[0];
		String newPanel = "P"+seenQR;
		if (seenQR < 0) newPanel = "None";
	
		//Update focus:
		// -- Remove expectation from focus
		this.metaSolver.removeFromCurrentFocus(expectation);
		SpatioTemporalSet ret = createPanelObservation(newPanel, value);
		
		// -- Add current observation to focus
		this.metaSolver.focus(ret);
		return ret;
		
	}

	protected boolean hasChanged(String value) {
		return true;
	}

	private boolean positionChanged(float[] newPose) {
		float epsilon = 0.01f; //1cm threshold for change
		for (int i = 0; i < pose.length; i++)
			if (Math.abs(pose[i]-newPose[i]) < epsilon) return false;
		return true;
	}


}
