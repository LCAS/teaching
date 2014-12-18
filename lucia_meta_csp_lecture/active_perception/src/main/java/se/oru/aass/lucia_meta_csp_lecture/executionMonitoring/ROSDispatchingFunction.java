package se.oru.aass.lucia_meta_csp_lecture.executionMonitoring;


import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;

import org.metacsp.dispatching.DispatchingFunction;
import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
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

public class ROSDispatchingFunction extends DispatchingFunction {

	private LuciaMetaConstraintSolver metaSolver;
	private SpatioTemporalSetNetworkSolver solver;
	private ActivityNetworkSolver activityNetworkSolver;
	protected SymbolicVariableActivity currentAct = null;
	private boolean isExecuting = false;
	private ConnectedNode rosNode = null;
	protected String robot = null;
	private ROSTopicSensor sensor;
	private int counter;
	private static int MIN_MESSAGES = 30;

	public ROSDispatchingFunction(String rob,  ActivityNetworkSolver activitySolver, ConnectedNode rosN) {
		super(rob);
		this.activityNetworkSolver = activitySolver;
		this.rosNode = rosN;
		this.robot = rob;
		//Subscribe to movebase feedback topic
		Subscriber<actionlib_msgs.GoalStatusArray> actionlibFeedback = rosNode.newSubscriber("/" + robot + "/move_base/status", actionlib_msgs.GoalStatusArray._TYPE);
		actionlibFeedback.addMessageListener(new MessageListener<actionlib_msgs.GoalStatusArray>() {
			@Override
			public void onNewMessage(actionlib_msgs.GoalStatusArray message) {
				if (message.getStatusList() != null && !message.getStatusList().isEmpty()) {	
					GoalStatus gs = message.getStatusList().get(0);
					System.out.println(">>>>>>>>>>>>>>>>>> (" + robot + ") ACTIONLIB SAYS: " + printStatus(gs.getStatus()));
					if(gs.getStatus() == (byte)3){
						finishCurrentActivity();
					}
				}
			}
		}, 10);
		
	}
	
	private String printStatus(byte n){
		if(n == (byte)3)
			return "Success";
		if(n == (byte)1)
			return "Active";
		if(n == (byte)4)
			return "Reject";
		return Byte.toString(n);
	}
	
	public ROSDispatchingFunction(String rob, LuciaMetaConstraintSolver metaSolver, ConnectedNode rosN, final ROSTopicSensor sens) {
		super(rob);
		this.metaSolver = metaSolver;
		this.solver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		this.activityNetworkSolver = ((SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0]).getActivitySolver();
		this.rosNode = rosN;
		this.robot = rob;
		this.sensor = sens;

		//Subscribe to movebase feedback topic
		Subscriber<actionlib_msgs.GoalStatusArray> actionlibFeedback = rosNode.newSubscriber("/" + robot + "/move_base/status", actionlib_msgs.GoalStatusArray._TYPE);
		actionlibFeedback.addMessageListener(new MessageListener<actionlib_msgs.GoalStatusArray>() {
			@Override
			public void onNewMessage(actionlib_msgs.GoalStatusArray message) {
				if (message.getStatusList() != null && !message.getStatusList().isEmpty()) {
					//goalID = message.getStatusList().size()-1;	
					GoalStatus gs = message.getStatusList().get(0);
					//If != ACTIVE
					if (isExecuting()) {
						System.out.println(">>>>>>>>>>>>>>>>>> (" + robot + ") ACTIONLIB SAYS: " + printStatus(gs.getStatus()));
						if (gs.getStatus() != (byte)1) {
							if (counter++ > MIN_MESSAGES) {
								finishCurrentActivity();
								long timeNow = rosNode.getCurrentTime().totalNsecs()/1000000;
								sensor.postSensorValue(sens.getRobotCurrentPoseString(), timeNow);
								counter = 0;
							}
						}
						else counter = 0;
					}
				}
			}
		}, 10);
	}

	private void sendGoal(final String robot, final String command, final Polygon destination) {
//		System.out.println(">>>> TO ROS: " + command + " " + destination);
		ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
		try { serviceClient = rosNode.newServiceClient(robot+"/sendGoal", sendGoal._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final sendGoalRequest request = serviceClient.newMessage();
		request.setX(destination.getPosition().x);
		request.setY(destination.getPosition().y);
		request.setTheta(destination.getOrientation());
//		System.out.println("GOAL ORIENTATION OF " + robot + "IS " + destination.getOrientation());
		request.setRotationAfter((byte)0);
		serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {

			@Override
			public void onSuccess(sendGoalResponse arg0) {
				setExecuting(true);
				//long timeNow = rosNode.getCurrentTime().totalNsecs()/1000000;
				//sensor.postSensorValue("LookAtQRCode", timeNow);
			}

			@Override
			public void onFailure(RemoteException arg0) { }
		});

	}

	public void finishCurrentActivity() {
		System.out.println(">>>>>>>>>>>>>>>>>>> (" + robot + ") FINISHED!!!");
		this.finish(currentAct);
		currentAct = null;
		setExecuting(false);
	}

	private boolean isExecuting() {
		return isExecuting;
	}

	private void setExecuting(boolean exec) {
		isExecuting = exec;
	}

	@Override
	public void dispatch(SymbolicVariableActivity act) {
		
		currentAct = act;
		String robot = act.getComponent();
		String command = act.getSymbolicVariable().getSymbols()[0];

		//Get the two constraint networks (one is low level, one is hi level)
		ConstraintNetwork activityNetwork = this.getConstraintNetwork();
		ConstraintNetwork spatioTemporalSetNetwork = solver.getConstraintNetwork();

		//Find observe activity to which this move_base is leading
		Constraint[] cons = activityNetwork.getOutgoingEdges(act);
		SymbolicVariableActivity observeAct = null;
		for (Constraint con : cons) {
			if (((SymbolicVariableActivity)con.getScope()[1]).getSymbolicVariable().getSymbols()[0].equals("Observe")) {
				observeAct = (SymbolicVariableActivity)con.getScope()[1];
				break;
			}
		}
		
		//Find polygon from which to compute destination
		Polygon destPoly = null;
		for (Variable var : spatioTemporalSetNetwork.getVariables()) {
			SpatioTemporalSet sts = (SpatioTemporalSet)var;
			if (sts.getActivity().equals(observeAct)) {
				destPoly = sts.getPolygon();
				System.out.println("-----------------------------------------------------------------------------");
				System.out.println("----------> DISPATCHING " + act.getSymbolicVariable().getSymbols()[0] +
						" for " + act.getComponent() +" to see "+ sts.getSet().getSymbols()[0] + " at polygon " + destPoly);
				break;
			}
		}

		this.sendGoal(robot, command, destPoly);

	}

	@Override
	public boolean skip(SymbolicVariableActivity act) {
		//Do not dispatch "Observe" activities
		if (act.getSymbolicVariable().getSymbols()[0].equals("Observe")) return true;
		return false;
	}

}
