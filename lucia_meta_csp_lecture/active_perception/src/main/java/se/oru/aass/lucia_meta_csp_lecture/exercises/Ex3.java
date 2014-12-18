package se.oru.aass.lucia_meta_csp_lecture.exercises;

import java.util.logging.Level;

import services.sendGoal;
import services.sendGoalRequest;
import services.sendGoalResponse;

import org.apache.commons.logging.Log;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.sensing.ConstraintNetworkAnimator;
import org.metacsp.sensing.InferenceCallback;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.logging.MetaCSPLogging;
import org.metacsp.utility.timelinePlotting.TimelinePublisher;
import org.metacsp.utility.timelinePlotting.TimelineVisualizer;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import se.oru.aass.lucia_meta_csp_lecture.executionMonitoring.ROSDispatchingFunction;


public class Ex3  extends AbstractNodeMain {


	private ConnectedNode connectedNode;
	private final String nodeName = "Ex3";
	private Ex3MetaConstraintSolver metaSolver;
	
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(nodeName);
	}

	@Override
	public void onStart(ConnectedNode cn) {
		
		MetaCSPLogging.setLevel(Ex3MetaConstraintSolver.class, Level.FINEST);

		this.connectedNode = cn;
		
		while (true) {
			try {
				this.connectedNode.getCurrentTime();
				break;
			}
			catch(NullPointerException e) { }
		}
		
		final Log log = connectedNode.getLog();
		log.info("Lucia CSP Node starting...");
		
		
		long origin = connectedNode.getCurrentTime().totalNsecs()/1000000;
		
		//Initializing meta-solver
		metaSolver = new Ex3MetaConstraintSolver(origin,origin+1000000,500);
		ActivityNetworkSolver temporalSolver = (ActivityNetworkSolver)metaSolver.getConstraintSolvers()[0];
		
		
		//creating robot's variables
		Variable var1 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_base");
		
		Variable var2 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_4");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_base");
		
		
		//#################################################################################
		//adding RobotSchedulingMetaConstraint Meta Constraint to Meta SOlver
		//TODO navigate through RobotSchedulingMetaConstraint meta-constraint and complete the implementation
		//#################################################################################
		RobotSchedulingMetaConstraint schedlingMetaConstraint = new RobotSchedulingMetaConstraint(null, null);
		metaSolver.addMetaConstraint(schedlingMetaConstraint);
		
		//add minimum duration for each move_base
		AllenIntervalConstraint minDuration1 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(2000, APSPSolver.INF));
		minDuration1.setFrom(var1);
		minDuration1.setTo(var1);
		temporalSolver.addConstraint(minDuration1);

		AllenIntervalConstraint minDuration2 = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(2000, APSPSolver.INF));
		minDuration2.setFrom(var2);
		minDuration2.setTo(var2);
		temporalSolver.addConstraint(minDuration2);
	
		
		
		//
		InferenceCallback cb = new InferenceCallback() {
			@Override
			public void doInference(long timeNow) {
				//performing backtrack in each clock
				metaSolver.clearResolvers();
				metaSolver.backtrack();
				
				//printing out the search tree if there is any added resolver
				if(metaSolver.getAddedResolvers().length > 0)
					metaSolver.draw();
				
			}
		};
		
		//creating animator to model the passing of time in the constraint network
		ConstraintNetworkAnimator animator = new ConstraintNetworkAnimator(temporalSolver, 1000, cb){
			@Override
			protected long getCurrentTimeInMillis() {
				return connectedNode.getCurrentTime().totalNsecs()/1000000;
			}
		};
		
		/*
		creating ROSDispatchingFunction for turtlebot_1 : triggers a ROS service call when a dispatchable variable appears in the constraint network
		triggers a ROS service call when a dispatchable variable
		appears in the constraint network
		 */
		ROSDispatchingFunction robotDispatchingFunction1 = new ROSDispatchingFunction("turtlebot_2", temporalSolver, this.connectedNode) {		
			@Override
			public boolean skip(SymbolicVariableActivity act) { return false; }
			
			@Override
			public void dispatch(SymbolicVariableActivity act) { 
				currentAct = act;
				sendGoal(robot, 3.0f, -1.0f, 0.3f);
			}
		};
		
		/*
		creating ROSDispatchingFunction for turtlebot_2 : triggers a ROS service call when a dispatchable variable appears in the constraint network
		triggers a ROS service call when a dispatchable variable
		appears in the constraint network
		 */
		ROSDispatchingFunction robotDispatchingFunction2 = new ROSDispatchingFunction("turtlebot_4", temporalSolver, this.connectedNode) {
			@Override
			public boolean skip(SymbolicVariableActivity act) { return false; }
			
			@Override
			public void dispatch(SymbolicVariableActivity act) { 
				currentAct = act;
				sendGoal(robot, -2.0f, -2.0f, 0.0f);
			}
		};
		
		//adding dispatchers to the animator
		animator.addDispatchingFunctions(temporalSolver, robotDispatchingFunction1, robotDispatchingFunction2);
		//#################################################################################
		//visualize timeline
		//#################################################################################
		TimelinePublisher tp = new TimelinePublisher((ActivityNetworkSolver)temporalSolver, new Bounds(0,120000), true, "turtlebot_2", "turtlebot_4");
		TimelineVisualizer tv = new TimelineVisualizer(tp);
		tv.startAutomaticUpdate(1000);
		
	}

	/*
	Implements the sendGoal services for each robot (e.g., "/turtlebot_1/sendGoal")
	service request is x, y and theta representing the goal pose and orientation (in radian)
	 */
	private void sendGoal(String robot, float x, float y, float theta) {
		ServiceClient<sendGoalRequest, sendGoalResponse> serviceClient = null;
		try { serviceClient = connectedNode.newServiceClient("/"+robot+"/sendGoal", sendGoal._TYPE); }
		catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		final sendGoalRequest request = serviceClient.newMessage();
		request.setX(x);
		request.setY(y);
		request.setTheta(theta);
		request.setRotationAfter((byte)0);
		serviceClient.call(request, new ServiceResponseListener<sendGoalResponse>() {
			@Override
			public void onSuccess(sendGoalResponse arg0) {System.out.println("Goal sent");}
			@Override
			public void onFailure(RemoteException arg0) { }
		});		
	}

}

