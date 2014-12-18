package se.oru.aass.lucia_meta_csp_lecture.util;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;

import java.util.ArrayList;
import java.util.HashMap;

import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.RawMessage;
import org.ros.message.Duration;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

public class PanelMarkerPublisher {
	
	private ConnectedNode connectedNode;
	private ArrayList<Polygon> polygons;
	private HashMap<Polygon,String> polygonsToPanelNamespaces;
	
	public PanelMarkerPublisher(ArrayList<Polygon> polygons, HashMap<Polygon,String> polygonsToPanelNamespaces, ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;
		this.polygons = polygons;
		this.polygonsToPanelNamespaces = polygonsToPanelNamespaces;
		this.publishMarkers();
	}
	
	public void publishMarkers() {
		final Publisher<visualization_msgs.MarkerArray> publisher = connectedNode.newPublisher("lucia_polygons", visualization_msgs.MarkerArray._TYPE);
		connectedNode.executeCancellableLoop(new CancellableLoop() {

		      @Override
		      protected void loop() throws InterruptedException {
		    	MarkerArray mArray = connectedNode.getTopicMessageFactory().newFromType(visualization_msgs.MarkerArray._TYPE);
		    	ArrayList<Marker> markers = new ArrayList<Marker>();
		    	for (int k = 0; k < polygons.size(); k++) {
		    		Polygon p = polygons.get(k);
			    	Marker m = connectedNode.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			    	m.getHeader().setFrameId("/map");
			    	ArrayList<Point> points = new ArrayList<Point>();
			    	for (Vec2 onePointV : p.getFullSpaceRepresentation()) {
			    		Point onePoint = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			    		onePoint.setX(onePointV.x);
			    		onePoint.setY(onePointV.y);
			    		onePoint.setZ(0.0);
			    		points.add(onePoint);
			    	}
			    	//Add first again to close the loop
			    	points.add(points.get(0));
			    	m.setPoints(points);
//			    	Pose pose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
//			    	Point center = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			    	//Quaternion orientation = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
			    	//orientation.setW(0);
			    	//orientation.setX(0);
			    	//orientation.setY(0);
			    	//orientation.setZ(0);
//			    	center.setX(p.getPosition().x);
//			    	center.setY(p.getPosition().y);
//			    	center.setZ(0.0);
//			    	pose.setPosition(center);
//			    	//pose.setOrientation(orientation);
//			    	m.setPose(pose);
			    	m.getScale().setX(0.1f);
			    	m.getScale().setY(0.1f);
			    	m.getScale().setZ(0.1f);
			    	m.getColor().setR(1.0f);
			    	m.getColor().setG(0.0f);
			    	m.getColor().setB(0.0f);
			    	m.getColor().setA(0.5f);
			    	m.setAction(visualization_msgs.Marker.ADD);
			    	m.setNs(polygonsToPanelNamespaces.get(p));
			    	m.setType(visualization_msgs.Marker.LINE_STRIP);
			    	m.setLifetime(new Duration(60.0));
			    	markers.add(m);
			    	
			    	Marker mArrow = connectedNode.getTopicMessageFactory().newFromType(visualization_msgs.Marker._TYPE);
			    	mArrow.setAction(visualization_msgs.Marker.ADD);
			    	mArrow.setNs(polygonsToPanelNamespaces.get(p)+"_orientation");
			    	mArrow.setType(visualization_msgs.Marker.ARROW);
			    	Pose arrowPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
			    	Point arrowPosition = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
			    	arrowPosition.setX(p.getPosition().x);
			    	arrowPosition.setY(p.getPosition().y);
			    	arrowPosition.setZ(0);
			    	arrowPose.setPosition(arrowPosition);
			    	float theta = p.getOrientation();
			    	Quaternion arrowQuat = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
			    	arrowQuat.setW(Math.cos(theta/2));
			    	arrowQuat.setX(0);
			    	arrowQuat.setY(0);
			    	arrowQuat.setZ(Math.sin(theta/2));
					arrowPose.setOrientation(arrowQuat);
			    	mArrow.setPose(arrowPose);
			    	mArrow.setLifetime(new Duration(60.0));
			    	mArrow.getHeader().setFrameId("/map");
			    	mArrow.getScale().setX(0.3);
			    	mArrow.getScale().setY(0.1);
			    	mArrow.getScale().setZ(0.1);
			    	mArrow.getColor().setR(1.0f);
			    	mArrow.getColor().setG(0.0f);
			    	mArrow.getColor().setB(0.0f);
			    	mArrow.getColor().setA(0.5f);
			    	markers.add(mArrow);
		    	}
		    	mArray.setMarkers(markers);
		        publisher.publish(mArray);
		        Thread.sleep(2000);
		      }
		    });
	}
	

}
