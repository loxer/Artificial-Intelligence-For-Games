package s0562844;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class Vertex {
	private Point2D location;
	private Vertex predecessor;

	private ArrayList<Vertex> connectedPointsList = new ArrayList<Vertex>();
	private ArrayList<Line2D> edgesList = new ArrayList<Line2D>();
	private Vertex[] connectedPoints;
	private Line2D[] edges;
	private float[] lengths;

	private Boolean finished = false;
	private Boolean queued = false;
	private float costs = Float.MAX_VALUE;
	private float extraCosts = 0;

	private Boolean connectedToCheckpoint = false;
	private float lengthToCheckpoint = Float.MAX_VALUE;

	public Vertex() {
		location = new Point2D.Float();
	}

	public Vertex(Point2D location) {
		this.location = location;
	}

	public Point2D getLocation() {
		return location;
	}

	public Vertex getPredecessor() {
		return predecessor;
	}

	public void setPredecessor(Vertex predecessor) {
		this.predecessor = predecessor;
	}

	public Vertex getConnectedPoint(int index) {
		return connectedPoints[index];
	}
	
	public Line2D[] getEdges() {
		return edges;
	}

	public void setEdgeAndPoint(Line2D edge, Vertex point) {
		edgesList.add(edge);
		connectedPointsList.add(point);
	}

	public int numberOfEdges() {
		return connectedPoints.length;
	}

	public void setReady(float extraDistance) {
		connectedPoints = new Vertex[connectedPointsList.size()];
		edges = new Line2D[connectedPoints.length];
		lengths = new float[connectedPoints.length];

		for (int i = 0; i < connectedPoints.length; i++) {
			connectedPoints[i] = connectedPointsList.get(i);
			edges[i] = edgesList.get(i);
			lengths[i] = (float) location.distanceSq(connectedPoints[i].getLocation()) + extraDistance + extraCosts;
		}
	}

	public void resetForQueue() {
		predecessor = null;
		finished = false;
		queued = false;
		costs = Float.MAX_VALUE;
	}

	public void resetLists() {
		connectedPointsList.clear();
		edgesList.clear();
	}

	public void setCosts(float newCosts) {
		costs = newCosts;
	}

	public float getCosts() {
		return costs;
	}

	public void setExtraCosts(float extra) {
		extraCosts += extra;

		if (extraCosts <= 0) {
			extraCosts = 0;
			return;
		}

		calculateNewLenghts(extraCosts);
	}

	public void resetExtraCosts() {
		calculateNewLenghts(-extraCosts);
		extraCosts = 0;
	}

	private void calculateNewLenghts(float value) {
		for (int i = 0; i < lengths.length; i++) {
			lengths[i] += value;
		}
	}

	public float getLenght(int index) {
		return lengths[index];
	}

	public Boolean isFinished() {
		return finished;
	}

	public void setFinished() {
		finished = true;
		queued = false;
	}

	public void setQueued() {
		queued = true;
	}

	public Boolean isQueued() {
		return queued;
	}

	public void printEdges() {
		System.out.println("Point " + toString() + " is connected to:");
		for (int i = 0; i < connectedPoints.length; i++) {
			System.out.println("* " + connectedPoints[i].getLocation().getX() + "/"
					+ connectedPoints[i].getLocation().getY() + "   || Distance: " + lengths[i]);
		}
		if (connectedToCheckpoint) {
			System.out.println("* Connection to current Checkpoint || Distance: " + lengthToCheckpoint);
		}
		System.out.println("--------");
	}

	@Override
	public String toString() {
		return location.getX() + "/" + location.getY();
	}
}