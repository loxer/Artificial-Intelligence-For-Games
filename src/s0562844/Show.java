package s0562844;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import org.lwjgl.opengl.GL11;
import org.lwjgl.util.vector.Vector2f;

import lenz.htw.ai4g.ai.Info;

public class Show {

	public Show(Info info) {
	}

	public static void expensivePoints(ArrayList<Point2D> expensivePoints) {
		int size = 10;
		for (int i = 0; i < expensivePoints.size(); i++) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glColor3d(1, 0, 0);
			GL11.glVertex2d(expensivePoints.get(i).getX() - size, expensivePoints.get(i).getY() + size);
			GL11.glVertex2d(expensivePoints.get(i).getX() + size, expensivePoints.get(i).getY() - size);
			GL11.glVertex2d(expensivePoints.get(i).getX() - size, expensivePoints.get(i).getY() - size);
			GL11.glVertex2d(expensivePoints.get(i).getX() + size, expensivePoints.get(i).getY() + size);
			GL11.glEnd();
		}
	}

	public static void carOrientationVector(Vector2f carPos, Vector2f carOrientationVector) {
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(1, 0, 1);
		GL11.glVertex2d(carPos.getX(), carPos.getY());
		GL11.glVertex2d(carOrientationVector.getX(), carOrientationVector.getY());
		GL11.glEnd();
	}

	public static void route(Vertex[] route) {
		int colour;
		if (route.length - 1 == 0) {
			colour = 0;
		} else if (route.length < 0) {
			return;
		} else {
			colour = 255 / (route.length - 1);
			for (int i = 0; i < route.length - 1; i++) {
				GL11.glBegin(GL11.GL_LINES);
				GL11.glColor3d(255 - colour * i, 255 - colour * i, 255 - colour * i);
				GL11.glVertex2d(route[i].getLocation().getX(), route[i].getLocation().getY());
				GL11.glVertex2d(route[i + 1].getLocation().getX(), route[i + 1].getLocation().getY());
				GL11.glEnd();
			}
		}
	}

	public static void helperPoints(Vertex vertices[]) {
		for (int i = 0; i < vertices.length; i++) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glColor3d(0, 0, 1);
			GL11.glVertex2d(vertices[i].getLocation().getX() - 2, vertices[i].getLocation().getY() + 2);
			GL11.glVertex2d(vertices[i].getLocation().getX() + 2, vertices[i].getLocation().getY() - 2);
			GL11.glVertex2d(vertices[i].getLocation().getX() - 2, vertices[i].getLocation().getY() - 2);
			GL11.glVertex2d(vertices[i].getLocation().getX() + 2, vertices[i].getLocation().getY() + 2);
			GL11.glEnd();
		}
	}

	public static void lineToCheckpoint(Vector2f carPos, double currentCheckpointX, double currentCheckpointY) {
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 1, 0);
		GL11.glVertex2d(carPos.getX(), carPos.getY());
		GL11.glVertex2d(currentCheckpointX, currentCheckpointY);
		GL11.glEnd();
	}

	public static void lineToDestination(Vector2f carPos, Vertex currentDrivingDestination) {
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 0, 1);
		GL11.glVertex2d(carPos.getX(), carPos.getY());
		GL11.glVertex2d(currentDrivingDestination.getLocation().getX(), currentDrivingDestination.getLocation().getY());
		GL11.glEnd();
	}

	public static void obstacleCorners(Rectangle2D[] obstacleCorners, float sizeOfObstacleCorners) {
		for (int i = 0; i < obstacleCorners.length; i++) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glVertex2d(obstacleCorners[i].getX(), obstacleCorners[i].getY());
			GL11.glVertex2d(obstacleCorners[i].getX() + sizeOfObstacleCorners,
					obstacleCorners[i].getY() + sizeOfObstacleCorners);
			GL11.glVertex2d(obstacleCorners[i].getX() + sizeOfObstacleCorners, obstacleCorners[i].getY());
			GL11.glVertex2d(obstacleCorners[i].getX(), obstacleCorners[i].getY() + sizeOfObstacleCorners);
			GL11.glColor3d(1, 1, 0);

			GL11.glEnd();
		}
	}

	public static void lineOfFutureRoutePoint(Vertex[] route, int currentRoutePoint) {
		int size = 20;
		if (route.length - 1 <= currentRoutePoint + 1) {
			return;
		}
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(1, 1, 0);
		GL11.glVertex2d(route[currentRoutePoint + 1].getLocation().getX() - size,
				route[currentRoutePoint + 1].getLocation().getY() + size);
		GL11.glVertex2d(route[currentRoutePoint + 1].getLocation().getX() - size,
				route[currentRoutePoint + 1].getLocation().getY() - size);
		GL11.glVertex2d(route[currentRoutePoint + 1].getLocation().getX() + size,
				route[currentRoutePoint + 1].getLocation().getY() + size);
		GL11.glVertex2d(route[currentRoutePoint + 1].getLocation().getX() + size,
				route[currentRoutePoint + 1].getLocation().getY() - size);
		GL11.glEnd();
	}

	public static void nextLine(Line2D showLine) {
		if (showLine == null) {
			return;
		}
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(1, 1, 0);
		GL11.glVertex2d(showLine.getX1(), showLine.getY1());
		GL11.glVertex2d(showLine.getX2(), showLine.getY2());
		GL11.glEnd();
	}

	public static void aHelperPointsConnections(Vertex vertices[]) {
		Vertex vertex = vertices[105];
		for (int i = 0; i < vertex.numberOfEdges(); i++) {
			GL11.glBegin(GL11.GL_LINES);
			GL11.glColor3d(1, 0, 1);
			GL11.glVertex2d(vertex.getLocation().getX(), vertex.getLocation().getY());
			GL11.glVertex2d(vertex.getConnectedPoint(i).getLocation().getX(),
					vertex.getConnectedPoint(i).getLocation().getY());
			GL11.glEnd();
		}
	}

	public static void printLines(Line2D[] lines) {
		for (int i = 0; i < lines.length; i++) {
			double length = Point2D.distanceSq(lines[i].getX1(), lines[i].getY1(), lines[i].getX2(), lines[i].getY2());
			GL11.glBegin(GL11.GL_LINES);
			GL11.glColor3d(1, 0.55, 0.12);
			GL11.glVertex2d(lines[i].getX1(), lines[i].getY1());
			GL11.glVertex2d(lines[i].getX2(), lines[i].getY2());
			GL11.glEnd();
		}
	}

	//
	// public static void showAvoidingVector(float x, float y) {
	// GL11.glBegin(GL11.GL_LINES);
	// GL11.glColor3d(1, 1, 0);
	// GL11.glVertex2d(carPos.getX(), carPos.getY());
	// GL11.glVertex2d(x, y);
	// GL11.glEnd();
	//
	// // info.getTrack().getObstacles()
	// // Vector2f.
	// }

}
