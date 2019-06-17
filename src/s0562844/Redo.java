package s0562844;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

public class Redo {
	private static final int MIN_SIZE_OBSTACLE_CORNERS = 5;
	private static final int MAX_SIZE_OBSTACLE_CORNERS = 30;
	

	public static Rectangle2D[] resizeObstacleCorners(Point2D[] obstaclePoints, float newSize) {
		
		if(newSize < MIN_SIZE_OBSTACLE_CORNERS) {
			newSize = MAX_SIZE_OBSTACLE_CORNERS;
		}
		
		if(newSize > MAX_SIZE_OBSTACLE_CORNERS) {
			newSize = MIN_SIZE_OBSTACLE_CORNERS;
		}
		
		Rectangle2D[] obstacleCorners = new Rectangle2D[obstaclePoints.length];
		double halfRectangle = newSize / 2;
		for (int i = 0; i < obstaclePoints.length; i++) {
			obstacleCorners[i] = new Rectangle2D.Double(obstaclePoints[i].getX() - halfRectangle,
					obstaclePoints[i].getY() - halfRectangle, newSize, newSize);
		}
		return obstacleCorners;
	}
}