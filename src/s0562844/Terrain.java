package s0562844;

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Terrain {
	private final int DISTANCE_OF_HELPER_POINTS = 50;
	private final int DISTANCE_FROM_CORNERS = 25;
	
	Polygon[] polygon;
	Point2D[] points;
	Line2D[] lines;
	Rectangle2D[] corners;

	public Terrain(Polygon[] polygon, float sizeOfObstacleCorners) {
		this.polygon = polygon;
		collectInformationOfPolygon(sizeOfObstacleCorners);
	}

	public Terrain(Polygon[] polygon) {
		this.polygon = polygon;
		collectInformationOfPolygon(0);
	}

	private void collectInformationOfPolygon(float sizeOfObstacleCorners) {
		int size = countPolygonPoints(polygon);

		this.points = new Point2D[size];
		this.lines = new Line2D[size];
		this.corners = new Rectangle2D[size];

		double halfRectangle = sizeOfObstacleCorners / 2;
		int pos = 0;

		for (int i = 0; i < polygon.length; i++) {
			int numberOfPoints = polygon[i].xpoints.length;

			for (int j = 1; j < numberOfPoints; j++) {

				points[pos] = new Point2D.Float(polygon[i].xpoints[j - 1], polygon[i].ypoints[j - 1]);
				lines[pos] = new Line2D.Float(polygon[i].xpoints[j - 1], polygon[i].ypoints[j - 1],
						polygon[i].xpoints[j], polygon[i].ypoints[j]);
				corners[pos] = new Rectangle2D.Double(points[pos].getX() - halfRectangle,
						points[pos].getY() - halfRectangle, sizeOfObstacleCorners, sizeOfObstacleCorners);
				pos++;
			}

			// connect the end with the beginning
			lines[pos] = new Line2D.Float(polygon[i].xpoints[numberOfPoints - 1],
					polygon[i].ypoints[numberOfPoints - 1], polygon[i].xpoints[0], polygon[i].ypoints[0]);

			// add the first point
			points[pos] = new Point2D.Float(polygon[i].xpoints[numberOfPoints - 1],
					polygon[i].ypoints[numberOfPoints - 1]);

			corners[pos] = new Rectangle2D.Double(points[pos].getX() - halfRectangle,
					points[pos].getY() - halfRectangle, sizeOfObstacleCorners, sizeOfObstacleCorners);

			pos++;
		}
	}

	public static int countPolygonPoints(Polygon[] polygon) {
		int numberOfPoints = 0;
		for (int i = 0; i < polygon.length; i++) {
			numberOfPoints += polygon[i].xpoints.length;
		}
		return numberOfPoints;
	}

	public Point2D[] getPoints() {
		return points;
	}

	public Line2D[] getLines() {
		return lines;
	}

	public Rectangle2D[] getCorners() {
		return corners;
	}

	public Boolean isInsideHere(Point2D point) {
		for (int i = 0; i < polygon.length; i++) {
			if (polygon[i].contains(point)) {
				return true;
			}
		}
		return false;
	}

	public ArrayList<Point2D> createHelperPoints() {
		ArrayList<Point2D> helperPoints = new ArrayList<Point2D>();
		
		for(int i = 0; i < lines.length; i++) {
			double x1 = lines[i].getX1();
			double x2 = lines[i].getX2();
			double y1 = lines[i].getY1();
			double y2 = lines[i].getY2();
			
			double distance = Point2D.distance(x1, y1, x2, y2);
			
//			Boolean x1IsLeft = x1 <= x2;
//			Boolean y1IsOnTop = y1 >= y2;
			
			int availableLength = (int) distance - DISTANCE_FROM_CORNERS*2;
			int numberOfNewHelperPoints = availableLength /  DISTANCE_OF_HELPER_POINTS + 1;

			for(int j = 0; j < numberOfNewHelperPoints; j++) {
				
				
				
//				float xCoord = Math.sqrt(lines[i].getX1())
				
				helperPoints.add(new Point2D.Float());
			}

		}
		
		return helperPoints;
	}

	private void test() {
		ArrayList<String> itemsList1 = new ArrayList();
		ArrayList<String> itemsList2 = new ArrayList();
		
		itemsList1.add("1");
		itemsList1.add("2");
		itemsList2.add("3");
		itemsList2.add("4");
		
		itemsList1.addAll(itemsList2);
		System.out.println("Merged list " + itemsList1);
	}
}
