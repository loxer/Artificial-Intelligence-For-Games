package s0562844;

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Terrain {

	Polygon[] polygon;
	Point2D[] points;
	Line2D[] lines;
	Rectangle2D[] corners;

	public Terrain(Polygon[] polygon, float sizeOfCorners) {
		this.polygon = polygon;
		collectInformationOfPolygon(sizeOfCorners);
	}

	public Terrain(Polygon[] polygon) {
		this.polygon = polygon;
		collectInformationOfPolygon(0);
	}

	private void collectInformationOfPolygon(float sizeOfCorners) {
		int size = countPolygonPoints(polygon);

		this.points = new Point2D[size];
		this.lines = new Line2D[size];
		this.corners = new Rectangle2D[size];

		double halfRectangle = sizeOfCorners / 2;
		int pos = 0;

		for (int i = 0; i < polygon.length; i++) {
			int numberOfPoints = polygon[i].xpoints.length;

			for (int j = 1; j < numberOfPoints; j++) {

				points[pos] = new Point2D.Float(polygon[i].xpoints[j - 1], polygon[i].ypoints[j - 1]);
				lines[pos] = new Line2D.Float(polygon[i].xpoints[j - 1], polygon[i].ypoints[j - 1],
						polygon[i].xpoints[j], polygon[i].ypoints[j]);
				corners[pos] = new Rectangle2D.Double(points[pos].getX() - halfRectangle,
						points[pos].getY() - halfRectangle, sizeOfCorners, sizeOfCorners);
				pos++;
			}

			// connect the end with the beginning
			lines[pos] = new Line2D.Float(polygon[i].xpoints[numberOfPoints - 1],
					polygon[i].ypoints[numberOfPoints - 1], polygon[i].xpoints[0], polygon[i].ypoints[0]);

			// add the first point
			points[pos] = new Point2D.Float(polygon[i].xpoints[numberOfPoints - 1],
					polygon[i].ypoints[numberOfPoints - 1]);

			corners[pos] = new Rectangle2D.Double(points[pos].getX() - halfRectangle,
					points[pos].getY() - halfRectangle, sizeOfCorners, sizeOfCorners);

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

	public Boolean isSomethingInTheWay(Line2D line) {
		for (int i = 0; i < lines.length; i++) {
			if (line.intersectsLine(lines[i]) || corners[i].intersectsLine(line)) {
				return true;
			}
		}
		return false;
	}
}
