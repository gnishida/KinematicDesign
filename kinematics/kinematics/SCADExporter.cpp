#include "SCADExporter.h"
#include <QFile>
#include <QTextStream>

namespace kinematics {

	void SCADExporter::save(const QString& filename, const QString& name, boost::shared_ptr<kinematics::BodyGeometry> body) {
		QFile file(filename);
		file.open(QIODevice::WriteOnly);
		QTextStream out(&file);

		int N = body->size();
		for (int i = 0; i < N; i++) {
			std::vector<glm::dvec2> pts = body->getActualPoints(i);
			std::vector<glm::dvec2> pts2 = body->getActualPoints2(i);
			glm::dvec2 center = (pts[0] + pts[pts.size() / 2]) * 0.5;
			float scale = glm::length(pts2[0] - center) / glm::length(pts[0] - center);
			float z = body->polygons[i].depth1;
			float depth = body->polygons[i].depth2 - body->polygons[i].depth1;

			out << "translate([" << center.x << ", " << center.y << ", " << z << "]) {" << endl;
			out << "\tlinear_extrude(height=" << depth << ", scale=" << scale << ") {" << endl;
			out << "\t\tpolygon( points=[";
			for (int j = 0; j < pts.size(); j++) {
				if (j > 0) out << ", ";
				out << "[" << pts[j].x - center.x << "," << pts[j].y - center.y << "]";
			}
			out << "]);" << endl;

			out << "\t}" << endl;
			out << "}" << endl;
		}

		file.close();
	}

	void SCADExporter::save(const QString& filename, const QString& name, const std::vector<glm::dvec2>& pts, const std::vector<std::vector<glm::dvec2>>& holes, double height) {
		QFile file(filename);
		file.open(QIODevice::WriteOnly);
		QTextStream out(&file);

		out << "difference() {" << endl;

		// outer polygon
		out << "\tlinear_extrude(height=" << height << ") {" << endl;
		out << "\t\tpolygon( points=[";
		for (int i = 0; i < pts.size(); i++) {
			if (i > 0) out << ", ";
			out << "[" << pts[i].x << "," << pts[i].y << "]";
		}
		out << "]);" << endl;
		out << "\t}" << endl;

		// holes
		for (int i = 0; i < holes.size(); i++) {
			out << "\ttranslate([0,0,-0.1]) {" << endl;
			out << "\t\tlinear_extrude(height=" << height + 0.2 << ") {" << endl;
			out << "\t\t\tpolygon( points=[";
			for (int j = 0; j < holes[i].size(); j++) {
				if (j > 0) out << ", ";
				out << "[" << holes[i][j].x << "," << holes[i][j].y << "]";
			}
			out << "]);" << endl;
			out << "\t\t}" << endl;
			out << "\t}" << endl;
		}

		out << "}" << endl;

		file.close();
	}

}