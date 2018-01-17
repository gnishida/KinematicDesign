#pragma once

#include <glew.h>
#include "Shader.h"
#include <kinematics.h>
#include <QGLWidget>
#include <QMouseEvent>
#include <QTimer>
#include "Camera.h"
#include "RenderManager.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Shape.h"
#include "Operation.h"
#include "Design.h"
#include "History.h"

class MainWindow;

class GLWidget3D : public QGLWidget {
	Q_OBJECT

public:
	static enum { MODE_SELECT = 0, MODE_MOVE, MODE_ROTATION, MODE_RESIZE, MODE_FIXED_RECTANGLE, MODE_FIXED_CIRCLE, MODE_FIXED_POLYGON, MODE_MOVING_RECTANGLE, MODE_MOVING_CIRCLE, MODE_MOVING_POLYGON, MODE_LINKAGE_REGION, MODE_LINKAGE_AVOIDANCE, MODE_KINEMATICS };
	static enum { LINKAGE_4R = 1, LINKAGE_RRRP = 2 };

public:
	MainWindow* mainWin;

	int mode;

	// camera
	Camera camera;
	glm::vec3 light_dir;
	glm::mat4 light_mvpMatrix;
	glm::vec3 spot_light_pos;

	// rendering engine
	RenderManager renderManager;

	// key status
	bool shiftPressed;
	bool ctrlPressed;

	bool first_paint;
	bool front_faced;

	boost::shared_ptr<canvas::Shape> current_shape;
	boost::shared_ptr<canvas::Operation> operation;
	boost::shared_ptr<canvas::Shape> selected_shape;
	std::vector<boost::shared_ptr<canvas::Shape>> copied_shapes;
	canvas::Design design;
	canvas::History history;

	std::vector<boost::shared_ptr<kinematics::LinkageSynthesis>> synthesis;

	std::vector<kinematics::Kinematics> kinematics;
	std::vector<kinematics::Solution> selected_solutions; // currently selected solution
	std::vector<std::vector<kinematics::Solution>> solutions; // all the candidates
	std::pair<int, int> selectedJoint;
	std::vector<kinematics::Object25D> fixed_bodies;
	std::vector<kinematics::Object25D> moving_bodies;
	int linkage_type;
	QTimer* animation_timer;
	bool collision_check;
	bool show_solutions;
	bool show_grid_lines;
	bool show_input_poses;

public:
	GLWidget3D(MainWindow *parent = 0);

	void drawScene();
	void render();
	void clear();
	void selectAll();
	void unselectAll();
	void deleteSelectedShapes();
	void undo();
	void redo();
	void copySelectedShapes();
	void pasteCopiedShapes();
	void setMode(int mode);
	void addLayer();
	void insertLayer();
	void deleteLayer();
	void setLayer(int layer_id);
	void open(const QString& filename);
	void save(const QString& filename);
	void saveSTL(const QString& dirname);
	void saveSCAD(const QString& dirname);
	void saveImage(const QString& filename);
	glm::dvec2 screenToWorldCoordinates(const glm::dvec2& p);
	glm::dvec2 screenToWorldCoordinates(double x, double y);
	glm::dvec2 worldToScreenCoordinates(const glm::dvec2& p);
	double scale();
	void update3DGeometry();
	void update3DGeometryFromKinematics();
	void calculateSolutions(int linkae_type, int num_samples, std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, double min_transmission_angle, const std::vector<double>& weights, int num_particles, int num_iterations, bool record_file);
	void constructKinematics();
	int findSolution(const std::vector<kinematics::Solution>& solutions, const glm::dvec2& pt, int joint_id);
	void run();
	void runBackward();
	void stop();
	void speedUp();
	void speedDown();
	void invertSpeed();
	void stepForward();
	void stepBackward();

	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

	public slots:
	void animation_update();

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintEvent(QPaintEvent *event);
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseDoubleClickEvent(QMouseEvent* e);
	void wheelEvent(QWheelEvent* e);
};
