#include "GLWidget3D.h"
#include "MainWindow.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <map>
#include <QDir>
#include <QTextStream>
#include <QDate>
#include <iostream>
#include <QProcess>
#include "Rectangle.h"
#include "Circle.h"
#include "Polygon.h"

GLWidget3D::GLWidget3D(MainWindow *parent) : QGLWidget(QGLFormat(QGL::SampleBuffers)) {
	this->mainWin = parent;
	ctrlPressed = false;
	shiftPressed = false;

	first_paint = true;
	front_faced = true;

	mode = MODE_SELECT;
	current_shape.reset();
	operation.reset();

	// add an empty design to the history as an initial state
	history.push(design);

	linkage_type = LINKAGE_4R;
	animation_timer = NULL;
	collision_check = true;
	restrict_motion_range = true;
	show_solutions = false;
	show_grid_lines = true;
	show_input_poses = true;

	// This is necessary to prevent the screen overdrawn by OpenGL
	setAutoFillBackground(false);

	// light direction for shadow mapping
	light_dir = glm::normalize(glm::vec3(-4, -5, -8));

	// model/view/projection matrices for shadow mapping
	glm::mat4 light_pMatrix = glm::ortho<float>(-100, 100, -100, 100, 0.1, 200);
	glm::mat4 light_mvMatrix = glm::lookAt(-light_dir * 50.0f, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	light_mvpMatrix = light_pMatrix * light_mvMatrix;

	// spot light
	spot_light_pos = glm::vec3(2, 2.5, 8);
}

/**
* Draw the scene.
*/
void GLWidget3D::drawScene() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(true);

	renderManager.renderAll();
}

void GLWidget3D::render() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PASS 1: Render to texture
	glUseProgram(renderManager.programs["pass1"]);

	glBindFramebuffer(GL_FRAMEBUFFER, renderManager.fragDataFB);
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderManager.fragDataTex[0], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, renderManager.fragDataTex[1], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, renderManager.fragDataTex[2], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, renderManager.fragDataTex[3], 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderManager.fragDepthTex, 0);

	// Set the list of draw buffers.
	GLenum DrawBuffers[4] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
	glDrawBuffers(4, DrawBuffers); // "3" is the size of DrawBuffers
	// Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		printf("+ERROR: GL_FRAMEBUFFER_COMPLETE false\n");
		exit(0);
	}

	glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["pass1"], "mvpMatrix"), 1, false, &camera.mvpMatrix[0][0]);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "lightDir"), light_dir.x, light_dir.y, light_dir.z);
	glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["pass1"], "light_mvpMatrix"), 1, false, &light_mvpMatrix[0][0]);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "spotLightPos"), spot_light_pos.x, spot_light_pos.y, spot_light_pos.z);
	glUniform3f(glGetUniformLocation(renderManager.programs["pass1"], "cameraPos"), camera.pos.x, camera.pos.y, camera.pos.z);

	glUniform1i(glGetUniformLocation(renderManager.programs["pass1"], "shadowMap"), 6);
	glActiveTexture(GL_TEXTURE6);
	glBindTexture(GL_TEXTURE_2D, renderManager.shadow.textureDepth);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	drawScene();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PASS 2: Create AO
	if (renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
		glUseProgram(renderManager.programs["ssao"]);
		glBindFramebuffer(GL_FRAMEBUFFER, renderManager.fragDataFB_AO);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderManager.fragAOTex, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, renderManager.fragDepthTex_AO, 0);
		GLenum DrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Always check that our framebuffer is ok
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			printf("++ERROR: GL_FRAMEBUFFER_COMPLETE false\n");
			exit(0);
		}

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["ssao"], "pixelSize"), 2.0f / this->width(), 2.0f / this->height());

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex0"), 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex1"), 2);
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "noiseTex"), 7);
		glActiveTexture(GL_TEXTURE7);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragNoiseTex);

		{
			glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["ssao"], "mvpMatrix"), 1, false, &camera.mvpMatrix[0][0]);
			glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["ssao"], "pMatrix"), 1, false, &camera.pMatrix[0][0]);
		}

		glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "uKernelSize"), renderManager.uKernelSize);
		glUniform3fv(glGetUniformLocation(renderManager.programs["ssao"], "uKernelOffsets"), renderManager.uKernelOffsets.size(), (const GLfloat*)renderManager.uKernelOffsets.data());

		glUniform1f(glGetUniformLocation(renderManager.programs["ssao"], "uPower"), renderManager.uPower);
		glUniform1f(glGetUniformLocation(renderManager.programs["ssao"], "uRadius"), renderManager.uRadius);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}
	else if (renderManager.renderingMode == RenderManager::RENDERING_MODE_LINE || renderManager.renderingMode == RenderManager::RENDERING_MODE_HATCHING) {
		glUseProgram(renderManager.programs["line"]);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["line"], "pixelSize"), 1.0f / this->width(), 1.0f / this->height());
		glUniformMatrix4fv(glGetUniformLocation(renderManager.programs["line"], "pMatrix"), 1, false, &camera.pMatrix[0][0]);
		if (renderManager.renderingMode == RenderManager::RENDERING_MODE_HATCHING) {
			glUniform1i(glGetUniformLocation(renderManager.programs["line"], "useHatching"), 1);
		}
		else {
			glUniform1i(glGetUniformLocation(renderManager.programs["line"], "useHatching"), 0);
		}

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex0"), 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex1"), 2);
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "tex3"), 4);
		glActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[3]);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["line"], "hatchingTexture"), 5);
		glActiveTexture(GL_TEXTURE5);
		glBindTexture(GL_TEXTURE_3D, renderManager.hatchingTextures);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}
	else if (renderManager.renderingMode == RenderManager::RENDERING_MODE_CONTOUR) {
		glUseProgram(renderManager.programs["contour"]);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUniform2f(glGetUniformLocation(renderManager.programs["contour"], "pixelSize"), 1.0f / this->width(), 1.0f / this->height());

		glUniform1i(glGetUniformLocation(renderManager.programs["contour"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Blur

	if (renderManager.renderingMode == RenderManager::RENDERING_MODE_BASIC || renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);

		glUseProgram(renderManager.programs["blur"]);
		glUniform2f(glGetUniformLocation(renderManager.programs["blur"], "pixelSize"), 2.0f / this->width(), 2.0f / this->height());
		//printf("pixelSize loc %d\n", glGetUniformLocation(vboRenderManager.programs["blur"], "pixelSize"));

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex0"), 1);//COLOR
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[0]);

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex1"), 2);//NORMAL
		glActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[1]);

		/*glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex2"), 3);
		glActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDataTex[2]);*/

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "depthTex"), 8);
		glActiveTexture(GL_TEXTURE8);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragDepthTex);

		glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "tex3"), 4);//AO
		glActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, renderManager.fragAOTex);

		if (renderManager.renderingMode == RenderManager::RENDERING_MODE_SSAO) {
			glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "ssao_used"), 1); // ssao used
		}
		else {
			glUniform1i(glGetUniformLocation(renderManager.programs["blur"], "ssao_used"), 0); // no ssao
		}

		glBindVertexArray(renderManager.secondPassVAO);

		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		glDepthFunc(GL_LEQUAL);

	}

	// REMOVE
	glActiveTexture(GL_TEXTURE0);
}

void GLWidget3D::clear() {
	design.clear();
	selected_shape.reset();

	// select 1st layer
	setLayer(0);

	// clear the kinematic data
	kinematics.clear();
	solutions.clear();
	selectedJoint = std::make_pair(-1, -1);

	// update 3D geometry
	update3DGeometry();

	update();
}

void GLWidget3D::selectAll() {
	design.selectAll();
	mode = MODE_SELECT;
	update();
}

void GLWidget3D::unselectAll() {
	design.unselectAll();
	current_shape.reset();
	update();
}

void GLWidget3D::deleteSelectedShapes() {
	history.push(design);

	design.deleteSelectedShapes();

	// update 3D geometry
	update3DGeometry();

	current_shape.reset();
	update();
}

void GLWidget3D::undo() {
	try {
		design = history.undo();

		// update 3D geometry
		update3DGeometry();

		update();
	}
	catch (char* ex) {
	}
}

void GLWidget3D::redo() {
	try {
		design = history.redo();

		// update 3D geometry
		update3DGeometry();

		update();
	}
	catch (char* ex) {
	}
}

void GLWidget3D::copySelectedShapes() {
	design.copySelectedShapes();
}

void GLWidget3D::pasteCopiedShapes() {
	design.pasteCopiedShapes();

	// update 3D geometry
	update3DGeometry();

	current_shape.reset();
	mode = MODE_SELECT;
	update();
}


void GLWidget3D::setMode(int mode) {
	if (this->mode != mode) {
		this->mode = mode;

		// clear
		unselectAll();
		selectedJoint = std::make_pair(-1, -1);

		update();
	}
}

void GLWidget3D::addLayer() {
	design.addLayer();
	setLayer(design.num_layers - 1);
}

void GLWidget3D::insertLayer() {
	design.insertLayer();
	setLayer(design.layer_id);
}

void GLWidget3D::deleteLayer() {
	if (design.deleteLayer()) {
		setLayer(design.layer_id);
	}
}

void GLWidget3D::setLayer(int layer_id) {
	design.unselectAll();
	design.layer_id = layer_id;
	current_shape.reset();

	// update 3D geometry
	update3DGeometry();

	// change the mode to SELECT
	setMode(MODE_SELECT);

	update();
}


void GLWidget3D::open(const QString& filename) {
	// if the animation is running, stop it.
	if (animation_timer) {
		stop();
	}

	design.load(filename);
	history.push(design);

	// update 3D geometry
	update3DGeometry();

	// no currently drawing shape
	current_shape.reset();

	mode = MODE_SELECT;

	// clear the kinematic data
	kinematics.clear();
	solutions.clear();

	// update the layer menu based on the loaded data
	mainWin->initLayerMenu(design.num_layers);

	update();
}

void GLWidget3D::save(const QString& filename) {
	design.save(filename);
}

void GLWidget3D::saveSTL(const QString& dirname) {
	//synthesis->saveSTL(dirname, kinematics);
}

void GLWidget3D::saveSCAD(const QString& dirname) {
	for (int i = 0; i < kinematics.size(); i++) {
		synthesis[kinematics[i].linkage_type]->saveSCAD(dirname, i, kinematics[i]);
	}
}

void GLWidget3D::saveImage(const QString& filename) {
	/*
	QPixmap pixmap(size());
	render(&pixmap);
	pixmap.save(filename);
	*/
	QImage image = grabFrameBuffer();
	image.save(filename);
}

glm::dvec2 GLWidget3D::screenToWorldCoordinates(const glm::dvec2& p) {
	return screenToWorldCoordinates(p.x, p.y);
}

glm::dvec2 GLWidget3D::screenToWorldCoordinates(double x, double y) {
	glm::vec2 offset = glm::vec2(camera.pos.x, -camera.pos.y) * (float)scale();
	return glm::dvec2((x - width() * 0.5 + offset.x) / scale(), -(y - height() * 0.5 + offset.y) / scale());
}

glm::dvec2 GLWidget3D::worldToScreenCoordinates(const glm::dvec2& p) {
	return glm::dvec2(width() * 0.5 + (p.x - camera.pos.x) * scale(), height() * 0.5 - (p.y - camera.pos.y) * scale());
}

double GLWidget3D::scale() {
	return camera.f() / camera.pos.z * height() * 0.5;
}

void GLWidget3D::update3DGeometry() {
	design.generate3DGeometry(renderManager);

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}

void GLWidget3D::update3DGeometryFromKinematics() {
	renderManager.removeObjects();
	std::vector<kinematics::Vertex> vertices;
	for (int i = 0; i < kinematics.size(); i++) {
		synthesis[kinematics[i].linkage_type]->generate3DGeometry(kinematics[i], vertices);
	}
	renderManager.addObject("kinematics", "", vertices, true);

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}

void GLWidget3D::calculateSolutions(int linkage_type, int num_samples, std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, double min_transmission_angle, const std::vector<double>& weights, int num_particles, int num_iterations, bool record_file) {
	mainWin->ui.statusBar->showMessage("Please wait for a moment...");

	// change the mode to kinematics
	setMode(MODE_KINEMATICS);
	mainWin->ui.actionKinematics->setChecked(true);

	time_t start = clock();

	this->linkage_type = linkage_type;

	// get the geometry of fixed rigid bodies, moving bodies, linkage regions
	fixed_bodies.clear();
	moving_bodies.resize(design.moving_bodies.size());
	std::vector<std::vector<glm::dmat3x3>> poses(design.moving_bodies.size());
	std::vector<std::vector<glm::dvec2>> linkage_region_pts;
	std::vector<std::vector<glm::dvec2>> linkage_avoidance_pts;
	for (int i = 0; i < design.fixed_bodies.size(); i++) {
		fixed_bodies.push_back(kinematics::Object25D(design.fixed_bodies[i]->getPoints(), -kinematics::options->body_depth, 0));
	}
	for (int i = 0; i < design.moving_bodies.size(); i++) {
		poses[i].resize(design.moving_bodies[i].poses.size());

		moving_bodies[i] = kinematics::Object25D(design.moving_bodies[i].poses[0]->getPoints(), -kinematics::options->body_depth, 0);

		// set pose matrices
		for (int j = 0; j < design.moving_bodies[i].poses.size(); j++) {
			poses[i][j] = design.moving_bodies[i].poses[j]->getModelMatrix();
		}

		if (design.moving_bodies[i].linkage_region) {
			linkage_region_pts.push_back(design.moving_bodies[i].linkage_region->getPoints());
		}
		else {
			// use a bounding box as a default linkage region
			canvas::BoundingBox bbox;
			for (int j = 0; j < design.fixed_bodies.size(); j++) {
				bbox.addPoints(design.fixed_bodies[j]->getPoints());
			}
			bbox.addPoints(design.moving_bodies[i].poses[0]->getPoints());
			linkage_region_pts.push_back({ bbox.minPt, glm::dvec2(bbox.minPt.x, bbox.maxPt.y), bbox.maxPt, glm::dvec2(bbox.maxPt.x, bbox.minPt.y) });
		}

		if (design.moving_bodies[i].linkage_avoidance) {
			linkage_avoidance_pts.push_back(design.moving_bodies[i].linkage_avoidance->getPoints());
		}
		else {
			linkage_avoidance_pts.push_back({});
		}
	}

	// merged fixed body
	std::vector<std::vector<glm::dvec2>> polygons(design.fixed_bodies.size());
	for (int i = 0; i < design.fixed_bodies.size(); i++) {
		polygons[i] = design.fixed_bodies[i]->getPoints();
	}
	polygons = kinematics::unionPolygon(polygons);
	std::vector<kinematics::Object25D> merged_fixed_bodies;
	for (int i = 0; i < polygons.size(); i++) {
		merged_fixed_bodies.push_back(kinematics::Object25D(polygons[i], -kinematics::options->body_depth, 0));
	}
	
	kinematics.clear();
	synthesis.clear();
	synthesis.resize(2);

	if (linkage_type & LINKAGE_4R) {
		synthesis[0] = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesis4R(merged_fixed_bodies, sigmas, avoid_branch_defect, min_transmission_angle, 1.0, weights));
	}
	if (linkage_type & LINKAGE_RRRP) {
		synthesis[1] = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisRRRP(merged_fixed_bodies, sigmas, avoid_branch_defect, min_transmission_angle, 1.0, weights));
	}

	solutions.clear();
	solutions.resize(moving_bodies.size(), {});
	selected_solutions.clear();
	selected_solutions.resize(moving_bodies.size());
	for (int i = 0; i < moving_bodies.size(); i++) {
		time_t start = clock();
		
		// calculate a distance mapt for the linkage region
		cv::Mat dist_map;
		kinematics::BBox dist_map_bbox;
		kinematics::LinkageSynthesis::createDistanceMapForLinkageRegion(linkage_region_pts[i], 5, dist_map_bbox, dist_map);

		std::vector<std::vector<kinematics::Solution>> current_solutions(2);

		// calculate the center of the valid regions
		kinematics::BBox bbox = kinematics::boundingBox(linkage_region_pts[i]);
		glm::dvec2 bbox_center = bbox.center();

		std::vector<glm::dvec2> enlarged_linkage_region_pts;

		int cnt = 0;
		for (int scale = 1; scale <= 3 && cnt == 0; scale++) {
			// calculate the enlarged linkage region for the sampling region
			enlarged_linkage_region_pts = kinematics::LinkageSynthesis::enlargePolygon(linkage_region_pts[i], bbox_center, scale);

			// calculate the bounding boxe of the valid regions
			kinematics::BBox enlarged_bbox = kinematics::boundingBox(enlarged_linkage_region_pts);

			// calculate the circle point curve and center point curve
			for (int j = 0; j < synthesis.size(); j++) {
				if (!synthesis[j]) continue;
				synthesis[j]->calculateSolution(poses[i], enlarged_linkage_region_pts, linkage_avoidance_pts[i], num_samples, moving_bodies[i], current_solutions[j]);
				cnt += current_solutions[j].size();
			}
		}

		time_t end = clock();
		std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for obtaining " << cnt << " candidates." << std::endl;

		if (cnt == 0) {
			mainWin->ui.statusBar->showMessage("No candidate was found.");
		}
		else {
			mainWin->ui.statusBar->showMessage("Running a particle filter...");
		}

		start = clock();

		kinematics::Kinematics kin;
		selected_solutions[i].cost = std::numeric_limits<double>::max();
		for (int j = 0; j < synthesis.size(); j++) {
			if (!synthesis[j]) continue;
			kinematics::Solution solution = synthesis[j]->findBestSolution(poses[i], current_solutions[j], enlarged_linkage_region_pts, dist_map, dist_map_bbox, linkage_avoidance_pts[i], moving_bodies[i], num_particles, num_iterations, record_file);
			if (solution.cost < selected_solutions[i].cost) {
				selected_solutions[i] = solution;
			}

			solutions[i].insert(solutions[i].end(), current_solutions[j].begin(), current_solutions[j].end());
		}

		std::vector<glm::dvec2> connector_pts;
		kin = synthesis[selected_solutions[i].linkage_type]->constructKinematics(selected_solutions[i].poses, selected_solutions[i].points, selected_solutions[i].zorder, moving_bodies[i], true, fixed_bodies, connector_pts);
		kinematics.push_back(kin);

		end = clock();
		std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for particle filter." << std::endl;
	}

	if (solutions[0].size() == 0) {
		mainWin->ui.statusBar->showMessage("No candidate was found.");
	}
	else {
		mainWin->ui.statusBar->showMessage("The linkage mechanism has been generated.");
	}

	// setup the kinematic system
	for (int i = 0; i < kinematics.size(); i++) {
		kinematics[i].diagram.initialize();
	}

	time_t end = clock();
	std::cout << "Total computation time was " << (double)(end - start) / CLOCKS_PER_SEC << " sec." << std::endl;
	
	// update 3D geometry from kinematics
	update3DGeometryFromKinematics();

	update();
}

/**
 * Construct a kinematic diagram based on the selected solution.
 */
void GLWidget3D::constructKinematics() {
	kinematics.clear();

	// construct kinamtics
	for (int i = 0; i < selected_solutions.size(); i++) {
		std::vector<glm::dvec2> connector_pts;
		kinematics::Kinematics kin = synthesis[selected_solutions[i].linkage_type]->constructKinematics(selected_solutions[i].poses, selected_solutions[i].points, selected_solutions[i].zorder, moving_bodies[i], true, fixed_bodies, connector_pts);
		kinematics.push_back(kin);
	}
	
	// setup the kinematic system
	for (int i = 0; i < kinematics.size(); i++) {
		kinematics[i].diagram.initialize();
	}
}

/**
* Find the closest solution.
*
* @param solutions	solution set
* @param pt		mouse position
* @param joint_id	0 -- driving crank / 1 -- follower
*/
int GLWidget3D::findSolution(const std::vector<kinematics::Solution>& solutions, const glm::dvec2& pt, int joint_id) {
	int ans = -1;
	double min_dist = std::numeric_limits<double>::max();

	for (int i = 0; i < solutions.size(); i++) {
		double dist = glm::length(solutions[i].points[joint_id] - pt);
		if (dist < min_dist) {
			min_dist = dist;
			ans = i;
		}
	}

	return ans;
}

void GLWidget3D::run() {
	if (animation_timer == NULL) {
		animation_timer = new QTimer(this);
		connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
		animation_timer->start(10);
	}
}

void GLWidget3D::stop() {
	if (animation_timer != NULL) {
		animation_timer->stop();
		delete animation_timer;
		animation_timer = NULL;
	}
}

void GLWidget3D::speedUp() {
	for (int i = 0; i < kinematics.size(); i++) {
		kinematics[i].speedUp();
	}
}

void GLWidget3D::speedDown() {
	for (int i = 0; i < kinematics.size(); i++) {
		kinematics[i].speedDown();
	}
}

void GLWidget3D::invertSpeed() {
	for (int i = 0; i < kinematics.size(); i++) {
		kinematics[i].invertSpeed();
	}
}

void GLWidget3D::stepForward() {
	if (animation_timer == NULL) {
		for (int i = 0; i < kinematics.size(); i++) {
			try {
				kinematics[i].stepForward(collision_check ? 1 : 0);
			}
			catch (char* ex) {
				kinematics[i].invertSpeed();
				//std::cerr << "Animation is stopped by error:" << std::endl;
				//std::cerr << ex << std::endl;
			}
		}

		update3DGeometryFromKinematics();

		update();
	}
}

void GLWidget3D::stepBackward() {
	if (animation_timer == NULL) {
		for (int i = 0; i < kinematics.size(); i++) {
			try {
				kinematics[i].stepBackward(collision_check ? 1 : 0);
			}
			catch (char* ex) {
				kinematics[i].invertSpeed();
				//std::cerr << "Animation is stopped by error:" << std::endl;
				//std::cerr << ex << std::endl;
			}
		}

		update3DGeometryFromKinematics();

		update();
	}
}

void GLWidget3D::keyPressEvent(QKeyEvent *e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Space:
		// start/stop the animation
		if (animation_timer == NULL) {
			run();
		}
		else {
			stop();
		}
		break;
	default:
		break;
	}
}

void GLWidget3D::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

void GLWidget3D::animation_update() {
	for (int i = 0; i < kinematics.size(); i++) {
		try {
			kinematics[i].stepForward(collision_check ? 1 : 0, true, restrict_motion_range);
		}
		catch (char* ex) {
			kinematics[i].invertSpeed();
			//std::cerr << "Animation is stopped by error:" << std::endl;
			//std::cerr << ex << std::endl;
		}
	}

	update3DGeometryFromKinematics();

	update();

}

/**
* This function is called once before the first call to paintGL() or resizeGL().
*/
void GLWidget3D::initializeGL() {
	// init glew
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		std::cout << "Error: " << glewGetErrorString(err) << std::endl;
	}

	if (glewIsSupported("GL_VERSION_4_2"))
		printf("Ready for OpenGL 4.2\n");
	else {
		printf("OpenGL 4.2 not supported\n");
		exit(1);
	}
	const GLubyte* text = glGetString(GL_VERSION);
	printf("VERSION: %s\n", text);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_TEXTURE_2D);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	glTexGenf(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glTexGenf(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
	glDisable(GL_TEXTURE_2D);

	glEnable(GL_TEXTURE_3D);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glDisable(GL_TEXTURE_3D);

	glEnable(GL_TEXTURE_2D_ARRAY);
	glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glDisable(GL_TEXTURE_2D_ARRAY);

	////////////////////////////////
	renderManager.init("", "", "", true, 8192);
	renderManager.resize(this->width(), this->height());

	glUniform1i(glGetUniformLocation(renderManager.programs["ssao"], "tex0"), 0);//tex0: 0
}

/**
* This function is called whenever the widget has been resized.
*/
void GLWidget3D::resizeGL(int width, int height) {
	height = height ? height : 1;
	glViewport(0, 0, width, height);
	camera.updatePMatrix(width, height);

	renderManager.resize(width, height);
}

/**
* This function is called whenever the widget needs to be painted.
*/
void GLWidget3D::paintEvent(QPaintEvent *event) {
	if (first_paint) {
		std::vector<kinematics::Vertex> vertices;
		kinematics::glutils::drawQuad(0.001, 0.001, glm::vec4(1, 1, 1, 1), glm::mat4(), vertices);
		renderManager.addObject("dummy", "", vertices, true);
		renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
		first_paint = false;
	}

	// draw by OpenGL
	makeCurrent();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	render();

	// unbind texture
	glActiveTexture(GL_TEXTURE0);

	// restore the settings for OpenGL
	glShadeModel(GL_FLAT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	QPainter painter(this);
	painter.setOpacity(1.0f);
	if (abs(camera.xrot) < 10 && abs(camera.yrot) < 10) {
		// draw grid
		if (show_grid_lines) {
			painter.save();
			painter.setPen(QPen(QColor(224, 224, 224), 1));
			for (int i = -200; i <= 200; i++) {
				glm::dvec2 p = worldToScreenCoordinates(glm::dvec2(i * 5, i * 5));
				painter.drawLine(p.x, 0, p.x, height());
				glm::dvec2 p2 = worldToScreenCoordinates(glm::dvec2(0, i * 5));
				painter.drawLine(0, p.y, width(), p.y);
			}
			painter.restore();
		}

		// draw 2D
		glm::vec2 offset = glm::vec2(camera.pos.x, -camera.pos.y) * (float)scale();
		if (mode != MODE_KINEMATICS) {
			// render unselected layers as background
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				for (int j = 0; j < design.moving_bodies[i].poses.size(); j++) {
					if (j == design.layer_id) continue;
					design.moving_bodies[i].poses[j]->draw(painter, QColor(0, 255, 0, 60), QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
				}
			}

			// make the unselected layers faded
			painter.setPen(QColor(255, 255, 255, 160));
			painter.setBrush(QColor(255, 255, 255, 160));
			painter.drawRect(0, 0, width(), height());

			// render selected layer
			for (int i = 0; i < design.fixed_bodies.size(); i++) {
				design.fixed_bodies[i]->draw(painter, QColor(150, 255, 0, 60), QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				design.moving_bodies[i].poses[design.layer_id]->draw(painter, QColor(0, 255, 0, 60), QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				if (design.moving_bodies[i].linkage_region) {
					design.moving_bodies[i].linkage_region->draw(painter, QColor(0, 0, 255, 30), QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
				}
				if (design.moving_bodies[i].linkage_avoidance) {
					design.moving_bodies[i].linkage_avoidance->draw(painter, QColor(255, 0, 0, 30), QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
				}
			}

			// render currently drawing shape
			if (current_shape) {
				current_shape->draw(painter, QColor(0, 0, 0, 0), QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}
		}
		else {
			// make the 3D faded
			painter.setPen(QColor(255, 255, 255, 160));
			painter.setBrush(QColor(255, 255, 255, 160));
			painter.drawRect(0, 0, width(), height());

			// draw solutions
			if (show_solutions && selectedJoint.first >= 0) {
				int linkage_id = selectedJoint.first;
				painter.setPen(QPen(QColor(128, 128, 255, 64), 1));
				painter.setBrush(QBrush(QColor(128, 128, 255, 64)));
				for (int i = 0; i < solutions[linkage_id].size(); i++) {
					if (selectedJoint.second < solutions[linkage_id][i].points.size()) {
						painter.drawEllipse(width() * 0.5 - offset.x + solutions[linkage_id][i].points[selectedJoint.second].x * scale(), height() * 0.5 - offset.y - solutions[linkage_id][i].points[selectedJoint.second].y * scale(), 3, 3);
					}
				}
			}

			// draw input poses
			if (show_input_poses) {
				painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
				painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
				for (int i = 0; i < design.moving_bodies.size(); i++) {
					for (int j = 0; j < design.moving_bodies[i].poses.size(); j++) {
						QPolygonF pts;
						std::vector<glm::dvec2>& body = design.moving_bodies[i].poses[j]->getPoints();
						for (int k = 0; k < body.size(); k++) {
							pts.push_back(QPointF(width() * 0.5 - offset.x + body[k].x * scale(), height() * 0.5 - offset.y - body[k].y * scale()));
						}
						pts.push_back(pts.front());
						painter.drawPolygon(pts);
					}
				}
			}

			// draw 2D mechanism
			for (int i = 0; i < kinematics.size(); i++) {
				kinematics[i].draw(painter, QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}
		}

		// draw axes
		if (show_grid_lines) {
			painter.save();
			painter.setPen(QPen(QColor(128, 128, 128), 1));
			glm::dvec2 x1 = worldToScreenCoordinates(glm::dvec2(-10000, 0));
			glm::dvec2 x2 = worldToScreenCoordinates(glm::dvec2(10000, 0));
			painter.drawLine(0, x1.y, width(), x2.y);
			glm::dvec2 y1 = worldToScreenCoordinates(glm::dvec2(0, -10000));
			glm::dvec2 y2 = worldToScreenCoordinates(glm::dvec2(0, 10000));
			painter.drawLine(y1.x, height(), y2.x, 0);
			painter.restore();
		}
	}
	painter.end();

	glEnable(GL_DEPTH_TEST);
}

/**
* This event handler is called when the mouse press events occur.
*/
void GLWidget3D::mousePressEvent(QMouseEvent *e) {
	// This is necessary to get key event occured even after the user selects a menu.
	setFocus();

	if (e->buttons() & Qt::LeftButton) {
		if (mode == MODE_SELECT) {
			// hit test for rotation marker
			glm::dvec2 rotate_pivot;
			if (design.hitTestRotationMarker(screenToWorldCoordinates(e->x(), e->y()), scale(), 10 / scale(), selected_shape, rotate_pivot)) {
				// start rotating
				mode = MODE_ROTATION;
				operation = boost::shared_ptr<canvas::Operation>(new canvas::RotateOperation(screenToWorldCoordinates(e->x(), e->y()), rotate_pivot));
				update();
				return;
			}

			// hit test for resize marker
			glm::dvec2 resize_pivot;
			if (design.hitTestResizeMarker(screenToWorldCoordinates(e->x(), e->y()), 10 / scale(), selected_shape, resize_pivot)) {
				// start resizing
				mode = MODE_RESIZE;
				operation = boost::shared_ptr<canvas::Operation>(new canvas::ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), resize_pivot));
				update();
				return;
			}

			// hit test for the shape
			if (design.hitTest(screenToWorldCoordinates(e->x(), e->y()), ctrlPressed, selected_shape)) {
				// start moving
				mode = MODE_MOVE;
				operation = boost::shared_ptr<canvas::Operation>(new canvas::MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
				update();
				return;
			}

			unselectAll();
		}
		else if (mode == MODE_FIXED_RECTANGLE || mode == MODE_MOVING_RECTANGLE) {
			if (!current_shape) {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Rectangle(screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_FIXED_CIRCLE || mode == MODE_MOVING_CIRCLE) {
			if (!current_shape) {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Circle(screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_POLYGON || mode == MODE_LINKAGE_REGION || mode == MODE_LINKAGE_AVOIDANCE) {
			if (current_shape) {
				current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
			}
			else {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Polygon(screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_KINEMATICS) {
			// convert the mouse position to the world coordinate system
			glm::dvec2 pt = screenToWorldCoordinates(e->x(), e->y());

			// select a joint to move
			selectedJoint = std::make_pair(-1, -1);
			double min_dist = 2;
			for (int i = 0; i < kinematics.size(); i++) {
				for (int j = 0; j < kinematics[i].diagram.joints.size(); j++) {
					double dist = glm::length(kinematics[i].diagram.joints[j]->pos - pt);
					if (dist < min_dist) {
						min_dist = dist;
						selectedJoint = std::make_pair(i, j);
					}
				}
			}
		}
	}
	else if (e->buttons() & Qt::RightButton) {
		camera.mousePress(e->x(), e->y());
	}
}

/**
* This event handler is called when the mouse move events occur.
*/

void GLWidget3D::mouseMoveEvent(QMouseEvent *e) {
	if (e->buttons() & Qt::RightButton) {
		if (shiftPressed) {
			camera.move(e->x(), e->y());
		}
		else {
			camera.rotate(e->x(), e->y(), (ctrlPressed ? 0.1 : 1));
		}
	}
	else if (mode == MODE_MOVE) {
		boost::shared_ptr<canvas::MoveOperation> op = boost::static_pointer_cast<canvas::MoveOperation>(operation);
		glm::dvec2 dir = screenToWorldCoordinates(e->x(), e->y()) - op->pivot;
		design.move(dir, renderManager);

		// update shadow map
		renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);

		op->pivot = screenToWorldCoordinates(e->x(), e->y());
		update();
	}
	else if (mode == MODE_ROTATION) {
		boost::shared_ptr<canvas::RotateOperation> op = boost::static_pointer_cast<canvas::RotateOperation>(operation);
		glm::dvec2 dir1 = op->pivot - op->rotation_center;
		glm::dvec2 dir2 = screenToWorldCoordinates(e->x(), e->y()) - op->rotation_center;
		double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
		design.rotate(theta, renderManager);

		// update shadow map
		renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);

		op->pivot = screenToWorldCoordinates(e->x(), e->y());
		update();
	}
	else if (mode == MODE_RESIZE) {
		boost::shared_ptr<canvas::ResizeOperation> op = boost::static_pointer_cast<canvas::ResizeOperation>(operation);
		glm::dvec2 resize_center = selected_shape->localCoordinate(op->resize_center);
		glm::dvec2 dir1 = selected_shape->localCoordinate(op->pivot) - resize_center;
		glm::dvec2 dir2 = selected_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())) - resize_center;
		glm::dvec2 resize_scale(dir2.x / dir1.x, dir2.y / dir1.y);
		design.resize(resize_scale, resize_center, renderManager);

		// update shadow map
		renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);

		op->pivot = screenToWorldCoordinates(e->x(), e->y());
		update();
	}
	else if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON || mode == MODE_LINKAGE_REGION || mode == MODE_LINKAGE_AVOIDANCE) {
		if (current_shape) {
			current_shape->updateByNewPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())), shiftPressed);
		}
	}
	else if (mode == MODE_KINEMATICS) {
		if (selectedJoint.first >= 0) {
			int linkage_id = selectedJoint.first;
			int joint_id = selectedJoint.second;
			glm::dvec2 pt = screenToWorldCoordinates(e->x(), e->y());

			// select a solution
			int selectedSolution = findSolution(solutions[linkage_id], pt, joint_id);
			if (selectedSolution >= 0) {
				selected_solutions[linkage_id] = solutions[linkage_id][selectedSolution];
			}

			// update the geometry
			constructKinematics();
			update();
		}
	}

	update();
}

/**
* This event handler is called when the mouse release events occur.
*/
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e) {
	if (e->button() == Qt::RightButton) {
		//if (abs(camera.xrot) < 20 && abs(camera.yrot) < 20) {
		camera.xrot = 0;
		camera.yrot = 0;
		camera.updateMVPMatrix();
		front_faced = true;
	}
	else if (mode == MODE_MOVE || mode == MODE_ROTATION || mode == MODE_RESIZE) {
		history.push(design);
		mode = MODE_SELECT;
	}
	else if (mode == MODE_KINEMATICS) {
		if (selectedJoint.first >= 0) {
			constructKinematics();
			update3DGeometryFromKinematics();
		}
	}

	update();
}

void GLWidget3D::mouseDoubleClickEvent(QMouseEvent* e) {
	if (e->button() == Qt::LeftButton) {
		if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON) {
			// The shape is created.
			current_shape->completeDrawing();
			design.fixed_bodies.push_back(current_shape->clone());
			design.fixed_bodies.back()->select();
		}
		else if (mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON) {
			// The shape is created.
			current_shape->completeDrawing();
			design.addMovingBody(current_shape);			
		}
		else if (mode == MODE_LINKAGE_REGION) {
			// The shape is created.
			current_shape->completeDrawing();
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				if (!design.moving_bodies[i].linkage_region) {
					design.moving_bodies[i].linkage_region = current_shape->clone();
					design.moving_bodies[i].linkage_region->select();
					break;
				}
			}
		}
		else if (mode == MODE_LINKAGE_AVOIDANCE) {
			// The shape is created.
			current_shape->completeDrawing();
			for (int i = 0; i < design.moving_bodies.size(); i++) {
				if (!design.moving_bodies[i].linkage_avoidance) {
					design.moving_bodies[i].linkage_avoidance = current_shape->clone();
					design.moving_bodies[i].linkage_avoidance->select();
					break;
				}
			}
		}

		if (mode == MODE_FIXED_RECTANGLE || mode == MODE_FIXED_CIRCLE || mode == MODE_FIXED_POLYGON || mode == MODE_MOVING_RECTANGLE || mode == MODE_MOVING_CIRCLE || mode == MODE_MOVING_POLYGON || mode == MODE_LINKAGE_REGION || mode == MODE_LINKAGE_AVOIDANCE) {
			// update 3D geometry
			update3DGeometry();

			mode = MODE_SELECT;
			history.push(design);
			current_shape.reset();
			operation.reset();
			mainWin->ui.actionSelect->setChecked(true);
		}
	}

	setMouseTracking(false);

	update();
}

void GLWidget3D::wheelEvent(QWheelEvent* e) {
	camera.zoom(e->delta() * 0.2);
	update();
}
