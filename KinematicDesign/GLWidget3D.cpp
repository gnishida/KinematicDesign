#include "GLWidget3D.h"
#include "MainWindow.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <map>
#include "GLUtils.h"
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
	layers.resize(2);
	layer_id = 0;
	current_shape.reset();
	operation.reset();

	linkage_type = LINKAGE_4R;
	animation_timer = NULL;
	collision_check = true;

	// This is necessary to prevent the screen overdrawn by OpenGL
	setAutoFillBackground(false);

	// light direction for shadow mapping
	light_dir = glm::normalize(glm::vec3(-4, -5, -8));

	// model/view/projection matrices for shadow mapping
	glm::mat4 light_pMatrix = glm::ortho<float>(-50, 50, -50, 50, 0.1, 200);
	glm::mat4 light_mvMatrix = glm::lookAt(-light_dir * 50.0f, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	light_mvpMatrix = light_pMatrix * light_mvMatrix;
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
	else if (renderManager.renderingMode == RenderManager::RENDERING_MODE_LINE || renderManager.renderingMode == RenderManager::RENDERING_MODE_HATCHING || renderManager.renderingMode == RenderManager::RENDERING_MODE_SKETCHY) {
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
	for (int i = 0; i < layers.size(); ++i) {
		layers[i].clear();
	}
	selected_shape.reset();

	// update 3D geometry
	update3DGeometry();

	update();
}

void GLWidget3D::selectAll() {
	layers[layer_id].selectAll();

	mode = MODE_SELECT;
	update();
}

void GLWidget3D::unselectAll() {
	layers[layer_id].unselectAll();

	current_shape.reset();
	update();
}

void GLWidget3D::deleteSelectedShapes() {
	for (int i = layers[layer_id].shapes.size() - 1; i >= 0; --i) {
		if (layers[layer_id].shapes[i]->isSelected()) {
			for (int l = 0; l < layers.size(); l++) {
				layers[l].shapes.erase(layers[l].shapes.begin() + i);
			}
		}
	}

	// update 3D geometry
	update3DGeometry();

	current_shape.reset();
	update();
}

void GLWidget3D::undo() {
	try {
		layers = history.undo();

		// update 3D geometry
		update3DGeometry();

		update();
	}
	catch (char* ex) {
	}
}

void GLWidget3D::redo() {
	try {
		layers = history.redo();

		// update 3D geometry
		update3DGeometry();

		update();
	}
	catch (char* ex) {
	}
}

void GLWidget3D::copySelectedShapes() {
	layers[layer_id].copySelectedShapes(copied_shapes);

	// update 3D geometry
	update3DGeometry();

	update();
}

void GLWidget3D::pasteCopiedShapes() {
	layers[layer_id].pasteCopiedShapes(copied_shapes);

	// update 3D geometry
	update3DGeometry();

	current_shape.reset();
	mode = MODE_SELECT;
	update();
}


void GLWidget3D::setMode(int mode) {
	if (this->mode != mode) {
		this->mode = mode;

		update();
	}
}

void GLWidget3D::addLayer() {
	layers.push_back(layers.back().clone());
	setLayer(layers.size() - 1);
}

void GLWidget3D::insertLayer() {
	layers.insert(layers.begin() + layer_id, layers[layer_id].clone());
	setLayer(layer_id);
}

void GLWidget3D::deleteLayer() {
	// we assume that there must be at least two layers.
	if (layers.size() <= 2) return;

	layers.erase(layers.begin() + layer_id);
	if (layer_id >= layers.size()) {
		layer_id--;
	}
	setLayer(layer_id);
}

void GLWidget3D::setLayer(int layer_id) {
	layers[this->layer_id].unselectAll();
	this->layer_id = layer_id;
	current_shape.reset();

	// update 3D geometry
	update3DGeometry();

	// change the mode to SELECT
	setMode(MODE_SELECT);

	update();
}


void GLWidget3D::open(const QString& filename) {
	QFile file(filename);
	if (!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

	// if the animation is running, stop it.
	if (animation_timer) {
		stop();
	}

	QDomDocument doc;
	doc.setContent(&file);

	QDomElement root = doc.documentElement();
	if (root.tagName() != "design")	throw "Invalid file format.";

	// clear the data
	layers.clear();
	selected_shape.reset();
	mode = MODE_SELECT;

	QDomNode layer_node = root.firstChild();
	while (!layer_node.isNull()) {
		if (layer_node.toElement().tagName() == "layer") {
			canvas::Layer layer;
			layer.load(layer_node.toElement());
			layers.push_back(layer);
		}

		layer_node = layer_node.nextSibling();
	}

	// select 1st layer to display
	layer_id = 0;

	// update 3D geometry
	update3DGeometry();

	// no currently drawing shape
	current_shape.reset();

	// update the layer menu based on the loaded data
	mainWin->initLayerMenu(layers.size());

	update();
}

void GLWidget3D::save(const QString& filename) {
	QFile file(filename);
	if (!file.open(QFile::WriteOnly)) throw "File cannot open.";

	QDomDocument doc;

	// set root node
	QDomElement root = doc.createElement("design");
	root.setAttribute("author", "Gen Nishida");
	root.setAttribute("version", "1.0");
	root.setAttribute("date", QDate::currentDate().toString("MM/dd/yyyy"));
	doc.appendChild(root);

	// write layers
	for (int i = 0; i < layers.size(); ++i) {
		QDomElement layer_node = layers[i].toXml(doc);
		root.appendChild(layer_node);
	}

	QTextStream out(&file);
	doc.save(out, 4);
}

glm::dvec2 GLWidget3D::screenToWorldCoordinates(const glm::dvec2& p) {
	return screenToWorldCoordinates(p.x, p.y);
}

glm::dvec2 GLWidget3D::screenToWorldCoordinates(double x, double y) {
	glm::vec2 offset = glm::vec2(camera.pos.x, -camera.pos.y) * (float)scale();
	return glm::dvec2((x - width() * 0.5 + offset.x) / scale(), -(y - height() * 0.5 + offset.y) / scale());
}

glm::dvec2 GLWidget3D::worldToScreenCoordinates(const glm::dvec2& p) {
	//return glm::dvec2(width() * 0.5 + p.x * camera.f() / camera.pos.z * width() * 0.5, height() * 0.5 - p.y * scale());
	return glm::dvec2(width() * 0.5 + (p.x - camera.pos.x) * scale(), height() * 0.5 - (p.y - camera.pos.y) * scale());
}

double GLWidget3D::scale() {
	return camera.f() / camera.pos.z * height() * 0.5;
}

void GLWidget3D::update3DGeometry() {
	renderManager.removeObjects();
	for (int i = 0; i < layers[layer_id].shapes.size(); i++) {
		if (layers[layer_id].shapes[i]->getSubType() == canvas::Shape::TYPE_BODY) {
			QString obj_name = QString("object_%1").arg(i);
			renderManager.addObject(obj_name, "", layers[layer_id].shapes[i]->getVertices(), true);
		}
	}

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}

void GLWidget3D::update3DGeometryFromKinematics() {
	renderManager.removeObjects();
	std::vector<Vertex> vertices;
	for (int i = 0; i < kinematics.size(); i++) {
		// generate geometry of rigid bodies
 		for (int j = 0; j < kinematics[i].diagram.bodies.size(); j++) {
			std::vector<glm::dvec2> points = kinematics[i].diagram.bodies[j]->getActualPoints();
			float z = kinematics[i].diagram.bodies[j]->polygon.depth1;
			float depth = kinematics[i].diagram.bodies[j]->polygon.depth2 - kinematics[i].diagram.bodies[j]->polygon.depth1;
			glutils::drawPrism(points, depth, glm::vec4(0.7, 1, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);
		}

		// generate geometry of links
		const float link_radius = 0.5f;
		const float link_depth = 0.3f;
		const float joint_radius = 0.25f;
		const float joint_depth = 0.15f;
		for (int j = 0; j < kinematics[i].diagram.links.size(); j++) {
			glm::dvec2& p1 = kinematics[i].diagram.links[j]->joints[0]->pos;
			glm::dvec2& p2 = kinematics[i].diagram.links[j]->joints[1]->pos;
			std::vector<glm::dvec2> pts = generateRoundedBarPolygon(glm::vec2(p1.x, p1.y), glm::vec2(p2.x, p2.y), link_radius);

			// HACK
			// This part is currently very unorganized.
			// For each type of mechanism, I have to hard code the depth of each link.
			// In the future, this part of code should be moved to the class of each type of mechanism.
			float z;
			if (j == 0) z = (link_depth + joint_depth) * 1;
			else if (j == 1) z = (link_depth + joint_depth) * 1;
			else z = (link_depth + joint_depth) * 2;
			glutils::drawPrism(pts, link_depth, glm::vec4(0.7, 0.7, 0.7, 1), glm::translate(glm::mat4(), glm::vec3(0, 0, z)), vertices);

			// joints
			float height = 0;
			if (j == 0) {
				height = (link_depth + joint_depth) * 1 + joint_depth;
			}
			else if (j == 1) {
				height = (link_depth + joint_depth) * 1 + joint_depth;
			}
			else {
				height = (link_depth + joint_depth) * 2 + joint_depth;
			}
			glutils::drawCylinderZ(joint_radius, joint_radius, joint_radius, joint_radius, height, glm::vec4(0.9, 0.9, 0.9, 1), glm::translate(glm::mat4(), glm::vec3(p1.x, p1.y, link_depth)), vertices, 24);
			glutils::drawCylinderZ(joint_radius, joint_radius, joint_radius, joint_radius, height, glm::vec4(0.9, 0.9, 0.9, 1), glm::translate(glm::mat4(), glm::vec3(p2.x, p2.y, link_depth)), vertices, 24);
		}
	}
	renderManager.addObject("kinematics", "", vertices, true);

	// update shadow map
	renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
}

std::vector<glm::dvec2> GLWidget3D::generateRoundedBarPolygon(const glm::dvec2& p1, const glm::dvec2& p2, float link_radius, int num_slices) {
	std::vector<glm::dvec2> ans;

	double theta0 = atan2(p2.y - p1.y, p2.x - p1.x) + glutils::M_PI * 0.5;
	for (int k = 0; k <= num_slices/2; k++) {
		if (p1 != p2 || k < num_slices / 2) {
			double theta = theta0 + glutils::M_PI / 12 * k;
			ans.push_back(glm::dvec2(cos(theta) * link_radius + p1.x, sin(theta) * link_radius + p1.y));
		}
	}
	theta0 += glutils::M_PI;
	for (int k = 0; k <= num_slices/2; k++) {
		if (p1 != p2 || k < num_slices / 2) {
			double theta = theta0 + glutils::M_PI / 12 * k;
			ans.push_back(glm::dvec2(cos(theta) * link_radius + p2.x, sin(theta) * link_radius + p2.y));
		}
	}

	return ans;
}

void GLWidget3D::calculateSolutions(int linkage_type, int num_samples, std::vector<std::pair<double, double>>& sigmas, bool avoid_branch_defect, bool rotatable_crank, double position_error_weight, double orientation_error_weight, double linkage_location_weight, double trajectory_weight, double size_weight) {
	mainWin->ui.statusBar->showMessage("Please wait for a moment...");

	// change the mode to kinematics
	setMode(MODE_KINEMATICS);
	mainWin->ui.actionKinematics->setChecked(true);

	time_t start = clock();

	this->linkage_type = linkage_type;

	// get the geometry of fixed rigid bodies, moving bodies, linkage regions
	fixed_body_pts.clear();
	body_pts.clear();
	linkage_region_pts.clear();
	poses.clear();
	for (int i = 0; i < layers[0].shapes.size(); i++) {
		int subtype = layers[0].shapes[i]->getSubType();
		if (subtype == canvas::Shape::TYPE_BODY) {
			glm::dmat3x3 mat0 = layers[0].shapes[i]->getModelMatrix();

			bool moved = false;
			for (int j = 0; j < layers.size(); j++) {
				glm::dmat3x3 mat = layers[j].shapes[i]->getModelMatrix();
				if (mat != mat0) {
					moved = true;
					break;
				}
			}

			if (moved) {
				body_pts.push_back(kinematics::Polygon25D(layers[0].shapes[i]->getPoints(), -10, 0));

				// calcualte poses of the moving body
				poses.resize(poses.size() + 1);
				for (int j = 0; j < layers.size(); j++) {
					poses.back().push_back(layers[j].shapes[i]->getModelMatrix());
				}
			}
			else {
				//fixed_body_pts.push_back(layers[0].shapes[i]->getPoints());
				fixed_body_pts.push_back(kinematics::Polygon25D(layers[0].shapes[i]->getPoints(), -10, 0));
			}
		}
		else if (subtype == canvas::Shape::TYPE_LINKAGE_REGION) {
			linkage_region_pts.push_back(layers[0].shapes[i]->getPoints());
		}
	}

	// if the linkage region is not specified, use a large enough region as default
	if (linkage_region_pts.size() < poses.size()) {
		int num_linkage_regions = linkage_region_pts.size();
		linkage_region_pts.resize(poses.size());
		for (int i = num_linkage_regions; i < poses.size(); i++) {
			linkage_region_pts[i].push_back(glm::dvec2(-40, -40));
			linkage_region_pts[i].push_back(glm::dvec2(-40, 40));
			linkage_region_pts[i].push_back(glm::dvec2(40, 40));
			linkage_region_pts[i].push_back(glm::dvec2(40, -40));
		}
	}

	kinematics.clear();

	solutions.resize(body_pts.size(), std::vector<kinematics::Solution>(2));
	for (int i = 0; i < body_pts.size(); i++) {
		time_t start = clock();

		boost::shared_ptr<kinematics::LinkageSynthesis> synthesis;
		if (linkage_type == LINKAGE_4R) {
			synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesis4R());
		}
		else if (linkage_type == LINKAGE_RRRP) {
			synthesis = boost::shared_ptr<kinematics::LinkageSynthesis>(new kinematics::LinkageSynthesisRRRP());
		}

		// calculate the circle point curve and center point curve
		synthesis->calculateSolution(poses[i], linkage_region_pts[i], num_samples, fixed_body_pts, body_pts[i], sigmas, rotatable_crank, avoid_branch_defect, 1.0, solutions[i]);

		if (solutions[i].size() == 0) {
			mainWin->ui.statusBar->showMessage("No candidate was found.");
		}
		else if (solutions[i].size() == 0) {
			mainWin->ui.statusBar->showMessage("1 candidate was found.");
		}
		else {
			mainWin->ui.statusBar->showMessage(QString("%1 candidates were found.").arg(solutions[i].size()));
		}

		time_t end = clock();
		std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for obtaining " << solutions[i].size() << " candidates." << std::endl;

		start = clock();
		if (linkage_type == LINKAGE_4R) {
			kinematics::Solution solution = synthesis->findBestSolution(poses[i], solutions[i], fixed_body_pts, body_pts[i], position_error_weight, orientation_error_weight, linkage_location_weight, trajectory_weight, size_weight);

			// construct a linkage
			kinematics::Kinematics kin;
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, solution.fixed_point[0])));
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, solution.fixed_point[1])));
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, solution.moving_point[0])));
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, solution.moving_point[1])));
			kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2]);
			kin.diagram.addLink(false, kin.diagram.joints[1], kin.diagram.joints[3]);
			kin.diagram.addLink(false, kin.diagram.joints[2], kin.diagram.joints[3]);

			// update the geometry
			kin.diagram.bodies.clear();
			kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[3], body_pts[i]);

			kinematics.push_back(kin);

			//updateDefectFlag(solution.poses, kinematics[0]);
		}
		else if (linkage_type == LINKAGE_RRRP) {
			kinematics::Solution solution = synthesis->findBestSolution(poses[i], solutions[i], fixed_body_pts, body_pts[i], position_error_weight, orientation_error_weight, linkage_location_weight, trajectory_weight, size_weight);

			// construct a linkage
			kinematics::Kinematics kin;
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, solution.fixed_point[0])));
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, solution.fixed_point[1])));
			kin.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, solution.moving_point[0])));
			kin.diagram.addJoint(boost::shared_ptr<kinematics::SliderHinge>(new kinematics::SliderHinge(3, false, solution.moving_point[1])));
			kin.diagram.addLink(true, kin.diagram.joints[0], kin.diagram.joints[2]);
			kin.diagram.addLink(false, kin.diagram.joints[1], kin.diagram.joints[3]);
			kin.diagram.addLink(false, kin.diagram.joints[2], kin.diagram.joints[3]);

			// update the geometry
			kin.diagram.bodies.clear();
			kin.diagram.addBody(kin.diagram.joints[2], kin.diagram.joints[3], body_pts[i]);

			kinematics.push_back(kin);

			//updateDefectFlag(solution.poses, kinematics[0]);
		}

		end = clock();
		std::cout << "Elapsed: " << (double)(end - start) / CLOCKS_PER_SEC << " sec for finding the best solution. " << std::endl;
	}














	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// connect the joints to the rigid bodies

	const float link_radius = 0.5f;
	const float link_depth = 0.3f;
	int N = fixed_body_pts.size();
	for (int i = 0; i < kinematics.size(); i++) {
		for (int j = 0; j < kinematics[i].diagram.joints.size(); j++) {
			boost::shared_ptr<kinematics::Joint>& joint = kinematics[i].diagram.joints[j];

			if (joint->ground) {
				// check if the joint is within the rigid body
				bool is_inside = false;
				for (int k = 0; k < N; k++) {
					if (kinematics::withinPolygon(fixed_body_pts[k].points, joint->pos)) {
						is_inside = true;
						break;
					}
				}

				glm::dvec2 closest_point;
				if (is_inside) {
					closest_point = joint->pos;
				}
				else {
					// find the closest point of a rigid body
					double min_dist = std::numeric_limits<double>::max();
					for (int k = 0; k < N; k++) {
						glm::dvec2 cp = kinematics::closestPoint(fixed_body_pts[k].points, joint->pos);

						// extend the point a little into the rigid body
						glm::dvec2 v = glm::normalize(cp - joint->pos);
						v *= link_radius;
						cp += v;

						double dist = glm::length(cp - joint->pos);
						if (dist < min_dist) {
							min_dist = dist;
							closest_point = cp;
						}
					}
				}

				// create a geometry to extend the body to the joint
				std::vector<glm::dvec2> pts = generateRoundedBarPolygon(closest_point, joint->pos, link_radius);
				fixed_body_pts.push_back(kinematics::Polygon25D(pts, 0, link_depth));
			}
		}
	}

	for (int i = 0; i < kinematics.size(); i++) {
		int N = kinematics[i].diagram.bodies.size();
		for (int j = 0; j < N; j++) {
			if (kinematics[i].diagram.bodies[j]->pivot1->ground || kinematics[i].diagram.bodies[j]->pivot2->ground) continue;

			std::vector<glm::dvec2> body_pts = kinematics[i].diagram.bodies[j]->getActualPoints();

			bool is_inside1 = kinematics::withinPolygon(body_pts, kinematics[i].diagram.bodies[j]->pivot1->pos);
			bool is_inside2 = kinematics::withinPolygon(body_pts, kinematics[i].diagram.bodies[j]->pivot2->pos);

			if (is_inside1) {
				std::vector<glm::dvec2> pts = generateRoundedBarPolygon(kinematics[i].diagram.bodies[j]->pivot1->pos, kinematics[i].diagram.bodies[j]->pivot1->pos, link_radius);

				// create a geometry to extend the body to the joint
				kinematics[i].diagram.addBody(kinematics[i].diagram.bodies[j]->pivot1, kinematics[i].diagram.bodies[j]->pivot2, kinematics::Polygon25D(pts, 0, link_depth));
			}
			
			if (is_inside2) {
				std::vector<glm::dvec2> pts = generateRoundedBarPolygon(kinematics[i].diagram.bodies[j]->pivot2->pos, kinematics[i].diagram.bodies[j]->pivot2->pos, link_radius);

				// create a geometry to extend the body to the joint
				kinematics[i].diagram.addBody(kinematics[i].diagram.bodies[j]->pivot1, kinematics[i].diagram.bodies[j]->pivot2, kinematics::Polygon25D(pts, 0, link_depth));
			}

			if (!is_inside1 && !is_inside2) {
				// find the closest point of a rigid body
				glm::dvec2 cp1 = kinematics::closestPoint(body_pts, kinematics[i].diagram.bodies[j]->pivot1->pos);

				// extend the point a little into the rigid body
				glm::dvec2 v1 = glm::normalize(cp1 - kinematics[i].diagram.bodies[j]->pivot1->pos);
				v1 *= link_radius;
				cp1 += v1;

				double dist1 = glm::length(cp1 - kinematics[i].diagram.bodies[j]->pivot1->pos);

				glm::dvec2 cp2 = kinematics::closestPoint(body_pts, kinematics[i].diagram.bodies[j]->pivot2->pos);

				// extend the point a little into the rigid body
				glm::dvec2 v2 = glm::normalize(cp2 - kinematics[i].diagram.bodies[j]->pivot2->pos);
				v2 *= link_radius;
				cp2 += v2;

				double dist2 = glm::length(cp2 - kinematics[i].diagram.bodies[j]->pivot2->pos);

				std::vector<glm::dvec2> pts;
				if (dist1 < dist2) {
					pts = generateRoundedBarPolygon(cp1, kinematics[i].diagram.bodies[j]->pivot1->pos, link_radius);
				}
				else {
					pts = generateRoundedBarPolygon(cp2, kinematics[i].diagram.bodies[j]->pivot2->pos, link_radius);
				}

				// create a geometry to extend the body to the joint
				kinematics[i].diagram.addBody(kinematics[i].diagram.bodies[j]->pivot1, kinematics[i].diagram.bodies[j]->pivot2, kinematics::Polygon25D(pts, 0, link_depth));
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

















	// add the fixed rigid bodies to the fixed joints of all the linkages
	for (int i = 0; i < fixed_body_pts.size(); i++) {
		for (int j = 0; j < kinematics.size(); j++) {
			kinematics[j].diagram.addBody(kinematics[j].diagram.joints[0], kinematics[j].diagram.joints[1], fixed_body_pts[i]);
		}
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
				kinematics[i].stepForward(collision_check);
			}
			catch (char* ex) {
				kinematics[i].invertSpeed();
				std::cerr << "Animation is stopped by error:" << std::endl;
				std::cerr << ex << std::endl;
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
				kinematics[i].stepBackward(collision_check);
			}
			catch (char* ex) {
				kinematics[i].invertSpeed();
				std::cerr << "Animation is stopped by error:" << std::endl;
				std::cerr << ex << std::endl;
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
			kinematics[i].stepForward(collision_check);
		}
		catch (char* ex) {
			kinematics[i].invertSpeed();
			//stop();
			std::cerr << "Animation is stopped by error:" << std::endl;
			std::cerr << ex << std::endl;
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
		std::vector<Vertex> vertices;
		glutils::drawQuad(0.001, 0.001, glm::vec4(1, 1, 1, 1), glm::mat4(), vertices);
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
		painter.save();
		painter.setPen(QPen(QColor(224, 224, 224), 1));
		for (int i = -200; i <= 200; i++) {
			glm::dvec2 p = worldToScreenCoordinates(glm::dvec2(i * 5, i * 5));
			painter.drawLine(p.x, 0, p.x, height());
			glm::dvec2 p2 = worldToScreenCoordinates(glm::dvec2(0, i * 5));
			painter.drawLine(0, p.y, width(), p.y);
		}
		painter.restore();

		// draw 2D
		glm::vec2 offset = glm::vec2(camera.pos.x, -camera.pos.y) * (float)scale();
		if (mode != MODE_KINEMATICS) {
			// render unselected layers as background
			for (int l = 0; l < layers.size(); ++l) {
				if (l == layer_id) continue;
				for (int i = 0; i < layers[l].shapes.size(); ++i) {
					if (layers[l].shapes[i]->getSubType() == canvas::Shape::TYPE_BODY) {
						layers[l].shapes[i]->draw(painter, QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
					}
				}
			}

			// make the unselected layers faded
			painter.setPen(QColor(255, 255, 255, 160));
			painter.setBrush(QColor(255, 255, 255, 160));
			painter.drawRect(0, 0, width(), height());

			// render selected layer
			for (int i = 0; i < layers[layer_id].shapes.size(); i++) {
				layers[layer_id].shapes[i]->draw(painter, QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}

			// render currently drawing shape
			if (current_shape) {
				current_shape->draw(painter, QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}
		}
		else {
			// make the 3D faded
			painter.setPen(QColor(255, 255, 255, 160));
			painter.setBrush(QColor(255, 255, 255, 160));
			painter.drawRect(0, 0, width(), height());

			for (int i = 0; i < kinematics.size(); i++) {
				kinematics[i].draw(painter, QPointF(width() * 0.5 - offset.x, height() * 0.5 - offset.y), scale());
			}
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
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (glm::length(layers[layer_id].shapes[i]->getRotationMarkerPosition(scale()) - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale()) {
					// start rotating
					mode = MODE_ROTATION;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::RotateOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(layers[layer_id].shapes[i]->getCenter())));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}
			}

			// hit test for resize marker
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				canvas::BoundingBox bbox = layers[layer_id].shapes[i]->boundingBox();
				if (glm::length(bbox.minPt - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale()) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(bbox.maxPt)));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(glm::dvec2(bbox.maxPt.x, bbox.minPt.y) - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale()) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(glm::dvec2(bbox.minPt.x, bbox.maxPt.y))));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(glm::dvec2(bbox.minPt.x, bbox.maxPt.y) - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale()) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(glm::dvec2(bbox.maxPt.x, bbox.minPt.y))));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}

				if (glm::length(bbox.maxPt - layers[layer_id].shapes[i]->localCoordinate(screenToWorldCoordinates(e->x(), e->y()))) < 10 / scale()) {
					// start resizing
					mode = MODE_RESIZE;
					operation = boost::shared_ptr<canvas::Operation>(new canvas::ResizeOperation(screenToWorldCoordinates(e->x(), e->y()), layers[layer_id].shapes[i]->worldCoordinate(bbox.minPt)));
					selected_shape = layers[layer_id].shapes[i];
					if (!layers[layer_id].shapes[i]->isSelected()) {
						unselectAll();
						layers[layer_id].shapes[i]->select();
					}
					update();
					return;
				}
			}

			// hit test for the selected shapes first
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->isSelected()) {
					if (layers[layer_id].shapes[i]->hit(screenToWorldCoordinates(e->x(), e->y()))) {
						// reselecting the already selected shapes
						mode = MODE_MOVE;
						operation = boost::shared_ptr<canvas::Operation>(new canvas::MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
						update();
						return;
					}
				}
			}

			// hit test for the shape
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->getSubType() == canvas::Shape::TYPE_BODY) {
					if (layers[layer_id].shapes[i]->hit(screenToWorldCoordinates(e->x(), e->y()))) {
						// start moving
						mode = MODE_MOVE;
						operation = boost::shared_ptr<canvas::Operation>(new canvas::MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
						if (!layers[layer_id].shapes[i]->isSelected()) {
							if (!ctrlPressed) {
								// If CTRL is not pressed, then deselect all other shapes.
								unselectAll();
							}
							layers[layer_id].shapes[i]->select();
						}
						update();
						return;
					}
				}
			}

			// hit test for the linkage region
			for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
				if (layers[layer_id].shapes[i]->getSubType() == canvas::Shape::TYPE_LINKAGE_REGION) {
					if (layers[layer_id].shapes[i]->hit(screenToWorldCoordinates(e->x(), e->y()))) {
						// start moving
						mode = MODE_MOVE;
						operation = boost::shared_ptr<canvas::Operation>(new canvas::MoveOperation(screenToWorldCoordinates(e->x(), e->y())));
						if (!layers[layer_id].shapes[i]->isSelected()) {
							if (!ctrlPressed) {
								// If CTRL is not pressed, then deselect all other shapes.
								unselectAll();
							}
							layers[layer_id].shapes[i]->select();
						}
						update();
						return;
					}
				}
			}

			unselectAll();
		}
		else if (mode == MODE_RECTANGLE) {
			if (!current_shape) {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Rectangle(canvas::Shape::TYPE_BODY, screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_CIRCLE) {
			if (!current_shape) {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Circle(canvas::Shape::TYPE_BODY, screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_POLYGON) {
			if (current_shape) {
				current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
			}
			else {
				// start drawing a rectangle
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Polygon(canvas::Shape::TYPE_BODY, screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
			}
		}
		else if (mode == MODE_LINKAGE_REGION) {
			if (current_shape) {
				current_shape->addPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())));
			}
			else {
				// start drawing a polygon
				unselectAll();
				current_shape = boost::shared_ptr<canvas::Shape>(new canvas::Polygon(canvas::Shape::TYPE_LINKAGE_REGION, screenToWorldCoordinates(e->x(), e->y())));
				current_shape->startDrawing();
				setMouseTracking(true);
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
	if (mode == MODE_MOVE) {
		boost::shared_ptr<canvas::MoveOperation> op = boost::static_pointer_cast<canvas::MoveOperation>(operation);
		glm::dvec2 dir = screenToWorldCoordinates(e->x(), e->y()) - op->pivot;
		for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
			if (layers[layer_id].shapes[i]->isSelected()) {
				layers[layer_id].shapes[i]->translate(dir);

				if (layers[layer_id].shapes[i]->getSubType() == canvas::Shape::TYPE_BODY) {
					// update 3D geometry
					QString obj_name = QString("object_%1").arg(i);
					renderManager.removeObject(obj_name);
					renderManager.addObject(obj_name, "", layers[layer_id].shapes[i]->getVertices(), true);

					// update shadow map
					renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
				}
			}
		}
		op->pivot = screenToWorldCoordinates(e->x(), e->y());
		update();
	}
	else if (mode == MODE_ROTATION) {
		boost::shared_ptr<canvas::RotateOperation> op = boost::static_pointer_cast<canvas::RotateOperation>(operation);
		glm::dvec2 dir1 = op->pivot - op->rotation_center;
		glm::dvec2 dir2 = screenToWorldCoordinates(e->x(), e->y()) - op->rotation_center;
		double theta = atan2(dir2.y, dir2.x) - atan2(dir1.y, dir1.x);
		for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
			if (layers[layer_id].shapes[i]->isSelected()) {
				layers[layer_id].shapes[i]->rotate(theta);

				if (layers[layer_id].shapes[i]->getSubType() == canvas::Shape::TYPE_BODY) {
					// update 3D geometry
					QString obj_name = QString("object_%1").arg(i);
					renderManager.removeObject(obj_name);
					renderManager.addObject(obj_name, "", layers[layer_id].shapes[i]->getVertices(), true);

					// update shadow map
					renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
				}
			}
		}
		op->pivot = screenToWorldCoordinates(e->x(), e->y());
		update();
	}
	else if (mode == MODE_RESIZE) {
		boost::shared_ptr<canvas::ResizeOperation> op = boost::static_pointer_cast<canvas::ResizeOperation>(operation);
		glm::dvec2 resize_center = selected_shape->localCoordinate(op->resize_center);
		glm::dvec2 dir1 = selected_shape->localCoordinate(op->pivot) - resize_center;
		glm::dvec2 dir2 = selected_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())) - resize_center;
		glm::dvec2 resize_scale(dir2.x / dir1.x, dir2.y / dir1.y);
		for (int i = 0; i < layers[layer_id].shapes.size(); ++i) {
			if (layers[layer_id].shapes[i]->isSelected()) {
				// resize the shape for all the layers in order to make the size of the shape the same across the layers
				for (int l = 0; l < layers.size(); l++) {
					layers[l].shapes[i]->resize(resize_scale, resize_center);
				}

				if (layers[layer_id].shapes[i]->getSubType() == canvas::Shape::TYPE_BODY) {
					// update 3D geometry
					QString obj_name = QString("object_%1").arg(i);
					renderManager.removeObject(obj_name);
					renderManager.addObject(obj_name, "", layers[layer_id].shapes[i]->getVertices(), true);

					// update shadow map
					renderManager.updateShadowMap(this, light_dir, light_mvpMatrix);
				}
			}
		}
		op->pivot = screenToWorldCoordinates(e->x(), e->y());
		update();
	}
	else if (mode == MODE_RECTANGLE || mode == MODE_CIRCLE || mode == MODE_POLYGON || mode == MODE_LINKAGE_REGION) {
		if (current_shape) {
			current_shape->updateByNewPoint(current_shape->localCoordinate(screenToWorldCoordinates(e->x(), e->y())), shiftPressed);
		}
	}
	else if (e->buttons() & Qt::RightButton) {
		if (shiftPressed) {
			camera.move(e->x(), e->y());
		}
		else {
			camera.rotate(e->x(), e->y(), (ctrlPressed ? 0.1 : 1));
		}
	}

	update();
}

/**
* This event handler is called when the mouse release events occur.
*/
void GLWidget3D::mouseReleaseEvent(QMouseEvent *e) {
	if (mode == MODE_MOVE || mode == MODE_ROTATION || mode == MODE_RESIZE) {
		history.push(layers);
		mode = MODE_SELECT;
	}
	else if (e->button() == Qt::RightButton) {
		//if (abs(camera.xrot) < 20 && abs(camera.yrot) < 20) {
		camera.xrot = 0;
		camera.yrot = 0;
		camera.updateMVPMatrix();
		front_faced = true;
		/*
		}
		else {
		front_faced = false;
		}
		*/
	}

	update();
}

void GLWidget3D::mouseDoubleClickEvent(QMouseEvent* e) {
	if (mode == MODE_RECTANGLE || mode == MODE_CIRCLE || mode == MODE_POLYGON || mode == MODE_LINKAGE_REGION) {
		if (e->button() == Qt::LeftButton) {
			if (current_shape) {
				// The shape is created.
				current_shape->completeDrawing();
				for (int i = 0; i < layers.size(); i++) {
					layers[i].shapes.push_back(current_shape->clone());
				}

				// update 3D geometry
				update3DGeometry();

				layers[layer_id].shapes.back()->select();
				mode = MODE_SELECT;
				history.push(layers);
				current_shape.reset();
				operation.reset();
				mainWin->ui.actionSelect->setChecked(true);
			}
		}
	}

	setMouseTracking(false);

	update();
}

void GLWidget3D::wheelEvent(QWheelEvent* e) {
	camera.zoom(e->delta() * 0.1);
	update();
}
