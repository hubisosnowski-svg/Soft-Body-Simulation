#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS

#define VERTICES 14000
#define GROUND_VERTICES 100
#define GROUND_FACES 100

#define FSIZE 2048

#define SHADOWSIZE 7000

#define SQ(x) ((x) * (x))
#define X(a, i) a[i][0]
#define Y(a, i) a[i][1]
#define Z(a, i) a[i][2]

#include <GL/glew.h>
#include <GL/wglew.h>
#include <GL/freeglut.h>
#include <commdlg.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <array>
#include <time.h>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <algorithm>
#include <cctype>

#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glut.h"
#include "imgui/backends/imgui_impl_opengl3.h"

#include "Vectord.h"
#include "TetMesh.h"

using namespace std;
using namespace glm;

inline Vectord operator^(Vectord v, Vectord u) {
	return { v.y * u.z - v.z * u.y,
			 v.z * u.x - v.x * u.z,
			 v.x * u.y - v.y * u.x };
}

inline GLdouble operator*(Vectord v, Vectord u) {
	return v.x * u.x + v.y * u.y + v.z * u.z;
}

struct collisionConstraint {
	GLuint i;
	dvec3 p1, n;
	GLdouble ln, lt;

	collisionConstraint() : i(), p1(), n(), ln(), lt() {}
	collisionConstraint(GLuint I, dvec3 P1, dvec3 N, GLdouble LN, GLdouble LT) : i(I), p1(P1), n(N), ln(LN), lt(LT) {}
};

GLuint verticesNum = 0, edgesNum = 0, facesNum = 0, tetrahedraNum = 0, groundVerticesNum = 0, groundFacesNum = 0, rampVerticesNum = 0, rampFacesNum = 0, parapet1VerticesNum = 0, parapet1FacesNum = 0, parapet2VerticesNum = 0, parapet2FacesNum = 0;

vec3 cameraPos = vec3(-6.0f, 0.0f, 4.0f);
vec3 lightPos1 = vec3(-16.0f, 20.0f, 16.0f);
vec3 lightPos2 = vec3(-70.0f, 0.0f, 55.0f);

GLfloat ambientLight[] = { 0.5f, 0.5f, 0.5f };
GLfloat diffuseLight1[] = { 0.5f, 0.5f, 0.5f };
GLfloat diffuseLight2[] = { 0.1f, 0.1f, 0.1f };

GLfloat xRot = M_PI / 7, yRot, WIDTH, HEIGHT, mouseX = 0.0f, mouseY = 0.0f, a = 0.002, p = 1, n = 20, h = 2.0f, thickness = 2.0f, us = 0.9, uk = 0.6, ep = 0.000f, dts = 10.0f, winHeight = 625.0f;
GLdouble currentKey, dt, g = -9.81, dx[3], l, delay = 1, mass = 1;

GLint64 time1, time2;

static auto previousTime = std::chrono::steady_clock::now();

vector<array<GLdouble, 3>> vel, pos;

vector<array<GLuint, 4>> tetrahedrons;
vector<GLdouble> V06;
GLdouble V6;

vector<GLuint> collidingVertices;
vector<array<vec3, 3>> collidingTriangles;

vector<collisionConstraint> collisionConstraints;

GLdouble grabConstraints[3];
vector<array<GLdouble, 3>> vertices;
GLdouble groundVertices[GROUND_VERTICES][3], rampVertices[GROUND_VERTICES][3], parapet1Vertices[GROUND_VERTICES][3], parapet2Vertices[GROUND_VERTICES][3];
vector<array<GLfloat, 3>> normals;
GLfloat groundNormals[GROUND_VERTICES][3], rampNormals[GROUND_VERTICES][3], parapet1Normals[GROUND_VERTICES][3], parapet2Normals[GROUND_VERTICES][3];
vector<array<GLuint, 3>> indices;
GLuint groundIndices[GROUND_FACES][3], rampIndices[GROUND_FACES][3], parapet1Indices[GROUND_FACES][3], parapet2Indices[GROUND_FACES][3];
vector<vector<GLuint>> faces;
vector<vec3> faceNormals;

vector<vector<GLuint>> dupIndices;
vector<GLuint> groundDupIndices[GROUND_VERTICES / 3], rampDupIndices[GROUND_VERTICES / 3], parapet1DupIndices[GROUND_VERTICES / 3], parapet2DupIndices[GROUND_VERTICES / 3];

GLuint programLoc, depthProgramLoc, vertexBuffer, normalBuffer, elementBuffer, groundVertexBuffer, groundNormalBuffer, groundElementBuffer, rampVertexBuffer, rampNormalBuffer, rampElementBuffer, parapet1VertexBuffer, parapet1NormalBuffer, parapet1ElementBuffer, parapet2VertexBuffer, parapet2NormalBuffer, parapet2ElementBuffer, FramebufferName = 0, depthTexture, grabbedTriangle;

bool mousePressed, findTriangle, grabbed = 0, movement = 1, settings = 0, distanceConstraints = 1, volumeConstraints = 1, bendConstraints = 1, gravity = 1, drawObject = 1, drawPoints = 0, drawMesh = 0, drawGrabConstraints = 0, deltaTime = 0;
vec4 lRayStart_NDC, lRayEnd_NDC, lRayStart_camera, lRayStart_world, lRayEnd_camera, lRayEnd_world, camera2;
vec3 sRay, dRay, grabPoint, initialGrabPoint;

static float color[3] = { 76.0f / 256.0f, 76.0f / 256.0f, 76.0f / 256.0f };

vec3 objectColor1, objectColor;

mat4 ProjectionMatrix, ViewMatrix, ModelMatrix;

bool ToggleSwitch(const char* label, const char* text1, const char* text2, bool* value) {
	ImGui::PushID(label);

	bool changed = false;

	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 2.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 3));

	bool pushColor1 = !*value;
	if (pushColor1) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));

	if (ImGui::Button(text1) && *value) {
		*value = false;
		changed = true;
	}

	if (pushColor1) ImGui::PopStyleColor();

	ImGui::SameLine(0, 2);

	bool pushColor2 = *value;
	if (pushColor2) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));

	if (ImGui::Button(text2) && !*value) {
		*value = true;
		changed = true;
	}

	if (pushColor2)ImGui::PopStyleColor();

	ImGui::PopStyleVar(2);
	ImGui::PopID();

	return changed;
}

struct Constraint {
	GLuint i0, i1;
	GLdouble l0;

	Constraint() : i0(), i1(), l0() {}
	Constraint(GLuint I1, GLuint I2, GLdouble L0) : i0(I1), i1(I2), l0(L0) {}
	void apply() {
		l = sqrt((vertices[i0][0] - vertices[i1][0]) * (vertices[i0][0] - vertices[i1][0]) +
			(vertices[i0][1] - vertices[i1][1]) * (vertices[i0][1] - vertices[i1][1]) +
			(vertices[i0][2] - vertices[i1][2]) * (vertices[i0][2] - vertices[i1][2]));

		if (l < 1e-8) return;
		if (abs(l - l0) < ep) return;

		dx[0] = -(verticesNum / mass / (2 * verticesNum / mass + a / (dt * dt))) * (l - l0) * (vertices[i0][0] - vertices[i1][0]) / l;
		dx[1] = -(verticesNum / mass / (2 * verticesNum / mass + a / (dt * dt))) * (l - l0) * (vertices[i0][1] - vertices[i1][1]) / l;
		dx[2] = -(verticesNum / mass / (2 * verticesNum / mass + a / (dt * dt))) * (l - l0) * (vertices[i0][2] - vertices[i1][2]) / l;

		for (auto d : dupIndices[i0]) {
			vertices[d][0] += dx[0];
			vertices[d][1] += dx[1];
			vertices[d][2] += dx[2];
		}

		for (auto d : dupIndices[i1]) {
			vertices[d][0] -= dx[0];
			vertices[d][1] -= dx[1];
			vertices[d][2] -= dx[2];
		}
	}
};

vector<Constraint> constraints, bendingConstraints;

#pragma comment(lib, "comdlg32.lib")

struct ModelEntry { string label, path; bool external; };

vector<ModelEntry> modelEntries;
vector<ModelEntry> externalModels;
string currentModelPath, currentModelName, modelStatus;
bool modelStatusOk = true, browseRequested = false;
char externalPathBuffer[512] = "";

float uiQuality = 0.0f, uiMaxVolume = 0.0f, uiFitSize = 1.25f;
bool uiPreserveSurface = true, uiFit = false, uiCenter = false;

string FileNameOf(const string& path) {
	size_t slash = path.find_last_of("/\\");
	return slash == string::npos ? path : path.substr(slash + 1);
}

string LowerCase(string s) {
	for (auto& c : s) c = (char)tolower((unsigned char)c);
	return s;
}

string CleanPath(const string& s) {
	const char* junk = " \t\r\n\"";
	size_t b = s.find_first_not_of(junk);
	if (b == string::npos) return "";
	size_t e = s.find_last_not_of(junk);
	return s.substr(b, e - b + 1);
}

bool IsColliderFile(const string& name) {
	string n = LowerCase(name);
	return n == "ground.obj" || n == "ramp.obj" || n == "parapet1.obj" || n == "parapet2.obj" || n == "scene.obj";
}

void RefreshModelList() {
	modelEntries.clear();

	WIN32_FIND_DATAA fd;
	HANDLE find = FindFirstFileA(".\\Presets\\*.obj", &fd);
	if (find != INVALID_HANDLE_VALUE) {
		do {
			if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;

			string name = fd.cFileName;
			string lower = LowerCase(name);
			if (lower.size() < 4 || lower.compare(lower.size() - 4, 4, ".obj") != 0) continue;
			if (IsColliderFile(name)) continue;

			modelEntries.push_back({ name, "./Presets/" + name, false });
		} while (FindNextFileA(find, &fd));
		FindClose(find);
	}

	sort(modelEntries.begin(), modelEntries.end(), [](const ModelEntry& x, const ModelEntry& y) { return LowerCase(x.label) < LowerCase(y.label); });

	for (auto& e : externalModels) {
		bool known = false;
		for (auto& m : modelEntries) if (m.path == e.path) known = true;
		if (!known) modelEntries.push_back({ "[ext] " + e.label, e.path, true });
	}
}

TetGenOptions MakeTetOptions() {
	TetGenOptions o;
	o.preserveSurface = uiPreserveSurface;
	o.qualityBound = uiQuality <= 0.0f ? 0.0 : fmaxf(1.1f, uiQuality);
	o.maxTetVolume = uiMaxVolume;
	o.fitSize = uiFit ? uiFitSize : 0.0;
	o.centerAtOrigin = uiCenter;
	return o;
}

void InstallSoftBody(const TetMesh& mesh, const SoftBodyTopology& topology, const string& objPath) {
	verticesNum = (GLuint)mesh.vertices.size();
	facesNum = (GLuint)mesh.surface.size();
	tetrahedraNum = (GLuint)mesh.tets.size();

	vertices = mesh.vertices;
	for (GLuint i = 0; i < verticesNum; i++) vertices[i][1] += h;

	indices = mesh.surface;
	tetrahedrons = mesh.tets;

	vel.assign(verticesNum, { 0.0, 0.0, 0.0 });
	pos = vertices;
	normals.assign(verticesNum, { 0.f, 1.f, 0.f });
	faceNormals.assign(facesNum, vec3(0.f));

	dupIndices.assign(verticesNum, vector<GLuint>());
	for (GLuint i = 0; i < verticesNum; i++) dupIndices[i].push_back(i);

	faces.assign(verticesNum, vector<GLuint>());
	for (GLuint j = 0; j < facesNum; j++) for (int k = 0; k < 3; k++) faces[indices[j][k]].push_back(j);

	V06.assign(tetrahedraNum, 0.0);
	for (GLuint t = 0; t < tetrahedraNum; t++) {
		const auto& p0 = vertices[tetrahedrons[t][0]];
		const auto& p1 = vertices[tetrahedrons[t][1]];
		const auto& p2 = vertices[tetrahedrons[t][2]];
		const auto& p3 = vertices[tetrahedrons[t][3]];

		Vectord side1 = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
		Vectord side2 = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };
		Vectord side3 = { p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2] };

		V06[t] = abs((side1 ^ side2) * side3);
	}

	auto restLength = [&](GLuint i, GLuint j) {
		return sqrt(SQ(X(vertices, j) - X(vertices, i)) + SQ(Y(vertices, j) - Y(vertices, i)) + SQ(Z(vertices, j) - Z(vertices, i)));
	};

	constraints.clear();
	bendingConstraints.clear();
	for (auto& e : topology.distancePairs) constraints.push_back({ e[0], e[1], restLength(e[0], e[1]) });
	for (auto& e : topology.bendingPairs) bendingConstraints.push_back({ e[0], e[1], restLength(e[0], e[1]) });

	edgesNum = (GLuint)constraints.size();

	grabbed = 0;
	findTriangle = 0;
	collidingVertices.clear();
	collidingTriangles.clear();
	collisionConstraints.clear();

	currentModelPath = objPath;
	currentModelName = FileNameOf(objPath);

	size_t interior = 0;
	for (char onSurface : topology.onSurface) if (!onSurface) interior++;
}

bool LoadSoftBody(const char* objPath, const TetGenOptions& options) {
	TetMesh mesh;
	string error;

	if (!TetrahedralizeObj(objPath, options, mesh, error)) {
		fprintf(stderr, "Soft body \"%s\": %s\n", objPath, error.c_str());
		return 0;
	}

	SoftBodyTopology topology;
	BuildTopology(mesh, topology);

	InstallSoftBody(mesh, topology, objPath);
	return 1;
}

struct SoftBodyJob {
	string path, error;
	TetGenOptions options;
	bool external = false;
	bool ok = false;
	TetMesh mesh;
	SoftBodyTopology topology;
	atomic<bool> done{ false };
};

shared_ptr<SoftBodyJob> softBodyJob;

void RequestModel(const string& path, bool newExternal) {
	if (softBodyJob || path.empty()) return;

	if (newExternal) { uiFit = true; uiCenter = true; }

	auto job = make_shared<SoftBodyJob>();
	job->path = path;
	job->options = MakeTetOptions();
	job->external = newExternal;
	softBodyJob = job;

	try {
		thread([job]() {
			try {
				job->ok = TetrahedralizeObj(job->path.c_str(), job->options, job->mesh, job->error);
				if (job->ok) BuildTopology(job->mesh, job->topology);
			}
			catch (const exception& e) { job->ok = false; job->error = e.what(); }
			catch (...) { job->ok = false; job->error = "unexpected error"; }
			job->done = true;
		}).detach();
	}
	catch (...) {
		softBodyJob.reset();
		modelStatus = "could not start the background thread";
		modelStatusOk = false;
	}

	winHeight = 640.0f;
}

string BrowseForObj() {
	char file[MAX_PATH] = "";

	OPENFILENAMEA ofn;
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = GetForegroundWindow();
	ofn.lpstrFilter = "Wavefront OBJ (*.obj)\0*.obj\0All files (*.*)\0*.*\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFile = file;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = "Choose a closed (watertight) OBJ for the soft body";
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | OFN_EXPLORER | OFN_HIDEREADONLY;

	return GetOpenFileNameA(&ofn) ? string(file) : string();
}

void PollSoftBodyJob() {
	if (browseRequested && !softBodyJob) {
		browseRequested = false;
		string path = BrowseForObj();
		time2 = glutGet(GLUT_ELAPSED_TIME);
		if (!path.empty()) RequestModel(path, true);
	}

	if (!softBodyJob || !softBodyJob->done) return;

	shared_ptr<SoftBodyJob> job = softBodyJob;
	softBodyJob.reset();

	if (job->ok) {
		InstallSoftBody(job->mesh, job->topology, job->path);

		if (job->external) {
			bool known = false;
			for (auto& e : externalModels) if (e.path == job->path) known = true;
			if (!known) externalModels.push_back({ FileNameOf(job->path), job->path, true });
		}
		RefreshModelList();

		modelStatus = "Loaded " + currentModelName + ": " + to_string(verticesNum) + " vertices, " + to_string(tetrahedraNum) + " tetrahedra";
		modelStatusOk = true;
	}
	else {
		modelStatus = FileNameOf(job->path) + ": " + job->error;
		modelStatusOk = false;
	}

	time2 = glutGet(GLUT_ELAPSED_TIME);
}

void DrawModelPicker() {
	const bool busy = softBodyJob != nullptr;

	ImGui::TextUnformatted("Soft body model");

	ImGui::PushItemWidth(-1);
	if (ImGui::BeginCombo("##model", currentModelName.c_str())) {
		for (size_t i = 0; i < modelEntries.size(); i++) {
			const bool selected = modelEntries[i].path == currentModelPath;

			ImGui::PushID((int)i);
			if (ImGui::Selectable(modelEntries[i].label.c_str(), selected) && !selected && !busy) RequestModel(modelEntries[i].path, false);
			if (selected) ImGui::SetItemDefaultFocus();
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();

	if (ImGui::Button("Browse...") && !busy) browseRequested = true;
	ImGui::SameLine();
	if (ImGui::Button("Reload") && !busy) RequestModel(currentModelPath, false);

	if (busy) {
		static const char* dots[] = { "", ".", "..", "..." };
		ImGui::Text("Tetrahedralizing %s%s", FileNameOf(softBodyJob->path).c_str(), dots[(int)(ImGui::GetTime() * 3.0) % 4]);
	}
	else if (!modelStatus.empty()) {
		ImGui::PushStyleColor(ImGuiCol_Text, modelStatusOk ? ImVec4(0.6f, 1.0f, 0.6f, 1.0f) : ImVec4(1.0f, 0.5f, 0.5f, 1.0f));
		ImGui::TextWrapped("%s", modelStatus.c_str());
		ImGui::PopStyleColor();
	}
}


bool LoadObj(const char* path, GLdouble(*v)[3], GLfloat(*vn)[3], GLuint(*f)[3], vector<GLuint>* dupIndices, GLuint& facesNumber, GLuint& vertices_number) {
	FILE* model = fopen(path, "r");

	vector<vec3> vn_obj;
	vector<GLuint>* vn_of_v = new vector<GLuint>[VERTICES];
	vector<GLuint>* vn_of_v2 = new vector<GLuint>[VERTICES];
	vector<GLuint>::iterator it;

	GLfloat x = 0.f, y = 0.f, z = 0.f;
	uint32_t v_i = 0, vt_i = 0, vn_i = 0, f_i = 0, v1 = 0, vn1 = 0, v1_obj = 0, v2 = 0, vn2 = 0, v2_obj = 0, v3 = 0, vn3 = 0, v3_obj = 0, t = 0;

	for (int i = 1; !feof(model); i++) {
		char c = fgetc(model);

		if (c == 'v') {
			c = fgetc(model);

			if (c == ' ') {
				if (fscanf(model, "%f %f %f", &x, &y, &z) != 3) printf("error reading vertex line\n");
				v[v_i][0] = x;
				v[v_i][1] = y;
				v[v_i][2] = z;

				dupIndices[v_i].push_back(v_i);
				v_i++;
				vertices_number++;
			}

			else if (c == 'n') {
				if (fscanf(model, "%f %f %f", &x, &y, &z) != 3) printf("error reading vertex line\n");

				vn_obj.push_back({ x, y, z });
			}

			else fscanf(model, "%*[^\n]");
		}

		else if (c == 'f') {
			if (getc(model) != ' ') {
				fscanf(model, "%*[^\n]");
				continue;
			}

			if (fscanf(model, "%u", &v1_obj) != 1) printf("error 1 reading face line %u\n", i);

			if (fgetc(model) == '/') {
				if (fscanf(model, "%u", &t) != 1) { if (fscanf(model, "/%u %u//%u %u//%u", &vn1, &v2_obj, &vn2, &v3_obj, &vn3) != 5) printf("error 2 reading face line %i\n", i); }

				else if (fgetc(model) == '/') { if (fscanf(model, "%u %u/%u/%u %u/%u/%u", &vn1, &v2_obj, &t, &vn2, &v3_obj, &t, &vn3) != 7) printf("error 3 reading face line %i\n", i); }
				
				else { if (fscanf(model, "%u/%*u %u/%*u", &v2_obj, &v3_obj) != 2) printf("error 4 reading face line %i\n", i); vn1 = vn2 = vn3 = 0; }
			}

			else { if (fscanf(model, "%u %u", &v2_obj, &v3_obj) != 2) printf("error 5 reading face line %i\n", 1); vn1 = vn2 = vn3 = 0; }

			v1 = v1_obj;
			v2 = v2_obj;
			v3 = v3_obj;

			if (vn1 && vn2 && vn3) {
				if (vn_of_v[v1_obj - 1].size()) {
					it = find(vn_of_v[v1_obj - 1].begin(), vn_of_v[v1_obj - 1].end(), vn1);

					if (it == vn_of_v[v1_obj - 1].end()) {
						v[v_i][0] = v[v1_obj - 1][0];
						v[v_i][1] = v[v1_obj - 1][1];
						v[v_i][2] = v[v1_obj - 1][2];

						v1 = v_i + 1;
						v_i++;
						dupIndices[v1_obj - 1].push_back(v1 - 1);

						vn_of_v[v1_obj - 1].push_back(vn1);
						vn_of_v2[v1_obj - 1].push_back(v1);
					}

					else v1 = *(vn_of_v2[v1_obj - 1].begin() + (it - vn_of_v[v1_obj - 1].begin()));
				}

				else {
					vn_of_v[v1_obj - 1].push_back(vn1);
					vn_of_v2[v1_obj - 1].push_back(v1);
				}

				vn[v1 - 1][0] = vn_obj[vn1 - 1].x;
				vn[v1 - 1][1] = vn_obj[vn1 - 1].y;
				vn[v1 - 1][2] = vn_obj[vn1 - 1].z;

				if (vn_of_v[v2_obj - 1].size()) {
					it = find(vn_of_v[v2_obj - 1].begin(), vn_of_v[v2_obj - 1].end(), vn2);

					if (it == vn_of_v[v2_obj - 1].end()) {
						v[v_i][0] = v[v2_obj - 1][0];
						v[v_i][1] = v[v2_obj - 1][1];
						v[v_i][2] = v[v2_obj - 1][2];

						v2 = v_i + 1;
						v_i++;
						dupIndices[v2_obj - 1].push_back(v2 - 1);

						vn_of_v[v2_obj - 1].push_back(vn2);
						vn_of_v2[v2_obj - 1].push_back(v2);
					}

					else v2 = *(vn_of_v2[v2_obj - 1].begin() + (it - vn_of_v[v2_obj - 1].begin()));
				}

				else { vn_of_v[v2_obj - 1].push_back(vn2); vn_of_v2[v2_obj - 1].push_back(v2); }

				vn[v2 - 1][0] = vn_obj[vn2 - 1].x;
				vn[v2 - 1][1] = vn_obj[vn2 - 1].y;
				vn[v2 - 1][2] = vn_obj[vn2 - 1].z;

				if (vn_of_v[v3_obj - 1].size()) {
					it = find(vn_of_v[v3_obj - 1].begin(), vn_of_v[v3_obj - 1].end(), vn3);

					if (it == vn_of_v[v3_obj - 1].end()) {
						v[v_i][0] = v[v3 - 1][0];
						v[v_i][1] = v[v3 - 1][1];
						v[v_i][2] = v[v3 - 1][2];

						v3 = v_i + 1;
						v_i++;
						dupIndices[v3_obj - 1].push_back(v3 - 1);

						vn_of_v[v3_obj - 1].push_back(vn3);
						vn_of_v2[v3_obj - 1].push_back(v3);
					}

					else v3 = *(vn_of_v2[v3_obj - 1].begin() + (it - vn_of_v[v3_obj - 1].begin()));
				}

				else {
					vn_of_v[v3_obj - 1].push_back(vn3);
					vn_of_v2[v3_obj - 1].push_back(v3);
				}

				vn[v3 - 1][0] = vn_obj[vn3 - 1].x;
				vn[v3 - 1][1] = vn_obj[vn3 - 1].y;
				vn[v3 - 1][2] = vn_obj[vn3 - 1].z;
			}

			f[f_i][0] = v1 - 1;
			f[f_i][1] = v2 - 1;
			f[f_i][2] = v3 - 1;

			f_i++;
		}

		else if (c != '\n') fscanf(model, "%*[^\n]");
	}

	fclose(model);

	facesNumber = f_i;

	vertices_number = facesNumber * 3;

	return 1;
}


GLuint LoadShader(string vertPath, string fragPath) {
	GLuint newVertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint newFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	char shaderText[FSIZE];

	FILE* file = fopen(vertPath.c_str(), "r");
	shaderText[fread(shaderText, 1, FSIZE, file)] = 0;
	fclose(file);

	GLcharARB* stringPointers[1];
	stringPointers[0] = shaderText;
	glShaderSource(newVertexShader, 1, (const GLcharARB**)stringPointers, NULL);


	file = fopen(fragPath.c_str(), "r");
	shaderText[fread(shaderText, 1, FSIZE, file)] = 0;
	fclose(file);

	stringPointers[0] = shaderText;
	glShaderSource(newFragmentShader, 1, (const GLcharARB**)stringPointers, NULL);

	glCompileShader(newVertexShader);
	glCompileShader(newFragmentShader);

	GLint success;
	glGetObjectParameterivARB(newVertexShader, GL_OBJECT_COMPILE_STATUS_ARB, &success);
	if (!success) {
		GLcharARB infoLog[2137];
		glGetInfoLogARB(newVertexShader, 2137, NULL, infoLog);
		fprintf(stderr, "Error in vertex shader compilation!\n");
		fprintf(stderr, "Info log: %s\n", infoLog);
		Sleep(10000);
		exit(0);
	}

	GLuint newProgram = glCreateProgramObjectARB();
	glAttachShader(newProgram, newVertexShader);
	glAttachShader(newProgram, newFragmentShader);

	glLinkProgram(newProgram);
	glGetObjectParameterivARB(newProgram, GL_OBJECT_LINK_STATUS_ARB, &success);
	if (!success) {
		GLcharARB infoLog[2137];
		glGetInfoLogARB(newProgram, 2137, NULL, infoLog);
		fprintf(stderr, "Error in program linkage!\n");
		fprintf(stderr, "Info log: %s\n", infoLog);
		Sleep(10000);
		exit(0);
	}

	glValidateProgramARB(newProgram);
	glGetObjectParameterivARB(newProgram, GL_OBJECT_VALIDATE_STATUS_ARB, &success);
	if (!success) {
		GLcharARB infoLog[2137];
		glGetInfoLogARB(newProgram, 2137, NULL, infoLog);
		fprintf(stderr, "Error in program linkage!\n");
		fprintf(stderr, "Info log: %s\n", infoLog);
		Sleep(10000);
		exit(0);
	}

	glDetachShader(newProgram, newVertexShader);
	glDetachShader(newProgram, newFragmentShader);

	glDeleteShader(newVertexShader);
	glDeleteShader(newFragmentShader);

	return newProgram;
}

void FindCollisions(GLdouble(*objectVertices)[3], GLfloat(*objectNormals)[3], GLuint(*objectIndices)[3], GLuint objectFacesNum) {
	bool collision;
	GLuint collisionTriangle;
	GLdouble depth, minDepth = -1.;
	dvec3 p1, p2, p3, normal, p, pn;

	for (int i = 0; i < verticesNum; i++) {
		collision = 1, minDepth = -1;
		p = dvec3(vertices[i][0], vertices[i][1], vertices[i][2]);

		for (int j = 0; j < objectFacesNum; j++) {
			p1 = dvec3(objectVertices[objectIndices[j][0]][0], objectVertices[objectIndices[j][0]][1], objectVertices[objectIndices[j][0]][2]);
			p2 = dvec3(objectVertices[objectIndices[j][1]][0], objectVertices[objectIndices[j][1]][1], objectVertices[objectIndices[j][1]][2]);
			p3 = dvec3(objectVertices[objectIndices[j][2]][0], objectVertices[objectIndices[j][2]][1], objectVertices[objectIndices[j][2]][2]);

			normal = dvec3(objectNormals[objectIndices[j][0]][0], objectNormals[objectIndices[j][0]][1], objectNormals[objectIndices[j][0]][2]);
			normal = normalize(normal);

			depth = -dot(p - p1, normal);
			if (depth <= 0.) { collision = 0; break; }

			if (minDepth <= 0.) {
				minDepth = depth;
				collisionTriangle = j;
			}

			else if (depth < minDepth) {
				pn = p + normal * depth;
				if (dot(normal, cross(p2 - p1, pn - p1)) > 0 && dot(normal, cross(p3 - p2, pn - p2)) > 0 && dot(normal, cross(p1 - p3, pn - p3)) > 0 || dot(normal, cross(p2 - p1, pn - p1)) < 0 && dot(normal, cross(p3 - p2, pn - p2)) < 0 && dot(normal, cross(p1 - p3, pn - p3)) < 0) {
					minDepth = depth;
					collisionTriangle = j;
				}
			}
		}

		if (collision) {
			p1 = dvec3(objectVertices[objectIndices[collisionTriangle][0]][0], objectVertices[objectIndices[collisionTriangle][0]][1], objectVertices[objectIndices[collisionTriangle][0]][2]);

			GLdouble ln = (minDepth * mass) / verticesNum, lt = 0.0;

			normal = dvec3(objectNormals[objectIndices[collisionTriangle][0]][0], objectNormals[objectIndices[collisionTriangle][0]][1], objectNormals[objectIndices[collisionTriangle][0]][2]);
			normal = normalize(normal);

			collisionConstraints.push_back({(GLuint)i, p1, normal, ln, lt});

			collidingVertices.push_back(i);
			collidingTriangles.push_back({ p1, p2, p3 });
		}
	}
}

void RecomputeNormals() {
	if (findTriangle) grabbed = 0;

	grabPoint = vec3(0.f, -1.f, 0.f);
	vec3 a, b, c, p, v, u, normal;

	for (int i = 0; i < facesNum; i++) {
		a = vec3((GLfloat)vertices[indices[i][0]][0],
			(GLfloat)vertices[indices[i][0]][1],
			(GLfloat)vertices[indices[i][0]][2]);
		b = vec3((GLfloat)vertices[indices[i][1]][0],
			(GLfloat)vertices[indices[i][1]][1],
			(GLfloat)vertices[indices[i][1]][2]);
		c = vec3((GLfloat)vertices[indices[i][2]][0],
			(GLfloat)vertices[indices[i][2]][1],
			(GLfloat)vertices[indices[i][2]][2]);

		normal = cross(b - a, c - a);

		faceNormals[i] = normal;

		p = sRay + dot(normal, a - sRay) / dot(normal, dRay) * dRay;

		if ((dot(normal, cross(b - a, p - a)) > 0 && dot(normal, cross(c - b, p - b)) > 0 && dot(normal, cross(a - c, p - c)) > 0 || dot(normal, cross(b - a, p - a)) < 0 && dot(normal, cross(c - b, p - b)) < 0 && dot(normal, cross(a - c, p - c)) < 0) && (!grabbed || length(p - sRay) < length(grabPoint - sRay))) {
			grabPoint = p;
			if (findTriangle) { grabbedTriangle = i; grabbed = 1; }
		}
	}

	for (int i = 0; i < verticesNum; i++) {
		normal = vec3(0.f);
		for (auto faceIndex : faces[i]) normal += faceNormals[faceIndex];

		normal = length(normal) > 1e-12f ? normalize(normal) : vec3(0.f, 1.f, 0.f);

		normals[i][0] = normal.x;
		normals[i][1] = normal.y;
		normals[i][2] = normal.z;
	}

	if (findTriangle) {
		findTriangle = 0;
		initialGrabPoint = grabPoint;

		a = vec3((GLfloat)vertices[indices[grabbedTriangle][0]][0],
			(GLfloat)vertices[indices[grabbedTriangle][0]][1],
			(GLfloat)vertices[indices[grabbedTriangle][0]][2]);
		b = vec3((GLfloat)vertices[indices[grabbedTriangle][1]][0],
			(GLfloat)vertices[indices[grabbedTriangle][1]][1],
			(GLfloat)vertices[indices[grabbedTriangle][1]][2]);
		c = vec3((GLfloat)vertices[indices[grabbedTriangle][2]][0],
			(GLfloat)vertices[indices[grabbedTriangle][2]][1],
			(GLfloat)vertices[indices[grabbedTriangle][2]][2]);

		grabConstraints[0] = length(a - grabPoint);
		grabConstraints[1] = length(b - grabPoint);
		grabConstraints[2] = length(c - grabPoint);
	}

	else grabPoint = sRay + dot(vec3(camera2), initialGrabPoint - sRay) / dot(vec3(camera2), dRay) * dRay;
}

void RecomputeRay() {
	lRayStart_NDC = vec4(2.f * (GLfloat)mouseX / (GLfloat)WIDTH, 2.f * (GLfloat)mouseY / (GLfloat)HEIGHT, -1.f, 1.f);
	lRayEnd_NDC = vec4(2.f * (GLfloat)mouseX / (GLfloat)WIDTH, 2.f * (GLfloat)mouseY / (GLfloat)HEIGHT, 0.f, 1.f);

	mat4 InverseProjectionMatrix = inverse(ProjectionMatrix);
	mat4 InverseViewMatrix = inverse(ViewMatrix);

	lRayStart_camera = InverseProjectionMatrix * lRayStart_NDC;       lRayStart_camera /= lRayStart_camera.w;
	lRayStart_world = InverseViewMatrix * lRayStart_camera;    lRayStart_world /= lRayStart_world.w;
	lRayEnd_camera = InverseProjectionMatrix * lRayEnd_NDC;         lRayEnd_camera /= lRayEnd_camera.w;
	lRayEnd_world = InverseViewMatrix * lRayEnd_camera;      lRayEnd_world /= lRayEnd_world.w;

	sRay = vec3(lRayStart_world);
	dRay = vec3(lRayStart_world - lRayEnd_world);
}

void TimerFunc(int value) {
	PollSoftBodyJob();

	collidingVertices = {};
	collidingTriangles = {};
	collisionConstraints = {};

	time1 = time2;
	time2 = glutGet(GLUT_ELAPSED_TIME);

	if (deltaTime == 0) dt = dts / (1000.0f * n);
	else dt = (time2 - time1) / (1000.0f * n);

	if (movement) {
		int temp = 0;

		for (int s = 0; s < n; s++) {
			for (int i = 0; i < verticesNum; i++) {
				if (gravity) vel[i][1] += g * dt;

				pos[i][0] = vertices[i][0];
				pos[i][1] = vertices[i][1];
				pos[i][2] = vertices[i][2];

				for (auto d : dupIndices[i]) {
					vertices[d][0] += vel[d][0] * dt;
					vertices[d][1] += vel[d][1] * dt;
					vertices[d][2] += vel[d][2] * dt;
				}
			}

			if (mousePressed && grabbed && !findTriangle) {
				for (int i = 0; i < 3; i++) {
					GLdouble l = sqrt((vertices[indices[grabbedTriangle][i]][0] - grabPoint.x) * (vertices[indices[grabbedTriangle][i]][0] - grabPoint.x) +
						(vertices[indices[grabbedTriangle][i]][1] - grabPoint.y) * (vertices[indices[grabbedTriangle][i]][1] - grabPoint.y) +
						(vertices[indices[grabbedTriangle][i]][2] - grabPoint.z) * (vertices[indices[grabbedTriangle][i]][2] - grabPoint.z));

					if (l < 1e-8) continue;

					GLdouble m = (verticesNum / mass / (2 * verticesNum / mass + 0.0f / (dt * dt))) * (l - grabConstraints[i]) / l;
					GLdouble dx[] = { m * (vertices[indices[grabbedTriangle][i]][0] - grabPoint.x),
								   m * (vertices[indices[grabbedTriangle][i]][1] - grabPoint.y),
								   m * (vertices[indices[grabbedTriangle][i]][2] - grabPoint.z) };

					for (auto d : dupIndices[indices[grabbedTriangle][i]]) {
						vertices[d][0] -= dx[0];
						vertices[d][1] -= dx[1];
						vertices[d][2] -= dx[2];
					}
				}
			}

			if (distanceConstraints) {
				for (size_t c = 0; c < constraints.size(); c++) {
					constraints[c].apply();
				}
			}

			if (bendConstraints) {
				for (size_t c = 0; c < bendingConstraints.size(); c++) {
					bendingConstraints[c].apply();
				}
			}

			if (volumeConstraints) {
				for (int v = 0; v < tetrahedraNum; v++) {
					Vectord side1_2 = { vertices[tetrahedrons[v][1]][0] - vertices[tetrahedrons[v][0]][0], vertices[tetrahedrons[v][1]][1] - vertices[tetrahedrons[v][0]][1], vertices[tetrahedrons[v][1]][2] - vertices[tetrahedrons[v][0]][2] }, side1_3 = { vertices[tetrahedrons[v][2]][0] - vertices[tetrahedrons[v][0]][0], vertices[tetrahedrons[v][2]][1] - vertices[tetrahedrons[v][0]][1], vertices[tetrahedrons[v][2]][2] - vertices[tetrahedrons[v][0]][2] }, side1_4 = { vertices[tetrahedrons[v][3]][0] - vertices[tetrahedrons[v][0]][0], vertices[tetrahedrons[v][3]][1] - vertices[tetrahedrons[v][0]][1], vertices[tetrahedrons[v][3]][2] - vertices[tetrahedrons[v][0]][2] }, side2_3 = { vertices[tetrahedrons[v][2]][0] - vertices[tetrahedrons[v][1]][0], vertices[tetrahedrons[v][2]][1] - vertices[tetrahedrons[v][1]][1], vertices[tetrahedrons[v][2]][2] - vertices[tetrahedrons[v][1]][2] }, side2_4 = { vertices[tetrahedrons[v][3]][0] - vertices[tetrahedrons[v][1]][0], vertices[tetrahedrons[v][3]][1] - vertices[tetrahedrons[v][1]][1], vertices[tetrahedrons[v][3]][2] - vertices[tetrahedrons[v][1]][2] };

					V6 = ((side1_2 ^ side1_3) * side1_4);
					if (V6 < 0.0) V6 = 0.0;

					Vectord NC1 = side2_4 ^ side2_3;
					Vectord NC2 = side1_3 ^ side1_4;
					Vectord NC3 = side1_4 ^ side1_2;
					Vectord NC4 = side1_2 ^ side1_3;

					GLdouble NC1_m = NC1.magnitude();
					GLdouble NC2_m = NC2.magnitude();
					GLdouble NC3_m = NC3.magnitude();
					GLdouble NC4_m = NC4.magnitude();
					GLdouble sumGradSquared = NC1_m * NC1_m + NC2_m * NC2_m + NC3_m * NC3_m + NC4_m * NC4_m;
					if (sumGradSquared < 1e-30) continue;
					GLdouble l = -(V6 - V06[v] * p) / sumGradSquared;

					NC1 *= l; NC2 *= l; NC3 *= l; NC4 *= l;

					for (auto d : dupIndices[tetrahedrons[v][0]]) {
						vertices[d][0] += NC1.x;
						vertices[d][1] += NC1.y;
						vertices[d][2] += NC1.z;
					}

					for (auto d : dupIndices[tetrahedrons[v][1]]) {
						vertices[d][0] += NC2.x;
						vertices[d][1] += NC2.y;
						vertices[d][2] += NC2.z;
					}

					for (auto d : dupIndices[tetrahedrons[v][2]]) {
						vertices[d][0] += NC3.x;
						vertices[d][1] += NC3.y;
						vertices[d][2] += NC3.z;
					}

					for (auto d : dupIndices[tetrahedrons[v][3]]) {
						vertices[d][0] += NC4.x;
						vertices[d][1] += NC4.y;
						vertices[d][2] += NC4.z;
					}
				}
			}

			if (s % 5 == 0) {
				FindCollisions(groundVertices, groundNormals, groundIndices, groundFacesNum);
				FindCollisions(rampVertices, rampNormals, rampIndices, rampFacesNum);
				FindCollisions(parapet1Vertices, parapet1Normals, parapet1Indices, parapet1FacesNum);
				FindCollisions(parapet2Vertices, parapet2Normals, parapet2Indices, parapet2FacesNum);
			}

			for (int i = 0; i < verticesNum; i++) {
				vel[i][0] = (vertices[i][0] - pos[i][0]) / dt;
				vel[i][1] = (vertices[i][1] - pos[i][1]) / dt;
				vel[i][2] = (vertices[i][2] - pos[i][2]) / dt;
			}

			dvec3 p, dx, v, dv = dvec3(0.), pp;
			GLdouble dotv, Cj;

			for (auto& C : collisionConstraints) {
				p = dvec3(vertices[C.i][0], vertices[C.i][1], vertices[C.i][2]);
				pp = dvec3(pos[C.i][0], pos[C.i][1], pos[C.i][2]);

				Cj = dot(p - C.p1, C.n);
				v = dvec3(vel[C.i][0], vel[C.i][1], vel[C.i][2]);
				dotv = dot(v, C.n);

				if (dotv < 0.0f) {
					dx = -C.n * dot(p - C.p1, C.n);
					dv = -C.n * dotv;

					dvec3 deltaP = p - pp;
					dvec3 deltaPt = deltaP - dot(deltaP, C.n) * C.n;
					GLdouble deltalt = (length(deltaPt) * mass) / verticesNum;
					GLdouble deltaln = (-Cj * mass) / verticesNum;

					dvec3 vn = dot(v, C.n) * C.n;
					dvec3 vt = v - vn;

					C.ln += deltaln;
					C.lt += deltalt;

					if (C.lt < us * C.ln) dx -= deltaPt;

					dv += -std::min(uk * abs(C.ln) / dt * (verticesNum / mass), length(vt)) * normalize(vt);

					if (isnan(length(dv))) dv = dvec3(0);

					vertices[C.i][0] += dx.x;
					vertices[C.i][1] += dx.y;
					vertices[C.i][2] += dx.z;

					vel[C.i][0] += dv.x;
					vel[C.i][1] += dv.y;
					vel[C.i][2] += dv.z;
				}
			}
		}
	}

	RecomputeNormals();

	glutTimerFunc(delay, TimerFunc, 0);
}

void MouseWheelFunc(GLint key, GLint dir, GLint x, GLint y) {
	GLfloat cameraPosLength = sqrt(cameraPos[0] * cameraPos[0] + cameraPos[1] * cameraPos[1] + cameraPos[2] * cameraPos[2]);

	if (dir > 0) {
		cameraPos[0] *= 1 - 1 / cameraPosLength;
		cameraPos[1] *= 1 - 1 / cameraPosLength;
		cameraPos[2] *= 1 - 1 / cameraPosLength;
	}
	else {
		cameraPos[0] *= 1 + 1 / cameraPosLength;
		cameraPos[1] *= 1 + 1 / cameraPosLength;
		cameraPos[2] *= 1 + 1 / cameraPosLength;
	}
}

void MouseFunc(GLint key, GLint state, GLint x, GLint y) {
	ImGui_ImplGLUT_MouseFunc(key, state, x, y);

	if (ImGui::GetIO().WantCaptureMouse) return;

	currentKey = key;

	if (key == GLUT_LEFT_BUTTON) {
		if (state == GLUT_UP) mousePressed = 0;

		else if (state == GLUT_DOWN) {
			mousePressed = 1;
			findTriangle = 1;
			RecomputeRay();
		}
	}

	else mousePressed = 0;
}

void PassiveMotionFunc(GLint x, GLint y) {
	ImGui_ImplGLUT_MotionFunc(x, y);

	if (ImGui::GetIO().WantCaptureMouse) return;

	mouseX = x - WIDTH / 2;
	mouseY = -y + HEIGHT / 2;
}

void MotionFunc(GLint x, GLint y) {
	ImGui_ImplGLUT_MotionFunc(x, y);

	if (ImGui::GetIO().WantCaptureMouse) return;

	if (currentKey == GLUT_RIGHT_BUTTON) {
		yRot -= 1.5f * (acos(2 * (x - WIDTH / 2) / WIDTH) - acos(2 * mouseX / WIDTH));
		xRot += 1.5f * (acos(2 * (-y + HEIGHT / 2) / HEIGHT) - acos(2 * mouseY / HEIGHT));

		if (xRot < -M_PI / 2.f) xRot = -M_PI / 2.f - 1e-4;
		if (xRot > M_PI / 2.f) xRot = M_PI / 2.f - 1e-4;
	}

	mouseX = x - WIDTH / 2.0f;
	mouseY = -y + HEIGHT / 2.0f;

	if (currentKey == GLUT_LEFT_BUTTON) RecomputeRay();
}

void KeyboardFunc(unsigned char key, int x, int y) {
	ImGui_ImplGLUT_KeyboardFunc(key, x, y);

	if (ImGui::GetIO().WantCaptureKeyboard) return;

	if (key == 27) exit(0);

	else if (key == 9) {
		if (settings) settings = 0;
		else settings = 1;
	}

	else if (key == 32) {
		if (movement) movement = 0;
		else movement = 1;
	}

	else if (key == 'r') {
		for (auto& v : vel) { v[0] = 0.; v[1] = 0.; v[2] = 0.; }

		GLdouble dx[] = { vertices[0][0],vertices[0][1] - h,vertices[0][2] };

		for (int j = 0; j < verticesNum; j++) { vertices[j][0] -= dx[0]; vertices[j][1] -= dx[1]; vertices[j][2] -= dx[2]; }
	}
}

void DrawScene(bool ifNormals) {
	GLuint objectColorLoc = glGetUniformLocation(programLoc, "objectColor");

	objectColor1 = vec3(color[0], color[1], color[2]);

	glUniform3f(objectColorLoc, objectColor1.x, objectColor1.y, objectColor1.z);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*)0);

	if (ifNormals) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);

	if (drawObject) { glDrawElements(GL_TRIANGLES, facesNum * 3, GL_UNSIGNED_INT, (void*)0); }

	glDisable(GL_CULL_FACE);

	if (drawPoints) {
		objectColor = vec3(0.0f / 256.0f, 256.0f / 256.0f, 0.0f / 256.0f);
		glUniform3f(objectColorLoc, objectColor.x * 2, objectColor.y * 2, objectColor.z * 2);

		glDisableVertexAttribArray(1);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDrawArrays(GL_POINTS, 0, verticesNum);

		glEnableVertexAttribArray(1);
	}

	glDisableVertexAttribArray(0);
	if (ifNormals) glDisableVertexAttribArray(1);

	if (drawMesh) {
		objectColor = vec3(0.0f / 256.0f, 256.0f / 256.0f, 0.0f / 256.0f);
		glUniform3f(objectColorLoc, objectColor.x * 2, objectColor.y * 2, objectColor.z * 2);

		for (int i = 0; i < tetrahedraNum; i++) {
			glLineWidth(thickness / length(vec3(camera2) - vec3(vertices[tetrahedrons[i][0]][0], vertices[tetrahedrons[i][0]][1], vertices[tetrahedrons[i][0]][2])));
			glBegin(GL_LINES);
				glVertex3f(vertices[tetrahedrons[i][0]][0], vertices[tetrahedrons[i][0]][1], vertices[tetrahedrons[i][0]][2]);
				glVertex3f(vertices[tetrahedrons[i][1]][0], vertices[tetrahedrons[i][1]][1], vertices[tetrahedrons[i][1]][2]);
			glEnd();

			glLineWidth(thickness / length(vec3(camera2) - vec3(vertices[tetrahedrons[i][0]][0], vertices[tetrahedrons[i][0]][1], vertices[tetrahedrons[i][0]][2])));
			glBegin(GL_LINES);
				glVertex3f(vertices[tetrahedrons[i][0]][0], vertices[tetrahedrons[i][0]][1], vertices[tetrahedrons[i][0]][2]);
				glVertex3f(vertices[tetrahedrons[i][2]][0], vertices[tetrahedrons[i][2]][1], vertices[tetrahedrons[i][2]][2]);
			glEnd();

			glLineWidth(thickness / length(vec3(camera2) - vec3(vertices[tetrahedrons[i][0]][0], vertices[tetrahedrons[i][0]][1], vertices[tetrahedrons[i][0]][2])));
			glBegin(GL_LINES);
				glVertex3f(vertices[tetrahedrons[i][0]][0], vertices[tetrahedrons[i][0]][1], vertices[tetrahedrons[i][0]][2]);
				glVertex3f(vertices[tetrahedrons[i][3]][0], vertices[tetrahedrons[i][3]][1], vertices[tetrahedrons[i][3]][2]);
			glEnd();

			glLineWidth(thickness / length(vec3(camera2) - vec3(vertices[tetrahedrons[i][1]][0], vertices[tetrahedrons[i][1]][1], vertices[tetrahedrons[i][1]][2])));
			glBegin(GL_LINES);
				glVertex3f(vertices[tetrahedrons[i][1]][0], vertices[tetrahedrons[i][1]][1], vertices[tetrahedrons[i][1]][2]);
				glVertex3f(vertices[tetrahedrons[i][2]][0], vertices[tetrahedrons[i][2]][1], vertices[tetrahedrons[i][2]][2]);
			glEnd();

			glLineWidth(thickness / length(vec3(camera2) - vec3(vertices[tetrahedrons[i][1]][0], vertices[tetrahedrons[i][1]][1], vertices[tetrahedrons[i][1]][2])));
			glBegin(GL_LINES);
				glVertex3f(vertices[tetrahedrons[i][1]][0], vertices[tetrahedrons[i][1]][1], vertices[tetrahedrons[i][1]][2]);
				glVertex3f(vertices[tetrahedrons[i][3]][0], vertices[tetrahedrons[i][3]][1], vertices[tetrahedrons[i][3]][2]);
			glEnd();

			glLineWidth(thickness / length(vec3(camera2) - vec3(vertices[tetrahedrons[i][2]][0], vertices[tetrahedrons[i][2]][1], vertices[tetrahedrons[i][2]][2])));
			glBegin(GL_LINES);
				glVertex3f(vertices[tetrahedrons[i][2]][0], vertices[tetrahedrons[i][2]][1], vertices[tetrahedrons[i][2]][2]);
				glVertex3f(vertices[tetrahedrons[i][3]][0], vertices[tetrahedrons[i][3]][1], vertices[tetrahedrons[i][3]][2]);
			glEnd();
		}
	}

	if (drawGrabConstraints && mousePressed && grabbed) {
		objectColor = vec3(0.0f / 256.0f, 256.0f / 256.0f, 0.0f / 256.0f);
		glUniform3f(objectColorLoc, objectColor.x * 2, objectColor.y * 2, objectColor.z * 2);

		glBegin(GL_LINES);
			glVertex3f(grabPoint.x, grabPoint.y, grabPoint.z);
			glVertex3f(vertices[indices[grabbedTriangle][0]][0], vertices[indices[grabbedTriangle][0]][1], vertices[indices[grabbedTriangle][0]][2]);

			glVertex3f(grabPoint.x, grabPoint.y, grabPoint.z);
			glVertex3f(vertices[indices[grabbedTriangle][1]][0], vertices[indices[grabbedTriangle][1]][1], vertices[indices[grabbedTriangle][1]][2]);

			glVertex3f(grabPoint.x, grabPoint.y, grabPoint.z);
			glVertex3f(vertices[indices[grabbedTriangle][2]][0], vertices[indices[grabbedTriangle][2]][1], vertices[indices[grabbedTriangle][2]][2]);
		glEnd();
	}

	glEnable(GL_CULL_FACE);

	objectColor = vec3(51.0f / 256.0f, 54.0f / 256.0f, 67.0f / 256.0f);
	glUniform3f(objectColorLoc, objectColor.x, objectColor.y, objectColor.z);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, groundVertexBuffer);
	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*)0);

	if (ifNormals) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, groundNormalBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, groundElementBuffer);

	glDrawElements(GL_TRIANGLES, groundFacesNum * 3, GL_UNSIGNED_INT, (void*)0);

	glDisableVertexAttribArray(0);
	if (ifNormals) glDisableVertexAttribArray(1);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, rampVertexBuffer);
	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*)0);

	if (ifNormals) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, rampNormalBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rampElementBuffer);

	glDrawElements(GL_TRIANGLES, rampFacesNum * 3, GL_UNSIGNED_INT, (void*)0);

	glDisableVertexAttribArray(0);
	if (ifNormals) glDisableVertexAttribArray(1);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, parapet1VertexBuffer);
	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*)0);

	if (ifNormals) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, parapet1NormalBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, parapet1ElementBuffer);

	glDrawElements(GL_TRIANGLES, parapet1FacesNum * 3, GL_UNSIGNED_INT, (void*)0);

	glDisableVertexAttribArray(0);
	if (ifNormals) glDisableVertexAttribArray(1);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, parapet2VertexBuffer);
	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(GLdouble), (void*)0);

	if (ifNormals) {
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, parapet2NormalBuffer);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, parapet2ElementBuffer);

	glDrawElements(GL_TRIANGLES, parapet2FacesNum * 3, GL_UNSIGNED_INT, (void*)0);

	glDisableVertexAttribArray(0);
	if (ifNormals) glDisableVertexAttribArray(1);
}

void RenderScene(void) {
	ImGuiIO& io = ImGui::GetIO();
	ImGuiStyle& style = ImGui::GetStyle();
	io.DisplaySize = ImVec2((float)WIDTH, (float)HEIGHT);

	io.Fonts->AddFontFromFileTTF("OpenSans-Bold.ttf", 14.0f);
	ImGui::StyleColorsClassic();

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGLUT_NewFrame();
	ImGui::NewFrame();

	style.WindowRounding = 3.0f;
	style.FrameRounding = 1.5f;

	ImGui::SetNextWindowPos(ImVec2(WIDTH - 300 - 3, 3), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(300, fminf(HEIGHT - 6.0f, winHeight)), ImGuiCond_Always);

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.18f, 0.18f, 0.18f, 0.83f));

	if (settings) {
		ImGui::Begin("Soft Body Details", nullptr, ImGuiWindowFlags_NoResize);
		DrawModelPicker();
		ImGui::Text(("Point Count: " + to_string(verticesNum)).c_str());
		ImGui::Text(("Edge Count: " + to_string(edgesNum)).c_str());
		ImGui::Text(("Tetrahedron Count: " + to_string(tetrahedraNum)).c_str());
		ImGui::Checkbox("Distance Constraints", &distanceConstraints);
		ImGui::Text("Inverse Stiffness");
		ImGui::SliderFloat(" ", &a, 0.0f, 0.1f, "%.003f", 0);
		ImGui::Checkbox("Volume Constraints", &volumeConstraints);
		ImGui::Text("Pressure Multiplier");
		ImGui::SliderFloat("  ", &p, 0.0f, 10.0f, "%0.1f", 0);
		ImGui::Checkbox("Bending Constraints", &bendConstraints);
		ImGui::Text("Friction");
		ImGui::SliderFloat("μₛ", &us, 0.0f, 10.0f, "%0.1f", 0);
		ImGui::SliderFloat("μₖ", &uk, 0.0f, 10.0f, "%0.1f", 0);
		ImGui::ColorPicker3("Color changer", color);
		ImGui::End();
	}
	else {
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.f, 0.f, 0.f, 0.f));
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

		ImGui::SetNextWindowPos(ImVec2(3, 3), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(WIDTH - 6, 54), ImGuiCond_Always);
		ImGui::Begin("Window1", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);
		ImGui::Text("[Tab] Open/Close Options");
		ImGui::Text("[Esc] Close Simulation");
		ImGui::End();

		ImGui::PopStyleColor();
		ImGui::PopStyleColor();
		ImGui::PopStyleVar();
	}

	ImGui::SetNextWindowPos(ImVec2(3, 3), ImGuiCond_Once);

	if (deltaTime == 0) ImGui::SetNextWindowSize(ImVec2(300, 395), ImGuiCond_Always);
	else ImGui::SetNextWindowSize(ImVec2(300, 370), ImGuiCond_Always);

	if (settings) {
		ImGui::Begin("Debug", nullptr, ImGuiWindowFlags_NoResize);
		ImGui::Text("[R] Reset Simulation");
		ImGui::Text("[Space] Pause/Unpause Simulation");
		ImGui::Text("[Tab] Open/Close Simulation Details");
		ImGui::Text("[Esc] Close Simulation");
		ImGui::Text("Frame Delta Time: %.2f ms", ImGui::GetIO().DeltaTime * 1000);
		ImGui::Text("FPS: %.2f", ImGui::GetIO().Framerate);
		ToggleSwitch("Delta Time", "Constant dt", "Real Time", &deltaTime);

		if (deltaTime == 0) ImGui::SliderFloat("ms", &dts, 1.0f, 100.0f, "%1.0f", 0);

		ImGui::Text("Iteration Count");
		ImGui::SliderFloat("  ", &n, 0.0f, 40.0f, "%1.0f", 0);
		ImGui::Text("Epsilon");
		ImGui::SliderFloat("   ", &ep, 0.0f, 0.1f, "%.3f", 0);
		ImGui::Checkbox("Gravity", &gravity);
		ImGui::Checkbox("Draw Object", &drawObject);
		ImGui::Checkbox("Draw Points", &drawPoints);
		ImGui::Checkbox("Draw Mesh", &drawMesh);
		ImGui::Checkbox("Draw Grab Constraints", &drawGrabConstraints);
		ImGui::End();
	}

	ImGui::PopStyleColor();

	glUniform3f(glGetUniformLocation(programLoc, "ambientLight"), ambientLight[0], ambientLight[1], ambientLight[2]);
	glUniform3f(glGetUniformLocation(programLoc, "diffuseLight0"), diffuseLight1[0], diffuseLight1[1], diffuseLight1[2]);
	glUniform3f(glGetUniformLocation(programLoc, "diffuseLight1"), diffuseLight2[0], diffuseLight2[1], diffuseLight2[2]);
	glUniform1i(glGetUniformLocation(programLoc, "time"), time1);
	
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, verticesNum * sizeof(vertices[0]), &vertices[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
	glBufferData(GL_ARRAY_BUFFER, verticesNum *sizeof(normals[0]), &normals[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, facesNum * sizeof(indices[0]), &indices[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, groundVertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, groundVerticesNum * sizeof(groundVertices[0]), &groundVertices[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, groundNormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, groundVerticesNum * sizeof(groundNormals[0]), &groundNormals[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, groundElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, groundFacesNum * sizeof(groundIndices[0]), &groundIndices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, rampVertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, rampVerticesNum * sizeof(rampVertices[0]), &rampVertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, rampNormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, rampVerticesNum * sizeof(rampNormals[0]), &rampNormals[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rampElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, rampFacesNum * sizeof(rampIndices[0]), &rampIndices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, parapet1VertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, parapet1VerticesNum * sizeof(parapet1Vertices[0]), &parapet1Vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, parapet1NormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, parapet1VerticesNum * sizeof(parapet1Normals[0]), &parapet1Normals[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, parapet1ElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, parapet1FacesNum * sizeof(parapet1Indices[0]), &parapet1Indices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, parapet2VertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, parapet2VerticesNum * sizeof(parapet2Vertices[0]), &parapet2Vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, parapet2NormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, parapet2VerticesNum * sizeof(parapet2Normals[0]), &parapet2Normals[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, parapet2ElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, parapet2FacesNum * sizeof(parapet2Indices[0]), &parapet2Indices[0], GL_STATIC_DRAW);

	GLuint depthMVPLoc = glGetUniformLocation(depthProgramLoc, "depthMVP");
	GLuint depthBiasMVPLoc = glGetUniformLocation(programLoc, "depthBiasMVP");
	GLuint MVPLoc = glGetUniformLocation(programLoc, "MVP");
	GLuint MLoc = glGetUniformLocation(programLoc, "M");
	GLuint VLoc = glGetUniformLocation(programLoc, "P");
	GLuint lightPosLoc = glGetUniformLocation(programLoc, "lightPos");
	GLuint shadowMapLoc = glGetUniformLocation(programLoc, "shadowMap");

	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);

	glDrawBuffer(GL_NONE);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) exit(2);

	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
	glViewport(0, 0, SHADOWSIZE, SHADOWSIZE);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(depthProgramLoc);

	vec3 lightInvDir = lightPos1;

	mat4 depthProjectionMatrix = ortho<float>(-100, 100, -100, 100, 0.1, 200);
	mat4 depthModelViewMatrix = lookAt(lightInvDir, vec3(0, 0, 0), vec3(0, 1, 0));
	mat4 depthMVP = depthProjectionMatrix * depthModelViewMatrix;

	glUniformMatrix4fv(depthMVPLoc, 1, GL_FALSE, &depthMVP[0][0]);

	DrawScene(0);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glViewport(0, 0, WIDTH, HEIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(programLoc);

	camera2 = vec4(cameraPos, 1.f) * rotate(rotate(mat4(1.f), xRot, vec3(cameraPos.z, 0.f, -cameraPos.x)), yRot, vec3(0.f, 1.f, 0.f));
    camera2 /= camera2.w;

    ProjectionMatrix = perspective<float>(45.f, WIDTH / HEIGHT, 1.f, 1000.f);
    ViewMatrix = lookAt(vec3(camera2), vec3(0.f), vec3(0.f, 1.f, 0.f));
    ModelMatrix = mat4(1.0f);

	mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
	mat4 biasMatrix(0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.5, 1.0);
	mat4 depthBiasMVP = biasMatrix * depthMVP;

	glUniformMatrix4fv(MVPLoc, 1, GL_FALSE, &MVP[0][0]);
	glUniformMatrix4fv(MLoc, 1, GL_FALSE, &ModelMatrix[0][0]);
	glUniformMatrix4fv(VLoc, 1, GL_FALSE, &ViewMatrix[0][0]);
	glUniformMatrix4fv(depthBiasMVPLoc, 1, GL_FALSE, &depthBiasMVP[0][0]);

	glUniform3f(lightPosLoc, lightPos1.x, lightPos1.y, lightPos1.z);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glUniform1i(shadowMapLoc, 0);

	DrawScene(1);

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glutSwapBuffers();
	glutPostRedisplay();
}

void SetupRC() {
	glShadeModel(GL_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glGenBuffers(1, &vertexBuffer);
	glGenBuffers(1, &normalBuffer);
	glGenBuffers(1, &elementBuffer);
	glGenBuffers(1, &groundVertexBuffer);
	glGenBuffers(1, &groundNormalBuffer);
	glGenBuffers(1, &groundElementBuffer);
	glGenBuffers(1, &rampVertexBuffer);
	glGenBuffers(1, &rampNormalBuffer);
	glGenBuffers(1, &rampElementBuffer);
	glGenBuffers(1, &parapet1VertexBuffer);
	glGenBuffers(1, &parapet1NormalBuffer);
	glGenBuffers(1, &parapet1ElementBuffer);
	glGenBuffers(1, &parapet2VertexBuffer);
	glGenBuffers(1, &parapet2NormalBuffer);
	glGenBuffers(1, &parapet2ElementBuffer);
	glGenFramebuffers(1, &FramebufferName);
	glGenTextures(1, &depthTexture);

	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, SHADOWSIZE, SHADOWSIZE, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

	glClearColor(0.82f, 0.87f, 0.99f, 1.0f);
}

void ChangeSize(GLint w, GLint h) {
	GLfloat fAspect;

	WIDTH = w;
	HEIGHT = h;

	if (h == 0) h = 1;

	glViewport(0, 0, w, h);

	fAspect = (GLfloat)w / (GLfloat)h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60.0f, fAspect, 1.0, 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

int main(int argc, char** argv) {
	RefreshModelList();
	if (!LoadSoftBody("./Presets/tire.obj", MakeTetOptions())) { Sleep(10000); return 1; }

	LoadObj("./Scene Files/ground.obj", groundVertices, groundNormals, groundIndices, groundDupIndices, groundFacesNum, groundVerticesNum);
	LoadObj("./Scene Files/ramp.obj", rampVertices, rampNormals, rampIndices, rampDupIndices, rampFacesNum, rampVerticesNum);
	LoadObj("./Scene Files/parapet1.obj", parapet1Vertices, parapet1Normals, parapet1Indices, parapet1DupIndices, parapet1FacesNum, parapet1VerticesNum);
	LoadObj("./Scene Files/parapet2.obj", parapet2Vertices, parapet2Normals, parapet2Indices, parapet2DupIndices, parapet2FacesNum, parapet2VerticesNum);

	glutInit(&argc, argv);

	glutSetOption(GLUT_MULTISAMPLE, 16);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutCreateWindow("Soft Rigid Body Simulation");

	glewInit();

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io;

	ImGui::StyleColorsDark();

	ImGui_ImplGLUT_Init();
	ImGui_ImplGLUT_InstallFuncs();
	ImGui_ImplOpenGL3_Init("#version 330");

	wglSwapIntervalEXT(0);

	static bool firstIteration = 1;

	if (firstIteration) { firstIteration = 0; programLoc = LoadShader("./main.vert", "./main.frag"); depthProgramLoc = LoadShader("./depth.vert", "./depth.frag"); }

	glutDisplayFunc(RenderScene);
	glutReshapeFunc(ChangeSize);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
	glutPassiveMotionFunc(PassiveMotionFunc);
	glutMouseWheelFunc(MouseWheelFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutTimerFunc(delay, TimerFunc, 0);
	glutFullScreen();

	SetupRC();

	glutMainLoop();
}