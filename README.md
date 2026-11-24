# Tetrahedral Soft-Body Physics Simulation

An interactive 3D soft-body physics simulation written in C++ using OpenGL, FreeGLUT, GLM, and Dear ImGui. The application loads closed (watertight) Wavefront `.obj` meshes, tetrahedralizes them in a background thread, and simulates deformable soft-body dynamics using Extended Position-Based Dynamics (XPBD) constraints alongside real-time collision detection and mouse-ray grabbing.

---

## Core Features

* **XPBD Soft-Body Solver:** Simulates deformable meshes across $n$ substeps per frame using:
  * **Distance Constraints:** Preserves edge lengths with configurable compliance ($a$).
  * **Bending Constraints:** Maintains angular/structural rigidity across adjacent faces.
  * **Volume Constraints:** Preserves the rest volume ($V_0$) of internal tetrahedra to prevent mesh collapse or inversion.
* **Asynchronous Tetrahedralization:** Converts surface `.obj` files (from a `./Presets/` directory or via a Windows file browser dialog) into volumetric tetrahedral meshes on a detached worker thread without freezing the UI.
* **Environment Collision & Friction:** Detects vertex penetration against static scene colliders (`ground`, `ramp`, `parapet1`, and `parapet2`) and resolves normal and tangential contact velocities.
* **Interactive Ray-Cast Grabbing:** Casts rays from screen-space mouse coordinates into world space to intersect surface triangles and apply spring-like pulling constraints to the soft body.
* **Dear ImGui Control Panel:** Provides live toggles for physics constraints, gravity, rendering modes (mesh, points, grab constraints), and model selection.

---

## Dependencies & Libraries

* **OpenGL / GLEW / WGLEW:** Core graphics rendering, VBOs, framebuffers, and shadow/depth mapping.
* **FreeGLUT:** Window management, input callbacks, and simulation timer loop.
* **GLM:** Vector and matrix mathematics (`vec3`, `vec4`, `dvec3`, `mat4`).
* **Dear ImGui:** Immediate-mode GUI (`imgui_impl_glut`, `imgui_impl_opengl3`).
* **Windows API (`commdlg.h`, `FindFirstFileA`):** Native file dialog for importing external `.obj` models and scanning preset directories.
* **Custom Headers:** `Vectord.h` (double-precision 3D vector math) and `TetMesh.h` (tetrahedralization and topology builder).

---

## Function & Method Reference

### Physics & Simulation Loop
* **`TimerFunc(int value)`**
  The primary simulation step executed periodically by FreeGLUT. Polls background tetrahedralization jobs, calculates substep $\Delta t$, integrates gravity and velocities, iteratively solves grab, distance, bending, and tetrahedral volume constraints, checks collisions against scene geometry every 5 substeps, and resolves contact impulses.
* **`Constraint::apply()`**
  Member method of the `Constraint` struct that enforces a rest-length ($l_0$) distance constraint between two vertices (`i0`, `i1`) using XPBD compliance ($a$) and updates any duplicated vertex indices (`dupIndices`).
* **`FindCollisions(GLdouble(*objectVertices)[3], GLfloat(*objectNormals)[3], GLuint(*objectIndices)[3], GLuint objectFacesNum)`**
  Tests all soft-body vertices against the faces of a static collider mesh. Computes signed penetration depth along face normals, verifies point-in-triangle projection, and populates `collisionConstraints`, `collidingVertices`, and `collidingTriangles`.

### Geometry, Normals & Ray Casting
* **`RecomputeNormals()`**
  Recalculates per-face normals and smooth area-weighted per-vertex normals for the deformed soft body. Simultaneously performs ray-triangle intersection testing to locate the closest surface triangle under the cursor (`grabbedTriangle`) and updates `grabPoint` and `grabConstraints`.
* **`RecomputeRay()`**
  Unprojects 2D screen-space mouse coordinates (`mouseX`, `mouseY`) through the inverse `ProjectionMatrix` and `ViewMatrix` to compute the world-space ray origin (`sRay`) and direction (`dRay`).

### Model Loading & Tetrahedralization
* **`InstallSoftBody(const TetMesh& mesh, const SoftBodyTopology& topology, const string& objPath)`**
  Initializes simulation state buffers (`vertices`, `vel`, `pos`, `normals`, `tetrahedrons`), offsets the mesh vertically by spawn height $h$, computes initial tetrahedral rest volumes (`V06`), and generates `constraints` and `bendingConstraints` from the topology.
* **`LoadSoftBody(const char* objPath, const TetGenOptions& options)`**
  Synchronously tetrahedralizes an `.obj` file via `TetrahedralizeObj`, builds its simulation topology via `BuildTopology`, and installs it into the scene.
* **`RequestModel(const string& path, bool newExternal)`**
  Spawns a detached `std::thread` (`SoftBodyJob`) to tetrahedralize a model asynchronously without blocking the render loop.
* **`PollSoftBodyJob()`**
  Checks if a file browse dialog was requested or if a background `SoftBodyJob` has finished. Installs the newly generated mesh on the main thread and updates UI status messages.
* **`LoadObj(const char* path, GLdouble(*v)[3], GLfloat(*vn)[3], GLuint(*f)[3], vector<GLuint>* dupIndices, GLuint& facesNumber, GLuint& vertices_number)`**
  Parses static Wavefront `.obj` files (such as the ground, ramp, and parapets), handling `v`, `vn`, and `f` definitions and splitting vertices that share positions with differing normals via `dupIndices`.
* **`MakeTetOptions()`**
  Packages current ImGui tetrahedralization parameters (`uiPreserveSurface`, `uiQuality`, `uiMaxVolume`, `uiFitSize`, `uiCenter`) into a `TetGenOptions` struct.

### File & UI Helpers
* **`RefreshModelList()`**
  Scans `./Presets/*.obj` using Win32 directory search functions, filters out static collider files, sorts presets alphabetically, and appends any user-loaded external models.
* **`BrowseForObj()`**
  Opens a native Windows `GetOpenFileNameA` dialog filtered for `.obj` files and returns the selected file path.
* **`DrawModelPicker()`**
  Renders the ImGui combo box for switching preset/external models, the **Browse...** and **Reload** buttons, and real-time status/progress indicators.
* **`ToggleSwitch(const char* label, const char* text1, const char* text2, bool* value)`**
  Custom ImGui widget that renders a two-state segmented toggle button and returns `true` when the boolean state changes.
* **`IsColliderFile(const string& name)`**
  Returns `true` if a filename matches reserved scene colliders (`ground.obj`, `ramp.obj`, `parapet1.obj`, `parapet2.obj`, or `scene.obj`).
* **`FileNameOf(const string& path)`**
  Extracts the base filename from a full or relative file path.
* **`LowerCase(string s)`**
  Converts all characters in a string to lowercase for case-insensitive comparisons and sorting.
* **`CleanPath(const string& s)`**
  Trims leading and trailing whitespace, newlines, and quotation marks from a path string.

### Shaders & Math Operators
* **`LoadShader(string vertPath, string fragPath)`**
  Reads vertex and fragment shader source files from disk, compiles and links them into an OpenGL shader program, logs compilation/linking errors if any occur, and returns the program handle.
* **`operator^(Vectord v, Vectord u)`**
  Inline operator overload computing the 3D cross product of two `Vectord` instances.
* **`operator*(Vectord v, Vectord u)`**
  Inline operator overload computing the scalar dot product of two `Vectord` instances.

---

> **Note:** The provided source snippet truncates near the end of `TimerFunc`, omitting the final collision friction resolution and OpenGL rendering/GLUT initialization callbacks (`Display`, `Reshape`, `main`, etc.). If you share the remainder of the file, those functions can be appended to this reference.
