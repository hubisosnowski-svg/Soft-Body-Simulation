# Tetrahedral Soft-Body Physics Simulation

An interactive 3D soft-body physics simulation written in C++ using OpenGL, FreeGLUT, GLM, and Dear ImGui. The application loads closed Wavefront `.obj` meshes, tetrahedralizes them in a background thread, and simulates deformable soft-body dynamics using Extended Position-Based Dynamics (XPBD) constraints alongside real-time collision detection and mouse-ray grabbing. This project was inspired on https://github.com/blackedout01 soft-rigid body simulation.

## Core Features

* **XPBD Soft-Body Solver:** Simulates deformable meshes across $n$ substeps per frame using:

  * **Distance Constraints:** Preserves edge lengths with configurable compliance ($a$).

  * **Bending Constraints:** Maintains angular/structural rigidity across adjacent faces.

  * **Volume Constraints:** Preserves the rest volume ($V_0$) of internal tetrahedra to prevent mesh collapse or inversion.

* **Asynchronous Tetrahedralization:** Converts surface `.obj` files (from a `./Presets/` directory or via a Windows file browser dialog) into volumetric tetrahedral meshes on a detached worker thread without freezing the UI.

* **Environment Collision & Friction:** Detects vertex penetration against static scene colliders (`ground`, `ramp`, `parapet1`, and `parapet2`) and resolves normal and tangential contact velocities.

* **Interactive Ray-Cast Grabbing:** Casts rays from screen-space mouse coordinates into world space to intersect surface triangles and apply spring-like pulling constraints to the soft body.

* **Dear ImGui Control Panel:** Provides live controls for model loading, tetrahedralization quality, XPBD physics parameters, constraint toggles, and debug visualization modes.

## Dear ImGui Controls & Parameters Reference

Based on the state variables and UI helpers in the source code, the Dear ImGui panel exposes controls across four functional categories:

### 1. Soft Body Model & Tetrahedralization (`DrawModelPicker` & `MakeTetOptions`)

* **Soft body model (Dropdown):** Selects and loads any `.obj` mesh found in `./Presets/` or previously imported external models. Selecting a model triggers asynchronous tetrahedralization on a background thread.

* **Browse... (Button):** Opens a native Windows file dialog (`BrowseForObj`) to import an external closed (watertight) `.obj` mesh. Automatically enables **Fit** and **Center** options on first load so arbitrary models scale cleanly into the scene.

* **Reload (Button):** Re-runs tetrahedralization on the currently active `.obj` file using the latest tetrahedralization settings below.

* **Preserve Surface (`uiPreserveSurface`):** Keeps the original input triangle surface mesh intact during tetrahedralization rather than allowing TetGen to remesh or split boundary triangles.

* **Quality Bound (`uiQuality`):** Sets the minimum radius-edge ratio quality bound for generated tetrahedra (clamped to $\ge 1.1$ when enabled, or disabled when $\le 0$). Lower values produce better-shaped, less sliver-like tetrahedra at the cost of higher element counts.

* **Max Volume (`uiMaxVolume`):** Caps the maximum allowable volume of any single generated tetrahedron, subdividing large interior elements for finer deformation resolution.

* **Fit (`uiFit`) & Fit Size (`uiFitSize`):** Uniformly scales the imported model so its largest dimension matches `uiFitSize` (default `1.25` units).

* **Center (`uiCenter`):** Translates the imported mesh so its bounding center sits at the local origin before applying the vertical spawn offset $h$.

### 2. Physics & Solver Parameters

* **Simulation / Movement (`movement`):** Plays or pauses the physics integration loop while keeping rendering and camera/UI interaction active.

* **Gravity (`gravity`, $g$):** Toggles downward gravitational acceleration ($g = -9.81\text{ m/s}^2$) on all soft-body vertices.

* **Substeps ($n$):** Sets the number of XPBD solver iterations performed per frame (default `20`). Higher values increase stiffness and stability.

* **Time Step Mode (`deltaTime`) & Fixed Step (`dts`):** Switches between a fixed frame time step (`dts` in milliseconds, default `10.0f`) and real elapsed wall-clock frame delta (`time2 - time1`).

* **Compliance ($a$):** Controls inverse stiffness in the XPBD distance and bending constraint solver (default `0.002`). Higher compliance makes the soft body stretchier and softer; `0` approaches rigid behavior.

* **Volume Preservation Multiplier ($p$):** Scales the target rest volume ($V_0 \cdot p$, default `1.0`) enforced by tetrahedral volume constraints—values $> 1$ inflate the object, while values $< 1$ deflate it.

* **Mass (`mass`):** Total mass of the soft body distributed across its vertices, affecting inertia, constraint weighting, and collision impulses.

* **Spawn Height ($h$):** Vertical Y-axis offset applied when spawning or reloading a soft body into the scene (default `2.0f`).

* **Static & Kinetic Friction ($\mu_s$, $\mu_k$):** Controls surface grip (`us = 0.9`) and sliding resistance (`uk = 0.6`) when vertices collide with static scene geometry.

### 3. Constraint Toggles

* **Distance Constraints (`distanceConstraints`):** Enables or disables edge-length preservation constraints between connected mesh vertices.

* **Bending Constraints (`bendConstraints`):** Enables or disables secondary cross-edge constraints that resist folding and creasing.

* **Volume Constraints (`volumeConstraints`):** Enables or disables per-tetrahedron volume preservation to prevent the soft body from collapsing flat.

### 4. Rendering & Visualization Options

* **Draw Object (`drawObject`):** Toggles rendering of the shaded, smooth-lit soft-body surface mesh.

* **Draw Mesh / Wireframe (`drawMesh`):** Toggles wireframe/edge overlay rendering of the soft-body topology.

* **Draw Points (`drawPoints`):** Toggles point-cloud visualization of individual simulation vertices.

* **Draw Grab Constraints (`drawGrabConstraints`):** Toggles debug visualization of the active mouse-ray grab point and spring attachment lines when dragging the mesh.

* **Object Color (`color`):** RGB color picker controlling the diffuse material color of the soft body.

* **Settings (`settings`):** Toggles visibility/expansion of the advanced settings panel.

## Dependencies & Libraries

* **OpenGL / GLEW / WGLEW:** Core graphics rendering, VBOs, framebuffers, and shadow/depth mapping.

* **FreeGLUT:** Window management, input callbacks, and simulation timer loop.

* **GLM:** Vector and matrix mathematics (`vec3`, `vec4`, `dvec3`, `mat4`).

* **Dear ImGui:** Immediate-mode GUI (`imgui_impl_glut`, `imgui_impl_opengl3`).

* **Windows API (`commdlg.h`, `FindFirstFileA`):** Native file dialog for importing external `.obj` models and scanning preset directories.

* **Custom Headers:** `Vectord.h` (double-precision 3D vector math) and `TetMesh.h` (tetrahedralization and topology builder).

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

* **`operator^(Vectord v, Vectord u)`**
  Inline operator overload computing the 3D cross product of two `Vectord` instances.

* **`operator*(Vectord v, Vectord u)`**
  Inline operator overload computing the scalar dot product of two `Vectord` instances.
