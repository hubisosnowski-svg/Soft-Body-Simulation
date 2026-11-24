// TetMesh.h
//
// OBJ surface mesh  ->  tetrahedral mesh (TetGen)  ->  topology used by the soft body solver.
//
// This header intentionally does NOT include tetgen.h. TetGen defines macros (REAL, PI, ...) and
// a pile of global names; they stay inside TetMesh.cpp so they can't clash with glm / imgui / main.cpp.
#pragma once

#include <array>
#include <string>
#include <vector>

struct TetGenOptions {
	// ---- TetGen switches -------------------------------------------------------------------
	// 'q' : max radius-edge ratio of tetrahedra (smaller = better shaped tets, more points).
	//       0 = no quality refinement: a plain constrained Delaunay tetrahedralization with no
	//       interior points, which is what your old .node/.ele files looked like.
	//       Sensible values when you want it: 1.2 .. 2.0
	double qualityBound = 0.0;

	// 'a' : maximum tetrahedron volume, in FINAL scene units^3 (fitSize/placement is applied first).
	//       0 = unlimited. If set while qualityBound == 0, q2.0 (TetGen's default) is used.
	double maxTetVolume = 0.0;

	// 'Y' : never add Steiner points on the surface, so the rendered surface is exactly your OBJ.
	//       If TetGen fails with it, the loader retries once without it.
	bool preserveSurface = true;

	// false = 'Q' (quiet). true = let TetGen print its progress.
	bool verbose = false;

	// ---- placement (applied to the OBJ before TetGen runs) ---------------------------------
	// > 0 : uniformly scale so the largest bounding box side equals this value. 0 = keep OBJ scale.
	double fitSize = 0.0;
	// true : translate the bounding box centre to (0,0,0). false = keep the OBJ position.
	bool centerAtOrigin = false;

	// Vertices closer than weldTolerance * (bbox diagonal) are merged (fixes OBJs exported with
	// per-face vertices, e.g. converted from STL). 0 = merge exact duplicates only.
	double weldTolerance = 1e-6;
};

struct TetMesh {
	std::vector<std::array<double, 3>>   vertices;
	// Tetrahedra, always positively oriented:  (v1-v0) x (v2-v0) . (v3-v0) > 0
	std::vector<std::array<unsigned, 4>> tets;
	// Boundary triangles of the tet mesh, wound counter-clockwise seen from OUTSIDE.
	std::vector<std::array<unsigned, 3>> surface;
};

// Reads an OBJ (positions + faces; normals/uvs/materials are ignored, quads and n-gons are
// triangulated), runs TetGen and returns the tet mesh. The OBJ must be ONE closed, watertight,
// non-self-intersecting shell. Returns false and fills `error` on failure.
bool TetrahedralizeObj(const char* objPath, const TetGenOptions& options, TetMesh& out, std::string& error);

struct SoftBodyTopology {
	// Edges that get a distance constraint.
	//   * every edge of the boundary surface
	//   * every edge that touches an interior vertex (only exist when q / a add interior points)
	//   Interior "chords" between two surface vertices are skipped (same as the old LoadTet).
	std::vector<std::array<unsigned, 2>> distancePairs;
	// For each surface edge shared by exactly two triangles: the two vertices opposite to that edge.
	std::vector<std::array<unsigned, 2>> bendingPairs;
	// 1 if the vertex is part of the boundary surface.
	std::vector<char> onSurface;
};

void BuildTopology(const TetMesh& mesh, SoftBodyTopology& topology);
