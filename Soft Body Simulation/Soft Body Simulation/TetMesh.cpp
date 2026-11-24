#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "TetMesh.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <sstream>


#ifndef TETLIBRARY
#define TETLIBRARY
#endif
#include "tetgen/tetgen.h"

namespace {

typedef std::array<double, 3>   P3;
typedef std::array<unsigned, 3> Tri;

bool ReadObj(const char* path, std::vector<P3>& pts, std::vector<Tri>& tris, std::string& err) {
	std::ifstream file(path);
	if (!file) { err = std::string("cannot open ") + path; return false; }

	std::string line;
	long lineNo = 0;

	while (std::getline(file, line)) {
		lineNo++;

		size_t s = line.find_first_not_of(" \t");
		if (s == std::string::npos || s + 1 >= line.size()) continue;

		const char c0 = line[s], c1 = line[s + 1];
		if (c1 != ' ' && c1 != '\t') continue;

		if (c0 == 'v') {
			P3 p;
			if (std::sscanf(line.c_str() + s + 1, "%lf %lf %lf", &p[0], &p[1], &p[2]) != 3) {
				err = "bad vertex on line " + std::to_string(lineNo);
				return false;
			}
			pts.push_back(p);
		}

		else if (c0 == 'f') {
			std::istringstream ss(line.substr(s + 1));
			std::string tok;
			std::vector<long> idx;

			while (ss >> tok) {
				char* end = nullptr;
				long i = std::strtol(tok.c_str(), &end, 10);
				if (end == tok.c_str() || i == 0) { err = "bad face on line " + std::to_string(lineNo); return false; }

				if (i < 0) i = (long)pts.size() + i + 1;
				if (i < 1 || i >(long)pts.size()) { err = "face index out of range on line " + std::to_string(lineNo); return false; }

				idx.push_back(i - 1);
			}

			if (idx.size() < 3) { err = "face with fewer than 3 vertices on line " + std::to_string(lineNo); return false; }

			for (size_t k = 1; k + 1 < idx.size(); k++)
				tris.push_back({ (unsigned)idx[0], (unsigned)idx[k], (unsigned)idx[k + 1] });
		}
	}

	if (pts.empty() || tris.empty()) { err = "no vertices / faces found"; return false; }
	return true;
}

void CleanSurface(std::vector<P3>& pts, std::vector<Tri>& tris, double relTol) {
	P3 lo = pts[0], hi = pts[0];
	for (auto& p : pts) for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
	const double diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) + (hi[2] - lo[2]) * (hi[2] - lo[2]));
	const double eps  = std::max(relTol * diag, 1e-12);

	std::map<std::array<long long, 3>, unsigned> cell;
	std::vector<unsigned> remap(pts.size());
	std::vector<P3> welded;

	for (size_t i = 0; i < pts.size(); i++) {
		std::array<long long, 3> key = { std::llround(pts[i][0] / eps), std::llround(pts[i][1] / eps), std::llround(pts[i][2] / eps) };
		auto it = cell.find(key);
		if (it == cell.end()) { it = cell.insert({ key, (unsigned)welded.size() }).first; welded.push_back(pts[i]); }
		remap[i] = it->second;
	}

	std::vector<Tri> clean;
	std::map<Tri, int> seen;
	for (auto t : tris) {
		for (auto& v : t) v = remap[v];
		if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;

		Tri sorted = t;
		std::sort(sorted.begin(), sorted.end());
		if (seen.count(sorted)) continue;
		seen[sorted] = 1;

		clean.push_back(t);
	}

	std::vector<int> used(welded.size(), -1);
	for (auto& t : clean) for (auto v : t) used[v] = 0;

	std::vector<P3> compact;
	for (size_t i = 0; i < welded.size(); i++) if (used[i] == 0) { used[i] = (int)compact.size(); compact.push_back(welded[i]); }
	for (auto& t : clean) for (auto& v : t) v = (unsigned)used[v];

	pts.swap(compact);
	tris.swap(clean);
}

void FitAndPlace(std::vector<P3>& pts, const TetGenOptions& o) {
	if (o.fitSize <= 0.0 && !o.centerAtOrigin) return;

	P3 lo = pts[0], hi = pts[0];
	for (auto& p : pts) for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }

	P3 c = { (lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5 };
	double extent = std::max({ hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2] });
	double s = (o.fitSize > 0.0 && extent > 0.0) ? o.fitSize / extent : 1.0;

	for (auto& p : pts) for (int k = 0; k < 3; k++)
		p[k] = (p[k] - c[k]) * s + (o.centerAtOrigin ? 0.0 : c[k]);
}

std::string BuildSwitches(const TetGenOptions& o, bool useY) {
	std::ostringstream s;
	s.imbue(std::locale::classic());
	s << std::fixed;

	s << "pz";

	double q = o.qualityBound;
	if (q <= 0.0 && o.maxTetVolume > 0.0) q = 2.0;
	if (q > 0.0) s << "q" << std::setprecision(4) << q;

	if (o.maxTetVolume > 0.0) s << "a" << std::setprecision(10) << o.maxTetVolume;

	if (useY) s << "Y";
	if (!o.verbose) s << "Q";

	return s.str();
}

const char* TetGenErrorText(int code) {
	switch (code) {
	case 1:  return "TetGen ran out of memory (try a larger maxTetVolume / smaller qualityBound)";
	case 2:  return "TetGen internal error";
	case 3:  return "the OBJ surface has self-intersections (it must be one clean closed shell)";
	case 4:  return "TetGen found a very small input feature (nearly intersecting edges)";
	case 5:  return "TetGen found two very close input facets";
	case 10: return "TetGen rejected the input";
	default: return "unknown TetGen error";
	}
}

bool RunTetGen(const std::vector<P3>& pts, const std::vector<Tri>& tris, const std::string& switches, tetgenio& out, int& code) {
	tetgenio in;
	in.firstnumber = 0;

	in.numberofpoints = (int)pts.size();
	in.pointlist = new REAL[pts.size() * 3];
	for (size_t i = 0; i < pts.size(); i++) for (int k = 0; k < 3; k++) in.pointlist[i * 3 + k] = pts[i][k];

	in.numberoffacets = (int)tris.size();
	in.facetlist = new tetgenio::facet[tris.size()];
	for (size_t i = 0; i < tris.size(); i++) {
		tetgenio::facet& f = in.facetlist[i];
		f.numberofpolygons = 1;
		f.polygonlist = new tetgenio::polygon[1];
		f.numberofholes = 0;
		f.holelist = NULL;

		tetgenio::polygon& p = f.polygonlist[0];
		p.numberofvertices = 3;
		p.vertexlist = new int[3];
		for (int k = 0; k < 3; k++) p.vertexlist[k] = (int)tris[i][k];
	}

	std::vector<char> sw(switches.begin(), switches.end());
	sw.push_back('\0');

	try { tetrahedralize(sw.data(), &in, &out); }
	catch (int c) { code = c; return false; }
	catch (...)   { code = -1; return false; }

	return true;
}

}

// ---------------------------------------------------------------------------------------------
bool TetrahedralizeObj(const char* objPath, const TetGenOptions& options, TetMesh& mesh, std::string& error) {
	mesh = TetMesh();

	std::vector<P3>  pts;
	std::vector<Tri> tris;
	if (!ReadObj(objPath, pts, tris, error)) return false;

	CleanSurface(pts, tris, options.weldTolerance);
	if (pts.size() < 4 || tris.size() < 4) { error = "surface is too small to enclose a volume"; return false; }

	FitAndPlace(pts, options);

	std::unique_ptr<tetgenio> out(new tetgenio);
	int code = 0;
	bool ok = RunTetGen(pts, tris, BuildSwitches(options, options.preserveSurface), *out, code);

	if (!ok && options.preserveSurface && code != 1 && code != 3) {
		std::fprintf(stderr, "TetGen failed with Y (%s), retrying without it - the surface may get extra vertices\n", TetGenErrorText(code));
		out.reset(new tetgenio);
		ok = RunTetGen(pts, tris, BuildSwitches(options, false), *out, code);
	}

	if (!ok) { error = TetGenErrorText(code); return false; }

	// ---- read the result -------------------------------------------------------------------------
	const int n = out->numberofpoints, T = out->numberoftetrahedra;
	if (T <= 0 || n <= 0) { error = "TetGen returned no tetrahedra (is the OBJ a closed volume?)"; return false; }
	if (out->numberofcorners != 4) { error = "TetGen returned 10-node tets"; return false; }

	int lo = INT_MAX, hi = INT_MIN;
	for (int i = 0; i < T * 4; i++) { lo = std::min(lo, out->tetrahedronlist[i]); hi = std::max(hi, out->tetrahedronlist[i]); }

	const int base = (lo >= 1 && hi == n) ? 1 : 0;
	if (lo - base < 0 || hi - base >= n) { error = "TetGen returned out-of-range indices"; return false; }

	std::vector<int> newIndex(n, -1);
	for (int i = 0; i < T * 4; i++) newIndex[out->tetrahedronlist[i] - base] = 0;
	for (int v = 0; v < n; v++) if (newIndex[v] == 0) {
		newIndex[v] = (int)mesh.vertices.size();
		mesh.vertices.push_back({ out->pointlist[v * 3], out->pointlist[v * 3 + 1], out->pointlist[v * 3 + 2] });
	}

	mesh.tets.reserve(T);
	for (int t = 0; t < T; t++) {
		std::array<unsigned, 4> tet;
		for (int k = 0; k < 4; k++) tet[k] = (unsigned)newIndex[out->tetrahedronlist[t * 4 + k] - base];

		const auto& a = mesh.vertices[tet[0]]; const auto& b = mesh.vertices[tet[1]];
		const auto& c = mesh.vertices[tet[2]]; const auto& d = mesh.vertices[tet[3]];

		double u[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
		double v[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
		double w[3] = { d[0] - a[0], d[1] - a[1], d[2] - a[2] };
		double V6 = (u[1] * v[2] - u[2] * v[1]) * w[0] + (u[2] * v[0] - u[0] * v[2]) * w[1] + (u[0] * v[1] - u[1] * v[0]) * w[2];

		if (V6 < 0.0) std::swap(tet[2], tet[3]);
		mesh.tets.push_back(tet);
	}

	static const int F[4][3] = { {0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3} };

	struct FaceRec { Tri key, tri; };
	std::vector<FaceRec> recs;
	recs.reserve(mesh.tets.size() * 4);

	for (auto& t : mesh.tets) for (int k = 0; k < 4; k++) {
		FaceRec r;
		r.tri = { t[F[k][0]], t[F[k][1]], t[F[k][2]] };
		r.key = r.tri;
		std::sort(r.key.begin(), r.key.end());
		recs.push_back(r);
	}

	std::sort(recs.begin(), recs.end(), [](const FaceRec& x, const FaceRec& y) { return x.key < y.key; });

	for (size_t i = 0; i < recs.size();) {
		size_t j = i;
		while (j < recs.size() && recs[j].key == recs[i].key) j++;

		if (j - i == 1) mesh.surface.push_back(recs[i].tri);
		else if (j - i > 2) { error = "TetGen produced a non-manifold tet mesh"; return false; }

		i = j;
	}

	if (mesh.surface.empty()) { error = "tet mesh has no boundary"; return false; }
	return true;
}

void BuildTopology(const TetMesh& m, SoftBodyTopology& topo) {
	topo = SoftBodyTopology();
	topo.onSurface.assign(m.vertices.size(), 0);
	for (auto& f : m.surface) for (int k = 0; k < 3; k++) topo.onSurface[f[k]] = 1;

	auto edgeKey = [](unsigned a, unsigned b) -> uint64_t {
		if (a > b) std::swap(a, b);
		return ((uint64_t)a << 32) | b;
	};

	std::vector<uint64_t> tetEdges;
	tetEdges.reserve(m.tets.size() * 6);
	static const int E[6][2] = { {0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3} };
	for (auto& t : m.tets) for (auto& e : E) tetEdges.push_back(edgeKey(t[e[0]], t[e[1]]));
	std::sort(tetEdges.begin(), tetEdges.end());
	tetEdges.erase(std::unique(tetEdges.begin(), tetEdges.end()), tetEdges.end());

	struct SurfEdge { uint64_t key; unsigned opposite; };
	std::vector<SurfEdge> surf;
	surf.reserve(m.surface.size() * 3);
	for (auto& f : m.surface) {
		surf.push_back({ edgeKey(f[0], f[1]), f[2] });
		surf.push_back({ edgeKey(f[1], f[2]), f[0] });
		surf.push_back({ edgeKey(f[2], f[0]), f[1] });
	}
	std::sort(surf.begin(), surf.end(), [](const SurfEdge& x, const SurfEdge& y) { return x.key < y.key; });

	for (uint64_t e : tetEdges) {
		const unsigned i = (unsigned)(e >> 32), j = (unsigned)(e & 0xffffffffu);

		auto first = std::lower_bound(surf.begin(), surf.end(), e, [](const SurfEdge& s, uint64_t k) { return s.key < k; });
		size_t count = 0;
		for (auto it = first; it != surf.end() && it->key == e; ++it) count++;

		const bool surfaceEdge = count > 0;

		if (surfaceEdge || !topo.onSurface[i] || !topo.onSurface[j]) topo.distancePairs.push_back({ i, j });

		if (count == 2 && first[0].opposite != first[1].opposite) topo.bendingPairs.push_back({ first[0].opposite, first[1].opposite });
	}
}
