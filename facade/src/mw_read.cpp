// MeshWeave additions: reading open CAD formats on the device
// (LOCAL_ONLY_L3_PLAN.md in the MeshWeave repository).
//
//  - mwImportIges: IGES through OCCT's own IGES translator (TKDEIGES), the same
//    calls the server makes (IGESControl_Reader, TransferRoots, OneShape).
//  - mwNurbsFace: one trimmed B-spline face from its poles, weights, knots and
//    the edges of its loops. It is what a Rhino .3dm reader needs: a Rhino Brep
//    face is a NURBS surface trimmed by NURBS edges.
//  - mwSewBody: faces sewn into one body at a tolerance, shape-fixed, closed
//    shells made into solids with any shell lying inside another kept as its
//    cavity; returns the body and how many shells closed.
//  - mwShellReport: solids / closed shells / open shells / loose faces / valid,
//    so the caller can decide whether a read part needs healing at all.
//
// Errors are std::runtime_error with the op name first, as in mw_edit.cpp.

#include "mw_access.h"

#include <emscripten/bind.h>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Geom_BSplineSurface.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Solid.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

template <class F> auto guardedRead(const char* op, F&& f) -> decltype(f()) {
    try {
        return f();
    } catch (const Standard_Failure& e) {
        throw std::runtime_error(std::string(op) + ": " + e.what());
    }
}

bool hasFace(const TopoDS_Shape& s) { return !s.IsNull() && TopExp_Explorer(s, TopAbs_FACE).More(); }

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

TopoDS_Shape unwrapOne(const TopoDS_Shape& s) {
    if (s.IsNull() || s.ShapeType() != TopAbs_COMPOUND) return s;
    TopoDS_Shape only;
    int n = 0;
    for (TopoDS_Iterator it(s); it.More(); it.Next()) {
        only = it.Value();
        if (++n > 1) return s;
    }
    return n == 1 ? only : s;
}

/// A point on the shell (its first vertex) to ask "is this inside that?".
bool firstPoint(const TopoDS_Shape& s, gp_Pnt& p) {
    TopExp_Explorer ex(s, TopAbs_VERTEX);
    if (!ex.More()) return false;
    p = BRep_Tool::Pnt(TopoDS::Vertex(ex.Current()));
    return true;
}

struct Built {
    TopoDS_Shape shape;
    int closed = 0;
    int open = 0;
};

/// Closed shells -> solids enclosing positive volume; a closed shell that lies
/// inside a larger one is that solid's cavity (reversed, added to it), not a
/// part of its own. Open shells and loose faces stay as they are.
Built nestedSolids(const TopoDS_Shape& sewn) {
    Built out;
    std::vector<TopoDS_Shell> closed;
    std::vector<TopoDS_Shape> rest;
    for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) {
        TopoDS_Shell sh = TopoDS::Shell(ex.Current());
        if (BRep_Tool::IsClosed(sh)) closed.push_back(sh); else rest.push_back(sh);
    }
    // Faces outside any shell: a body whose whole boundary is one closed face
    // (an untrimmed sphere) sews to that face. Put them in one shell and test.
    std::vector<TopoDS_Shape> loose;
    for (TopExp_Explorer ex(sewn, TopAbs_FACE, TopAbs_SHELL); ex.More(); ex.Next()) loose.push_back(ex.Current());
    if (!loose.empty()) {
        BRep_Builder b;
        TopoDS_Shell sh;
        b.MakeShell(sh);
        for (auto& f : loose) b.Add(sh, f);
        if (BRep_Tool::IsClosed(sh)) closed.push_back(sh);
        else rest.insert(rest.end(), loose.begin(), loose.end());
    }
    // Each closed shell on its own, outward, with its enclosed volume.
    struct S { TopoDS_Solid solid; TopoDS_Shell shell; double vol; int parent = -1; };
    std::vector<S> ss;
    for (auto& sh : closed) {
        BRepBuilderAPI_MakeSolid ms(sh);
        if (!ms.IsDone()) { rest.push_back(sh); continue; }
        ShapeFix_Solid fix(ms.Solid());
        fix.Perform();
        TopoDS_Shape fs = fix.Solid();
        TopoDS_Solid so;
        if (fs.ShapeType() == TopAbs_SOLID) so = TopoDS::Solid(fs);
        else {
            TopExp_Explorer e(fs, TopAbs_SOLID);
            if (!e.More()) { rest.push_back(sh); continue; }
            so = TopoDS::Solid(e.Current());
        }
        double v = volumeOf(so);
        if (v < 0) { so.Reverse(); v = -v; }
        TopoDS_Shell outer;
        TopExp_Explorer se(so, TopAbs_SHELL);
        if (se.More()) outer = TopoDS::Shell(se.Current());
        ss.push_back({so, outer, v});
    }
    std::sort(ss.begin(), ss.end(), [](const S& a, const S& b) { return a.vol > b.vol; });
    // The smallest enclosing larger solid is the parent; an even depth is a
    // body, an odd depth a cavity of its parent.
    for (size_t i = 0; i < ss.size(); ++i) {
        gp_Pnt p;
        if (!firstPoint(ss[i].shell, p)) continue;
        for (size_t j = i; j-- > 0;) {
            BRepClass3d_SolidClassifier c(ss[j].solid, p, 1e-6);
            if (c.State() == TopAbs_IN) { ss[i].parent = static_cast<int>(j); break; }
        }
    }
    std::vector<int> depth(ss.size(), 0);
    for (size_t i = 0; i < ss.size(); ++i) depth[i] = ss[i].parent < 0 ? 0 : depth[ss[i].parent] + 1;
    BRep_Builder b;
    std::vector<TopoDS_Solid> bodies(ss.size());
    for (size_t i = 0; i < ss.size(); ++i) {
        if (depth[i] % 2) continue;
        b.MakeSolid(bodies[i]);
        b.Add(bodies[i], ss[i].shell);
    }
    for (size_t i = 0; i < ss.size(); ++i) {
        if (depth[i] % 2 == 0) continue;
        TopoDS_Shape cav = ss[i].shell.Reversed();
        b.Add(bodies[ss[i].parent], cav);
    }
    TopoDS_Compound c;
    b.MakeCompound(c);
    int n = 0;
    for (size_t i = 0; i < ss.size(); ++i) if (depth[i] % 2 == 0) { b.Add(c, bodies[i]); ++n; }
    for (auto& r : rest) { b.Add(c, r); ++n; }
    out.closed = static_cast<int>(ss.size());
    out.open = static_cast<int>(rest.size());
    out.shape = n == 0 ? sewn : unwrapOne(c);
    return out;
}

/// Flat (value) knots -> OCCT's (distinct value, multiplicity) pairs.
void compress(const std::vector<double>& flat, NCollection_Array1<double>*& k, NCollection_Array1<int>*& m) {
    std::vector<double> v;
    std::vector<int> mu;
    for (double x : flat) {
        if (!v.empty() && std::abs(x - v.back()) < 1e-12) mu.back()++;
        else { v.push_back(x); mu.push_back(1); }
    }
    k = new NCollection_Array1<double>(1, static_cast<int>(v.size()));
    m = new NCollection_Array1<int>(1, static_cast<int>(mu.size()));
    for (size_t i = 0; i < v.size(); ++i) { k->SetValue(static_cast<int>(i) + 1, v[i]); m->SetValue(static_cast<int>(i) + 1, mu[i]); }
}

} // namespace

/// IGES bytes -> one shape (the translator's OneShape), refused in words when
/// the file reads to nothing.
uint32_t mwImportIges(OcctKernel& k, const std::string& data) {
    return guardedRead("importIges", [&] {
        const char* path = "/tmp/import.igs";
        FILE* f = std::fopen(path, "wb");
        if (!f) throw std::runtime_error("importIges: cannot create temp file");
        std::fwrite(data.data(), 1, data.size(), f);
        std::fclose(f);
        IGESControl_Reader r;
        IFSelect_ReturnStatus st = r.ReadFile(path);
        std::remove(path);
        if (st != IFSelect_RetDone) throw std::runtime_error("importIges: the file could not be read as IGES");
        r.TransferRoots();
        if (r.NbShapes() == 0) throw std::runtime_error("importIges: no shapes in this IGES file");
        TopoDS_Shape s = r.OneShape();
        if (!hasFace(s)) throw std::runtime_error("importIges: the file was read but holds no surfaces");
        return MwEdit::store(k, s);
    });
}

/// One trimmed NURBS face.
///  poles: x,y,z per pole, u-major (pole (i,j) at (i*nv + j)*3), euclidean.
///  weights: nu*nv (empty = non-rational).
///  uKnots / vKnots: FULL clamped knot vectors (nu+du+1, nv+dv+1 long).
///  edges: kernel edge ids for every loop's trims in order; edgeRev: 1 where
///  the trim runs against its edge; loopSizes: how many of `edges` each loop
///  takes, outer loop first. No loops: the face is the surface's own bounds.
uint32_t mwNurbsFace(OcctKernel& k, std::vector<double> poles, std::vector<double> weights, int nu, int nv,
                     int du, int dv, std::vector<double> uKnots, std::vector<double> vKnots,
                     std::vector<uint32_t> edges, std::vector<int> edgeRev, std::vector<int> loopSizes,
                     bool reversed) {
    return guardedRead("nurbsFace", [&] {
        if (nu < 2 || nv < 2 || poles.size() != static_cast<size_t>(nu * nv * 3))
            throw std::runtime_error("nurbsFace: pole grid does not match its counts");
        if (uKnots.size() != static_cast<size_t>(nu + du + 1) || vKnots.size() != static_cast<size_t>(nv + dv + 1))
            throw std::runtime_error("nurbsFace: knot vector length does not match poles and degree");
        if (!weights.empty() && weights.size() != static_cast<size_t>(nu * nv))
            throw std::runtime_error("nurbsFace: weights do not match the poles");
        NCollection_Array2<gp_Pnt> P(1, nu, 1, nv);
        NCollection_Array2<double> W(1, nu, 1, nv);
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                size_t o = static_cast<size_t>(i * nv + j);
                P.SetValue(i + 1, j + 1, gp_Pnt(poles[o * 3], poles[o * 3 + 1], poles[o * 3 + 2]));
                W.SetValue(i + 1, j + 1, weights.empty() || weights[o] == 0 ? 1.0 : weights[o]);
            }
        NCollection_Array1<double>* uk; NCollection_Array1<int>* um;
        NCollection_Array1<double>* vk; NCollection_Array1<int>* vm;
        compress(uKnots, uk, um);
        compress(vKnots, vk, vm);
        Handle(Geom_BSplineSurface) surf;
        try {
            surf = new Geom_BSplineSurface(P, W, *uk, *vk, *um, *vm, du, dv, false, false);
        } catch (...) {
            delete uk; delete um; delete vk; delete vm;
            throw;
        }
        delete uk; delete um; delete vk; delete vm;

        if (edgeRev.size() != edges.size()) throw std::runtime_error("nurbsFace: one reverse flag per edge");
        std::vector<TopoDS_Wire> wires;
        size_t at = 0;
        for (int n : loopSizes) {
            if (n < 0 || at + static_cast<size_t>(n) > edges.size())
                throw std::runtime_error("nurbsFace: loop sizes exceed the edges given");
            BRepBuilderAPI_MakeWire mw;
            int added = 0;
            for (int t = 0; t < n; ++t, ++at) {
                TopoDS_Edge e = TopoDS::Edge(MwEdit::get(k, edges[at]));
                if (edgeRev[at]) e.Reverse();
                mw.Add(e);
                ++added;
            }
            if (added && mw.IsDone()) wires.push_back(mw.Wire());
        }
        TopoDS_Face face;
        if (!wires.empty()) {
            BRepBuilderAPI_MakeFace mf(surf, wires[0], true);
            if (!mf.IsDone()) throw std::runtime_error("nurbsFace: the face could not be trimmed by its outer loop");
            face = mf.Face();
            for (size_t w = 1; w < wires.size(); ++w) {
                BRepBuilderAPI_MakeFace m2(face, wires[w]);
                if (m2.IsDone()) face = m2.Face();
            }
        } else {
            BRepBuilderAPI_MakeFace mf(surf, 1e-6);
            if (!mf.IsDone()) throw std::runtime_error("nurbsFace: the face has no usable boundary");
            face = mf.Face();
        }
        ShapeFix_Face fix(face);
        fix.Perform();
        TopoDS_Face f = fix.Face();
        if (reversed) f.Reverse();
        return MwEdit::store(k, f);
    });
}

/// Faces (or any shapes) sewn at `tol`, shape-fixed, closed shells made into
/// solids with cavities kept. Returns [shapeId, closedShells, openPieces].
std::vector<double> mwSewBody(OcctKernel& k, std::vector<uint32_t> ids, double tol) {
    return guardedRead("sewBody", [&] {
        if (ids.empty()) throw std::runtime_error("sewBody: no faces");
        BRepBuilderAPI_Sewing sew(tol);
        for (uint32_t id : ids) sew.Add(MwEdit::get(k, id));
        sew.Perform();
        ShapeFix_Shape fix(sew.SewedShape());
        fix.Perform();
        Built b = nestedSolids(fix.Shape());
        return std::vector<double>{static_cast<double>(MwEdit::store(k, b.shape)), static_cast<double>(b.closed),
                                   static_cast<double>(b.open)};
    });
}

/// A read part as it stands: [solids, closedShells, openShells, looseFaces, valid(0/1)].
std::vector<double> mwShellReport(OcctKernel& k, uint32_t id) {
    return guardedRead("shellReport", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        double solids = 0, closed = 0, open = 0, loose = 0;
        for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++solids;
        for (TopExp_Explorer ex(s, TopAbs_SHELL); ex.More(); ex.Next())
            (BRep_Tool::IsClosed(ex.Current()) ? closed : open) += 1;
        for (TopExp_Explorer ex(s, TopAbs_FACE, TopAbs_SHELL); ex.More(); ex.Next()) ++loose;
        double valid = !s.IsNull() && BRepCheck_Analyzer(s, false).IsValid() ? 1 : 0;
        return std::vector<double>{solids, closed, open, loose, valid};
    });
}

/// Closed shells of a read part into solids (cavities kept), the rest kept.
/// Returns [shapeId, closedShells, openPieces]. A part already all solids
/// comes back as itself.
std::vector<double> mwSolidify(OcctKernel& k, uint32_t id) {
    return guardedRead("solidify", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        Built b = nestedSolids(s);
        return std::vector<double>{static_cast<double>(MwEdit::store(k, b.shape)), static_cast<double>(b.closed),
                                   static_cast<double>(b.open)};
    });
}

EMSCRIPTEN_BINDINGS(meshweave_read) {
    using namespace emscripten;
    function("mwImportIges", &mwImportIges);
    function("mwNurbsFace", &mwNurbsFace);
    function("mwSewBody", &mwSewBody);
    function("mwShellReport", &mwShellReport);
    function("mwSolidify", &mwSolidify);
}
