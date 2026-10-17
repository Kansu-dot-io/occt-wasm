// MeshWeave Edit additions to the occt-wasm facade (MIT OR Apache-2.0, like
// the rest of the facade). OCCT itself is not modified.
//
// Why a hand-written file and not codegen specs: these functions are bound as
// free functions in their own EMSCRIPTEN_BINDINGS block, so the upstream spec
// list, the generated files and the WASI crate stay byte-identical to upstream
// and a rebase onto a new upstream release touches one header line
// (`friend struct MwEdit;` in occt_kernel.h).
//
// What they add over upstream:
//  - INDEX-based face history. Upstream's *WithHistory methods report faces as
//    hashes modulo a bound, which can collide. Here every face is named by its
//    0-based position in TopExp::MapShapes(shape, TopAbs_FACE), the order the
//    editor's face ids are kept in, for the input(s) and the result. The history
//    comes from OCCT's own Modified / Generated / IsDeleted (BRepTools_History).
//  - History on every op the editor needs, including the ones upstream has none
//    for: splitter, defeaturing, draft, unify-same-domain, shape fix, sewing.
//  - A tessellation whose triangles are grouped by face index and whose edge
//    polylines are in edge-map order, plus the face/edge topology the ids are
//    built from (surface kind and parameters, area, centroid, the faces each
//    edge bounds).
//
// History encoding (EvolutionData, reused from upstream so no new binding type):
//   modified  = [inFace, n, outFace_1 .. outFace_n, ...]   (unchanged faces: n=1, out=its index)
//   generated = [inFace, n, outFace_1 .. outFace_n, ...]
//   deleted   = [inFace, ...]
// Input faces of a two-shape op are numbered a's faces first, then b's (offset
// by a's face count).
//
// Errors are std::runtime_error with the op name first. When OCCT can say which
// faces or edges failed, the message ends in " @F<i>,<j>" or " @E<i>,<j>"
// (indices as above) so the editor can highlight them.

#include "occt_kernel.h"

#include <emscripten/bind.h>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools_History.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_HSequence.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <NCollection_Vec3.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using ShapeList = NCollection_List<TopoDS_Shape>;
using ShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using AncestorMap =
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

/// The kernel's arena is private; occt_kernel.h names this struct a friend.
struct MwEdit {
    static uint32_t store(OcctKernel& k, const TopoDS_Shape& s) { return k.store(s); }
    static const TopoDS_Shape& get(OcctKernel& k, uint32_t id) { return k.get(id); }
};

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

ShapeMap faceMap(const TopoDS_Shape& s) {
    ShapeMap m;
    TopExp::MapShapes(s, TopAbs_FACE, m);
    return m;
}

ShapeMap edgeMap(const TopoDS_Shape& s) {
    ShapeMap m;
    TopExp::MapShapes(s, TopAbs_EDGE, m);
    return m;
}

std::string refList(char kind, const std::vector<int>& idx) {
    if (idx.empty()) return "";
    std::ostringstream o;
    o << " @" << kind;
    for (size_t i = 0; i < idx.size(); ++i) o << (i ? "," : "") << idx[i];
    return o.str();
}

/// Faces of `s` picked by 0-based index; throws naming any index out of range.
std::vector<TopoDS_Face> pickFaces(const TopoDS_Shape& s, const std::vector<int>& idx,
                                   const char* op) {
    ShapeMap m = faceMap(s);
    std::vector<TopoDS_Face> out;
    for (int i : idx) {
        if (i < 0 || i >= m.Extent())
            throw std::runtime_error(std::string(op) + ": no face " + std::to_string(i) +
                                     " (the shape has " + std::to_string(m.Extent()) + ")");
        out.push_back(TopoDS::Face(m.FindKey(i + 1)));
    }
    return out;
}

std::vector<TopoDS_Edge> pickEdges(const TopoDS_Shape& s, const std::vector<int>& idx,
                                   const char* op) {
    ShapeMap m = edgeMap(s);
    std::vector<TopoDS_Edge> out;
    for (int i : idx) {
        if (i < 0 || i >= m.Extent())
            throw std::runtime_error(std::string(op) + ": no edge " + std::to_string(i) +
                                     " (the shape has " + std::to_string(m.Extent()) + ")");
        out.push_back(TopoDS::Edge(m.FindKey(i + 1)));
    }
    return out;
}

/// A compound holding exactly one solid becomes that solid (fillet, chamfer and
/// the booleans wrap single results), so the next op sees a Solid.
TopoDS_Shape unwrap(const TopoDS_Shape& s) {
    if (s.IsNull() || s.ShapeType() != TopAbs_COMPOUND) return s;
    TopoDS_Shape only;
    int n = 0;
    for (TopoDS_Iterator it(s); it.More(); it.Next()) {
        only = it.Value();
        if (++n > 1) return s;
    }
    return n == 1 ? only : s;
}

/// The face history of `inputs` (in order) into `result`, read from `h` (null
/// means every input face that is still in the result is unchanged).
/// `extra` adds images by hand: input face index -> shapes whose final images
/// (through `h`) are that face's modified images (push/pull's moved cap).
EvolutionData evolution(uint32_t resultId, const std::vector<TopoDS_Shape>& inputs,
                        const TopoDS_Shape& result, const Handle(BRepTools_History) & h,
                        const std::map<int, std::vector<TopoDS_Shape>>& extra = {}) {
    EvolutionData evo;
    evo.resultId = resultId;
    ShapeMap rmap = faceMap(result);

    auto finalImages = [&](const TopoDS_Shape& s, std::vector<int>& out) {
        if (!h.IsNull() && h->IsRemoved(s)) return;
        const ShapeList* mods = nullptr;
        if (!h.IsNull() && h->HasModified() && !(mods = &h->Modified(s))->IsEmpty()) {
            for (const TopoDS_Shape& m : *mods) {
                int j = rmap.FindIndex(m);
                if (j > 0) out.push_back(j - 1);
            }
            return;
        }
        int j = rmap.FindIndex(s);
        if (j > 0) out.push_back(j - 1);
    };

    int base = 0;
    for (const TopoDS_Shape& in : inputs) {
        ShapeMap imap = faceMap(in);
        for (int i = 1; i <= imap.Extent(); ++i) {
            const TopoDS_Shape& f = imap.FindKey(i);
            int gi = base + i - 1;
            std::vector<int> imgs;
            auto ex = extra.find(gi);
            if (ex != extra.end()) {
                for (const TopoDS_Shape& s : ex->second) finalImages(s, imgs);
            } else {
                finalImages(f, imgs);
            }
            if (imgs.empty()) {
                evo.deleted.push_back(gi);
            } else {
                evo.modified.push_back(gi);
                evo.modified.push_back(static_cast<int>(imgs.size()));
                evo.modified.insert(evo.modified.end(), imgs.begin(), imgs.end());
            }
            if (!h.IsNull() && h->HasGenerated()) {
                std::vector<int> gens;
                for (const TopoDS_Shape& g : h->Generated(f)) {
                    int j = rmap.FindIndex(g);
                    if (j > 0) gens.push_back(j - 1);
                }
                if (!gens.empty()) {
                    evo.generated.push_back(gi);
                    evo.generated.push_back(static_cast<int>(gens.size()));
                    evo.generated.insert(evo.generated.end(), gens.begin(), gens.end());
                }
            }
        }
        base += imap.Extent();
    }
    return evo;
}

template <class Algo>
Handle(BRepTools_History) historyOf(const std::vector<TopoDS_Shape>& inputs, Algo& algo) {
    ShapeList args;
    for (const TopoDS_Shape& s : inputs) args.Append(s);
    return new BRepTools_History(args, algo);
}

/// Sewing has no MakeShape interface; its history is written out by hand.
Handle(BRepTools_History) sewingHistory(const std::vector<TopoDS_Shape>& inputs,
                                        BRepBuilderAPI_Sewing& sew) {
    Handle(BRepTools_History) h = new BRepTools_History;
    for (const TopoDS_Shape& in : inputs) {
        for (TopExp_Explorer ex(in, TopAbs_FACE); ex.More(); ex.Next()) {
            const TopoDS_Shape& f = ex.Current();
            if (sew.IsModified(f)) h->AddModified(f, sew.Modified(f));
        }
    }
    return h;
}

bool isValid(const TopoDS_Shape& s) { return !s.IsNull() && BRepCheck_Analyzer(s, false).IsValid(); }

/// Shells that close become solids (with outward orientation); others stay shells.
TopoDS_Shape solidify(const TopoDS_Shape& sewn) {
    BRep_Builder b;
    TopoDS_Compound out;
    b.MakeCompound(out);
    int n = 0;
    for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) {
        TopoDS_Shell sh = TopoDS::Shell(ex.Current());
        if (BRep_Tool::IsClosed(sh)) {
            BRepBuilderAPI_MakeSolid ms(sh);
            if (ms.IsDone()) {
                ShapeFix_Solid fix(ms.Solid());
                fix.Perform();
                b.Add(out, fix.Solid());
                ++n;
                continue;
            }
        }
        b.Add(out, sh);
        ++n;
    }
    for (TopExp_Explorer ex(sewn, TopAbs_FACE, TopAbs_SHELL); ex.More(); ex.Next()) {
        b.Add(out, ex.Current());
        ++n;
    }
    return n == 0 ? sewn : unwrap(out);
}

double bboxDiag(const TopoDS_Shape& s) {
    Bnd_Box box;
    BRepBndLib::Add(s, box, false);
    if (box.IsVoid()) return 1.0;
    return std::sqrt(box.SquareExtent());
}

gp_Pnt bboxCenter(const TopoDS_Shape& s) {
    Bnd_Box box;
    BRepBndLib::Add(s, box, false);
    if (box.IsVoid()) return gp_Pnt(0, 0, 0);
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    return gp_Pnt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
}

/// A planar face big enough to cut all of `s`.
TopoDS_Face cuttingFace(const TopoDS_Shape& s, const gp_Pln& pln) {
    gp_Pnt c = bboxCenter(s);
    double L = 2.0 * (bboxDiag(s) + pln.Location().Distance(c)) + 1.0;
    double u0, v0;
    // Centre the face on the projection of the box centre.
    gp_Vec d(pln.Location(), c);
    u0 = d.Dot(gp_Vec(pln.XAxis().Direction()));
    v0 = d.Dot(gp_Vec(pln.YAxis().Direction()));
    BRepBuilderAPI_MakeFace mf(pln, u0 - L, u0 + L, v0 - L, v0 + L);
    if (!mf.IsDone()) throw std::runtime_error("split: could not build the cutting plane");
    return mf.Face();
}

/// Outward unit normal of a planar face, or throws.
gp_Dir planarNormal(const TopoDS_Face& f, const char* op, int idx) {
    BRepAdaptor_Surface a(f, false);
    if (a.GetType() != GeomAbs_Plane)
        throw std::runtime_error(std::string(op) + ": face is not planar" + refList('F', {idx}));
    gp_Dir n = a.Plane().Axis().Direction();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    return n;
}

int bodyCount(const TopoDS_Shape& s, std::vector<TopoDS_Shape>& bodies) {
    for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) bodies.push_back(ex.Current());
    if (bodies.empty())
        for (TopExp_Explorer ex(s, TopAbs_SHELL); ex.More(); ex.Next()) bodies.push_back(ex.Current());
    if (bodies.empty()) bodies.push_back(s);
    return static_cast<int>(bodies.size());
}

// Mesh output buffers, copied out by the caller straight after mwMesh.
std::vector<float> gPos, gNrm, gEdgePts;
std::vector<int32_t> gFaceRanges, gEdgeRanges;

template <class F> auto guarded(const char* op, F&& f) -> decltype(f()) {
    try {
        return f();
    } catch (const Standard_Failure& e) {
        throw std::runtime_error(std::string(op) + ": " + e.what());
    }
}

} // namespace

// ===========================================================================
// Topology and tessellation
// ===========================================================================

/// [nFaces, nEdges, nBodies, bbox xmin ymin zmin xmax ymax zmax, per face 16
/// doubles, per edge 8 doubles]. The bbox is BRepBndLib::AddOptimal without
/// triangulation or tolerances.
/// Face: surf (0 plane, 1 cylinder, 2 cone, 3 sphere, 4 torus, 5 other), area,
///   centroid xyz, dir xyz (plane normal, outward / axis; NaN when none),
///   origin xyz (plane location / axis location / centre), r1, r2 (radius,
///   major+minor, cone ref radius + semi-angle in radians), body, reversed.
/// Edge: curve (0 line, 1 circle, 2 ellipse, 3 bspline/bezier, 4 other),
///   length, face0, face1 (-1 = none; face1 == face0 on a seam), midpoint xyz,
///   degenerate (0/1).
std::vector<double> mwTopology(OcctKernel& k, uint32_t id) {
    return guarded("topology", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        ShapeMap fm = faceMap(s), em = edgeMap(s);
        std::vector<TopoDS_Shape> bodies;
        int nb = bodyCount(s, bodies);
        std::vector<int> bodyOf(fm.Extent(), -1);
        for (int b = 0; b < nb; ++b)
            for (TopExp_Explorer ex(bodies[b], TopAbs_FACE); ex.More(); ex.Next()) {
                int j = fm.FindIndex(ex.Current());
                if (j > 0 && bodyOf[j - 1] < 0) bodyOf[j - 1] = b;
            }

        std::vector<double> out;
        out.reserve(9 + 16 * fm.Extent() + 8 * em.Extent());
        out.push_back(fm.Extent());
        out.push_back(em.Extent());
        out.push_back(nb);
        Bnd_Box box;
        BRepBndLib::AddOptimal(s, box, false, false);
        double bb[6] = {0, 0, 0, 0, 0, 0};
        if (!box.IsVoid()) box.Get(bb[0], bb[1], bb[2], bb[3], bb[4], bb[5]);
        out.insert(out.end(), bb, bb + 6);
        for (int i = 1; i <= fm.Extent(); ++i) {
            const TopoDS_Face& f = TopoDS::Face(fm.FindKey(i));
            GProp_GProps g;
            BRepGProp::SurfaceProperties(f, g);
            gp_Pnt c = g.CentreOfMass();
            BRepAdaptor_Surface a(f, false);
            bool rev = f.Orientation() == TopAbs_REVERSED;
            double code = 5, dx = kNaN, dy = kNaN, dz = kNaN, ox = kNaN, oy = kNaN, oz = kNaN,
                   r1 = kNaN, r2 = kNaN;
            auto setDir = [&](const gp_Dir& d) { dx = d.X(); dy = d.Y(); dz = d.Z(); };
            auto setO = [&](const gp_Pnt& p) { ox = p.X(); oy = p.Y(); oz = p.Z(); };
            switch (a.GetType()) {
            case GeomAbs_Plane: {
                gp_Pln p = a.Plane();
                gp_Dir n = p.Axis().Direction();
                if (rev) n.Reverse();
                code = 0; setDir(n); setO(p.Location());
                break;
            }
            case GeomAbs_Cylinder: {
                gp_Cylinder cy = a.Cylinder();
                code = 1; setDir(cy.Axis().Direction()); setO(cy.Location()); r1 = cy.Radius();
                break;
            }
            case GeomAbs_Cone: {
                gp_Cone co = a.Cone();
                code = 2; setDir(co.Axis().Direction()); setO(co.Location());
                r1 = co.RefRadius(); r2 = co.SemiAngle();
                break;
            }
            case GeomAbs_Sphere: {
                gp_Sphere sp = a.Sphere();
                code = 3; setO(sp.Location()); r1 = sp.Radius();
                break;
            }
            case GeomAbs_Torus: {
                gp_Torus to = a.Torus();
                code = 4; setDir(to.Axis().Direction()); setO(to.Location());
                r1 = to.MajorRadius(); r2 = to.MinorRadius();
                break;
            }
            default:
                break;
            }
            double row[16] = {code, g.Mass(), c.X(), c.Y(), c.Z(), dx, dy, dz, ox, oy, oz, r1, r2,
                              static_cast<double>(bodyOf[i - 1]), rev ? 1.0 : 0.0, 0.0};
            out.insert(out.end(), row, row + 16);
        }

        AncestorMap anc;
        TopExp::MapShapesAndUniqueAncestors(s, TopAbs_EDGE, TopAbs_FACE, anc);
        for (int i = 1; i <= em.Extent(); ++i) {
            const TopoDS_Edge& e = TopoDS::Edge(em.FindKey(i));
            bool degen = BRep_Tool::Degenerated(e);
            double code = 4, len = 0;
            gp_Pnt mid(0, 0, 0);
            if (!degen) {
                BRepAdaptor_Curve c(e);
                switch (c.GetType()) {
                case GeomAbs_Line: code = 0; break;
                case GeomAbs_Circle: code = 1; break;
                case GeomAbs_Ellipse: code = 2; break;
                case GeomAbs_BSplineCurve:
                case GeomAbs_BezierCurve: code = 3; break;
                default: break;
                }
                GProp_GProps g;
                BRepGProp::LinearProperties(e, g);
                len = g.Mass();
                mid = c.Value((c.FirstParameter() + c.LastParameter()) / 2);
            }
            int f0 = -1, f1 = -1;
            int ai = anc.FindIndex(e);
            if (ai > 0) {
                const ShapeList& fl = anc.FindFromIndex(ai);
                int n = 0;
                for (const TopoDS_Shape& f : fl) {
                    int j = fm.FindIndex(f) - 1;
                    if (n == 0) f0 = j;
                    else if (n == 1) f1 = j;
                    ++n;
                }
            }
            // A seam bounds one face on both sides.
            if (f1 < 0 && f0 >= 0 && !degen) {
                const TopoDS_Face& f = TopoDS::Face(fm.FindKey(f0 + 1));
                if (BRep_Tool::IsClosed(e, f)) f1 = f0;
            }
            double row[8] = {code, len, double(f0), double(f1), mid.X(), mid.Y(), mid.Z(),
                             degen ? 1.0 : 0.0};
            out.insert(out.end(), row, row + 8);
        }
        return out;
    });
}

/// Mesh `id` into a non-indexed soup grouped by face (face-map order) plus an
/// edge polyline per edge (edge-map order). Returns
/// [posPtr, nrmPtr, nTris, faceRangesPtr, nFaces, edgePtsPtr, nEdgeFloats,
///  edgeRangesPtr, nEdges, facesWithoutMesh]. faceRanges = [triStart, triCount]
/// per face, edgeRanges = [firstFloat, nFloats] per edge. The buffers live until
/// the next call.
std::vector<uint32_t> mwMesh(OcctKernel& k, uint32_t id, double lin, double ang) {
    return guarded("tessellate", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        BRepMesh_IncrementalMesh mesher(s, lin, false, ang, false);
        if (!mesher.IsDone()) throw std::runtime_error("tessellate: meshing failed");
        ShapeMap fm = faceMap(s), em = edgeMap(s);
        gPos.clear(); gNrm.clear(); gEdgePts.clear(); gFaceRanges.clear(); gEdgeRanges.clear();
        uint32_t nTri = 0, missing = 0;
        std::vector<Handle(Poly_Triangulation)> tris(fm.Extent() + 1);
        std::vector<TopLoc_Location> locs(fm.Extent() + 1);
        for (int i = 1; i <= fm.Extent(); ++i) {
            const TopoDS_Face& f = TopoDS::Face(fm.FindKey(i));
            TopLoc_Location loc;
            Handle(Poly_Triangulation) t = BRep_Tool::Triangulation(f, loc);
            tris[i] = t; locs[i] = loc;
            gFaceRanges.push_back(static_cast<int32_t>(nTri));
            if (t.IsNull() || t->NbTriangles() == 0) {
                gFaceRanges.push_back(0);
                ++missing;
                continue;
            }
            if (!t->HasNormals()) BRepLib_ToolTriangulatedShape::ComputeNormals(f, t);
            const gp_Trsf& tr = loc.Transformation();
            bool ident = loc.IsIdentity();
            bool rev = f.Orientation() == TopAbs_REVERSED;
            for (int ti = 1; ti <= t->NbTriangles(); ++ti) {
                int n1, n2, n3;
                t->Triangle(ti).Get(n1, n2, n3);
                if (rev) std::swap(n2, n3);
                for (int n : {n1, n2, n3}) {
                    gp_Pnt p = t->Node(n);
                    if (!ident) p.Transform(tr);
                    gPos.push_back(float(p.X())); gPos.push_back(float(p.Y())); gPos.push_back(float(p.Z()));
                    gp_Dir d(0, 0, 1);
                    if (t->HasNormals()) {
                        NCollection_Vec3<float> nv;
                        t->Normal(n, nv);
                        if (nv.x() != 0.f || nv.y() != 0.f || nv.z() != 0.f) d = gp_Dir(nv.x(), nv.y(), nv.z());
                    }
                    if (rev) d.Reverse();
                    if (!ident) d.Transform(tr);
                    gNrm.push_back(float(d.X())); gNrm.push_back(float(d.Y())); gNrm.push_back(float(d.Z()));
                }
            }
            gFaceRanges.push_back(t->NbTriangles());
            nTri += t->NbTriangles();
        }

        AncestorMap anc;
        TopExp::MapShapesAndUniqueAncestors(s, TopAbs_EDGE, TopAbs_FACE, anc);
        for (int i = 1; i <= em.Extent(); ++i) {
            const TopoDS_Edge& e = TopoDS::Edge(em.FindKey(i));
            gEdgeRanges.push_back(static_cast<int32_t>(gEdgePts.size()));
            size_t before = gEdgePts.size();
            if (!BRep_Tool::Degenerated(e)) {
                bool done = false;
                int ai = anc.FindIndex(e);
                if (ai > 0) {
                    for (const TopoDS_Shape& f : anc.FindFromIndex(ai)) {
                        int fi = fm.FindIndex(f);
                        if (fi <= 0 || tris[fi].IsNull()) continue;
                        Handle(Poly_PolygonOnTriangulation) poly =
                            BRep_Tool::PolygonOnTriangulation(e, tris[fi], locs[fi]);
                        if (poly.IsNull()) continue;
                        const gp_Trsf& tr = locs[fi].Transformation();
                        for (int n = 1; n <= poly->NbNodes(); ++n) {
                            gp_Pnt p = tris[fi]->Node(poly->Node(n)).Transformed(tr);
                            gEdgePts.push_back(float(p.X())); gEdgePts.push_back(float(p.Y())); gEdgePts.push_back(float(p.Z()));
                        }
                        done = true;
                        break;
                    }
                }
                if (!done) {
                    BRepAdaptor_Curve c(e);
                    GCPnts_TangentialDeflection td(c, ang, lin);
                    for (int n = 1; n <= td.NbPoints(); ++n) {
                        gp_Pnt p = td.Value(n);
                        gEdgePts.push_back(float(p.X())); gEdgePts.push_back(float(p.Y())); gEdgePts.push_back(float(p.Z()));
                    }
                }
            }
            gEdgeRanges.push_back(static_cast<int32_t>(gEdgePts.size() - before));
        }
        auto ptr = [](const void* p) { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p)); };
        return std::vector<uint32_t>{ptr(gPos.data()), ptr(gNrm.data()), nTri,
                                     ptr(gFaceRanges.data()), uint32_t(fm.Extent()),
                                     ptr(gEdgePts.data()), uint32_t(gEdgePts.size()),
                                     ptr(gEdgeRanges.data()), uint32_t(em.Extent()), missing};
    });
}

/// Free the mesh buffers (they are otherwise kept for the next call).
void mwMeshFree() {
    std::vector<float>().swap(gPos);
    std::vector<float>().swap(gNrm);
    std::vector<float>().swap(gEdgePts);
    std::vector<int32_t>().swap(gFaceRanges);
    std::vector<int32_t>().swap(gEdgeRanges);
}

/// [volume, area, xmin, ymin, zmin, xmax, ymax, zmax, cx, cy, cz]. Volume is 0
/// and the centroid is the surface centroid when the shape holds no solid.
std::vector<double> mwMeasure(OcctKernel& k, uint32_t id) {
    return guarded("measure", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        GProp_GProps sg;
        BRepGProp::SurfaceProperties(s, sg);
        bool solid = TopExp_Explorer(s, TopAbs_SOLID).More();
        GProp_GProps vg;
        if (solid) BRepGProp::VolumeProperties(s, vg);
        Bnd_Box box;
        BRepBndLib::AddOptimal(s, box, false, false);
        double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
        if (!box.IsVoid()) box.Get(x0, y0, z0, x1, y1, z1);
        gp_Pnt c = solid ? vg.CentreOfMass() : sg.CentreOfMass();
        return std::vector<double>{solid ? vg.Mass() : 0.0, sg.Mass(), x0, y0, z0, x1, y1, z1,
                                   c.X(), c.Y(), c.Z()};
    });
}

/// Distance between two sub-shapes of `id`: kind 0 face, 1 edge, 2 body (by
/// index). Returns [value, pa xyz, pb xyz].
std::vector<double> mwDistance(OcctKernel& k, uint32_t id, int kindA, int idxA, int kindB, int idxB) {
    return guarded("distance", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        auto pick = [&](int kind, int idx) -> TopoDS_Shape {
            if (kind == 0) return pickFaces(s, {idx}, "distance")[0];
            if (kind == 1) return pickEdges(s, {idx}, "distance")[0];
            std::vector<TopoDS_Shape> bodies;
            int nb = bodyCount(s, bodies);
            if (idx < 0 || idx >= nb) throw std::runtime_error("distance: no body " + std::to_string(idx));
            return bodies[idx];
        };
        BRepExtrema_DistShapeShape d(pick(kindA, idxA), pick(kindB, idxB));
        if (!d.IsDone() || d.NbSolution() < 1) throw std::runtime_error("distance: no solution");
        gp_Pnt a = d.PointOnShape1(1), b = d.PointOnShape2(1);
        return std::vector<double>{d.Value(), a.X(), a.Y(), a.Z(), b.X(), b.Y(), b.Z()};
    });
}

/// Body `index` of `id` as its own shape.
uint32_t mwBody(OcctKernel& k, uint32_t id, int index) {
    return guarded("body", [&] {
        std::vector<TopoDS_Shape> bodies;
        int nb = bodyCount(MwEdit::get(k, id), bodies);
        if (index < 0 || index >= nb) throw std::runtime_error("body: no body " + std::to_string(index));
        return MwEdit::store(k, bodies[index]);
    });
}

// ===========================================================================
// Operations with index history
// ===========================================================================

/// Remove faces and let the neighbours grow over the gap (OCCT defeaturing).
EvolutionData mwDefeature(OcctKernel& k, uint32_t id, std::vector<int> faces) {
    return guarded("deleteFaces", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        BRepAlgoAPI_Defeaturing d;
        d.SetShape(s);
        ShapeList rm;
        for (const TopoDS_Face& f : pickFaces(s, faces, "deleteFaces")) rm.Append(f);
        d.AddFacesToRemove(rm);
        d.SetRunParallel(false);
        d.SetToFillHistory(true);
        d.Build();
        if (!d.IsDone() || d.HasErrors())
            throw std::runtime_error("deleteFaces: the neighbouring faces cannot close the gap" +
                                     refList('F', faces));
        TopoDS_Shape r = unwrap(d.Shape());
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, d.History());
    });
}

namespace {
/// A face spanning a closed wire: planar where the wire is planar, else an
/// N-sided filling tangent (G1) to the faces the boundary edges belong to.
TopoDS_Face capWire(const TopoDS_Wire& w, const AncestorMap& anc) {
    BRepBuilderAPI_MakeFace planar(w, true);
    if (planar.IsDone()) return planar.Face();
    BRepOffsetAPI_MakeFilling fill;
    for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(ex.Current());
        int ai = anc.FindIndex(e);
        if (ai > 0 && anc.FindFromIndex(ai).Size() == 1)
            fill.Add(e, TopoDS::Face(anc.FindFromIndex(ai).First()), GeomAbs_G1);
        else
            fill.Add(e, GeomAbs_C0);
    }
    fill.Build();
    if (!fill.IsDone()) throw std::runtime_error("fillHole: the filling surface failed");
    return TopoDS::Face(fill.Shape());
}

/// Sew `faces` into `base` (an open shell or a solid's shell) and close what closes.
EvolutionData sewIn(OcctKernel& k, const TopoDS_Shape& base, const std::vector<TopoDS_Face>& faces,
                    const std::vector<TopoDS_Shape>& historyInputs, const char* op) {
    double tol = std::max(1e-6, 1e-6 * bboxDiag(base));
    BRepBuilderAPI_Sewing sew(tol);
    sew.Add(base);
    for (const TopoDS_Face& f : faces) sew.Add(f);
    sew.Perform();
    TopoDS_Shape r = solidify(sew.SewedShape());
    Handle(BRepTools_History) h = sewingHistory(historyInputs, sew);
    uint32_t rid = MwEdit::store(k, r);
    (void)op;
    return evolution(rid, historyInputs, r, h);
}
} // namespace

/// Remove faces and leave the opening (cap=false) or close each opening with a
/// new face (cap=true).
EvolutionData mwRemoveFaces(OcctKernel& k, uint32_t id, std::vector<int> faces, bool cap) {
    return guarded("deleteFaces", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        std::vector<TopoDS_Face> rm = pickFaces(s, faces, "deleteFaces");
        ShapeMap rmSet;
        for (const TopoDS_Face& f : rm) rmSet.Add(f);
        BRep_Builder b;
        TopoDS_Compound shells;
        b.MakeCompound(shells);
        std::vector<TopoDS_Shape> bodies;
        bodyCount(s, bodies);
        for (const TopoDS_Shape& body : bodies) {
            TopoDS_Shell sh;
            b.MakeShell(sh);
            int n = 0;
            for (TopExp_Explorer ex(body, TopAbs_FACE); ex.More(); ex.Next())
                if (!rmSet.Contains(ex.Current())) { b.Add(sh, ex.Current()); ++n; }
            if (n) b.Add(shells, sh);
        }
        TopoDS_Shape open = unwrap(shells);
        if (!cap) {
            uint32_t rid = MwEdit::store(k, open);
            return evolution(rid, {s}, open, nullptr);
        }
        ShapeAnalysis_FreeBounds fb(open, std::max(1e-7, 1e-7 * bboxDiag(open)), false, false);
        AncestorMap anc;
        TopExp::MapShapesAndUniqueAncestors(open, TopAbs_EDGE, TopAbs_FACE, anc);
        std::vector<TopoDS_Face> caps;
        for (TopExp_Explorer ex(fb.GetClosedWires(), TopAbs_WIRE); ex.More(); ex.Next())
            caps.push_back(capWire(TopoDS::Wire(ex.Current()), anc));
        if (caps.empty()) throw std::runtime_error("deleteFaces: no closed opening to cap" + refList('F', faces));
        // History is against the original shape: removed faces are not in the
        // sewing input, so they read as deleted.
        return sewIn(k, open, caps, {s}, "deleteFaces");
    });
}

/// Close an opening bounded by the given edges with a new face and sew it in.
EvolutionData mwFillHole(OcctKernel& k, uint32_t id, std::vector<int> edges) {
    return guarded("fillHole", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        Handle(NCollection_HSequence<TopoDS_Shape>) seq = new NCollection_HSequence<TopoDS_Shape>;
        for (const TopoDS_Edge& e : pickEdges(s, edges, "fillHole")) seq->Append(e);
        Handle(NCollection_HSequence<TopoDS_Shape>) wires =
            ShapeAnalysis_FreeBounds::ConnectEdgesToWires(seq, 1e-6 * bboxDiag(s), false);
        AncestorMap anc;
        TopExp::MapShapesAndUniqueAncestors(s, TopAbs_EDGE, TopAbs_FACE, anc);
        std::vector<TopoDS_Face> caps;
        for (int i = 1; i <= wires->Length(); ++i) {
            TopoDS_Wire w = TopoDS::Wire(wires->Value(i));
            if (!BRep_Tool::IsClosed(w))
                throw std::runtime_error("fillHole: the picked edges do not close a loop" + refList('E', edges));
            caps.push_back(capWire(w, anc));
        }
        return sewIn(k, s, caps, {s}, "fillHole");
    });
}

/// Constant or per-edge radius fillet.
EvolutionData mwFillet(OcctKernel& k, uint32_t id, std::vector<int> edges, std::vector<double> radii) {
    return guarded("fillet", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        if (radii.empty() || (radii.size() != 1 && radii.size() != edges.size()))
            throw std::runtime_error("fillet: give one radius, or one per edge");
        std::vector<TopoDS_Edge> es = pickEdges(s, edges, "fillet");
        BRepFilletAPI_MakeFillet m(s);
        for (size_t i = 0; i < es.size(); ++i) m.Add(radii.size() == 1 ? radii[0] : radii[i], es[i]);
        // A radius that cannot fit can also throw from inside the blend; that is
        // the same failure, and it should name the edges too.
        try { m.Build(); } catch (const Standard_Failure&) {}
        if (!m.IsDone()) {
            ShapeMap em = edgeMap(s);
            std::vector<int> bad;
            for (int c = 1; c <= m.NbFaultyContours(); ++c) {
                int ic = m.FaultyContour(c);
                for (int j = 1; j <= m.NbEdges(ic); ++j) bad.push_back(em.FindIndex(m.Edge(ic, j)) - 1);
            }
            throw std::runtime_error("fillet: the radius does not fit" + refList('E', bad.empty() ? edges : bad));
        }
        TopoDS_Shape r = unwrap(m.Shape());
        if (!isValid(r)) throw std::runtime_error("fillet: the result is not a valid solid" + refList('E', edges));
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, historyOf({s}, m));
    });
}

/// Equal (d2 <= 0) or two-distance chamfer; d1 is measured on the edge's first face.
EvolutionData mwChamfer(OcctKernel& k, uint32_t id, std::vector<int> edges, double d1, double d2) {
    return guarded("chamfer", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        AncestorMap anc;
        TopExp::MapShapesAndUniqueAncestors(s, TopAbs_EDGE, TopAbs_FACE, anc);
        BRepFilletAPI_MakeChamfer m(s);
        for (const TopoDS_Edge& e : pickEdges(s, edges, "chamfer")) {
            if (d2 > 0) m.Add(d1, d2, e, TopoDS::Face(anc.FindFromKey(e).First()));
            else m.Add(d1, e);
        }
        try { m.Build(); } catch (const Standard_Failure&) {}
        if (!m.IsDone()) throw std::runtime_error("chamfer: the distance does not fit" + refList('E', edges));
        TopoDS_Shape r = unwrap(m.Shape());
        if (!isValid(r)) throw std::runtime_error("chamfer: the result is not a valid solid" + refList('E', edges));
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, historyOf({s}, m));
    });
}

/// Hollow with walls of `thickness` (material inside the current skin). Open
/// faces are removed; with none the part becomes a closed hollow.
EvolutionData mwShell(OcctKernel& k, uint32_t id, std::vector<int> openFaces, double thickness) {
    return guarded("shell", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        double tol = std::max(1e-6, 1e-6 * bboxDiag(s));
        if (openFaces.empty()) {
            BRepOffsetAPI_MakeOffsetShape off;
            off.PerformByJoin(s, -thickness, tol);
            if (!off.IsDone()) throw std::runtime_error("shell: the inner skin failed");
            BRepAlgoAPI_Cut cut(s, off.Shape());
            if (!cut.IsDone() || cut.HasErrors()) throw std::runtime_error("shell: the hollow failed");
            TopoDS_Shape r = unwrap(cut.Shape());
            uint32_t rid = MwEdit::store(k, r);
            return evolution(rid, {s}, r, historyOf({s}, cut));
        }
        ShapeList open;
        for (const TopoDS_Face& f : pickFaces(s, openFaces, "shell")) open.Append(f);
        BRepOffsetAPI_MakeThickSolid m;
        m.MakeThickSolidByJoin(s, open, -thickness, tol);
        if (!m.IsDone()) throw std::runtime_error("shell: the wall does not fit" + refList('F', openFaces));
        TopoDS_Shape r = unwrap(m.Shape());
        if (!isValid(r)) throw std::runtime_error("shell: the result is not a valid solid" + refList('F', openFaces));
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, historyOf({s}, m));
    });
}

/// Push/pull planar faces by `distance` along their outward normals (a prism of
/// each face, fused when > 0, cut when < 0). The moved face keeps its id.
EvolutionData mwOffsetFaces(OcctKernel& k, uint32_t id, std::vector<int> faces, double distance) {
    return guarded("offsetFaces", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        if (distance == 0) throw std::runtime_error("offsetFaces: distance is 0");
        std::vector<TopoDS_Face> fs = pickFaces(s, faces, "offsetFaces");
        ShapeList tools;
        std::vector<std::unique_ptr<BRepPrimAPI_MakePrism>> prisms;
        std::map<int, std::vector<TopoDS_Shape>> extra;
        for (size_t i = 0; i < fs.size(); ++i) {
            gp_Dir n = planarNormal(fs[i], "offsetFaces", faces[i]);
            gp_Vec v = gp_Vec(n) * distance;
            prisms.emplace_back(new BRepPrimAPI_MakePrism(fs[i], v, true));
            if (!prisms.back()->IsDone()) throw std::runtime_error("offsetFaces: prism failed" + refList('F', {faces[i]}));
            tools.Append(prisms.back()->Shape());
            extra[faces[i]] = {prisms.back()->LastShape(fs[i])};
        }
        ShapeList args;
        args.Append(s);
        std::unique_ptr<BRepAlgoAPI_BooleanOperation> op;
        if (distance > 0) op.reset(new BRepAlgoAPI_Fuse); else op.reset(new BRepAlgoAPI_Cut);
        op->SetArguments(args);
        op->SetTools(tools);
        op->Build();
        if (!op->IsDone() || op->HasErrors()) throw std::runtime_error("offsetFaces: the boolean failed" + refList('F', faces));
        TopoDS_Shape r = unwrap(op->Shape());
        uint32_t rid = MwEdit::store(k, r);
        std::vector<TopoDS_Shape> ins{s};
        for (const TopoDS_Shape& t : tools) ins.push_back(t);
        Handle(BRepTools_History) h = historyOf(ins, *op);
        return evolution(rid, {s}, r, h, extra);
    });
}

/// Draft faces about a neutral plane; the pull direction is the plane normal.
EvolutionData mwDraft(OcctKernel& k, uint32_t id, std::vector<int> faces, double ox, double oy,
                      double oz, double nx, double ny, double nz, double angleDeg) {
    return guarded("draft", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        gp_Dir n(nx, ny, nz);
        gp_Pln neutral(gp_Pnt(ox, oy, oz), n);
        BRepOffsetAPI_DraftAngle d(s);
        std::vector<TopoDS_Face> fs = pickFaces(s, faces, "draft");
        for (size_t i = 0; i < fs.size(); ++i) {
            d.Add(fs[i], n, angleDeg * M_PI / 180.0, neutral);
            if (!d.AddDone()) throw std::runtime_error("draft: this face cannot take the draft" + refList('F', {faces[i]}));
        }
        d.Build();
        if (!d.IsDone()) throw std::runtime_error("draft: failed" + refList('F', faces));
        TopoDS_Shape r = unwrap(d.Shape());
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, historyOf({s}, d));
    });
}

/// op 0 union, 1 subtract (a - b), 2 intersect. Input faces: a's, then b's.
EvolutionData mwBoolean(OcctKernel& k, int op, uint32_t a, uint32_t b) {
    return guarded("boolean", [&] {
        const TopoDS_Shape& sa = MwEdit::get(k, a);
        const TopoDS_Shape& sb = MwEdit::get(k, b);
        std::unique_ptr<BRepAlgoAPI_BooleanOperation> m;
        if (op == 0) m.reset(new BRepAlgoAPI_Fuse);
        else if (op == 1) m.reset(new BRepAlgoAPI_Cut);
        else if (op == 2) m.reset(new BRepAlgoAPI_Common);
        else throw std::runtime_error("boolean: op must be 0, 1 or 2");
        ShapeList args, tools;
        args.Append(sa);
        tools.Append(sb);
        m->SetArguments(args);
        m->SetTools(tools);
        m->Build();
        if (!m->IsDone() || m->HasErrors()) throw std::runtime_error("boolean: failed");
        TopoDS_Shape r = unwrap(m->Shape());
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {sa, sb}, r, historyOf({sa, sb}, *m));
    });
}

/// Split by a plane; keep 0 both, 1 the side the normal points to, 2 the other.
EvolutionData mwSplitPlane(OcctKernel& k, uint32_t id, double ox, double oy, double oz, double nx,
                           double ny, double nz, int keep) {
    return guarded("splitByPlane", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        gp_Pln pln(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz));
        BRepAlgoAPI_Splitter sp;
        ShapeList args, tools;
        args.Append(s);
        tools.Append(cuttingFace(s, pln));
        sp.SetArguments(args);
        sp.SetTools(tools);
        sp.Build();
        if (!sp.IsDone() || sp.HasErrors()) throw std::runtime_error("splitByPlane: failed");
        TopoDS_Shape r = sp.Shape();
        if (keep != 0) {
            BRep_Builder b;
            TopoDS_Compound kept;
            b.MakeCompound(kept);
            int n = 0;
            std::vector<TopoDS_Shape> bodies;
            bodyCount(r, bodies);
            for (const TopoDS_Shape& body : bodies) {
                GProp_GProps g;
                if (body.ShapeType() == TopAbs_SOLID) BRepGProp::VolumeProperties(body, g);
                else BRepGProp::SurfaceProperties(body, g);
                double side = gp_Vec(pln.Location(), g.CentreOfMass()).Dot(gp_Vec(pln.Axis().Direction()));
                if ((keep == 1 && side > 0) || (keep == 2 && side < 0)) { b.Add(kept, body); ++n; }
            }
            if (n == 0) throw std::runtime_error("splitByPlane: nothing of the part is on the kept side");
            r = kept;
        }
        r = unwrap(r);
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, sp.History());
    });
}

/// Split the given faces where they meet `toolId` (any shape) or, when toolId
/// is 0, the plane (o, n). The body stays one body; only the faces divide.
EvolutionData mwSplitFaces(OcctKernel& k, uint32_t id, std::vector<int> faces, uint32_t toolId,
                           double ox, double oy, double oz, double nx, double ny, double nz) {
    return guarded("splitFaces", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        TopoDS_Shape tool = toolId ? MwEdit::get(k, toolId)
                                   : TopoDS_Shape(cuttingFace(s, gp_Pln(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz))));
        BRep_Builder b;
        TopoDS_Compound cutEdges;
        b.MakeCompound(cutEdges);
        int nEdges = 0;
        std::vector<TopoDS_Face> fs = pickFaces(s, faces, "splitFaces");
        for (size_t i = 0; i < fs.size(); ++i) {
            BRepAlgoAPI_Section sec(fs[i], tool, false);
            sec.Approximation(true);
            sec.Build();
            if (!sec.IsDone()) throw std::runtime_error("splitFaces: section failed" + refList('F', {faces[i]}));
            int here = 0;
            for (TopExp_Explorer ex(sec.Shape(), TopAbs_EDGE); ex.More(); ex.Next()) { b.Add(cutEdges, ex.Current()); ++here; }
            if (!here) throw std::runtime_error("splitFaces: the tool does not cross this face" + refList('F', {faces[i]}));
            nEdges += here;
        }
        BRepAlgoAPI_Splitter sp;
        ShapeList args, tools;
        args.Append(s);
        tools.Append(cutEdges);
        sp.SetArguments(args);
        sp.SetTools(tools);
        sp.Build();
        if (!sp.IsDone() || sp.HasErrors()) throw std::runtime_error("splitFaces: failed" + refList('F', faces));
        TopoDS_Shape r = unwrap(sp.Shape());
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, sp.History());
    });
}

/// Merge faces that lie on the same surface (and edges on the same curve).
EvolutionData mwUnify(OcctKernel& k, uint32_t id) {
    return guarded("unifyFaces", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        ShapeUpgrade_UnifySameDomain u(s, true, true, false);
        u.Build();
        TopoDS_Shape r = u.Shape();
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, u.History());
    });
}

/// Sew open seams within `tol`, then ShapeFix the result at that precision.
EvolutionData mwHeal(OcctKernel& k, uint32_t id, double tol) {
    return guarded("heal", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        BRepBuilderAPI_Sewing sew(tol);
        sew.Add(s);
        sew.Perform();
        TopoDS_Shape sewn = solidify(sew.SewedShape());
        Handle(BRepTools_History) h = sewingHistory({s}, sew);
        ShapeFix_Shape fix(sewn);
        fix.SetPrecision(tol);
        fix.SetMaxTolerance(std::max(tol * 10, 1e-3));
        fix.Perform();
        TopoDS_Shape r = unwrap(fix.Shape());
        Handle(BRepTools_History) fh = fix.Context()->History();
        if (!fh.IsNull()) h->Merge(fh);
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, h);
    });
}

/// Apply a row-major 3x4 matrix (rotation/scale part + translation column).
EvolutionData mwTransform(OcctKernel& k, uint32_t id, std::vector<double> m) {
    return guarded("transform", [&] {
        if (m.size() != 12) throw std::runtime_error("transform: the matrix needs 12 numbers");
        const TopoDS_Shape& s = MwEdit::get(k, id);
        gp_Trsf t;
        bool rigid = true;
        try {
            t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
        } catch (const Standard_Failure&) {
            rigid = false;
        }
        TopoDS_Shape r;
        Handle(BRepTools_History) h;
        if (rigid) {
            BRepBuilderAPI_Transform tr(s, t, true);
            r = tr.Shape();
            h = historyOf({s}, tr);
        } else {
            gp_GTrsf g;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 4; ++j) g.SetValue(i + 1, j + 1, m[i * 4 + j]);
            BRepBuilderAPI_GTransform tr(s, g, true);
            r = tr.Shape();
            h = historyOf({s}, tr);
        }
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, h);
    });
}

/// The mirror image of `id` in the plane (o, n) (the copy only).
EvolutionData mwMirror(OcctKernel& k, uint32_t id, double ox, double oy, double oz, double nx,
                       double ny, double nz) {
    return guarded("mirror", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        gp_Trsf t;
        t.SetMirror(gp_Ax2(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz)));
        BRepBuilderAPI_Transform tr(s, t, true);
        TopoDS_Shape r = tr.Shape();
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, historyOf({s}, tr));
    });
}

/// The original fused with count-1 copies. kind 0 linear (a = dir xyz, p =
/// spacing), 1 circular (o = axis origin, a = axis dir, p = total angle deg).
EvolutionData mwPattern(OcctKernel& k, uint32_t id, int kind, double ox, double oy, double oz,
                        double ax, double ay, double az, double p, int count) {
    return guarded("pattern", [&] {
        const TopoDS_Shape& s = MwEdit::get(k, id);
        if (count < 2) throw std::runtime_error("pattern: count must be at least 2");
        ShapeList args, tools;
        args.Append(s);
        gp_Dir a(ax, ay, az);
        for (int i = 1; i < count; ++i) {
            gp_Trsf t;
            if (kind == 0) t.SetTranslation(gp_Vec(a) * (p * i));
            else {
                bool full = std::abs(std::abs(p) - 360.0) < 1e-9;
                double step = p / (full ? count : (count - 1));
                t.SetRotation(gp_Ax1(gp_Pnt(ox, oy, oz), a), step * i * M_PI / 180.0);
            }
            tools.Append(BRepBuilderAPI_Transform(s, t, true).Shape());
        }
        BRepAlgoAPI_Fuse fu;
        fu.SetArguments(args);
        fu.SetTools(tools);
        fu.Build();
        if (!fu.IsDone() || fu.HasErrors()) throw std::runtime_error("pattern: the union failed");
        TopoDS_Shape r = unwrap(fu.Shape());
        uint32_t rid = MwEdit::store(k, r);
        return evolution(rid, {s}, r, historyOf({s}, fu));
    });
}

namespace {
TopoDS_Shape asFace(const TopoDS_Shape& p) {
    if (p.ShapeType() == TopAbs_WIRE) {
        BRepBuilderAPI_MakeFace mf(TopoDS::Wire(p), true);
        if (!mf.IsDone()) throw std::runtime_error("profile: the wire is not a closed planar loop");
        return mf.Face();
    }
    return p;
}
} // namespace

/// Extrude a profile (face, wire or compound of faces) by vector v; symmetric
/// centres the solid on the profile (total length |v|). Every face is new.
EvolutionData mwExtrude(OcctKernel& k, uint32_t profile, double vx, double vy, double vz, bool symmetric) {
    return guarded("extrude", [&] {
        TopoDS_Shape p = asFace(MwEdit::get(k, profile));
        gp_Vec v(vx, vy, vz);
        if (symmetric) {
            gp_Trsf t;
            t.SetTranslation(v * -0.5);
            p = BRepBuilderAPI_Transform(p, t, true).Shape();
        }
        BRepPrimAPI_MakePrism m(p, v, true);
        if (!m.IsDone()) throw std::runtime_error("extrude: failed");
        TopoDS_Shape r = unwrap(m.Shape());
        uint32_t rid = MwEdit::store(k, r);
        EvolutionData e;
        e.resultId = rid;
        return e;
    });
}

EvolutionData mwRevolve(OcctKernel& k, uint32_t profile, double ox, double oy, double oz, double dx,
                        double dy, double dz, double angleDeg) {
    return guarded("revolve", [&] {
        TopoDS_Shape p = asFace(MwEdit::get(k, profile));
        gp_Ax1 ax(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz));
        std::unique_ptr<BRepPrimAPI_MakeRevol> m;
        if (std::abs(angleDeg) >= 360.0) m.reset(new BRepPrimAPI_MakeRevol(p, ax, true));
        else m.reset(new BRepPrimAPI_MakeRevol(p, ax, angleDeg * M_PI / 180.0, true));
        if (!m->IsDone()) throw std::runtime_error("revolve: failed");
        TopoDS_Shape r = unwrap(m->Shape());
        uint32_t rid = MwEdit::store(k, r);
        EvolutionData e;
        e.resultId = rid;
        return e;
    });
}

/// A planar face from a closed polyline (xyz triples): the test and sketch path.
uint32_t mwPolygonFace(OcctKernel& k, std::vector<double> xyz) {
    return guarded("polygonFace", [&] {
        if (xyz.size() < 9 || xyz.size() % 3) throw std::runtime_error("polygonFace: need 3+ points");
        BRepBuilderAPI_MakePolygon poly;
        for (size_t i = 0; i < xyz.size(); i += 3) poly.Add(gp_Pnt(xyz[i], xyz[i + 1], xyz[i + 2]));
        poly.Close();
        BRepBuilderAPI_MakeFace mf(poly.Wire(), true);
        if (!mf.IsDone()) throw std::runtime_error("polygonFace: not a planar loop");
        return MwEdit::store(k, mf.Face());
    });
}

EMSCRIPTEN_BINDINGS(meshweave_edit) {
    using namespace emscripten;
    function("mwTopology", &mwTopology);
    function("mwMesh", &mwMesh);
    function("mwMeshFree", &mwMeshFree);
    function("mwMeasure", &mwMeasure);
    function("mwDistance", &mwDistance);
    function("mwBody", &mwBody);
    function("mwDefeature", &mwDefeature);
    function("mwRemoveFaces", &mwRemoveFaces);
    function("mwFillHole", &mwFillHole);
    function("mwFillet", &mwFillet);
    function("mwChamfer", &mwChamfer);
    function("mwShell", &mwShell);
    function("mwOffsetFaces", &mwOffsetFaces);
    function("mwDraft", &mwDraft);
    function("mwBoolean", &mwBoolean);
    function("mwSplitPlane", &mwSplitPlane);
    function("mwSplitFaces", &mwSplitFaces);
    function("mwUnify", &mwUnify);
    function("mwHeal", &mwHeal);
    function("mwTransform", &mwTransform);
    function("mwMirror", &mwMirror);
    function("mwPattern", &mwPattern);
    function("mwExtrude", &mwExtrude);
    function("mwRevolve", &mwRevolve);
    function("mwPolygonFace", &mwPolygonFace);
}
