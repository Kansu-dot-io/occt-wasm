// MeshWeave additions: generic OCCT geometry/topology bindings for MeshWeave's
// device-side readers (MIT OR Apache-2.0, like the rest of the facade). OCCT
// itself is not modified.
//
// What this exposes is plain OCCT, under OCCT's own class and method names, so
// code written against the usual Python binding of OCCT (OCP) can be ported to
// JavaScript nearly word for word:
//  - gp value types (gp_Pnt, gp_Dir, gp_Vec, gp_Ax1/2/3, 2-D counterparts, gp_Trsf),
//    TopLoc_Location, Bnd_Box, GProp_GProps, the TColgp/TColStd arrays;
//  - Geom / Geom2d curves and surfaces as `opencascade::handle<T>` smart pointers
//    (embind downcasts a returned handle to its most derived bound class, so a
//    Geom_Plane is usable wherever a Geom_Surface is, and vice versa);
//  - TopoDS shapes, BRep_Builder, BRep_Tool, TopExp, the BRepBuilderAPI makers,
//    sewing, ShapeFix_Face, projections, intersections, interpolation, mass
//    properties, BRepMesh and Poly_Triangulation for reading triangles back.
//
// Conventions (the JS side relies on them):
//  - static member functions are bound with OCP's `_s` suffix (BRep_Tool.Pnt_s);
//  - out-parameters a Python binding returns as a tuple come back as a JS array
//    in the same order (BRep_Tool.Range_s -> [first, last], Bnd_Box.Get -> 6);
//  - where two C++ overloads take the same number of arguments, the extra ones
//    are bound under `<Name>__<tag>` (constructors as static `__new_<tag>`) and
//    the JS wrapper picks one by argument type;
//  - functions taking a specific TopoDS subtype accept any TopoDS_Shape and cast
//    with TopoDS:: (which throws Standard_TypeMismatch on a wrong type);
//  - TopoDS_Shape.TShape() returns the TShape's address as a number, usable as
//    a Map/Set key for "the same underlying shape";
//  - mwWhat(thrower) recovers the message of a C++ exception (Standard_Failure
//    or std::exception) that reached JS: the thrower rethrows it into C++.
//  - mwStoreShape / mwGetShape move a shape into / out of the kernel's arena.

#include "mw_access.h"

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <Adaptor3d_Curve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_EdgeError.hxx>
#include <BRepBuilderAPI_FaceError.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GC_MakeSegment.hxx>
#include <GProp_GProps.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2dConvert.hxx>
#include <Geom2dConvert_CompCurveToBSplineCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BoundedCurve.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Geometry.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomConvert.hxx>
#include <GeomConvert_ApproxSurface.hxx>
#include <GeomConvert_CompCurveToBSplineCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BoundedCurve.hxx>
#include <Geom_BoundedSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Conic.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Geometry.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_SweptSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Transient.hxx>
#include <Standard_Type.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_HArray1.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_Map.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_State.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Datum3D.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_CompSolid.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>

#include <cstdint>
#include <cstdlib>
#include <cxxabi.h>
#include <typeinfo>
#include <exception>
#include <string>

// OCCT 8 retired the TColgp/TColStd/TopTools alias headers (deprecated, not
// installed); the classes keep those names here and on the JS side.
using TColgp_Array1OfPnt = NCollection_Array1<gp_Pnt>;
using TColgp_Array1OfPnt2d = NCollection_Array1<gp_Pnt2d>;
using TColStd_Array1OfReal = NCollection_Array1<double>;
using TColStd_Array1OfInteger = NCollection_Array1<int>;
using TColgp_Array2OfPnt = NCollection_Array2<gp_Pnt>;
using TColStd_Array2OfReal = NCollection_Array2<double>;
using TColgp_HArray1OfPnt = NCollection_HArray1<gp_Pnt>;
using TColgp_HArray1OfPnt2d = NCollection_HArray1<gp_Pnt2d>;
using TColStd_HArray1OfReal = NCollection_HArray1<double>;
using TopTools_MapOfShape = NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>;
using TopTools_IndexedMapOfShape = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

// opencascade::handle<T> as an embind smart pointer. The reference count lives
// in the object (Standard_Transient), so sharing is intrusive: a handle of a
// base type and one of a derived type can point at the same object.
namespace emscripten {
template <typename T> struct smart_ptr_trait<opencascade::handle<T>> {
    using PointerType = opencascade::handle<T>;
    using element_type = T;
    static T* get(const PointerType& p) { return p.get(); }
    static sharing_policy get_sharing_policy() { return sharing_policy::INTRUSIVE; }
    static PointerType* share(T* p, EM_VAL) { return new PointerType(p); }
    static PointerType* construct_null() { return new PointerType(); }
};
} // namespace emscripten

using namespace emscripten;
#define H(T) opencascade::handle<T>

namespace {

val arr() { return val::array(); }
template <class... A> val arrOf(A... xs) {
    val a = val::array();
    (a.call<void>("push", val(xs)), ...);
    return a;
}

double keyOf(const TopoDS_Shape& s) {
    return static_cast<double>(reinterpret_cast<std::uintptr_t>(s.TShape().get()));
}

// A stable identity for "the same shape" as TopoDS_Shape::IsEqual sees it: the
// TShape, the location (its chain of datum/power items, which is what
// TopLoc_Location equality compares) and the orientation.
std::string shapeKey(const TopoDS_Shape& s) {
    std::string k = std::to_string(reinterpret_cast<std::uintptr_t>(s.TShape().get()));
    for (TopLoc_Location l = s.Location(); !l.IsIdentity(); l = l.NextLocation()) {
        k += ":" + std::to_string(reinterpret_cast<std::uintptr_t>(l.FirstDatum().get())) + "^" +
             std::to_string(l.FirstPower());
    }
    k += "/" + std::to_string(static_cast<int>(s.Orientation()));
    return k;
}

std::string what(val thrower) {
    try {
        thrower();
    } catch (const Standard_Failure& f) {
        const char* m = f.what();
        int st = 0;
        char* dm = abi::__cxa_demangle(typeid(f).name(), nullptr, nullptr, &st);
        std::string name = (st == 0 && dm) ? dm : "Standard_Failure";
        std::free(dm);
        return (m && *m) ? name + ": " + m : name;
    } catch (const std::exception& e) {
        return e.what();
    } catch (...) {
        return "unknown C++ exception";
    }
    return "";
}

uint32_t storeShape(OcctKernel& k, const TopoDS_Shape& s) { return MwEdit::store(k, s); }
TopoDS_Shape getShape(OcctKernel& k, uint32_t id) { return MwEdit::get(k, id); }

// Static-only OCCT holders (some are namespaces in OCCT 8) are bound as empty
// classes carrying the `_s` functions.
template <int N> struct Statics {};

std::string typeName(const Standard_Transient& t) { return t.DynamicType()->Name(); }

} // namespace

// ── gp ────────────────────────────────────────────────────────────────────────
static void bindGp() {
    class_<gp_Pnt>("gp_Pnt")
        .constructor<>()
        .constructor<double, double, double>()
        .function("X", &gp_Pnt::X)
        .function("Y", &gp_Pnt::Y)
        .function("Z", &gp_Pnt::Z)
        .function("SetX", &gp_Pnt::SetX)
        .function("SetY", &gp_Pnt::SetY)
        .function("SetZ", &gp_Pnt::SetZ)
        .function("SetCoord", +[](gp_Pnt& p, double x, double y, double z) { p.SetCoord(x, y, z); })
        .function("Coord", +[](const gp_Pnt& p) { return arrOf(p.X(), p.Y(), p.Z()); })
        .function("Distance", &gp_Pnt::Distance)
        .function("SquareDistance", &gp_Pnt::SquareDistance)
        .function("IsEqual", &gp_Pnt::IsEqual)
        .function("Transformed", &gp_Pnt::Transformed)
        .function("Transform", &gp_Pnt::Transform)
        .function("Translated", +[](const gp_Pnt& p, const gp_Vec& v) { return p.Translated(v); })
        .function("Translated", +[](const gp_Pnt& p, const gp_Pnt& a, const gp_Pnt& b) { return p.Translated(a, b); })
        .function("Translate", +[](gp_Pnt& p, const gp_Vec& v) { p.Translate(v); })
        .function("Rotated", &gp_Pnt::Rotated)
        .function("Scaled", &gp_Pnt::Scaled);

    class_<gp_Pnt2d>("gp_Pnt2d")
        .constructor<>()
        .constructor<double, double>()
        .function("X", &gp_Pnt2d::X)
        .function("Y", &gp_Pnt2d::Y)
        .function("SetX", &gp_Pnt2d::SetX)
        .function("SetY", &gp_Pnt2d::SetY)
        .function("SetCoord", +[](gp_Pnt2d& p, double x, double y) { p.SetCoord(x, y); })
        .function("Coord", +[](const gp_Pnt2d& p) { return arrOf(p.X(), p.Y()); })
        .function("Distance", &gp_Pnt2d::Distance)
        .function("SquareDistance", &gp_Pnt2d::SquareDistance)
        .function("IsEqual", &gp_Pnt2d::IsEqual)
        .function("Translated", +[](const gp_Pnt2d& p, const gp_Vec2d& v) { return p.Translated(v); })
        .function("Translate", +[](gp_Pnt2d& p, const gp_Vec2d& v) { p.Translate(v); });

    class_<gp_Vec>("gp_Vec")
        .constructor<>()
        .constructor<double, double, double>()
        .constructor<const gp_Dir&>()
        .constructor<const gp_Pnt&, const gp_Pnt&>()
        .function("X", &gp_Vec::X)
        .function("Y", &gp_Vec::Y)
        .function("Z", &gp_Vec::Z)
        .function("SetCoord", +[](gp_Vec& v, double x, double y, double z) { v.SetCoord(x, y, z); })
        .function("Coord", +[](const gp_Vec& v) { return arrOf(v.X(), v.Y(), v.Z()); })
        .function("Magnitude", &gp_Vec::Magnitude)
        .function("SquareMagnitude", &gp_Vec::SquareMagnitude)
        .function("Normalize", &gp_Vec::Normalize)
        .function("Normalized", &gp_Vec::Normalized)
        .function("Reverse", &gp_Vec::Reverse)
        .function("Reversed", &gp_Vec::Reversed)
        .function("Dot", &gp_Vec::Dot)
        .function("Cross", &gp_Vec::Cross)
        .function("Crossed", &gp_Vec::Crossed)
        .function("CrossMagnitude", &gp_Vec::CrossMagnitude)
        .function("Add", &gp_Vec::Add)
        .function("Added", &gp_Vec::Added)
        .function("Subtract", &gp_Vec::Subtract)
        .function("Subtracted", &gp_Vec::Subtracted)
        .function("Multiply", &gp_Vec::Multiply)
        .function("Multiplied", &gp_Vec::Multiplied)
        .function("Divided", &gp_Vec::Divided)
        .function("Scale", &gp_Vec::Scale)
        .function("Scaled", &gp_Vec::Scaled)
        .function("Angle", &gp_Vec::Angle)
        .function("IsParallel", &gp_Vec::IsParallel)
        .function("Transformed", &gp_Vec::Transformed);

    class_<gp_Vec2d>("gp_Vec2d")
        .constructor<>()
        .constructor<double, double>()
        // 2 arguments: (x, y) is the constructor; (P1, P2) is __new_pp.
        .class_function("__new_pp", +[](const gp_Pnt2d& a, const gp_Pnt2d& b) { return gp_Vec2d(a, b); })
        .function("X", &gp_Vec2d::X)
        .function("Y", &gp_Vec2d::Y)
        .function("Magnitude", &gp_Vec2d::Magnitude)
        .function("SquareMagnitude", &gp_Vec2d::SquareMagnitude)
        .function("Normalize", &gp_Vec2d::Normalize)
        .function("Normalized", &gp_Vec2d::Normalized)
        .function("Reversed", &gp_Vec2d::Reversed)
        .function("Dot", &gp_Vec2d::Dot)
        .function("Crossed", &gp_Vec2d::Crossed)
        .function("Added", &gp_Vec2d::Added)
        .function("Subtracted", &gp_Vec2d::Subtracted)
        .function("Multiplied", &gp_Vec2d::Multiplied)
        .function("Scaled", &gp_Vec2d::Scaled)
        .function("Angle", &gp_Vec2d::Angle);

    class_<gp_Dir>("gp_Dir")
        .constructor<>()
        .constructor<double, double, double>()
        .constructor<const gp_Vec&>()
        .function("X", &gp_Dir::X)
        .function("Y", &gp_Dir::Y)
        .function("Z", &gp_Dir::Z)
        .function("Coord", +[](const gp_Dir& d) { return arrOf(d.X(), d.Y(), d.Z()); })
        .function("Dot", &gp_Dir::Dot)
        .function("Crossed", &gp_Dir::Crossed)
        .function("Reverse", &gp_Dir::Reverse)
        .function("Reversed", &gp_Dir::Reversed)
        .function("Angle", &gp_Dir::Angle)
        .function("IsParallel", &gp_Dir::IsParallel)
        .function("IsEqual", &gp_Dir::IsEqual)
        .function("IsOpposite", &gp_Dir::IsOpposite)
        .function("IsNormal", &gp_Dir::IsNormal)
        .function("Transformed", &gp_Dir::Transformed);

    class_<gp_Dir2d>("gp_Dir2d")
        .constructor<>()
        .constructor<double, double>()
        .constructor<const gp_Vec2d&>()
        .function("X", &gp_Dir2d::X)
        .function("Y", &gp_Dir2d::Y)
        .function("Dot", &gp_Dir2d::Dot)
        .function("Reversed", &gp_Dir2d::Reversed)
        .function("Angle", &gp_Dir2d::Angle);

    class_<gp_Ax1>("gp_Ax1")
        .constructor<>()
        .constructor<const gp_Pnt&, const gp_Dir&>()
        .function("Location", +[](const gp_Ax1& a) { return a.Location(); })
        .function("Direction", +[](const gp_Ax1& a) { return a.Direction(); })
        .function("SetLocation", &gp_Ax1::SetLocation)
        .function("SetDirection", &gp_Ax1::SetDirection)
        .function("Reversed", &gp_Ax1::Reversed);

    class_<gp_Ax2>("gp_Ax2")
        .constructor<>()
        .constructor<const gp_Pnt&, const gp_Dir&>()
        .constructor<const gp_Pnt&, const gp_Dir&, const gp_Dir&>()
        .function("Location", +[](const gp_Ax2& a) { return a.Location(); })
        .function("Direction", +[](const gp_Ax2& a) { return a.Direction(); })
        .function("XDirection", +[](const gp_Ax2& a) { return a.XDirection(); })
        .function("YDirection", +[](const gp_Ax2& a) { return a.YDirection(); })
        .function("Axis", +[](const gp_Ax2& a) { return a.Axis(); });

    class_<gp_Ax3>("gp_Ax3")
        .constructor<>()
        .constructor<const gp_Ax2&>()
        .constructor<const gp_Pnt&, const gp_Dir&>()
        .constructor<const gp_Pnt&, const gp_Dir&, const gp_Dir&>()
        .function("Location", +[](const gp_Ax3& a) { return a.Location(); })
        .function("Direction", +[](const gp_Ax3& a) { return a.Direction(); })
        .function("XDirection", +[](const gp_Ax3& a) { return a.XDirection(); })
        .function("YDirection", +[](const gp_Ax3& a) { return a.YDirection(); })
        .function("Axis", +[](const gp_Ax3& a) { return a.Axis(); })
        .function("Ax2", &gp_Ax3::Ax2)
        .function("Direct", &gp_Ax3::Direct);

    class_<gp_Lin2d>("gp_Lin2d")
        .constructor<const gp_Pnt2d&, const gp_Dir2d&>()
        .function("Location", +[](const gp_Lin2d& l) { return l.Location(); })
        .function("Direction", +[](const gp_Lin2d& l) { return l.Direction(); })
        .function("Distance", +[](const gp_Lin2d& l, const gp_Pnt2d& p) { return l.Distance(p); });

    class_<gp_Circ>("gp_Circ")
        .function("Radius", &gp_Circ::Radius)
        .function("Location", +[](const gp_Circ& c) { return c.Location(); })
        .function("Position", +[](const gp_Circ& c) { return c.Position(); })
        .function("Axis", +[](const gp_Circ& c) { return c.Axis(); });

    class_<gp_Elips>("gp_Elips")
        .function("MajorRadius", &gp_Elips::MajorRadius)
        .function("MinorRadius", &gp_Elips::MinorRadius)
        .function("Location", +[](const gp_Elips& c) { return c.Location(); })
        .function("Position", +[](const gp_Elips& c) { return c.Position(); });

    class_<gp_Pln>("gp_Pln")
        .constructor<const gp_Ax3&>()
        .constructor<const gp_Pnt&, const gp_Dir&>()
        .function("Distance", +[](const gp_Pln& p, const gp_Pnt& q) { return p.Distance(q); })
        .function("Location", +[](const gp_Pln& p) { return p.Location(); })
        .function("Position", +[](const gp_Pln& p) { return p.Position(); })
        .function("Axis", +[](const gp_Pln& p) { return p.Axis(); });

    class_<gp_Trsf>("gp_Trsf")
        .constructor<>()
        .function("SetTranslation", +[](gp_Trsf& t, const gp_Vec& v) { t.SetTranslation(v); })
        .function("SetRotation", +[](gp_Trsf& t, const gp_Ax1& a, double ang) { t.SetRotation(a, ang); })
        .function("SetScale", +[](gp_Trsf& t, const gp_Pnt& p, double s) { t.SetScale(p, s); })
        .function("Multiplied", &gp_Trsf::Multiplied)
        .function("Inverted", &gp_Trsf::Inverted)
        .function("IsNegative", &gp_Trsf::IsNegative)
        .function("ScaleFactor", &gp_Trsf::ScaleFactor)
        .function("Value", &gp_Trsf::Value);

    class_<TopLoc_Location>("TopLoc_Location")
        .constructor<>()
        .constructor<const gp_Trsf&>()
        .function("IsIdentity", &TopLoc_Location::IsIdentity)
        .function("Transformation", +[](const TopLoc_Location& l) { return gp_Trsf(l.Transformation()); })
        .function("Inverted", &TopLoc_Location::Inverted)
        .function("Multiplied", &TopLoc_Location::Multiplied);
}

// ── enums ─────────────────────────────────────────────────────────────────────
static void bindEnums() {
    enum_<TopAbs_ShapeEnum>("TopAbs_ShapeEnum")
        .value("TopAbs_COMPOUND", TopAbs_COMPOUND)
        .value("TopAbs_COMPSOLID", TopAbs_COMPSOLID)
        .value("TopAbs_SOLID", TopAbs_SOLID)
        .value("TopAbs_SHELL", TopAbs_SHELL)
        .value("TopAbs_FACE", TopAbs_FACE)
        .value("TopAbs_WIRE", TopAbs_WIRE)
        .value("TopAbs_EDGE", TopAbs_EDGE)
        .value("TopAbs_VERTEX", TopAbs_VERTEX)
        .value("TopAbs_SHAPE", TopAbs_SHAPE);
    enum_<TopAbs_Orientation>("TopAbs_Orientation")
        .value("TopAbs_FORWARD", TopAbs_FORWARD)
        .value("TopAbs_REVERSED", TopAbs_REVERSED)
        .value("TopAbs_INTERNAL", TopAbs_INTERNAL)
        .value("TopAbs_EXTERNAL", TopAbs_EXTERNAL);
    enum_<TopAbs_State>("TopAbs_State")
        .value("TopAbs_IN", TopAbs_IN)
        .value("TopAbs_OUT", TopAbs_OUT)
        .value("TopAbs_ON", TopAbs_ON)
        .value("TopAbs_UNKNOWN", TopAbs_UNKNOWN);
    enum_<GeomAbs_Shape>("GeomAbs_Shape")
        .value("GeomAbs_C0", GeomAbs_C0)
        .value("GeomAbs_G1", GeomAbs_G1)
        .value("GeomAbs_C1", GeomAbs_C1)
        .value("GeomAbs_G2", GeomAbs_G2)
        .value("GeomAbs_C2", GeomAbs_C2)
        .value("GeomAbs_C3", GeomAbs_C3)
        .value("GeomAbs_CN", GeomAbs_CN);
    enum_<GeomAbs_SurfaceType>("GeomAbs_SurfaceType")
        .value("GeomAbs_Plane", GeomAbs_Plane)
        .value("GeomAbs_Cylinder", GeomAbs_Cylinder)
        .value("GeomAbs_Cone", GeomAbs_Cone)
        .value("GeomAbs_Sphere", GeomAbs_Sphere)
        .value("GeomAbs_Torus", GeomAbs_Torus)
        .value("GeomAbs_BezierSurface", GeomAbs_BezierSurface)
        .value("GeomAbs_BSplineSurface", GeomAbs_BSplineSurface)
        .value("GeomAbs_SurfaceOfRevolution", GeomAbs_SurfaceOfRevolution)
        .value("GeomAbs_SurfaceOfExtrusion", GeomAbs_SurfaceOfExtrusion)
        .value("GeomAbs_OffsetSurface", GeomAbs_OffsetSurface)
        .value("GeomAbs_OtherSurface", GeomAbs_OtherSurface);
    enum_<GeomAbs_CurveType>("GeomAbs_CurveType")
        .value("GeomAbs_Line", GeomAbs_Line)
        .value("GeomAbs_Circle", GeomAbs_Circle)
        .value("GeomAbs_Ellipse", GeomAbs_Ellipse)
        .value("GeomAbs_Hyperbola", GeomAbs_Hyperbola)
        .value("GeomAbs_Parabola", GeomAbs_Parabola)
        .value("GeomAbs_BezierCurve", GeomAbs_BezierCurve)
        .value("GeomAbs_BSplineCurve", GeomAbs_BSplineCurve)
        .value("GeomAbs_OffsetCurve", GeomAbs_OffsetCurve)
        .value("GeomAbs_OtherCurve", GeomAbs_OtherCurve);
    enum_<BRepBuilderAPI_EdgeError>("BRepBuilderAPI_EdgeError")
        .value("BRepBuilderAPI_EdgeDone", BRepBuilderAPI_EdgeDone)
        .value("BRepBuilderAPI_PointProjectionFailed", BRepBuilderAPI_PointProjectionFailed)
        .value("BRepBuilderAPI_ParameterOutOfRange", BRepBuilderAPI_ParameterOutOfRange)
        .value("BRepBuilderAPI_DifferentPointsOnClosedCurve", BRepBuilderAPI_DifferentPointsOnClosedCurve)
        .value("BRepBuilderAPI_PointWithInfiniteParameter", BRepBuilderAPI_PointWithInfiniteParameter)
        .value("BRepBuilderAPI_DifferentsPointAndParameter", BRepBuilderAPI_DifferentsPointAndParameter)
        .value("BRepBuilderAPI_LineThroughIdenticPoints", BRepBuilderAPI_LineThroughIdenticPoints);
    enum_<BRepBuilderAPI_FaceError>("BRepBuilderAPI_FaceError")
        .value("BRepBuilderAPI_FaceDone", BRepBuilderAPI_FaceDone)
        .value("BRepBuilderAPI_NoFace", BRepBuilderAPI_NoFace)
        .value("BRepBuilderAPI_NotPlanar", BRepBuilderAPI_NotPlanar)
        .value("BRepBuilderAPI_CurveProjectionFailed", BRepBuilderAPI_CurveProjectionFailed)
        .value("BRepBuilderAPI_ParametersOutOfRange", BRepBuilderAPI_ParametersOutOfRange);
}

// ── Standard_Transient, Geom, Geom2d (handles) ───────────────────────────────
static void bindGeom() {
    class_<Standard_Transient>("Standard_Transient")
        .smart_ptr<H(Standard_Transient)>("Handle_Standard_Transient")
        .function("DynamicType", +[](const Standard_Transient& t) { return H(Standard_Type)(t.DynamicType()); })
        .function("IsKind", +[](const Standard_Transient& t, std::string name) { return t.IsKind(name.c_str()); })
        .function("TypeName", &typeName);
    class_<Standard_Type, base<Standard_Transient>>("Standard_Type")
        .smart_ptr<H(Standard_Type)>("Handle_Standard_Type")
        .function("Name", +[](const Standard_Type& t) { return std::string(t.Name()); });

    class_<Geom_Geometry, base<Standard_Transient>>("Geom_Geometry")
        .smart_ptr<H(Geom_Geometry)>("Handle_Geom_Geometry")
        .function("Copy", +[](const Geom_Geometry& g) { return g.Copy(); })
        .function("Rotate", &Geom_Geometry::Rotate)
        .function("Rotated", &Geom_Geometry::Rotated)
        .function("Scale", &Geom_Geometry::Scale)
        .function("Scaled", &Geom_Geometry::Scaled)
        .function("Translate", +[](Geom_Geometry& g, const gp_Vec& v) { g.Translate(v); })
        .function("Translated", +[](const Geom_Geometry& g, const gp_Vec& v) { return g.Translated(v); })
        .function("Transform", &Geom_Geometry::Transform)
        .function("Transformed", &Geom_Geometry::Transformed);

    class_<Geom_Surface, base<Geom_Geometry>>("Geom_Surface")
        .smart_ptr<H(Geom_Surface)>("Handle_Geom_Surface")
        .function("Value", &Geom_Surface::Value)
        .function("D0", &Geom_Surface::D0)
        .function("D1", &Geom_Surface::D1)
        .function("D2", &Geom_Surface::D2)
        .function("Bounds", +[](const Geom_Surface& s) {
            double a, b, c, d;
            s.Bounds(a, b, c, d);
            return arrOf(a, b, c, d);
        })
        .function("IsUClosed", &Geom_Surface::IsUClosed)
        .function("IsVClosed", &Geom_Surface::IsVClosed)
        .function("IsUPeriodic", &Geom_Surface::IsUPeriodic)
        .function("IsVPeriodic", &Geom_Surface::IsVPeriodic)
        .function("UPeriod", &Geom_Surface::UPeriod)
        .function("VPeriod", &Geom_Surface::VPeriod)
        .function("IsCNu", &Geom_Surface::IsCNu)
        .function("IsCNv", &Geom_Surface::IsCNv)
        .function("Continuity", &Geom_Surface::Continuity)
        .function("UIso", &Geom_Surface::UIso)
        .function("VIso", &Geom_Surface::VIso)
        .function("UReverse", &Geom_Surface::UReverse)
        .function("VReverse", &Geom_Surface::VReverse)
        .function("UReversed", &Geom_Surface::UReversed)
        .function("VReversed", &Geom_Surface::VReversed);

    class_<Geom_ElementarySurface, base<Geom_Surface>>("Geom_ElementarySurface")
        .smart_ptr<H(Geom_ElementarySurface)>("Handle_Geom_ElementarySurface")
        .function("Position", +[](const Geom_ElementarySurface& s) { return s.Position(); })
        .function("SetPosition", &Geom_ElementarySurface::SetPosition)
        .function("Location", +[](const Geom_ElementarySurface& s) { return s.Location(); })
        .function("Axis", +[](const Geom_ElementarySurface& s) { return s.Axis(); });

    class_<Geom_Plane, base<Geom_ElementarySurface>>("Geom_Plane")
        .smart_ptr<H(Geom_Plane)>("Handle_Geom_Plane")
        .constructor(+[](const gp_Ax3& a) { return H(Geom_Plane)(new Geom_Plane(a)); })
        .constructor(+[](const gp_Pnt& p, const gp_Dir& d) { return H(Geom_Plane)(new Geom_Plane(p, d)); })
        .function("Pln", &Geom_Plane::Pln);
    class_<Geom_CylindricalSurface, base<Geom_ElementarySurface>>("Geom_CylindricalSurface")
        .smart_ptr<H(Geom_CylindricalSurface)>("Handle_Geom_CylindricalSurface")
        .constructor(+[](const gp_Ax3& a, double r) { return H(Geom_CylindricalSurface)(new Geom_CylindricalSurface(a, r)); })
        .function("Radius", &Geom_CylindricalSurface::Radius);
    class_<Geom_ConicalSurface, base<Geom_ElementarySurface>>("Geom_ConicalSurface")
        .smart_ptr<H(Geom_ConicalSurface)>("Handle_Geom_ConicalSurface")
        .constructor(+[](const gp_Ax3& a, double ang, double r) {
            return H(Geom_ConicalSurface)(new Geom_ConicalSurface(a, ang, r));
        })
        .function("SemiAngle", &Geom_ConicalSurface::SemiAngle)
        .function("RefRadius", &Geom_ConicalSurface::RefRadius)
        .function("Apex", &Geom_ConicalSurface::Apex);
    class_<Geom_SphericalSurface, base<Geom_ElementarySurface>>("Geom_SphericalSurface")
        .smart_ptr<H(Geom_SphericalSurface)>("Handle_Geom_SphericalSurface")
        .constructor(+[](const gp_Ax3& a, double r) { return H(Geom_SphericalSurface)(new Geom_SphericalSurface(a, r)); })
        .function("Radius", &Geom_SphericalSurface::Radius);
    class_<Geom_ToroidalSurface, base<Geom_ElementarySurface>>("Geom_ToroidalSurface")
        .smart_ptr<H(Geom_ToroidalSurface)>("Handle_Geom_ToroidalSurface")
        .constructor(+[](const gp_Ax3& a, double R, double r) {
            return H(Geom_ToroidalSurface)(new Geom_ToroidalSurface(a, R, r));
        })
        .function("MajorRadius", &Geom_ToroidalSurface::MajorRadius)
        .function("MinorRadius", &Geom_ToroidalSurface::MinorRadius);

    class_<Geom_BoundedSurface, base<Geom_Surface>>("Geom_BoundedSurface")
        .smart_ptr<H(Geom_BoundedSurface)>("Handle_Geom_BoundedSurface");
    class_<Geom_BSplineSurface, base<Geom_BoundedSurface>>("Geom_BSplineSurface")
        .smart_ptr<H(Geom_BSplineSurface)>("Handle_Geom_BSplineSurface")
        .constructor(+[](const TColgp_Array2OfPnt& p, const TColStd_Array2OfReal& w, const TColStd_Array1OfReal& uk,
                         const TColStd_Array1OfReal& vk, const TColStd_Array1OfInteger& um,
                         const TColStd_Array1OfInteger& vm, int du, int dv, bool up, bool vp) {
            return H(Geom_BSplineSurface)(new Geom_BSplineSurface(p, w, uk, vk, um, vm, du, dv, up, vp));
        })
        .constructor(+[](const TColgp_Array2OfPnt& p, const TColStd_Array1OfReal& uk, const TColStd_Array1OfReal& vk,
                         const TColStd_Array1OfInteger& um, const TColStd_Array1OfInteger& vm, int du, int dv, bool up,
                         bool vp) {
            return H(Geom_BSplineSurface)(new Geom_BSplineSurface(p, uk, vk, um, vm, du, dv, up, vp));
        })
        .function("NbUKnots", &Geom_BSplineSurface::NbUKnots)
        .function("NbVKnots", &Geom_BSplineSurface::NbVKnots)
        .function("NbUPoles", &Geom_BSplineSurface::NbUPoles)
        .function("NbVPoles", &Geom_BSplineSurface::NbVPoles)
        .function("UKnot", &Geom_BSplineSurface::UKnot)
        .function("VKnot", &Geom_BSplineSurface::VKnot)
        .function("UMultiplicity", &Geom_BSplineSurface::UMultiplicity)
        .function("VMultiplicity", &Geom_BSplineSurface::VMultiplicity)
        .function("UDegree", &Geom_BSplineSurface::UDegree)
        .function("VDegree", &Geom_BSplineSurface::VDegree)
        .function("Pole", +[](const Geom_BSplineSurface& s, int i, int j) { return gp_Pnt(s.Pole(i, j)); })
        .function("Weight", &Geom_BSplineSurface::Weight)
        .function("IsURational", &Geom_BSplineSurface::IsURational)
        .function("IsVRational", &Geom_BSplineSurface::IsVRational)
        .function("SetUPeriodic", &Geom_BSplineSurface::SetUPeriodic)
        .function("SetVPeriodic", &Geom_BSplineSurface::SetVPeriodic)
        .function("SetUNotPeriodic", &Geom_BSplineSurface::SetUNotPeriodic)
        .function("SetVNotPeriodic", &Geom_BSplineSurface::SetVNotPeriodic)
        .function("Segment", +[](Geom_BSplineSurface& s, double u0, double u1, double v0, double v1) {
            s.Segment(u0, u1, v0, v1);
        })
        .function("RemoveUKnot", &Geom_BSplineSurface::RemoveUKnot)
        .function("RemoveVKnot", &Geom_BSplineSurface::RemoveVKnot);

    class_<Geom_SweptSurface, base<Geom_Surface>>("Geom_SweptSurface")
        .smart_ptr<H(Geom_SweptSurface)>("Handle_Geom_SweptSurface")
        .function("BasisCurve", +[](const Geom_SweptSurface& s) { return s.BasisCurve(); })
        .function("Direction", +[](const Geom_SweptSurface& s) { return s.Direction(); });
    class_<Geom_SurfaceOfLinearExtrusion, base<Geom_SweptSurface>>("Geom_SurfaceOfLinearExtrusion")
        .smart_ptr<H(Geom_SurfaceOfLinearExtrusion)>("Handle_Geom_SurfaceOfLinearExtrusion")
        .constructor(+[](const H(Geom_Curve) & c, const gp_Dir& d) {
            return H(Geom_SurfaceOfLinearExtrusion)(new Geom_SurfaceOfLinearExtrusion(c, d));
        });
    class_<Geom_SurfaceOfRevolution, base<Geom_SweptSurface>>("Geom_SurfaceOfRevolution")
        .smart_ptr<H(Geom_SurfaceOfRevolution)>("Handle_Geom_SurfaceOfRevolution")
        .constructor(+[](const H(Geom_Curve) & c, const gp_Ax1& a) {
            return H(Geom_SurfaceOfRevolution)(new Geom_SurfaceOfRevolution(c, a));
        })
        .function("Axis", &Geom_SurfaceOfRevolution::Axis)
        .function("Location", +[](const Geom_SurfaceOfRevolution& s) { return s.Location(); });

    class_<Geom_OffsetSurface, base<Geom_Surface>>("Geom_OffsetSurface")
        .smart_ptr<H(Geom_OffsetSurface)>("Handle_Geom_OffsetSurface")
        .constructor(+[](const H(Geom_Surface) & s, double off) {
            return H(Geom_OffsetSurface)(new Geom_OffsetSurface(s, off));
        })
        .constructor(+[](const H(Geom_Surface) & s, double off, bool noC0Check) {
            return H(Geom_OffsetSurface)(new Geom_OffsetSurface(s, off, noC0Check));
        })
        .function("BasisSurface", +[](const Geom_OffsetSurface& s) { return s.BasisSurface(); })
        .function("Offset", &Geom_OffsetSurface::Offset);

    class_<Geom_RectangularTrimmedSurface, base<Geom_BoundedSurface>>("Geom_RectangularTrimmedSurface")
        .smart_ptr<H(Geom_RectangularTrimmedSurface)>("Handle_Geom_RectangularTrimmedSurface")
        .constructor(+[](const H(Geom_Surface) & s, double u1, double u2, double v1, double v2) {
            return H(Geom_RectangularTrimmedSurface)(new Geom_RectangularTrimmedSurface(s, u1, u2, v1, v2));
        })
        .class_function("__new_param", +[](const H(Geom_Surface) & s, double p1, double p2, bool utrim, bool sense) {
            return H(Geom_RectangularTrimmedSurface)(new Geom_RectangularTrimmedSurface(s, p1, p2, utrim, sense));
        })
        .function("BasisSurface", +[](const Geom_RectangularTrimmedSurface& s) { return s.BasisSurface(); });

    // curves
    class_<Geom_Curve, base<Geom_Geometry>>("Geom_Curve")
        .smart_ptr<H(Geom_Curve)>("Handle_Geom_Curve")
        .function("Value", &Geom_Curve::Value)
        .function("D0", &Geom_Curve::D0)
        .function("D1", &Geom_Curve::D1)
        .function("D2", &Geom_Curve::D2)
        .function("FirstParameter", &Geom_Curve::FirstParameter)
        .function("LastParameter", &Geom_Curve::LastParameter)
        .function("IsClosed", &Geom_Curve::IsClosed)
        .function("IsPeriodic", &Geom_Curve::IsPeriodic)
        .function("Period", &Geom_Curve::Period)
        .function("IsCN", &Geom_Curve::IsCN)
        .function("Continuity", &Geom_Curve::Continuity)
        .function("Reverse", &Geom_Curve::Reverse)
        .function("Reversed", +[](const Geom_Curve& c) { return H(Geom_Curve)(c.Reversed()); })
        .function("ReversedParameter", &Geom_Curve::ReversedParameter);
    class_<Geom_Line, base<Geom_Curve>>("Geom_Line")
        .smart_ptr<H(Geom_Line)>("Handle_Geom_Line")
        .constructor(+[](const gp_Ax1& a) { return H(Geom_Line)(new Geom_Line(a)); })
        .constructor(+[](const gp_Pnt& p, const gp_Dir& d) { return H(Geom_Line)(new Geom_Line(p, d)); })
        .function("Position", +[](const Geom_Line& l) { return l.Position(); });
    class_<Geom_Conic, base<Geom_Curve>>("Geom_Conic")
        .smart_ptr<H(Geom_Conic)>("Handle_Geom_Conic")
        .function("Position", +[](const Geom_Conic& c) { return c.Position(); })
        .function("Location", +[](const Geom_Conic& c) { return c.Location(); })
        .function("Axis", +[](const Geom_Conic& c) { return c.Axis(); })
        .function("XAxis", &Geom_Conic::XAxis)
        .function("YAxis", &Geom_Conic::YAxis);
    class_<Geom_Circle, base<Geom_Conic>>("Geom_Circle")
        .smart_ptr<H(Geom_Circle)>("Handle_Geom_Circle")
        .constructor(+[](const gp_Ax2& a, double r) { return H(Geom_Circle)(new Geom_Circle(a, r)); })
        .function("Radius", &Geom_Circle::Radius)
        .function("Circ", &Geom_Circle::Circ);
    class_<Geom_Ellipse, base<Geom_Conic>>("Geom_Ellipse")
        .smart_ptr<H(Geom_Ellipse)>("Handle_Geom_Ellipse")
        .constructor(+[](const gp_Ax2& a, double R, double r) { return H(Geom_Ellipse)(new Geom_Ellipse(a, R, r)); })
        .function("MajorRadius", &Geom_Ellipse::MajorRadius)
        .function("MinorRadius", &Geom_Ellipse::MinorRadius)
        .function("Elips", &Geom_Ellipse::Elips);
    class_<Geom_BoundedCurve, base<Geom_Curve>>("Geom_BoundedCurve")
        .smart_ptr<H(Geom_BoundedCurve)>("Handle_Geom_BoundedCurve")
        .function("StartPoint", &Geom_BoundedCurve::StartPoint)
        .function("EndPoint", &Geom_BoundedCurve::EndPoint);
    class_<Geom_BSplineCurve, base<Geom_BoundedCurve>>("Geom_BSplineCurve")
        .smart_ptr<H(Geom_BSplineCurve)>("Handle_Geom_BSplineCurve")
        .constructor(+[](const TColgp_Array1OfPnt& p, const TColStd_Array1OfReal& k, const TColStd_Array1OfInteger& m,
                         int deg) { return H(Geom_BSplineCurve)(new Geom_BSplineCurve(p, k, m, deg)); })
        .constructor(+[](const TColgp_Array1OfPnt& p, const TColStd_Array1OfReal& w, const TColStd_Array1OfReal& k,
                         const TColStd_Array1OfInteger& m, int deg, bool per) {
            return H(Geom_BSplineCurve)(new Geom_BSplineCurve(p, w, k, m, deg, per));
        })
        .constructor(+[](const TColgp_Array1OfPnt& p, const TColStd_Array1OfReal& w, const TColStd_Array1OfReal& k,
                         const TColStd_Array1OfInteger& m, int deg, bool per, bool checkRational) {
            return H(Geom_BSplineCurve)(new Geom_BSplineCurve(p, w, k, m, deg, per, checkRational));
        })
        // 5 arguments: weighted (poles, weights, knots, mults, degree) is the constructor;
        // (poles, knots, mults, degree, periodic) is __new_periodic.
        .constructor(+[](const TColgp_Array1OfPnt& p, const TColStd_Array1OfReal& w, const TColStd_Array1OfReal& k,
                         const TColStd_Array1OfInteger& m, int deg) {
            return H(Geom_BSplineCurve)(new Geom_BSplineCurve(p, w, k, m, deg));
        })
        .class_function("__new_periodic", +[](const TColgp_Array1OfPnt& p, const TColStd_Array1OfReal& k,
                                              const TColStd_Array1OfInteger& m, int deg, bool per) {
            return H(Geom_BSplineCurve)(new Geom_BSplineCurve(p, k, m, deg, per));
        })
        .function("NbKnots", &Geom_BSplineCurve::NbKnots)
        .function("NbPoles", &Geom_BSplineCurve::NbPoles)
        .function("Knot", &Geom_BSplineCurve::Knot)
        .function("Multiplicity", &Geom_BSplineCurve::Multiplicity)
        .function("Pole", +[](const Geom_BSplineCurve& c, int i) { return gp_Pnt(c.Pole(i)); })
        .function("Weight", &Geom_BSplineCurve::Weight)
        .function("Degree", &Geom_BSplineCurve::Degree)
        .function("IsRational", &Geom_BSplineCurve::IsRational)
        .function("FirstUKnotIndex", &Geom_BSplineCurve::FirstUKnotIndex)
        .function("LastUKnotIndex", &Geom_BSplineCurve::LastUKnotIndex)
        .function("SetPeriodic", &Geom_BSplineCurve::SetPeriodic)
        .function("SetNotPeriodic", &Geom_BSplineCurve::SetNotPeriodic)
        .function("Segment", +[](Geom_BSplineCurve& c, double u1, double u2) { c.Segment(u1, u2); });
    class_<Geom_TrimmedCurve, base<Geom_BoundedCurve>>("Geom_TrimmedCurve")
        .smart_ptr<H(Geom_TrimmedCurve)>("Handle_Geom_TrimmedCurve")
        .constructor(+[](const H(Geom_Curve) & c, double u1, double u2) {
            return H(Geom_TrimmedCurve)(new Geom_TrimmedCurve(c, u1, u2));
        })
        .constructor(+[](const H(Geom_Curve) & c, double u1, double u2, bool sense) {
            return H(Geom_TrimmedCurve)(new Geom_TrimmedCurve(c, u1, u2, sense));
        })
        .constructor(+[](const H(Geom_Curve) & c, double u1, double u2, bool sense, bool adjust) {
            return H(Geom_TrimmedCurve)(new Geom_TrimmedCurve(c, u1, u2, sense, adjust));
        })
        .function("BasisCurve", +[](const Geom_TrimmedCurve& c) { return c.BasisCurve(); })
        .function("SetTrim", +[](Geom_TrimmedCurve& c, double u1, double u2) { c.SetTrim(u1, u2); });

    // 2-D curves
    class_<Geom2d_Geometry, base<Standard_Transient>>("Geom2d_Geometry")
        .smart_ptr<H(Geom2d_Geometry)>("Handle_Geom2d_Geometry")
        .function("Copy", +[](const Geom2d_Geometry& g) { return g.Copy(); })
        .function("Translate", +[](Geom2d_Geometry& g, const gp_Vec2d& v) { g.Translate(v); })
        .function("Translated", +[](const Geom2d_Geometry& g, const gp_Vec2d& v) { return g.Translated(v); });
    class_<Geom2d_Curve, base<Geom2d_Geometry>>("Geom2d_Curve")
        .smart_ptr<H(Geom2d_Curve)>("Handle_Geom2d_Curve")
        .function("Value", &Geom2d_Curve::Value)
        .function("D0", &Geom2d_Curve::D0)
        .function("D1", &Geom2d_Curve::D1)
        .function("FirstParameter", &Geom2d_Curve::FirstParameter)
        .function("LastParameter", &Geom2d_Curve::LastParameter)
        .function("IsClosed", &Geom2d_Curve::IsClosed)
        .function("IsPeriodic", &Geom2d_Curve::IsPeriodic)
        .function("Period", &Geom2d_Curve::Period)
        .function("Continuity", &Geom2d_Curve::Continuity)
        .function("Reverse", &Geom2d_Curve::Reverse)
        .function("Reversed", +[](const Geom2d_Curve& c) { return H(Geom2d_Curve)(c.Reversed()); })
        .function("ReversedParameter", &Geom2d_Curve::ReversedParameter);
    class_<Geom2d_Line, base<Geom2d_Curve>>("Geom2d_Line")
        .smart_ptr<H(Geom2d_Line)>("Handle_Geom2d_Line")
        .constructor(+[](const gp_Pnt2d& p, const gp_Dir2d& d) { return H(Geom2d_Line)(new Geom2d_Line(p, d)); })
        .function("Location", +[](const Geom2d_Line& l) { return l.Location(); })
        .function("Direction", +[](const Geom2d_Line& l) { return l.Direction(); });
    class_<Geom2d_BoundedCurve, base<Geom2d_Curve>>("Geom2d_BoundedCurve")
        .smart_ptr<H(Geom2d_BoundedCurve)>("Handle_Geom2d_BoundedCurve")
        .function("StartPoint", &Geom2d_BoundedCurve::StartPoint)
        .function("EndPoint", &Geom2d_BoundedCurve::EndPoint);
    class_<Geom2d_BSplineCurve, base<Geom2d_BoundedCurve>>("Geom2d_BSplineCurve")
        .smart_ptr<H(Geom2d_BSplineCurve)>("Handle_Geom2d_BSplineCurve")
        .constructor(+[](const TColgp_Array1OfPnt2d& p, const TColStd_Array1OfReal& k,
                         const TColStd_Array1OfInteger& m,
                         int deg) { return H(Geom2d_BSplineCurve)(new Geom2d_BSplineCurve(p, k, m, deg)); })
        .constructor(+[](const TColgp_Array1OfPnt2d& p, const TColStd_Array1OfReal& w, const TColStd_Array1OfReal& k,
                         const TColStd_Array1OfInteger& m, int deg) {
            return H(Geom2d_BSplineCurve)(new Geom2d_BSplineCurve(p, w, k, m, deg));
        })
        .constructor(+[](const TColgp_Array1OfPnt2d& p, const TColStd_Array1OfReal& w, const TColStd_Array1OfReal& k,
                         const TColStd_Array1OfInteger& m, int deg, bool per) {
            return H(Geom2d_BSplineCurve)(new Geom2d_BSplineCurve(p, w, k, m, deg, per));
        })
        .class_function("__new_periodic", +[](const TColgp_Array1OfPnt2d& p, const TColStd_Array1OfReal& k,
                                              const TColStd_Array1OfInteger& m, int deg, bool per) {
            return H(Geom2d_BSplineCurve)(new Geom2d_BSplineCurve(p, k, m, deg, per));
        })
        .function("NbKnots", &Geom2d_BSplineCurve::NbKnots)
        .function("NbPoles", &Geom2d_BSplineCurve::NbPoles)
        .function("Knot", &Geom2d_BSplineCurve::Knot)
        .function("Multiplicity", &Geom2d_BSplineCurve::Multiplicity)
        .function("Pole", +[](const Geom2d_BSplineCurve& c, int i) { return gp_Pnt2d(c.Pole(i)); })
        .function("Weight", &Geom2d_BSplineCurve::Weight)
        .function("Degree", &Geom2d_BSplineCurve::Degree)
        .function("IsRational", &Geom2d_BSplineCurve::IsRational)
        .function("SetPeriodic", &Geom2d_BSplineCurve::SetPeriodic)
        .function("SetNotPeriodic", &Geom2d_BSplineCurve::SetNotPeriodic)
        .function("Segment", +[](Geom2d_BSplineCurve& c, double u1, double u2) { c.Segment(u1, u2); });
    class_<Geom2d_TrimmedCurve, base<Geom2d_BoundedCurve>>("Geom2d_TrimmedCurve")
        .smart_ptr<H(Geom2d_TrimmedCurve)>("Handle_Geom2d_TrimmedCurve")
        .constructor(+[](const H(Geom2d_Curve) & c, double u1, double u2) {
            return H(Geom2d_TrimmedCurve)(new Geom2d_TrimmedCurve(c, u1, u2));
        })
        .constructor(+[](const H(Geom2d_Curve) & c, double u1, double u2, bool sense) {
            return H(Geom2d_TrimmedCurve)(new Geom2d_TrimmedCurve(c, u1, u2, sense));
        })
        .function("BasisCurve", +[](const Geom2d_TrimmedCurve& c) { return c.BasisCurve(); });
}

// ── arrays ────────────────────────────────────────────────────────────────────
template <class A, class T> static void bindArray1(const char* name) {
    class_<A>(name)
        .template constructor<int, int>()
        .function("Value", +[](const A& a, int i) { return T(a.Value(i)); })
        .function("SetValue", +[](A& a, int i, const T& v) { a.SetValue(i, v); })
        .function("Length", +[](const A& a) { return a.Length(); })
        .function("Lower", +[](const A& a) { return a.Lower(); })
        .function("Upper", +[](const A& a) { return a.Upper(); });
}
template <class A, class T> static void bindHArray1(const char* name, const char* hname) {
    class_<A>(name)
        .template smart_ptr<H(A)>(hname)
        .constructor(+[](int lo, int hi) { return H(A)(new A(lo, hi)); })
        .function("Value", +[](const A& a, int i) { return T(a.Value(i)); })
        .function("SetValue", +[](A& a, int i, const T& v) { a.SetValue(i, v); })
        .function("Length", +[](const A& a) { return a.Length(); })
        .function("Lower", +[](const A& a) { return a.Lower(); })
        .function("Upper", +[](const A& a) { return a.Upper(); });
}
template <class A, class T> static void bindArray2(const char* name) {
    class_<A>(name)
        .template constructor<int, int, int, int>()
        .function("Value", +[](const A& a, int r, int c) { return T(a.Value(r, c)); })
        .function("SetValue", +[](A& a, int r, int c, const T& v) { a.SetValue(r, c, v); })
        .function("ColLength", +[](const A& a) { return a.ColLength(); })
        .function("RowLength", +[](const A& a) { return a.RowLength(); })
        .function("LowerRow", +[](const A& a) { return a.LowerRow(); })
        .function("UpperRow", +[](const A& a) { return a.UpperRow(); })
        .function("LowerCol", +[](const A& a) { return a.LowerCol(); })
        .function("UpperCol", +[](const A& a) { return a.UpperCol(); });
}
static void bindArrays() {
    bindArray1<TColgp_Array1OfPnt, gp_Pnt>("TColgp_Array1OfPnt");
    bindArray1<TColgp_Array1OfPnt2d, gp_Pnt2d>("TColgp_Array1OfPnt2d");
    bindArray1<TColStd_Array1OfReal, double>("TColStd_Array1OfReal");
    bindArray1<TColStd_Array1OfInteger, int>("TColStd_Array1OfInteger");
    bindArray2<TColgp_Array2OfPnt, gp_Pnt>("TColgp_Array2OfPnt");
    bindArray2<TColStd_Array2OfReal, double>("TColStd_Array2OfReal");
    bindHArray1<TColgp_HArray1OfPnt, gp_Pnt>("TColgp_HArray1OfPnt", "Handle_TColgp_HArray1OfPnt");
    bindHArray1<TColgp_HArray1OfPnt2d, gp_Pnt2d>("TColgp_HArray1OfPnt2d", "Handle_TColgp_HArray1OfPnt2d");
    bindHArray1<TColStd_HArray1OfReal, double>("TColStd_HArray1OfReal", "Handle_TColStd_HArray1OfReal");
}

// ── converters, adaptors, projections, intersections, interpolation ──────────
static void bindAlgo() {
    class_<Statics<877>>("GeomConvert")
        .class_function("CurveToBSplineCurve_s",
                        +[](const H(Geom_Curve) & c) { return GeomConvert::CurveToBSplineCurve(c); })
        .class_function("SurfaceToBSplineSurface_s",
                        +[](const H(Geom_Surface) & s) { return GeomConvert::SurfaceToBSplineSurface(s); });
    class_<Statics<882>>("Geom2dConvert")
        .class_function("CurveToBSplineCurve_s",
                        +[](const H(Geom2d_Curve) & c) { return Geom2dConvert::CurveToBSplineCurve(c); });
    class_<GeomConvert_CompCurveToBSplineCurve>("GeomConvert_CompCurveToBSplineCurve")
        .constructor<const H(Geom_BoundedCurve)&>()
        .function("Add", +[](GeomConvert_CompCurveToBSplineCurve& c, const H(Geom_BoundedCurve) & n, double tol) {
            return c.Add(n, tol);
        })
        .function("Add", +[](GeomConvert_CompCurveToBSplineCurve& c, const H(Geom_BoundedCurve) & n, double tol,
                             bool after) { return c.Add(n, tol, after); })
        .function("Add", +[](GeomConvert_CompCurveToBSplineCurve& c, const H(Geom_BoundedCurve) & n, double tol,
                             bool after, bool withRatio, int minM) { return c.Add(n, tol, after, withRatio, minM); })
        .function("BSplineCurve", &GeomConvert_CompCurveToBSplineCurve::BSplineCurve);
    class_<Geom2dConvert_CompCurveToBSplineCurve>("Geom2dConvert_CompCurveToBSplineCurve")
        .constructor<const H(Geom2d_BoundedCurve)&>()
        .function("Add", +[](Geom2dConvert_CompCurveToBSplineCurve& c, const H(Geom2d_BoundedCurve) & n, double tol) {
            return c.Add(n, tol);
        })
        .function("Add", +[](Geom2dConvert_CompCurveToBSplineCurve& c, const H(Geom2d_BoundedCurve) & n, double tol,
                             bool after) { return c.Add(n, tol, after); })
        .function("BSplineCurve", &Geom2dConvert_CompCurveToBSplineCurve::BSplineCurve);
    class_<GeomConvert_ApproxSurface>("GeomConvert_ApproxSurface")
        .constructor<const H(Geom_Surface)&, double, GeomAbs_Shape, GeomAbs_Shape, int, int, int, int>()
        .function("IsDone", &GeomConvert_ApproxSurface::IsDone)
        .function("HasResult", &GeomConvert_ApproxSurface::HasResult)
        .function("MaxError", &GeomConvert_ApproxSurface::MaxError)
        .function("Surface", &GeomConvert_ApproxSurface::Surface);

    class_<Adaptor3d_Curve>("Adaptor3d_Curve")
        .function("FirstParameter", &Adaptor3d_Curve::FirstParameter)
        .function("LastParameter", &Adaptor3d_Curve::LastParameter)
        .function("Value", &Adaptor3d_Curve::Value)
        .function("GetType", &Adaptor3d_Curve::GetType)
        .function("IsClosed", &Adaptor3d_Curve::IsClosed)
        .function("IsPeriodic", &Adaptor3d_Curve::IsPeriodic);
    class_<GeomAdaptor_Curve, base<Adaptor3d_Curve>>("GeomAdaptor_Curve")
        .constructor<const H(Geom_Curve)&>()
        .constructor<const H(Geom_Curve)&, double, double>()
        .function("Curve", +[](const GeomAdaptor_Curve& a) { return a.Curve(); });
    class_<BRepAdaptor_Curve, base<Adaptor3d_Curve>>("BRepAdaptor_Curve")
        .constructor(+[](const TopoDS_Shape& e) { return new BRepAdaptor_Curve(TopoDS::Edge(e)); }, allow_raw_pointers())
        .constructor(+[](const TopoDS_Shape& e, const TopoDS_Shape& f) {
            return new BRepAdaptor_Curve(TopoDS::Edge(e), TopoDS::Face(f));
        }, allow_raw_pointers())
        // The edge's 3-D curve as an adaptor over the same range (location not applied).
        .function("Curve", +[](const BRepAdaptor_Curve& a) {
            const GeomAdaptor_Curve& g = a.Curve();
            return GeomAdaptor_Curve(g.Curve(), g.FirstParameter(), g.LastParameter());
        })
        .function("Trsf", +[](const BRepAdaptor_Curve& a) { return gp_Trsf(a.Trsf()); })
        .function("Is3DCurve", +[](const BRepAdaptor_Curve& a) { return a.Is3DCurve(); })
        .function("Edge", +[](const BRepAdaptor_Curve& a) { return TopoDS_Edge(a.Edge()); });
    class_<BRepAdaptor_Surface>("BRepAdaptor_Surface")
        .constructor(+[](const TopoDS_Shape& f) { return new BRepAdaptor_Surface(TopoDS::Face(f)); }, allow_raw_pointers())
        .constructor(+[](const TopoDS_Shape& f, bool restriction) {
            return new BRepAdaptor_Surface(TopoDS::Face(f), restriction);
        }, allow_raw_pointers())
        .function("GetType", +[](const BRepAdaptor_Surface& a) { return a.GetType(); })
        .function("FirstUParameter", +[](const BRepAdaptor_Surface& a) { return a.FirstUParameter(); })
        .function("LastUParameter", +[](const BRepAdaptor_Surface& a) { return a.LastUParameter(); })
        .function("FirstVParameter", +[](const BRepAdaptor_Surface& a) { return a.FirstVParameter(); })
        .function("LastVParameter", +[](const BRepAdaptor_Surface& a) { return a.LastVParameter(); })
        .function("Value", +[](const BRepAdaptor_Surface& a, double u, double v) { return a.Value(u, v); })
        .function("IsUPeriodic", +[](const BRepAdaptor_Surface& a) { return a.IsUPeriodic(); })
        .function("IsVPeriodic", +[](const BRepAdaptor_Surface& a) { return a.IsVPeriodic(); });

    class_<GCPnts_AbscissaPoint>("GCPnts_AbscissaPoint")
        .class_function("Length_s", +[](const Adaptor3d_Curve& c) { return GCPnts_AbscissaPoint::Length(c); })
        .class_function("Length_s",
                        +[](const Adaptor3d_Curve& c, double tol) { return GCPnts_AbscissaPoint::Length(c, tol); })
        .class_function("Length_s", +[](const Adaptor3d_Curve& c, double u1, double u2) {
            return GCPnts_AbscissaPoint::Length(c, u1, u2);
        })
        .class_function("Length_s", +[](const Adaptor3d_Curve& c, double u1, double u2, double tol) {
            return GCPnts_AbscissaPoint::Length(c, u1, u2, tol);
        });
    class_<GCPnts_UniformAbscissa>("GCPnts_UniformAbscissa")
        .constructor(+[](const Adaptor3d_Curve& c, int n, double tol) { return new GCPnts_UniformAbscissa(c, n, tol); },
                     allow_raw_pointers())
        .constructor(+[](const Adaptor3d_Curve& c, int n, double u1, double u2, double tol) {
            return new GCPnts_UniformAbscissa(c, n, u1, u2, tol);
        }, allow_raw_pointers())
        .function("IsDone", &GCPnts_UniformAbscissa::IsDone)
        .function("NbPoints", &GCPnts_UniformAbscissa::NbPoints)
        .function("Parameter", &GCPnts_UniformAbscissa::Parameter);
    class_<GC_MakeSegment>("GC_MakeSegment")
        .constructor<const gp_Pnt&, const gp_Pnt&>()
        .function("IsDone", +[](const GC_MakeSegment& m) { return m.IsDone(); })
        .function("Value", +[](const GC_MakeSegment& m) { return m.Value(); });

    class_<GeomAPI_ProjectPointOnCurve>("GeomAPI_ProjectPointOnCurve")
        .constructor<const gp_Pnt&, const H(Geom_Curve)&>()
        .constructor<const gp_Pnt&, const H(Geom_Curve)&, double, double>()
        .function("NbPoints", &GeomAPI_ProjectPointOnCurve::NbPoints)
        .function("Point", &GeomAPI_ProjectPointOnCurve::Point)
        .function("Parameter", +[](const GeomAPI_ProjectPointOnCurve& p, int i) { return p.Parameter(i); })
        .function("Distance", &GeomAPI_ProjectPointOnCurve::Distance)
        .function("NearestPoint", &GeomAPI_ProjectPointOnCurve::NearestPoint)
        .function("LowerDistanceParameter", &GeomAPI_ProjectPointOnCurve::LowerDistanceParameter)
        .function("LowerDistance", &GeomAPI_ProjectPointOnCurve::LowerDistance);
    class_<GeomAPI_ProjectPointOnSurf>("GeomAPI_ProjectPointOnSurf")
        .constructor<const gp_Pnt&, const H(Geom_Surface)&>()
        .constructor(+[](const gp_Pnt& p, const H(Geom_Surface) & s, double tol) {
            return new GeomAPI_ProjectPointOnSurf(p, s, tol);
        }, allow_raw_pointers())
        .function("NbPoints", &GeomAPI_ProjectPointOnSurf::NbPoints)
        .function("Point", &GeomAPI_ProjectPointOnSurf::Point)
        .function("Parameters", +[](const GeomAPI_ProjectPointOnSurf& p, int i) {
            double u, v;
            p.Parameters(i, u, v);
            return arrOf(u, v);
        })
        .function("Distance", &GeomAPI_ProjectPointOnSurf::Distance)
        .function("NearestPoint", &GeomAPI_ProjectPointOnSurf::NearestPoint)
        .function("LowerDistanceParameters", +[](const GeomAPI_ProjectPointOnSurf& p) {
            double u, v;
            p.LowerDistanceParameters(u, v);
            return arrOf(u, v);
        })
        .function("LowerDistance", &GeomAPI_ProjectPointOnSurf::LowerDistance);
    class_<Geom2dAPI_ProjectPointOnCurve>("Geom2dAPI_ProjectPointOnCurve")
        .constructor<const gp_Pnt2d&, const H(Geom2d_Curve)&>()
        .constructor<const gp_Pnt2d&, const H(Geom2d_Curve)&, double, double>()
        .function("NbPoints", &Geom2dAPI_ProjectPointOnCurve::NbPoints)
        .function("Point", &Geom2dAPI_ProjectPointOnCurve::Point)
        .function("Parameter", +[](const Geom2dAPI_ProjectPointOnCurve& p, int i) { return p.Parameter(i); })
        .function("Distance", &Geom2dAPI_ProjectPointOnCurve::Distance)
        .function("NearestPoint", &Geom2dAPI_ProjectPointOnCurve::NearestPoint)
        .function("LowerDistanceParameter", &Geom2dAPI_ProjectPointOnCurve::LowerDistanceParameter)
        .function("LowerDistance", &Geom2dAPI_ProjectPointOnCurve::LowerDistance);

    class_<GeomAPI_IntSS>("GeomAPI_IntSS")
        .constructor<const H(Geom_Surface)&, const H(Geom_Surface)&, double>()
        .function("IsDone", &GeomAPI_IntSS::IsDone)
        .function("NbLines", &GeomAPI_IntSS::NbLines)
        .function("Line", &GeomAPI_IntSS::Line);
    class_<GeomAPI_Interpolate>("GeomAPI_Interpolate")
        .constructor<const H(TColgp_HArray1OfPnt)&, bool, double>()
        .constructor<const H(TColgp_HArray1OfPnt)&, const H(TColStd_HArray1OfReal)&, bool, double>()
        .function("Perform", &GeomAPI_Interpolate::Perform)
        .function("IsDone", &GeomAPI_Interpolate::IsDone)
        .function("Curve", +[](const GeomAPI_Interpolate& i) { return i.Curve(); });
    class_<Geom2dAPI_Interpolate>("Geom2dAPI_Interpolate")
        .constructor<const H(TColgp_HArray1OfPnt2d)&, bool, double>()
        .constructor<const H(TColgp_HArray1OfPnt2d)&, const H(TColStd_HArray1OfReal)&, bool, double>()
        .function("Perform", &Geom2dAPI_Interpolate::Perform)
        .function("IsDone", &Geom2dAPI_Interpolate::IsDone)
        .function("Curve", +[](const Geom2dAPI_Interpolate& i) { return i.Curve(); });
    class_<GeomAPI_PointsToBSplineSurface>("GeomAPI_PointsToBSplineSurface")
        .constructor<const TColgp_Array2OfPnt&>()
        .constructor<const TColgp_Array2OfPnt&, int, int, GeomAbs_Shape, double>()
        .function("IsDone", &GeomAPI_PointsToBSplineSurface::IsDone)
        .function("Surface", +[](const GeomAPI_PointsToBSplineSurface& p) { return p.Surface(); });

    class_<ShapeAnalysis_Surface, base<Standard_Transient>>("ShapeAnalysis_Surface")
        .smart_ptr<H(ShapeAnalysis_Surface)>("Handle_ShapeAnalysis_Surface")
        .constructor(+[](const H(Geom_Surface) & s) { return H(ShapeAnalysis_Surface)(new ShapeAnalysis_Surface(s)); })
        .function("ValueOfUV", &ShapeAnalysis_Surface::ValueOfUV)
        .function("NextValueOfUV", +[](ShapeAnalysis_Surface& a, const gp_Pnt2d& prev, const gp_Pnt& p, double preci) {
            return a.NextValueOfUV(prev, p, preci);
        })
        .function("NextValueOfUV", +[](ShapeAnalysis_Surface& a, const gp_Pnt2d& prev, const gp_Pnt& p, double preci,
                                       double maxpreci) { return a.NextValueOfUV(prev, p, preci, maxpreci); })
        .function("Gap", &ShapeAnalysis_Surface::Gap)
        .function("Surface", +[](const ShapeAnalysis_Surface& a) { return a.Surface(); });
}

// ── topology ──────────────────────────────────────────────────────────────────
static void bindTopo() {
    class_<TopoDS_Shape>("TopoDS_Shape")
        .constructor<>()
        .function("IsNull", &TopoDS_Shape::IsNull)
        .function("Nullify", &TopoDS_Shape::Nullify)
        .function("ShapeType", &TopoDS_Shape::ShapeType)
        .function("Orientation", +[](const TopoDS_Shape& s) { return s.Orientation(); })
        .function("Orientation", +[](TopoDS_Shape& s, TopAbs_Orientation o) { s.Orientation(o); })
        .function("Oriented", &TopoDS_Shape::Oriented)
        .function("Reverse", &TopoDS_Shape::Reverse)
        .function("Reversed", &TopoDS_Shape::Reversed)
        .function("Complement", &TopoDS_Shape::Complement)
        .function("Complemented", &TopoDS_Shape::Complemented)
        .function("IsSame", &TopoDS_Shape::IsSame)
        .function("IsEqual", &TopoDS_Shape::IsEqual)
        .function("IsPartner", &TopoDS_Shape::IsPartner)
        .function("Location", +[](const TopoDS_Shape& s) { return TopLoc_Location(s.Location()); })
        .function("Location", +[](TopoDS_Shape& s, const TopLoc_Location& l) { s.Location(l); })
        .function("Located", +[](const TopoDS_Shape& s, const TopLoc_Location& l) { return s.Located(l); })
        .function("Moved", +[](const TopoDS_Shape& s, const TopLoc_Location& l) { return s.Moved(l); })
        .function("Closed", +[](const TopoDS_Shape& s) { return s.Closed(); })
        .function("Closed", +[](TopoDS_Shape& s, bool c) { s.Closed(c); })
        .function("Free", +[](const TopoDS_Shape& s) { return s.Free(); })
        .function("NbChildren", &TopoDS_Shape::NbChildren)
        .function("TShape", &keyOf);
    class_<TopoDS_Vertex, base<TopoDS_Shape>>("TopoDS_Vertex").constructor<>();
    class_<TopoDS_Edge, base<TopoDS_Shape>>("TopoDS_Edge").constructor<>();
    class_<TopoDS_Wire, base<TopoDS_Shape>>("TopoDS_Wire").constructor<>();
    class_<TopoDS_Face, base<TopoDS_Shape>>("TopoDS_Face").constructor<>();
    class_<TopoDS_Shell, base<TopoDS_Shape>>("TopoDS_Shell").constructor<>();
    class_<TopoDS_Solid, base<TopoDS_Shape>>("TopoDS_Solid").constructor<>();
    class_<TopoDS_CompSolid, base<TopoDS_Shape>>("TopoDS_CompSolid").constructor<>();
    class_<TopoDS_Compound, base<TopoDS_Shape>>("TopoDS_Compound").constructor<>();

    class_<Statics<1084>>("TopoDS")
        .class_function("Vertex_s", +[](const TopoDS_Shape& s) { return TopoDS::Vertex(s); })
        .class_function("Edge_s", +[](const TopoDS_Shape& s) { return TopoDS::Edge(s); })
        .class_function("Wire_s", +[](const TopoDS_Shape& s) { return TopoDS::Wire(s); })
        .class_function("Face_s", +[](const TopoDS_Shape& s) { return TopoDS::Face(s); })
        .class_function("Shell_s", +[](const TopoDS_Shape& s) { return TopoDS::Shell(s); })
        .class_function("Solid_s", +[](const TopoDS_Shape& s) { return TopoDS::Solid(s); })
        .class_function("CompSolid_s", +[](const TopoDS_Shape& s) { return TopoDS::CompSolid(s); })
        .class_function("Compound_s", +[](const TopoDS_Shape& s) { return TopoDS::Compound(s); });

    class_<TopExp_Explorer>("TopExp_Explorer")
        .constructor<>()
        .constructor<const TopoDS_Shape&, TopAbs_ShapeEnum>()
        .constructor<const TopoDS_Shape&, TopAbs_ShapeEnum, TopAbs_ShapeEnum>()
        .function("Init", +[](TopExp_Explorer& e, const TopoDS_Shape& s, TopAbs_ShapeEnum t) { e.Init(s, t); })
        .function("Init", +[](TopExp_Explorer& e, const TopoDS_Shape& s, TopAbs_ShapeEnum t, TopAbs_ShapeEnum a) {
            e.Init(s, t, a);
        })
        .function("More", &TopExp_Explorer::More)
        .function("Next", &TopExp_Explorer::Next)
        .function("Current", +[](const TopExp_Explorer& e) { return e.Current(); })
        .function("Value", +[](const TopExp_Explorer& e) { return e.Value(); })
        .function("ReInit", &TopExp_Explorer::ReInit)
        .function("Depth", &TopExp_Explorer::Depth);
    class_<TopoDS_Iterator>("TopoDS_Iterator")
        .constructor<const TopoDS_Shape&>()
        .constructor<const TopoDS_Shape&, bool, bool>()
        .function("More", &TopoDS_Iterator::More)
        .function("Next", &TopoDS_Iterator::Next)
        .function("Value", +[](const TopoDS_Iterator& i) { return i.Value(); });
    class_<Statics<1114>>("TopExp")
        .class_function("FirstVertex_s", +[](const TopoDS_Shape& e) { return TopExp::FirstVertex(TopoDS::Edge(e)); })
        .class_function("FirstVertex_s",
                        +[](const TopoDS_Shape& e, bool c) { return TopExp::FirstVertex(TopoDS::Edge(e), c); })
        .class_function("LastVertex_s", +[](const TopoDS_Shape& e) { return TopExp::LastVertex(TopoDS::Edge(e)); })
        .class_function("LastVertex_s",
                        +[](const TopoDS_Shape& e, bool c) { return TopExp::LastVertex(TopoDS::Edge(e), c); })
        .class_function("Vertices_s", +[](const TopoDS_Shape& e) {
            TopoDS_Vertex a, b;
            TopExp::Vertices(TopoDS::Edge(e), a, b);
            return arrOf(a, b);
        })
        .class_function("Vertices_s", +[](const TopoDS_Shape& e, bool c) {
            TopoDS_Vertex a, b;
            TopExp::Vertices(TopoDS::Edge(e), a, b, c);
            return arrOf(a, b);
        })
        .class_function("MapShapes_s", +[](const TopoDS_Shape& s, TopAbs_ShapeEnum t, TopTools_IndexedMapOfShape& m) {
            TopExp::MapShapes(s, t, m);
        });
    class_<TopTools_MapOfShape>("TopTools_MapOfShape")
        .constructor<>()
        .function("Add", +[](TopTools_MapOfShape& m, const TopoDS_Shape& s) { return m.Add(s); })
        .function("Contains", +[](const TopTools_MapOfShape& m, const TopoDS_Shape& s) { return m.Contains(s); })
        .function("Remove", +[](TopTools_MapOfShape& m, const TopoDS_Shape& s) { return m.Remove(s); })
        .function("Extent", +[](const TopTools_MapOfShape& m) { return m.Extent(); })
        .function("Clear", +[](TopTools_MapOfShape& m) { m.Clear(); });
    class_<TopTools_IndexedMapOfShape>("TopTools_IndexedMapOfShape")
        .constructor<>()
        .function("Add", +[](TopTools_IndexedMapOfShape& m, const TopoDS_Shape& s) { return m.Add(s); })
        .function("Contains",
                  +[](const TopTools_IndexedMapOfShape& m, const TopoDS_Shape& s) { return m.Contains(s); })
        .function("FindIndex",
                  +[](const TopTools_IndexedMapOfShape& m, const TopoDS_Shape& s) { return m.FindIndex(s); })
        .function("FindKey", +[](const TopTools_IndexedMapOfShape& m, int i) { return TopoDS_Shape(m.FindKey(i)); })
        .function("Extent", +[](const TopTools_IndexedMapOfShape& m) { return m.Extent(); });

    class_<BRep_Builder>("BRep_Builder")
        .constructor<>()
        .function("MakeWire", +[](const BRep_Builder& b, TopoDS_Shape& w) { b.MakeWire(TopoDS::Wire(w)); })
        .function("MakeShell", +[](const BRep_Builder& b, TopoDS_Shape& s) { b.MakeShell(TopoDS::Shell(s)); })
        .function("MakeSolid", +[](const BRep_Builder& b, TopoDS_Shape& s) { b.MakeSolid(TopoDS::Solid(s)); })
        .function("MakeCompound",
                  +[](const BRep_Builder& b, TopoDS_Shape& c) { b.MakeCompound(TopoDS::Compound(c)); })
        .function("MakeEdge", +[](const BRep_Builder& b, TopoDS_Shape& e) { b.MakeEdge(TopoDS::Edge(e)); })
        .function("MakeEdge", +[](const BRep_Builder& b, TopoDS_Shape& e, const H(Geom_Curve) & c, double tol) {
            b.MakeEdge(TopoDS::Edge(e), c, tol);
        })
        .function("MakeFace", +[](const BRep_Builder& b, TopoDS_Shape& f, const H(Geom_Surface) & s, double tol) {
            b.MakeFace(TopoDS::Face(f), s, tol);
        })
        .function("MakeFace", +[](const BRep_Builder& b, TopoDS_Shape& f, const H(Geom_Surface) & s,
                                  const TopLoc_Location& l, double tol) { b.MakeFace(TopoDS::Face(f), s, l, tol); })
        .function("MakeVertex", +[](const BRep_Builder& b, TopoDS_Shape& v, const gp_Pnt& p, double tol) {
            b.MakeVertex(TopoDS::Vertex(v), p, tol);
        })
        .function("Add", +[](const BRep_Builder& b, TopoDS_Shape& s, const TopoDS_Shape& c) { b.Add(s, c); })
        .function("Remove", +[](const BRep_Builder& b, TopoDS_Shape& s, const TopoDS_Shape& c) { b.Remove(s, c); })
        .function("UpdateVertex",
                  +[](const BRep_Builder& b, const TopoDS_Shape& v, double tol) { b.UpdateVertex(TopoDS::Vertex(v), tol); })
        .function("UpdateVertex", +[](const BRep_Builder& b, const TopoDS_Shape& v, const gp_Pnt& p, double tol) {
            b.UpdateVertex(TopoDS::Vertex(v), p, tol);
        })
        .function("UpdateEdge", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom_Curve) & c, double tol) {
            b.UpdateEdge(TopoDS::Edge(e), c, tol);
        })
        // 4 arguments: (E, C2d, Face, tol) here; (E, C3d, Location, tol) is UpdateEdge__c3dLoc.
        .function("UpdateEdge", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom2d_Curve) & c,
                                    const TopoDS_Shape& f, double tol) {
            b.UpdateEdge(TopoDS::Edge(e), c, TopoDS::Face(f), tol);
        })
        .function("UpdateEdge__c3dLoc", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom_Curve) & c,
                                            const TopLoc_Location& l,
                                            double tol) { b.UpdateEdge(TopoDS::Edge(e), c, l, tol); })
        // 5 arguments: (E, C2d, Surface, Location, tol) here; (E, C1, C2, Face, tol) is UpdateEdge__twoFace.
        .function("UpdateEdge", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom2d_Curve) & c,
                                    const H(Geom_Surface) & s, const TopLoc_Location& l,
                                    double tol) { b.UpdateEdge(TopoDS::Edge(e), c, s, l, tol); })
        .function("UpdateEdge__twoFace", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom2d_Curve) & c1,
                                             const H(Geom2d_Curve) & c2, const TopoDS_Shape& f, double tol) {
            b.UpdateEdge(TopoDS::Edge(e), c1, c2, TopoDS::Face(f), tol);
        })
        .function("UpdateEdge", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom2d_Curve) & c1,
                                    const H(Geom2d_Curve) & c2, const H(Geom_Surface) & s, const TopLoc_Location& l,
                                    double tol) { b.UpdateEdge(TopoDS::Edge(e), c1, c2, s, l, tol); })
        .function("UpdateFace",
                  +[](const BRep_Builder& b, const TopoDS_Shape& f, double tol) { b.UpdateFace(TopoDS::Face(f), tol); })
        .function("Range", +[](const BRep_Builder& b, const TopoDS_Shape& e, double f, double l) {
            b.Range(TopoDS::Edge(e), f, l);
        })
        // 4 arguments: (E, first, last, only3d) here; (E, Face, first, last) is Range__face.
        .function("Range", +[](const BRep_Builder& b, const TopoDS_Shape& e, double f, double l, bool only3d) {
            b.Range(TopoDS::Edge(e), f, l, only3d);
        })
        .function("Range__face", +[](const BRep_Builder& b, const TopoDS_Shape& e, const TopoDS_Shape& face, double f,
                                     double l) { b.Range(TopoDS::Edge(e), TopoDS::Face(face), f, l); })
        .function("Range", +[](const BRep_Builder& b, const TopoDS_Shape& e, const H(Geom_Surface) & s,
                               const TopLoc_Location& loc, double f,
                               double l) { b.Range(TopoDS::Edge(e), s, loc, f, l); })
        .function("SameRange",
                  +[](const BRep_Builder& b, const TopoDS_Shape& e, bool v) { b.SameRange(TopoDS::Edge(e), v); })
        .function("SameParameter",
                  +[](const BRep_Builder& b, const TopoDS_Shape& e, bool v) { b.SameParameter(TopoDS::Edge(e), v); })
        .function("Degenerated",
                  +[](const BRep_Builder& b, const TopoDS_Shape& e, bool v) { b.Degenerated(TopoDS::Edge(e), v); })
        .function("NaturalRestriction", +[](const BRep_Builder& b, const TopoDS_Shape& f, bool v) {
            b.NaturalRestriction(TopoDS::Face(f), v);
        });

    class_<Statics<1223>>("BRep_Tool")
        .class_function("Pnt_s", +[](const TopoDS_Shape& v) { return BRep_Tool::Pnt(TopoDS::Vertex(v)); })
        .class_function("Tolerance_s", +[](const TopoDS_Shape& s) {
            switch (s.ShapeType()) {
            case TopAbs_VERTEX: return BRep_Tool::Tolerance(TopoDS::Vertex(s));
            case TopAbs_EDGE: return BRep_Tool::Tolerance(TopoDS::Edge(s));
            default: return BRep_Tool::Tolerance(TopoDS::Face(s));
            }
        })
        .class_function("Range_s", +[](const TopoDS_Shape& e) {
            double f, l;
            BRep_Tool::Range(TopoDS::Edge(e), f, l);
            return arrOf(f, l);
        })
        .class_function("Range_s", +[](const TopoDS_Shape& e, const TopoDS_Shape& face) {
            double f, l;
            BRep_Tool::Range(TopoDS::Edge(e), TopoDS::Face(face), f, l);
            return arrOf(f, l);
        })
        // OCP's (edge, first, last) form: the doubles are only placeholders; the
        // range is BRep_Tool.Range_s.
        .class_function("Curve_s", +[](const TopoDS_Shape& e, double, double) {
            double f, l;
            return BRep_Tool::Curve(TopoDS::Edge(e), f, l);
        })
        .class_function("Curve_s", +[](const TopoDS_Shape& e, TopLoc_Location& loc, double, double) {
            double f, l;
            return BRep_Tool::Curve(TopoDS::Edge(e), loc, f, l);
        })
        .class_function("CurveOnSurface_s", +[](const TopoDS_Shape& e, const TopoDS_Shape& face, double, double) {
            double f, l;
            return BRep_Tool::CurveOnSurface(TopoDS::Edge(e), TopoDS::Face(face), f, l);
        })
        .class_function("Surface_s", +[](const TopoDS_Shape& f) { return BRep_Tool::Surface(TopoDS::Face(f)); })
        .class_function("Surface_s", +[](const TopoDS_Shape& f, TopLoc_Location& l) {
            return H(Geom_Surface)(BRep_Tool::Surface(TopoDS::Face(f), l));
        })
        .class_function("Triangulation_s", +[](const TopoDS_Shape& f, TopLoc_Location& l) {
            return H(Poly_Triangulation)(BRep_Tool::Triangulation(TopoDS::Face(f), l));
        })
        .class_function("IsClosed_s", +[](const TopoDS_Shape& s) { return BRep_Tool::IsClosed(s); })
        .class_function("IsClosed_s", +[](const TopoDS_Shape& e, const TopoDS_Shape& f) {
            return BRep_Tool::IsClosed(TopoDS::Edge(e), TopoDS::Face(f));
        })
        .class_function("Degenerated_s", +[](const TopoDS_Shape& e) { return BRep_Tool::Degenerated(TopoDS::Edge(e)); })
        .class_function("SameParameter_s",
                        +[](const TopoDS_Shape& e) { return BRep_Tool::SameParameter(TopoDS::Edge(e)); })
        .class_function("SameRange_s", +[](const TopoDS_Shape& e) { return BRep_Tool::SameRange(TopoDS::Edge(e)); })
        .class_function("Parameter_s", +[](const TopoDS_Shape& v, const TopoDS_Shape& e) {
            return BRep_Tool::Parameter(TopoDS::Vertex(v), TopoDS::Edge(e));
        })
        .class_function("Parameters_s", +[](const TopoDS_Shape& v, const TopoDS_Shape& f) {
            return BRep_Tool::Parameters(TopoDS::Vertex(v), TopoDS::Face(f));
        });

    class_<Statics<1278>>("BRepTools")
        .class_function("Clean_s", +[](const TopoDS_Shape& s) { BRepTools::Clean(s); })
        .class_function("Clean_s", +[](const TopoDS_Shape& s, bool force) { BRepTools::Clean(s, force); })
        .class_function("UVBounds_s", +[](const TopoDS_Shape& f) {
            double a, b, c, d;
            BRepTools::UVBounds(TopoDS::Face(f), a, b, c, d);
            return arrOf(a, b, c, d);
        })
        .class_function("OuterWire_s", +[](const TopoDS_Shape& f) { return BRepTools::OuterWire(TopoDS::Face(f)); });
    class_<Statics<1287>>("BRepLib")
        .class_function("SameParameter_s", +[](const TopoDS_Shape& s) {
            if (s.ShapeType() == TopAbs_EDGE) BRepLib::SameParameter(TopoDS::Edge(s));
            else BRepLib::SameParameter(s);
        })
        .class_function("SameParameter_s", +[](const TopoDS_Shape& s, double tol) {
            if (s.ShapeType() == TopAbs_EDGE) BRepLib::SameParameter(TopoDS::Edge(s), tol);
            else BRepLib::SameParameter(s, tol);
        })
        .class_function("SameParameter_s",
                        +[](const TopoDS_Shape& s, double tol, bool forced) { BRepLib::SameParameter(s, tol, forced); })
        .class_function("BuildCurves3d_s", +[](const TopoDS_Shape& s) { return BRepLib::BuildCurves3d(s); })
        .class_function("BuildCurves3d_s", +[](const TopoDS_Shape& s, double tol) {
            if (s.ShapeType() == TopAbs_EDGE) return BRepLib::BuildCurve3d(TopoDS::Edge(s), tol);
            return BRepLib::BuildCurves3d(s, tol);
        })
        .class_function("BuildCurve3d_s",
                        +[](const TopoDS_Shape& e, double tol) { return BRepLib::BuildCurve3d(TopoDS::Edge(e), tol); })
        .class_function("OrientClosedSolid_s",
                        +[](TopoDS_Shape& s) { return BRepLib::OrientClosedSolid(TopoDS::Solid(s)); });

    class_<Bnd_Box>("Bnd_Box")
        .constructor<>()
        .function("IsVoid", &Bnd_Box::IsVoid)
        .function("Get", +[](const Bnd_Box& b) {
            double a, c, d, e, f, g;
            b.Get(a, c, d, e, f, g);
            return arrOf(a, c, d, e, f, g);
        })
        .function("Add", +[](Bnd_Box& b, const gp_Pnt& p) { b.Add(p); })
        .function("Enlarge", &Bnd_Box::Enlarge)
        .function("SetGap", &Bnd_Box::SetGap)
        .function("GetGap", &Bnd_Box::GetGap)
        .function("CornerMin", &Bnd_Box::CornerMin)
        .function("CornerMax", &Bnd_Box::CornerMax);
    class_<Statics<1322>>("BRepBndLib")
        .class_function("Add_s", +[](const TopoDS_Shape& s, Bnd_Box& b) { BRepBndLib::Add(s, b); })
        .class_function("Add_s", +[](const TopoDS_Shape& s, Bnd_Box& b, bool tri) { BRepBndLib::Add(s, b, tri); })
        .class_function("AddOptimal_s", +[](const TopoDS_Shape& s, Bnd_Box& b, bool tri, bool tol) {
            BRepBndLib::AddOptimal(s, b, tri, tol);
        });

    class_<GProp_GProps>("GProp_GProps")
        .constructor<>()
        .function("Mass", &GProp_GProps::Mass)
        .function("CentreOfMass", &GProp_GProps::CentreOfMass);
    class_<Statics<1333>>("BRepGProp")
        .class_function("VolumeProperties_s",
                        +[](const TopoDS_Shape& s, GProp_GProps& p) { BRepGProp::VolumeProperties(s, p); })
        // 3 arguments: (S, props, eps) here, returning the error; (S, props, onlyClosed) is __bool.
        .class_function("VolumeProperties_s", +[](const TopoDS_Shape& s, GProp_GProps& p, double eps) {
            return BRepGProp::VolumeProperties(s, p, eps);
        })
        .class_function("VolumeProperties__bool_s", +[](const TopoDS_Shape& s, GProp_GProps& p, bool onlyClosed) {
            BRepGProp::VolumeProperties(s, p, onlyClosed);
        })
        .class_function("VolumeProperties_s", +[](const TopoDS_Shape& s, GProp_GProps& p, double eps, bool onlyClosed) {
            return BRepGProp::VolumeProperties(s, p, eps, onlyClosed);
        })
        .class_function("SurfaceProperties_s",
                        +[](const TopoDS_Shape& s, GProp_GProps& p) { BRepGProp::SurfaceProperties(s, p); })
        .class_function("SurfaceProperties_s", +[](const TopoDS_Shape& s, GProp_GProps& p, double eps) {
            return BRepGProp::SurfaceProperties(s, p, eps);
        })
        .class_function("LinearProperties_s",
                        +[](const TopoDS_Shape& s, GProp_GProps& p) { BRepGProp::LinearProperties(s, p); });

    class_<BRepClass3d_SolidClassifier>("BRepClass3d_SolidClassifier")
        .constructor<const TopoDS_Shape&, const gp_Pnt&, double>()
        .function("State", +[](const BRepClass3d_SolidClassifier& c) { return c.State(); });

    class_<BRepCheck_Analyzer>("BRepCheck_Analyzer")
        .constructor<const TopoDS_Shape&>()
        .constructor<const TopoDS_Shape&, bool>()
        .function("IsValid", +[](const BRepCheck_Analyzer& a) { return a.IsValid(); });
}

// ── builders, sewing, healing, meshing ────────────────────────────────────────
static void bindBuild() {
    class_<BRepBuilderAPI_MakeVertex>("BRepBuilderAPI_MakeVertex")
        .constructor<const gp_Pnt&>()
        .function("Vertex", +[](BRepBuilderAPI_MakeVertex& m) { return TopoDS_Vertex(m.Vertex()); })
        .function("Shape", +[](BRepBuilderAPI_MakeVertex& m) { return TopoDS_Shape(m.Shape()); });

    using ME = BRepBuilderAPI_MakeEdge;
    class_<ME>("BRepBuilderAPI_MakeEdge")
        .constructor(+[](const H(Geom_Curve) & c) { return new ME(c); }, allow_raw_pointers())
        // 2 arguments: (V1, V2) is the constructor; (C2d, Surface) __new_c2dSurf; (P1, P2) __new_pp.
        .constructor(+[](const TopoDS_Shape& a, const TopoDS_Shape& b) {
            return new ME(TopoDS::Vertex(a), TopoDS::Vertex(b));
        }, allow_raw_pointers())
        .class_function("__new_c2dSurf", +[](const H(Geom2d_Curve) & c, const H(Geom_Surface) & s) {
            return new ME(c, s);
        }, allow_raw_pointers(), return_value_policy::take_ownership())
        .class_function("__new_pp", +[](const gp_Pnt& a, const gp_Pnt& b) { return new ME(a, b); },
                        allow_raw_pointers(), return_value_policy::take_ownership())
        // 3 arguments: (C, p1, p2) is the constructor; (C, V1, V2) __new_cvv.
        .constructor(+[](const H(Geom_Curve) & c, double a, double b) { return new ME(c, a, b); }, allow_raw_pointers())
        .class_function("__new_cvv", +[](const H(Geom_Curve) & c, const TopoDS_Shape& a, const TopoDS_Shape& b) {
            return new ME(c, TopoDS::Vertex(a), TopoDS::Vertex(b));
        }, allow_raw_pointers(), return_value_policy::take_ownership())
        .constructor(+[](const H(Geom2d_Curve) & c, const H(Geom_Surface) & s, double a, double b) {
            return new ME(c, s, a, b);
        }, allow_raw_pointers())
        .constructor(+[](const H(Geom_Curve) & c, const TopoDS_Shape& v1, const TopoDS_Shape& v2, double a, double b) {
            return new ME(c, TopoDS::Vertex(v1), TopoDS::Vertex(v2), a, b);
        }, allow_raw_pointers())
        .constructor(+[](const H(Geom2d_Curve) & c, const H(Geom_Surface) & s, const TopoDS_Shape& v1,
                         const TopoDS_Shape& v2, double a,
                         double b) { return new ME(c, s, TopoDS::Vertex(v1), TopoDS::Vertex(v2), a, b); },
                     allow_raw_pointers())
        .function("IsDone", +[](const ME& m) { return m.IsDone(); })
        .function("Error", &ME::Error)
        .function("Edge", +[](ME& m) { return TopoDS_Edge(m.Edge()); })
        .function("Shape", +[](ME& m) { return TopoDS_Shape(m.Shape()); })
        .function("Vertex1", +[](const ME& m) { return TopoDS_Vertex(m.Vertex1()); })
        .function("Vertex2", +[](const ME& m) { return TopoDS_Vertex(m.Vertex2()); });

    using MW = BRepBuilderAPI_MakeWire;
    class_<MW>("BRepBuilderAPI_MakeWire")
        .constructor<>()
        .constructor(+[](const TopoDS_Shape& e) { return new MW(TopoDS::Edge(e)); }, allow_raw_pointers())
        .function("Add", +[](MW& m, const TopoDS_Shape& s) {
            if (s.ShapeType() == TopAbs_WIRE) m.Add(TopoDS::Wire(s));
            else m.Add(TopoDS::Edge(s));
        })
        .function("IsDone", +[](const MW& m) { return m.IsDone(); })
        .function("Wire", +[](MW& m) { return TopoDS_Wire(m.Wire()); })
        .function("Shape", +[](MW& m) { return TopoDS_Shape(m.Shape()); });

    using MF = BRepBuilderAPI_MakeFace;
    class_<MF>("BRepBuilderAPI_MakeFace")
        .constructor<>()
        // 1 argument: a wire (planar face on it) or a face (to add wires to).
        .constructor(+[](const TopoDS_Shape& s) {
            return s.ShapeType() == TopAbs_FACE ? new MF(TopoDS::Face(s)) : new MF(TopoDS::Wire(s));
        }, allow_raw_pointers())
        // 2 arguments: (Surface, tol) is the constructor; (Face, Wire) __new_faceWire;
        // (Wire, onlyPlane) __new_wireBool.
        .constructor(+[](const H(Geom_Surface) & s, double tol) { return new MF(s, tol); }, allow_raw_pointers())
        .class_function("__new_faceWire", +[](const TopoDS_Shape& f, const TopoDS_Shape& w) {
            return new MF(TopoDS::Face(f), TopoDS::Wire(w));
        }, allow_raw_pointers(), return_value_policy::take_ownership())
        .class_function("__new_wireBool", +[](const TopoDS_Shape& w, bool onlyPlane) {
            return new MF(TopoDS::Wire(w), onlyPlane);
        }, allow_raw_pointers(), return_value_policy::take_ownership())
        .constructor(+[](const H(Geom_Surface) & s, const TopoDS_Shape& w, bool inside) {
            return new MF(s, TopoDS::Wire(w), inside);
        }, allow_raw_pointers())
        .constructor(+[](const H(Geom_Surface) & s, double u1, double u2, double v1, double v2, double tol) {
            return new MF(s, u1, u2, v1, v2, tol);
        }, allow_raw_pointers())
        .function("Add", +[](MF& m, const TopoDS_Shape& w) { m.Add(TopoDS::Wire(w)); })
        .function("IsDone", +[](const MF& m) { return m.IsDone(); })
        .function("Error", &MF::Error)
        .function("Face", +[](MF& m) { return TopoDS_Face(m.Face()); })
        .function("Shape", +[](MF& m) { return TopoDS_Shape(m.Shape()); });

    using MS = BRepBuilderAPI_MakeSolid;
    class_<MS>("BRepBuilderAPI_MakeSolid")
        .constructor<>()
        .constructor(+[](const TopoDS_Shape& s) {
            return s.ShapeType() == TopAbs_SOLID ? new MS(TopoDS::Solid(s)) : new MS(TopoDS::Shell(s));
        }, allow_raw_pointers())
        .function("Add", +[](MS& m, const TopoDS_Shape& s) { m.Add(TopoDS::Shell(s)); })
        .function("IsDone", +[](const MS& m) { return m.IsDone(); })
        .function("Solid", +[](MS& m) { return TopoDS_Solid(m.Solid()); })
        .function("Shape", +[](MS& m) { return TopoDS_Shape(m.Shape()); });

    using SW = BRepBuilderAPI_Sewing;
    class_<SW>("BRepBuilderAPI_Sewing")
        .constructor<>()
        .constructor<double>()
        .constructor<double, bool, bool, bool, bool>()
        .function("Add", &SW::Add)
        .function("Perform", +[](SW& s) { s.Perform(); })
        .function("SewedShape", +[](const SW& s) { return TopoDS_Shape(s.SewedShape()); })
        .function("IsModified", &SW::IsModified)
        .function("Modified", +[](const SW& s, const TopoDS_Shape& f) { return TopoDS_Shape(s.Modified(f)); })
        .function("NbFreeEdges", &SW::NbFreeEdges)
        .function("NbMultipleEdges", &SW::NbMultipleEdges)
        .function("NbDegeneratedShapes", &SW::NbDegeneratedShapes);

    class_<BRepBuilderAPI_Copy>("BRepBuilderAPI_Copy")
        .constructor<const TopoDS_Shape&>()
        .constructor<const TopoDS_Shape&, bool, bool>()
        .function("Shape", +[](BRepBuilderAPI_Copy& c) { return TopoDS_Shape(c.Shape()); });
    class_<BRepBuilderAPI_Transform>("BRepBuilderAPI_Transform")
        .constructor<const TopoDS_Shape&, const gp_Trsf&>()
        .constructor<const TopoDS_Shape&, const gp_Trsf&, bool>()
        .function("Shape", +[](BRepBuilderAPI_Transform& c) { return TopoDS_Shape(c.Shape()); });

    using MFill = BRepOffsetAPI_MakeFilling;
    class_<MFill>("BRepOffsetAPI_MakeFilling")
        .constructor<>()
        .function("Add", +[](MFill& m, const gp_Pnt& p) { return m.Add(p); })
        .function("Add", +[](MFill& m, const TopoDS_Shape& e, GeomAbs_Shape o) { return m.Add(TopoDS::Edge(e), o); })
        // 3 arguments: (Edge, Order, IsBound) here; (Edge, SupportFace, Order) is Add__support.
        .function("Add", +[](MFill& m, const TopoDS_Shape& e, GeomAbs_Shape o, bool bound) {
            return m.Add(TopoDS::Edge(e), o, bound);
        })
        .function("Add__support", +[](MFill& m, const TopoDS_Shape& e, const TopoDS_Shape& f, GeomAbs_Shape o) {
            return m.Add(TopoDS::Edge(e), TopoDS::Face(f), o);
        })
        .function("Build", +[](MFill& m) { m.Build(); })
        .function("IsDone", +[](const MFill& m) { return m.IsDone(); })
        .function("Shape", +[](MFill& m) { return TopoDS_Shape(m.Shape()); })
        .function("Face", +[](MFill& m) { return TopoDS::Face(m.Shape()); });

    class_<ShapeBuild_ReShape, base<Standard_Transient>>("ShapeBuild_ReShape")
        .smart_ptr<H(ShapeBuild_ReShape)>("Handle_ShapeBuild_ReShape")
        .constructor(+[]() { return H(ShapeBuild_ReShape)(new ShapeBuild_ReShape()); })
        .function("Apply", +[](ShapeBuild_ReShape& r, const TopoDS_Shape& s) { return r.Apply(s); })
        .function("Replace",
                  +[](ShapeBuild_ReShape& r, const TopoDS_Shape& a, const TopoDS_Shape& b) { r.Replace(a, b); })
        .function("Remove", +[](ShapeBuild_ReShape& r, const TopoDS_Shape& s) { r.Remove(s); })
        .function("Value", +[](const ShapeBuild_ReShape& r, const TopoDS_Shape& s) { return r.Value(s); })
        .function("IsRecorded", +[](const ShapeBuild_ReShape& r, const TopoDS_Shape& s) { return r.IsRecorded(s); });
    class_<ShapeFix_Face, base<Standard_Transient>>("ShapeFix_Face")
        .smart_ptr<H(ShapeFix_Face)>("Handle_ShapeFix_Face")
        .constructor(+[]() { return H(ShapeFix_Face)(new ShapeFix_Face()); })
        .constructor(+[](const TopoDS_Shape& f) { return H(ShapeFix_Face)(new ShapeFix_Face(TopoDS::Face(f))); })
        .function("Init", +[](ShapeFix_Face& x, const TopoDS_Shape& f) { x.Init(TopoDS::Face(f)); })
        .function("SetContext", +[](ShapeFix_Face& x, const H(ShapeBuild_ReShape) & c) { x.SetContext(c); })
        .function("SetPrecision", +[](ShapeFix_Face& x, double p) { x.SetPrecision(p); })
        .function("SetMaxTolerance", +[](ShapeFix_Face& x, double t) { x.SetMaxTolerance(t); })
        .function("Perform", +[](ShapeFix_Face& x) { return x.Perform(); })
        .function("Face", +[](const ShapeFix_Face& x) { return x.Face(); })
        .function("Result", +[](const ShapeFix_Face& x) { return x.Result(); });
    class_<ShapeFix_Shape, base<Standard_Transient>>("ShapeFix_Shape")
        .smart_ptr<H(ShapeFix_Shape)>("Handle_ShapeFix_Shape")
        .constructor(+[](const TopoDS_Shape& s) { return H(ShapeFix_Shape)(new ShapeFix_Shape(s)); })
        .function("SetContext", +[](ShapeFix_Shape& x, const H(ShapeBuild_ReShape) & c) { x.SetContext(c); })
        .function("Perform", +[](ShapeFix_Shape& x) { return x.Perform(); })
        .function("Shape", +[](const ShapeFix_Shape& x) { return x.Shape(); });

    class_<BRepMesh_IncrementalMesh>("BRepMesh_IncrementalMesh")
        .constructor<const TopoDS_Shape&, double>()
        .constructor<const TopoDS_Shape&, double, bool, double, bool>()
        .function("IsDone", +[](const BRepMesh_IncrementalMesh& m) { return m.IsDone(); });
    class_<Poly_Triangle>("Poly_Triangle")
        .constructor<>()
        .function("Get", +[](const Poly_Triangle& t) {
            int a, b, c;
            t.Get(a, b, c);
            return arrOf(a, b, c);
        })
        .function("Value", +[](const Poly_Triangle& t, int i) { return t.Value(i); });
    class_<Poly_Triangulation, base<Standard_Transient>>("Poly_Triangulation")
        .smart_ptr<H(Poly_Triangulation)>("Handle_Poly_Triangulation")
        .function("NbNodes", &Poly_Triangulation::NbNodes)
        .function("NbTriangles", &Poly_Triangulation::NbTriangles)
        .function("Node", +[](const Poly_Triangulation& t, int i) { return gp_Pnt(t.Node(i)); })
        .function("Triangle", +[](const Poly_Triangulation& t, int i) { return Poly_Triangle(t.Triangle(i)); })
        .function("HasNormals", &Poly_Triangulation::HasNormals)
        // Whole arrays in one call: nodes as [x0,y0,z0, x1,...] with `trsf` applied,
        // triangles as [a0,b0,c0, ...] (1-based node indices, as Node(i) takes them).
        .function("NodesXYZ", +[](const Poly_Triangulation& t, const gp_Trsf& trsf) {
            val a = val::array();
            for (int i = 1; i <= t.NbNodes(); ++i) {
                gp_Pnt p = t.Node(i).Transformed(trsf);
                a.call<void>("push", p.X());
                a.call<void>("push", p.Y());
                a.call<void>("push", p.Z());
            }
            return a;
        })
        .function("TrianglesIdx", +[](const Poly_Triangulation& t) {
            val a = val::array();
            for (int i = 1; i <= t.NbTriangles(); ++i) {
                int x, y, z;
                t.Triangle(i).Get(x, y, z);
                a.call<void>("push", x);
                a.call<void>("push", y);
                a.call<void>("push", z);
            }
            return a;
        });
}

EMSCRIPTEN_BINDINGS(meshweave_geom) {
    bindGp();
    bindEnums();
    bindGeom();
    bindArrays();
    bindAlgo();
    bindTopo();
    bindBuild();
    function("mwWhat", &what);
    function("mwStoreShape", &storeShape);
    function("mwGetShape", &getShape);
    function("mwShapeKey", &shapeKey);
    function("mwTShapeKey", &keyOf);
}
