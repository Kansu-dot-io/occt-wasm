# MeshWeave's build of occt-wasm

meshweave.app runs this package in the browser (its Edit workspace and its
in-browser STEP/BREP reader). This branch is upstream
[andymai/occt-wasm](https://github.com/andymai/occt-wasm) 5.6.0 plus:

- `facade/src/mw_edit.cpp` — index-based Modified/Generated/IsDeleted face
  history for every editing operation (splitter, defeaturing, filling, fillet,
  chamfer, shell, draft, booleans, unify, shape fix, transforms, patterns), a
  face- and edge-indexed tessellation, topology and measurement. Bound as free
  functions (`Module.mwSplitPlane(kernel, ...)` etc.) in their own
  `EMSCRIPTEN_BINDINGS` block.
- one `friend struct MwEdit;` line in `facade/include/occt_kernel.h`.
- `facade/src/mw_read.cpp` (since v5.6.0-mw.4) — reading open formats on the
  device: IGES through OCCT's IGES translator (`mwImportIges`; `libTKDEIGES.a`
  is linked again), a trimmed NURBS face from poles/knots/edges for the Rhino
  .3dm reader (`mwNurbsFace`), sewing into solids with cavities kept
  (`mwSewBody`, `mwSolidify`) and a shell report (`mwShellReport`).
  `facade/include/mw_access.h` holds the `MwEdit` arena accessor both files use.
- `facade/src/mw_geom.cpp` (since v5.6.0-mw.5) — generic OCCT geometry and
  topology bindings for MeshWeave's device-side readers: the gp value types,
  Geom/Geom2d curves and surfaces as `opencascade::handle<T>` embind smart
  pointers, TopoDS shapes, BRep_Builder/BRep_Tool/TopExp, the BRepBuilderAPI
  makers, sewing, ShapeFix_Face, projections, GeomAPI_IntSS, interpolation,
  mass properties, BRepMesh and Poly_Triangulation, under OCCT's own names
  (statics with a `_s` suffix, tuple out-parameters as arrays). Plus
  `mwStoreShape`/`mwGetShape` (a shape into / out of the kernel arena) and
  `mwWhat` (the class and message of an OCCT exception that reached JS).
- `.gitmodules` points at [Kansu-dot-io/OCCT](https://github.com/Kansu-dot-io/OCCT),
  a mirror of the OCCT commit upstream uses (`git ls-tree HEAD occt`): OCCT
  8.0.1 plus two upstream patches. MeshWeave does not modify OCCT.

## Licences

The facade is MIT OR Apache-2.0 (LICENSE-MIT, LICENSE-APACHE). OpenCASCADE is
LGPL-2.1 with the Open CASCADE exception. To rebuild the exact module, or to
relink it against a modified OCCT, run `meshweave/build.sh` (docker); the
published package and its checksum are on this repository's Releases page.
