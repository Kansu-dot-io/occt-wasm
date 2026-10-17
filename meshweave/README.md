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
- `.gitmodules` points at [Kansu-dot-io/OCCT](https://github.com/Kansu-dot-io/OCCT),
  a mirror of the OCCT commit upstream uses (`git ls-tree HEAD occt`): OCCT
  8.0.1 plus two upstream patches. MeshWeave does not modify OCCT.

## Licences

The facade is MIT OR Apache-2.0 (LICENSE-MIT, LICENSE-APACHE). OpenCASCADE is
LGPL-2.1 with the Open CASCADE exception. To rebuild the exact module, or to
relink it against a modified OCCT, run `meshweave/build.sh` (docker); the
published package and its checksum are on this repository's Releases page.
