#!/usr/bin/env bash
# Rebuild the occt-wasm package MeshWeave ships (meshweave.app), from this fork.
#
# This is the LGPL-2.1 relinking path for the OpenCASCADE code inside
# occt-wasm.wasm: anyone can rebuild the exact module, or relink it against a
# modified OCCT, with this script. What goes in:
#   - OCCT: the `occt` submodule (github.com/Kansu-dot-io/OCCT, a mirror of
#     github.com/andymai/OCCT), pinned by commit. MeshWeave does not modify it.
#   - the facade (MIT OR Apache-2.0): upstream occt-wasm, plus the hand-written
#     facade/src/mw_edit.cpp and one `friend` line in facade/include/occt_kernel.h.
#
# Usage:  meshweave/build.sh [out-dir]          (needs docker)
#   FROM_SOURCE=1 meshweave/build.sh            builds the OCCT static libs from
#                                               occt/ first (~1 h; needs the
#                                               submodule checked out) instead of
#                                               using the prebuilt builder image.
# Output: <out-dir>/occt-wasm-<version>.tgz (the npm package) and SHA256SUMS.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(mkdir -p "${1:-$ROOT/meshweave/out}" && cd "${1:-$ROOT/meshweave/out}" && pwd)"
OCCT_REV="$(git -C "$ROOT" rev-parse --short=7 HEAD:occt)"

if [[ "${FROM_SOURCE:-0}" == 1 ]]; then
    IMG="mw-occt-wasm-builder:${OCCT_REV}"
    docker build -f "$ROOT/Dockerfile.builder" -t "$IMG" "$ROOT"
else
    # Upstream's prebuilt OCCT static libs for exactly this OCCT commit
    # (Dockerfile.builder, tagged with the submodule's short rev).
    IMG="ghcr.io/andymai/occt-wasm-builder:${OCCT_REV}"
    docker pull "$IMG"
fi

docker run --rm -v "$ROOT":/src:ro -v "$OUT":/out "$IMG" bash -euo pipefail -c '
    cd /workspace
    for p in .cargo Cargo.toml Cargo.lock rust-toolchain.toml xtask crate facade scripts ts package.json package-lock.json README.md; do
        rm -rf "./$p"; cp -a "/src/$p" "./$p"
    done
    rm -rf ts/node_modules ts/dist dist
    cargo xtask build --release
    cd ts
    npm ci --no-audit --no-fund
    npm run build
    npm pack --pack-destination /out
    cd /out && sha256sum *.tgz > SHA256SUMS
    cp /workspace/dist/occt-wasm.wasm /out/ && sha256sum occt-wasm.wasm >> SHA256SUMS
    chown -R "$(stat -c %u:%g /out)" /out
'
cat "$OUT/SHA256SUMS"
