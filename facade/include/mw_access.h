// MeshWeave: the one door into the kernel's private shape arena, shared by
// mw_edit.cpp and mw_read.cpp. occt_kernel.h names MwEdit a friend.
#pragma once
#include "occt_kernel.h"

struct MwEdit {
    static uint32_t store(OcctKernel& k, const TopoDS_Shape& s) { return k.store(s); }
    static const TopoDS_Shape& get(OcctKernel& k, uint32_t id) { return k.get(id); }
};
