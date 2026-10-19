#include "view3d.h"

#include <algorithm>
#include <cmath>

#include "widgets.h"

namespace smt::ui::v3 {

namespace {

constexpr float kNear = 0.02f;

ImVec2 add(ImVec2 a, ImVec2 b) { return {a.x + b.x, a.y + b.y}; }
ImVec2 sub(ImVec2 a, ImVec2 b) { return {a.x - b.x, a.y - b.y}; }
ImVec2 mul(ImVec2 a, float s) { return {a.x * s, a.y * s}; }

ImU32 shade(ImU32 c, float k) {
    const auto ch = [&](int shift) {
        const float v = static_cast<float>((c >> shift) & 0xFF) * k;
        return static_cast<ImU32>(std::clamp(v, 0.0f, 255.0f)) << shift;
    };
    return ch(IM_COL32_R_SHIFT) | ch(IM_COL32_G_SHIFT) | ch(IM_COL32_B_SHIFT) | (c & IM_COL32_A_MASK);
}

}  // namespace

float length(V3 a) { return std::sqrt(dot(a, a)); }

V3 normalized(V3 a) {
    const float n = length(a);
    return n > 1e-9f ? a * (1.0f / n) : V3{0, 0, 0};
}

V3 rotate(const Quatf& q, V3 v) {
    // v' = v + 2w (u x v) + 2 u x (u x v)
    const V3 u{q.x, q.y, q.z};
    const V3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}

Quatf conj(const Quatf& q) { return {q.w, -q.x, -q.y, -q.z}; }

V3 lerp(V3 a, V3 b, float t) { return a + (b - a) * t; }

// ---------------------------------------------------------------- orbit

bool Orbit::update(const char* id, ImVec2 size, float minZoom, float maxZoom) {
    ImGui::InvisibleButton(id, ImVec2(std::max(size.x, 1.0f), std::max(size.y, 1.0f)),
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImGuiIO& io = ImGui::GetIO();
    bool interacting = false;
    if (hovered) ImGui::SetItemKeyOwner(ImGuiKey_MouseWheelY);
    if (active && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)) {
        yaw -= io.MouseDelta.x * 0.009f;
        pitch += io.MouseDelta.y * 0.007f;
        pitch = std::clamp(pitch, -1.2f, 1.2f);
        interacting = true;
    }
    if (hovered && io.MouseWheel != 0.0f) {
        zoom = std::clamp(zoom * std::pow(0.87f, io.MouseWheel), minZoom, maxZoom);
        interacting = true;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        reset();
        interacting = true;
    }
    if (active) interacting = true;
    if (interacting) lastInteraction = ImGui::GetTime();
    if (hovered && !active) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    return interacting;
}

// ---------------------------------------------------------------- scene

void Scene::begin(ImDrawList* dl, ImVec2 min, ImVec2 max, const Camera& cam) {
    m_dl = dl;
    m_min = min;
    m_max = max;
    m_center = ImVec2((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f);
    m_focal = (max.y - min.y) * 0.5f / std::tan(cam.fov * 0.5f);
    const float cp = std::cos(cam.pitch), sp = std::sin(cam.pitch);
    const V3 dir{cp * std::sin(cam.yaw), sp, cp * std::cos(cam.yaw)};
    m_eye = cam.target + dir * cam.distance;
    m_f = normalized(cam.target - m_eye);
    m_r = normalized(cross(m_f, V3{0, 1, 0}));
    if (length(m_r) < 0.5f) m_r = V3{1, 0, 0};
    m_u = cross(m_r, m_f);
    m_prims.clear();
    dl->PushClipRect(min, max, true);
}

bool Scene::project(V3 p, ImVec2& out, float* depth) const {
    const V3 d = p - m_eye;
    const float z = dot(d, m_f);
    if (depth) *depth = z;
    if (z < kNear) return false;
    out = ImVec2(m_center.x + dot(d, m_r) / z * m_focal, m_center.y - dot(d, m_u) / z * m_focal);
    return true;
}

bool Scene::clipToView(V3& a, V3& b) const {
    const float za = dot(a - m_eye, m_f), zb = dot(b - m_eye, m_f);
    if (za < kNear && zb < kNear) return false;
    if (za < kNear) a = lerp(a, b, (kNear - za) / (zb - za));
    if (zb < kNear) b = lerp(b, a, (kNear - zb) / (za - zb));
    return true;
}

void Scene::bgLine(V3 a, V3 b, ImU32 col, float width) {
    if (!clipToView(a, b)) return;
    ImVec2 pa, pb;
    if (project(a, pa) && project(b, pb)) m_dl->AddLine(pa, pb, col, width);
}

void Scene::line(V3 a, V3 b, ImU32 col, float width, bool perspectiveWidth) {
    if (!clipToView(a, b)) return;
    ImVec2 pa, pb;
    float da = 0, db = 0;
    if (!project(a, pa, &da) || !project(b, pb, &db)) return;
    const float depth = (da + db) * 0.5f;
    const float w = perspectiveWidth ? std::max(1.0f, width * m_focal / depth) : width;
    m_prims.push_back({Kind::Line, depth, pa, pb, {}, {}, w, col, 0, nullptr, false});
}

void Scene::sphere(V3 c, float radius, ImU32 col, bool shaded) {
    ImVec2 p;
    float d = 0;
    if (!project(c, p, &d)) return;
    m_prims.push_back({Kind::Sphere, d, p, {}, {}, {}, std::max(1.5f, radius * m_focal / d), col, 0, nullptr, shaded});
}

void Scene::point(V3 c, float pixels, ImU32 col) {
    ImVec2 p;
    float d = 0;
    if (!project(c, p, &d)) return;
    m_prims.push_back({Kind::Dot, d, p, {}, {}, {}, pixels, col, 0, nullptr, false});
}

void Scene::ring(V3 c, float pixels, ImU32 col, float width) {
    ImVec2 p;
    float d = 0;
    if (!project(c, p, &d)) return;
    m_prims.push_back({Kind::Ring, d - 1e-4f, p, {}, {}, {}, pixels, col, 0, nullptr, false});
    m_prims.back().c.x = width;
}

void Scene::arrow(V3 from, V3 to, ImU32 col, float width) {
    ImVec2 pa, pb;
    float da = 0, db = 0;
    if (!project(from, pa, &da) || !project(to, pb, &db)) return;
    m_prims.push_back({Kind::Arrow, (da + db) * 0.5f, pa, pb, {}, {}, width, col, 0, nullptr, false});
}

void Scene::quad(V3 a, V3 b, V3 c, V3 d, ImU32 fill, ImU32 outline) {
    ImVec2 pa, pb, pc, pd;
    float za = 0, zb = 0, zc = 0, zd = 0;
    if (!project(a, pa, &za) || !project(b, pb, &zb) || !project(c, pc, &zc) || !project(d, pd, &zd)) return;
    m_prims.push_back({Kind::Quad, (za + zb + zc + zd) * 0.25f, pa, pb, pc, pd, 0.0f, fill, outline, nullptr, false});
}

void Scene::label(V3 p, const char* text, ImU32 col) {
    ImVec2 s;
    float d = 0;
    if (!project(p, s, &d)) return;
    m_prims.push_back({Kind::Label, d - 1e-3f, s, {}, {}, {}, 0.0f, col, 0, text, false});
}

void Scene::box(V3 center, const Quatf& q, V3 half, ImU32 color, ImU32 topColor) {
    // Faces: outward local normal, then 4 corners (sign patterns)
    struct Face {
        V3 n;
        int corners[4][3];
    };
    static const Face kFaces[6] = {
        {{1, 0, 0}, {{1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}}},
        {{-1, 0, 0}, {{-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1}, {-1, 1, -1}}},
        {{0, 1, 0}, {{-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}, {1, 1, -1}}},
        {{0, -1, 0}, {{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}}},
        {{0, 0, 1}, {{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}}},
        {{0, 0, -1}, {{-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}, {1, -1, -1}}},
    };
    const V3 light = normalized(V3{0.35f, 1.0f, 0.55f});
    for (int f = 0; f < 6; ++f) {
        const Face& face = kFaces[f];
        const V3 n = rotate(q, face.n);
        V3 pts[4];
        for (int k = 0; k < 4; ++k) {
            const V3 local{half.x * static_cast<float>(face.corners[k][0]), half.y * static_cast<float>(face.corners[k][1]),
                           half.z * static_cast<float>(face.corners[k][2])};
            pts[k] = center + rotate(q, local);
        }
        const V3 mid = (pts[0] + pts[2]) * 0.5f;
        if (dot(n, mid - m_eye) >= 0.0f) continue;  // back face
        const float k = 0.50f + 0.50f * std::max(0.0f, dot(n, light));
        const ImU32 base = (face.n.z > 0.5f) ? topColor : color;
        quad(pts[0], pts[1], pts[2], pts[3], shade(base, k), alpha(col::white, 0.10f));
    }
}

void Scene::bgDisc(V3 center, float radius, ImU32 col) {
    constexpr int kN = 24;
    ImVec2 pts[kN];
    for (int i = 0; i < kN; ++i) {
        const float a = 6.2831853f * static_cast<float>(i) / kN;
        if (!project(center + V3{std::cos(a) * radius, 0.0f, std::sin(a) * radius}, pts[i])) return;
    }
    m_dl->AddConvexPolyFilled(pts, kN, col);
}

void Scene::circle(V3 center, V3 axis, float radius, ImU32 col, float width) {
    axis = normalized(axis);
    V3 t = std::fabs(axis.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0};
    const V3 u = normalized(cross(axis, t));
    const V3 v = cross(axis, u);
    constexpr int kN = 48;
    V3 prev = center + u * radius;
    for (int i = 1; i <= kN; ++i) {
        const float a = 6.2831853f * static_cast<float>(i) / kN;
        const V3 cur = center + (u * std::cos(a) + v * std::sin(a)) * radius;
        line(prev, cur, col, width);
        prev = cur;
    }
}

void Scene::grid(V3 center, float halfSize, float step, ImU32 col) {
    const int n = static_cast<int>(halfSize / step);
    constexpr int kSeg = 8;
    for (int axis = 0; axis < 2; ++axis) {
        for (int i = -n; i <= n; ++i) {
            const float off = static_cast<float>(i) * step;
            for (int s = 0; s < kSeg; ++s) {
                const float t0 = -halfSize + 2 * halfSize * static_cast<float>(s) / kSeg;
                const float t1 = -halfSize + 2 * halfSize * static_cast<float>(s + 1) / kSeg;
                const float tm = (t0 + t1) * 0.5f;
                const float r = std::sqrt(off * off + tm * tm) / halfSize;
                const float fade = std::clamp(1.0f - r, 0.0f, 1.0f);
                if (fade <= 0.0f) continue;
                V3 a = axis == 0 ? V3{center.x + off, center.y, center.z + t0} : V3{center.x + t0, center.y, center.z + off};
                V3 b = axis == 0 ? V3{center.x + off, center.y, center.z + t1} : V3{center.x + t1, center.y, center.z + off};
                bgLine(a, b, alpha(col, fade * fade), i == 0 ? 1.6f : 1.0f);
            }
        }
    }
}

void Scene::flush() {
    std::stable_sort(m_prims.begin(), m_prims.end(), [](const Prim& a, const Prim& b) { return a.depth > b.depth; });
    // Style alpha (dimmed while disabled)
    const float styleAlpha = ImGui::GetStyle().Alpha;
    if (styleAlpha < 1.0f)
        for (Prim& p : m_prims) {
            p.col = alpha(p.col, styleAlpha);
            if (p.col2) p.col2 = alpha(p.col2, styleAlpha);
        }
    for (const Prim& p : m_prims) {
        switch (p.kind) {
        case Kind::Line:
            m_dl->AddLine(p.a, p.b, p.col, p.size);
            // Round caps
            if (p.size > 3.0f) {
                m_dl->AddCircleFilled(p.a, p.size * 0.5f, p.col, 12);
                m_dl->AddCircleFilled(p.b, p.size * 0.5f, p.col, 12);
            }
            break;
        case Kind::Sphere: {
            const float r = p.size;
            if (p.flag) {
                m_dl->AddCircleFilled(p.a, r, shade(p.col, 0.62f), 24);
                m_dl->AddCircleFilled(sub(p.a, ImVec2(r * 0.10f, r * 0.10f)), r * 0.84f, p.col, 24);
                m_dl->AddCircleFilled(sub(p.a, ImVec2(r * 0.32f, r * 0.34f)), r * 0.30f, mix(p.col, col::white, 0.45f),
                                      16);
            } else {
                m_dl->AddCircleFilled(p.a, r, p.col, 20);
            }
            break;
        }
        case Kind::Dot:
            m_dl->AddCircleFilled(p.a, p.size, p.col, 10);
            break;
        case Kind::Ring:
            m_dl->AddCircle(p.a, p.size, p.col, 20, p.c.x);
            break;
        case Kind::Quad:
            m_dl->AddQuadFilled(p.a, p.b, p.c, p.d, p.col);
            if (p.col2) m_dl->AddQuad(p.a, p.b, p.c, p.d, p.col2, 1.0f);
            break;
        case Kind::Label:
            m_dl->AddText(fonts.small, fonts.small->FontSize, add(p.a, ImVec2(6, -8)), p.col, p.text);
            break;
        case Kind::Arrow: {
            const ImVec2 d = sub(p.b, p.a);
            const float len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len < 1.0f) {
                m_dl->AddCircleFilled(p.a, p.size, p.col, 10);
                break;
            }
            const ImVec2 u = mul(d, 1.0f / len);
            const ImVec2 n(-u.y, u.x);
            const float head = std::min(len * 0.45f, 7.0f + p.size * 2.2f);
            const ImVec2 base = sub(p.b, mul(u, head));
            m_dl->AddLine(p.a, base, p.col, p.size);
            m_dl->AddTriangleFilled(p.b, add(base, mul(n, head * 0.5f)), sub(base, mul(n, head * 0.5f)), p.col);
            break;
        }
        }
    }
    m_prims.clear();
    m_dl->PopClipRect();
}

}  // namespace smt::ui::v3
