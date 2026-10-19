// Small software 3D renderer on top of ImDrawList (skeleton, tracker and mag views):
// an orbit camera, perspective projection and depth-sorted primitives.
#pragma once

#include <vector>

#include "driver_link.h"  // Vec3f, Quatf
#include "imgui.h"

namespace smt::ui::v3 {

struct V3 {
    float x = 0, y = 0, z = 0;
    V3() = default;
    V3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    V3(const Vec3f& v) : x(v.x), y(v.y), z(v.z) {}  // NOLINT: implicit on purpose
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float length(V3 a);
V3 normalized(V3 a);
V3 rotate(const Quatf& q, V3 v);
Quatf conj(const Quatf& q);
V3 lerp(V3 a, V3 b, float t);

struct Camera {
    float yaw = 0.0f;  // around +Y; 0 = looking toward -Z from +Z
    float pitch = 0.2f;  // positive = looking down from above
    float distance = 3.0f;
    float fov = 0.75f;  // vertical, radians
    V3 target;
};

// Orbit interaction over the item region just submitted (drag: rotate, wheel: zoom,
// double click: reset to the defaults). Returns true while the user interacts.
struct Orbit {
    float yaw = 0.0f, pitch = 0.0f;  // offsets added by the user
    float zoom = 1.0f;  // distance multiplier
    double lastInteraction = -1e9;  // ImGui time
    bool update(const char* id, ImVec2 size, float minZoom = 0.35f, float maxZoom = 3.0f);
    void reset() { yaw = pitch = 0.0f, zoom = 1.0f; }
};

class Scene {
public:
    void begin(ImDrawList* dl, ImVec2 min, ImVec2 max, const Camera& cam);
    // Projects a world point; false when behind the camera. depth: distance along the view
    bool project(V3 p, ImVec2& out, float* depth = nullptr) const;
    // Pixels per meter at a given depth
    float scaleAt(float depth) const { return m_focal / depth; }

    // Background layer: drawn first, in order (floor grid)
    void bgLine(V3 a, V3 b, ImU32 col, float width);
    // Depth sorted layer
    void line(V3 a, V3 b, ImU32 col, float width, bool perspectiveWidth = false);
    void sphere(V3 c, float radius, ImU32 col, bool shaded = true);
    void point(V3 c, float pixels, ImU32 col);
    void ring(V3 c, float pixels, ImU32 col, float width);
    void arrow(V3 from, V3 to, ImU32 col, float width);
    void quad(V3 a, V3 b, V3 c, V3 d, ImU32 fill, ImU32 outline = 0);
    void label(V3 p, const char* text, ImU32 col);
    // Oriented box (half extents in its local frame), with per-face shading
    void box(V3 center, const Quatf& q, V3 half, ImU32 color, ImU32 topColor);
    // Horizontal disc on the background layer (shadows)
    void bgDisc(V3 center, float radius, ImU32 col);
    // Circle (outline) in the plane normal to `axis`, sorted per segment
    void circle(V3 center, V3 axis, float radius, ImU32 col, float width);
    // Floor grid around center (y = height)
    void grid(V3 center, float halfSize, float step, ImU32 col);

    void flush();

    [[nodiscard]] V3 eye() const { return m_eye; }
    [[nodiscard]] V3 forward() const { return m_f; }

private:
    enum class Kind { Line, Sphere, Dot, Ring, Quad, Label, Arrow };
    struct Prim {
        Kind kind;
        float depth;
        ImVec2 a, b, c, d;
        float size;
        ImU32 col, col2;
        const char* text;
        bool flag;
    };
    bool clipToView(V3& a, V3& b) const;

    ImDrawList* m_dl = nullptr;
    ImVec2 m_min, m_max, m_center;
    float m_focal = 1.0f;
    V3 m_eye, m_r, m_u, m_f;
    std::vector<Prim> m_prims;
};

}  // namespace smt::ui::v3
