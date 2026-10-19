// Small 3D math helpers. World frame is SteamVR's: right-handed, +Y up, -Z forward.
#pragma once

#include <array>
#include <cmath>
#include <vector>

namespace smt {

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }
// Wraps an angle to (-pi, pi]
inline double wrapAngle(double a) {
	a = std::fmod(a + kPi, 2 * kPi);
	if (a < 0) a += 2 * kPi;
	return a - kPi;
}

struct Vec3 {
	double x = 0, y = 0, z = 0;

	Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3 operator-() const { return {-x, -y, -z}; }
	Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
	Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
	Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
	Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
	double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
	double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

	double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
	Vec3 cross(const Vec3& o) const {
		return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
	}
	double norm() const { return std::sqrt(dot(*this)); }
	Vec3 normalized() const {
		double n = norm();
		return n > 1e-12 ? *this / n : Vec3{};
	}
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

struct Mat3 {
	// Row-major
	std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};

	static Mat3 zero() { Mat3 r; r.m.fill(0); return r; }
	double operator()(int r, int c) const { return m[r * 3 + c]; }
	double& operator()(int r, int c) { return m[r * 3 + c]; }
	Vec3 operator*(const Vec3& v) const {
		return {m[0] * v.x + m[1] * v.y + m[2] * v.z,
				m[3] * v.x + m[4] * v.y + m[5] * v.z,
				m[6] * v.x + m[7] * v.y + m[8] * v.z};
	}
	Mat3 operator*(const Mat3& o) const {
		Mat3 r = zero();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++) r(i, j) += (*this)(i, k) * o(k, j);
		return r;
	}
	Mat3 operator*(double s) const { Mat3 r = *this; for (auto& v : r.m) v *= s; return r; }
	Mat3 transposed() const {
		Mat3 r;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) r(i, j) = (*this)(j, i);
		return r;
	}
	double det() const {
		return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
			 + m[2] * (m[3] * m[7] - m[4] * m[6]);
	}
	Mat3 inverse() const;
};

struct Quat {
	double w = 1, x = 0, y = 0, z = 0;

	static Quat fromAxisAngle(const Vec3& axis, double angle) {
		Vec3 a = axis.normalized();
		double s = std::sin(angle / 2);
		return {std::cos(angle / 2), a.x * s, a.y * s, a.z * s};
	}
	static Quat yaw(double angle) { return fromAxisAngle({0, 1, 0}, angle); }
	// Shortest rotation taking unit vector a onto unit vector b
	static Quat fromTwoVectors(const Vec3& a, const Vec3& b);
	static Quat fromMatrix(const Mat3& m);

	Quat operator*(const Quat& q) const {
		return {w * q.w - x * q.x - y * q.y - z * q.z,
				w * q.x + x * q.w + y * q.z - z * q.y,
				w * q.y - x * q.z + y * q.w + z * q.x,
				w * q.z + x * q.y - y * q.x + z * q.w};
	}
	Quat conj() const { return {w, -x, -y, -z}; }
	double dot(const Quat& q) const { return w * q.w + x * q.x + y * q.y + z * q.z; }
	Quat normalized() const {
		double n = std::sqrt(dot(*this));
		return n > 1e-12 ? Quat{w / n, x / n, y / n, z / n} : Quat{};
	}
	Vec3 rotate(const Vec3& v) const {
		// v + 2w(u x v) + 2u x (u x v)
		Vec3 u{x, y, z};
		Vec3 t = u.cross(v) * 2.0;
		return v + t * w + u.cross(t);
	}
	Mat3 toMatrix() const {
		Mat3 r;
		r.m = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
			   2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
			   2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
		return r;
	}
	// Rotation angle in [0, pi]
	double angle() const { return 2 * std::acos(std::fmin(1.0, std::fabs(w))); }
	// Heading of the rotated -Z (forward) axis around +Y, in radians
	double yawAngle() const {
		Vec3 f = rotate({0, 0, -1});
		return std::atan2(-f.x, -f.z);
	}
	// Rotation vector (axis * angle), shortest path
	Vec3 toRotationVector() const;
	static Quat fromRotationVector(const Vec3& r) {
		double a = r.norm();
		return a < 1e-12 ? Quat{} : fromAxisAngle(r / a, a);
	}
};

Quat slerp(const Quat& a, Quat b, double t);
// Keeps only the rotation around +Y (swing-twist decomposition)
Quat yawOnly(const Quat& q);

// Eigen decomposition of a symmetric n x n matrix (row-major), Jacobi method.
// Returns eigenvalues ascending; eigenvectors as rows in the same order.
void symmetricEigen(int n, std::vector<double> a, std::vector<double>& values,
					std::vector<double>& vectors);

}  // namespace smt
