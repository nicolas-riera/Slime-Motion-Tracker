#include "slimelite/linalg.h"

#include <algorithm>
#include <numeric>

namespace slimelite {

Mat3 Mat3::inverse() const {
	double d = det();
	Mat3 r;
	if (std::fabs(d) < 1e-15) return r;
	r.m = {(m[4] * m[8] - m[5] * m[7]) / d, (m[2] * m[7] - m[1] * m[8]) / d,
		   (m[1] * m[5] - m[2] * m[4]) / d, (m[5] * m[6] - m[3] * m[8]) / d,
		   (m[0] * m[8] - m[2] * m[6]) / d, (m[2] * m[3] - m[0] * m[5]) / d,
		   (m[3] * m[7] - m[4] * m[6]) / d, (m[1] * m[6] - m[0] * m[7]) / d,
		   (m[0] * m[4] - m[1] * m[3]) / d};
	return r;
}

Quat Quat::fromTwoVectors(const Vec3& a, const Vec3& b) {
	Vec3 u = a.normalized(), v = b.normalized();
	double d = u.dot(v);
	if (d < -0.999999) {
		Vec3 axis = Vec3{1, 0, 0}.cross(u);
		if (axis.norm() < 1e-6) axis = Vec3{0, 1, 0}.cross(u);
		return fromAxisAngle(axis, kPi);
	}
	Vec3 c = u.cross(v);
	return Quat{1 + d, c.x, c.y, c.z}.normalized();
}

Quat Quat::fromMatrix(const Mat3& m) {
	double tr = m(0, 0) + m(1, 1) + m(2, 2);
	Quat q;
	if (tr > 0) {
		double s = std::sqrt(tr + 1.0) * 2;
		q = {0.25 * s, (m(2, 1) - m(1, 2)) / s, (m(0, 2) - m(2, 0)) / s, (m(1, 0) - m(0, 1)) / s};
	} else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
		double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
		q = {(m(2, 1) - m(1, 2)) / s, 0.25 * s, (m(0, 1) + m(1, 0)) / s, (m(0, 2) + m(2, 0)) / s};
	} else if (m(1, 1) > m(2, 2)) {
		double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
		q = {(m(0, 2) - m(2, 0)) / s, (m(0, 1) + m(1, 0)) / s, 0.25 * s, (m(1, 2) + m(2, 1)) / s};
	} else {
		double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
		q = {(m(1, 0) - m(0, 1)) / s, (m(0, 2) + m(2, 0)) / s, (m(1, 2) + m(2, 1)) / s, 0.25 * s};
	}
	return q.normalized();
}

Vec3 Quat::toRotationVector() const {
	Quat q = w < 0 ? Quat{-w, -x, -y, -z} : *this;
	double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
	if (s < 1e-12) return {2 * q.x, 2 * q.y, 2 * q.z};
	double a = 2 * std::atan2(s, q.w);
	return Vec3{q.x, q.y, q.z} * (a / s);
}

Quat slerp(const Quat& a, Quat b, double t) {
	double d = a.dot(b);
	if (d < 0) {
		b = {-b.w, -b.x, -b.y, -b.z};
		d = -d;
	}
	if (d > 0.9995) {
		return Quat{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
					a.z + (b.z - a.z) * t}
			.normalized();
	}
	double th = std::acos(d);
	double sa = std::sin((1 - t) * th) / std::sin(th), sb = std::sin(t * th) / std::sin(th);
	return Quat{a.w * sa + b.w * sb, a.x * sa + b.x * sb, a.y * sa + b.y * sb, a.z * sa + b.z * sb};
}

Quat yawOnly(const Quat& q) {
	Quat t{q.w, 0, q.y, 0};
	double n = std::sqrt(t.w * t.w + t.y * t.y);
	if (n < 1e-9) {
		// Pure 180 degree swing: fall back to the heading of the forward axis
		return Quat::yaw(q.yawAngle());
	}
	return {t.w / n, 0, t.y / n, 0};
}

void symmetricEigen(int n, std::vector<double> a, std::vector<double>& values,
					std::vector<double>& vectors) {
	std::vector<double> v(n * n, 0.0);
	for (int i = 0; i < n; i++) v[i * n + i] = 1.0;
	for (int sweep = 0; sweep < 100; sweep++) {
		double off = 0;
		for (int p = 0; p < n; p++)
			for (int q = p + 1; q < n; q++) off += a[p * n + q] * a[p * n + q];
		if (off < 1e-30) break;
		for (int p = 0; p < n; p++) {
			for (int q = p + 1; q < n; q++) {
				double apq = a[p * n + q];
				if (std::fabs(apq) < 1e-300) continue;
				double app = a[p * n + p], aqq = a[q * n + q];
				double theta = (aqq - app) / (2 * apq);
				double t = (theta >= 0 ? 1.0 : -1.0)
						 / (std::fabs(theta) + std::sqrt(theta * theta + 1));
				double c = 1 / std::sqrt(t * t + 1), s = t * c;
				for (int k = 0; k < n; k++) {
					double akp = a[k * n + p], akq = a[k * n + q];
					a[k * n + p] = c * akp - s * akq;
					a[k * n + q] = s * akp + c * akq;
				}
				for (int k = 0; k < n; k++) {
					double apk = a[p * n + k], aqk = a[q * n + k];
					a[p * n + k] = c * apk - s * aqk;
					a[q * n + k] = s * apk + c * aqk;
				}
				for (int k = 0; k < n; k++) {
					double vkp = v[k * n + p], vkq = v[k * n + q];
					v[k * n + p] = c * vkp - s * vkq;
					v[k * n + q] = s * vkp + c * vkq;
				}
			}
		}
	}
	std::vector<int> idx(n);
	std::iota(idx.begin(), idx.end(), 0);
	std::sort(idx.begin(), idx.end(), [&](int i, int j) { return a[i * n + i] < a[j * n + j]; });
	values.assign(n, 0);
	vectors.assign(n * n, 0);
	for (int r = 0; r < n; r++) {
		values[r] = a[idx[r] * n + idx[r]];
		for (int k = 0; k < n; k++) vectors[r * n + k] = v[k * n + idx[r]];
	}
}

}  // namespace slimelite
