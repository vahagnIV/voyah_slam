#!/usr/bin/env python3
"""Absolute trajectory error of a full_slam trajectory against TUM-VI mocap.

Usage: tools/ate.py <trajectory> <groundtruth.csv> [--align sim3|se3] [--max_dt S]

<trajectory>: TUM lines `timestamp tx ty tz qx qy qz qw` (seconds,
camera-to-world) as written by `--trajectory_out`; `# map <id>` lines start
a map's section. Only the largest section is evaluated.

<groundtruth.csv>: TUM-VI mocap (`mav0/mocap0/data.csv`): `timestamp [ns],
p_x, p_y, p_z, q_w, q_x, q_y, q_z`.

Each pose is associated with the nearest mocap sample within --max_dt
(20 ms); the camera centres are aligned to the mocap positions with Umeyama
(sim3: rotation, translation and scale; se3: scale 1) and the RMSE of the
aligned positions is printed. The camera-to-body offset is not applied, so a
few centimetres of the error is the lever arm. Pure Python: no numpy needed.
"""

import argparse
import bisect
import math
import sys


def read_trajectory(path):
    """Returns {map id: [(t, (x, y, z))]}; poses before any `# map` line go to map None."""
    sections = {}
    current = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                words = line[1:].split()
                if len(words) == 2 and words[0] == 'map':
                    current = int(words[1])
                continue
            values = [float(v) for v in line.split()]
            sections.setdefault(current, []).append((values[0], tuple(values[1:4])))
    return sections


def read_mocap(path):
    """Returns sorted [(t seconds, (x, y, z))]."""
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            values = line.split(',')
            samples.append((int(values[0]) * 1e-9, tuple(float(v) for v in values[1:4])))
    samples.sort()
    return samples


def associate(poses, mocap, max_dt):
    """Pairs (estimated, true) positions whose timestamps differ by at most max_dt."""
    times = [t for t, _ in mocap]
    pairs = []
    for t, position in poses:
        i = bisect.bisect_left(times, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(times) and (best is None or abs(times[j] - t) < abs(times[best] - t)):
                best = j
        if best is not None and abs(times[best] - t) <= max_dt:
            pairs.append((position, mocap[best][1]))
    return pairs


def mean(points):
    n = float(len(points))
    return tuple(sum(p[k] for p in points) / n for k in range(3))


def sub(a, b):
    return tuple(a[k] - b[k] for k in range(3))


def dot(a, b):
    return sum(a[k] * b[k] for k in range(3))


def jacobi_eigen(a):
    """Eigenvalues and eigenvectors (columns) of a small symmetric matrix."""
    n = len(a)
    a = [row[:] for row in a]
    v = [[float(i == j) for j in range(n)] for i in range(n)]
    for _ in range(100):
        off = sum(a[i][j] ** 2 for i in range(n) for j in range(n) if i != j)
        if off < 1e-24:
            break
        for p in range(n):
            for q in range(p + 1, n):
                if abs(a[p][q]) < 1e-300:
                    continue
                theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
                t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c
                for k in range(n):
                    akp, akq = a[k][p], a[k][q]
                    a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
                for k in range(n):
                    apk, aqk = a[p][k], a[q][k]
                    a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
                for k in range(n):
                    vkp, vkq = v[k][p], v[k][q]
                    v[k][p], v[k][q] = c * vkp - s * vkq, s * vkp + c * vkq
    return [a[i][i] for i in range(n)], v


def umeyama(source, target, with_scale):
    """(s, R, t) minimising sum |target - (s R source + t)|^2.

    The rotation is the proper rotation of Umeyama (1991), computed in closed
    form from Horn's quaternion eigenproblem; the scale is Umeyama's.
    """
    mu_x, mu_y = mean(source), mean(target)
    xs = [sub(p, mu_x) for p in source]
    ys = [sub(p, mu_y) for p in target]
    # Cross-covariance S[i][j] = sum x_i y_j.
    S = [[sum(x[i] * y[j] for x, y in zip(xs, ys)) for j in range(3)] for i in range(3)]
    (sxx, sxy, sxz), (syx, syy, syz), (szx, szy, szz) = S
    N = [
        [sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ]
    values, vectors = jacobi_eigen(N)
    k = max(range(4), key=lambda i: values[i])
    w, x, y, z = (vectors[i][k] for i in range(4))
    R = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]

    def rotate(p):
        return tuple(dot(R[i], p) for i in range(3))

    scale = 1.0
    if with_scale:
        variance = sum(dot(p, p) for p in xs)
        scale = sum(dot(yy, rotate(xx)) for xx, yy in zip(xs, ys)) / variance
    r_mu_x = rotate(mu_x)
    t = tuple(mu_y[i] - scale * r_mu_x[i] for i in range(3))
    return scale, rotate, t


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('trajectory')
    parser.add_argument('groundtruth')
    parser.add_argument('--align', choices=['sim3', 'se3'], default='sim3')
    parser.add_argument('--max_dt', type=float, default=0.02, help='association window, s')
    args = parser.parse_args()

    sections = read_trajectory(args.trajectory)
    if not sections:
        sys.exit('no poses in ' + args.trajectory)
    sizes = ', '.join('map %s: %d' % (m, len(p)) for m, p in sections.items())
    map_id, poses = max(sections.items(), key=lambda item: len(item[1]))
    pairs = associate(poses, read_mocap(args.groundtruth), args.max_dt)
    print('sections: %s; largest: map %s' % (sizes, map_id))
    print('matched poses: %d of %d' % (len(pairs), len(poses)))
    if len(pairs) < 3:
        sys.exit('too few matched poses to align')

    source = [p for p, _ in pairs]
    target = [q for _, q in pairs]
    scale, rotate, t = umeyama(source, target, args.align == 'sim3')
    errors = []
    for p, q in pairs:
        r = rotate(p)
        aligned = tuple(scale * r[i] + t[i] for i in range(3))
        errors.append(math.sqrt(sum((aligned[i] - q[i]) ** 2 for i in range(3))))
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    print('alignment: %s, scale %.4f' % (args.align, scale))
    print('ATE RMSE: %.4f m (mean %.4f, max %.4f)' % (rmse, sum(errors) / len(errors), max(errors)))


if __name__ == '__main__':
    main()
