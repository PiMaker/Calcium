using System.Numerics;
using System.Collections.Concurrent;

public static class Calibration
{
    const float MinimumRotation = 0.025f; // Ignore pairs with very small motion
    static readonly double[,] RotationNormal = new double[4, 4];
    static readonly ConcurrentQueue<(Matrix4x4 A, Matrix4x4 B)> Pairs = new();

    static bool _active;
    static Matrix4x4 _targetInverse;
    static Matrix4x4 _prevHmdInverse;

    internal static Lock CalibrationLock = new();
    internal static bool Active => _active;

    public const int CalibrationSteps = 4;
    public const int MaxSamples = 128;
    public static int CollectedSampleCount => Pairs.Count;

    static readonly double[,] _rotationScratch4x4 = new double[4, 4];
    static readonly double[,] _rotationScratch3x3 = new double[3, 3];

    internal static void Stop()
    {
        if (_active)
        {
            _active = false;
            _targetInverse = Matrix4x4.Identity;
            _prevHmdInverse = Matrix4x4.Identity;
            Array.Clear(RotationNormal);
            Pairs.Clear();
        }
    }

    // Algorithm based on Hand-Eye calibration with fixed AX = XB.
    // We don't calculate a translation between tracking spaces here, instead
    // we focus on recovering only the fixed offset of the mount.
    //
    // Matrix chain:
    // - targetInverse (A^-1): target tracking space -> target-local
    // - M: target-local -> HMD-local (the fixed mount offset we solve)
    // - hmd (B): HMD-local -> HMD tracking space
    //
    // The mount is rigid, so the whole chain targetInverse * M * hmd has to
    // be a physical constant: Deltas in targetInverse and hmd cancel out.
    // Comparing each pair of samples gives A * M = M * B, where A
    // is the relative motion the target measured and B the same motion
    // measured by the HMD. Or conjugated: A = M * B * M^-1.
    //
    // One pair leaves M undetermined (many transforms conjugate B into A),
    // so pairs accumulate and M is solved in two stages over all of them:
    //
    // 1. Rotation, from orientations only (SolveRotation). Each pair
    //    linearly constrains the 4 components of M's quaternion; the
    //    least-squares fit over all pairs is the eigenvector of a symmetric
    //    4x4 matrix with the smallest eigenvalue.
    //
    // 2. Translation and uniform scale, rotation fixed (SolveSimilarity).
    //    Write A = (Ra, a), B = (Rb, b), M = (scale * Rm, t): Rm turns
    //    target axes into HMD axes, scale converts target-space distances,
    //    t is the offset from the target tracker origin to the HMD origin.
    //    If both origins coincided, the measured translations would agree
    //    up to scale once Rm is applied. But if they pivot about different
    //    points, a rotation sweeps the offset point along an arc. That arc term
    //    is t * (Id - Rb): zero without rotation, growing with the offset's
    //    distance from the rotation axis.
    //        t * (Id - Rb) + scale * (a * Rm) = b
    //    Why: a is the target's own displacement - but the target origin is
    //    a body point held at offset t from the HMD's pivot. The body motion
    //    (rotation Rb about the HMD origin, translation b) carries that
    //    point to t * Rb + b. Subtracting the start point t gives its
    //    displacement in HMD axes: b - t * (Id - Rb). That must equal the
    //    same displacement measured by the target, turned into HMD axes and
    //    units: scale * (a * Rm).
    //    Each pair gives 3 equations in the 4 unknowns (t, scale), stacked
    //    into normal equations and solved as a linear system.
    internal static bool Update(Matrix4x4 targetInverse, Matrix4x4 hmd, out Matrix4x4 result, int calibrateUpTo)
    {
        result = Matrix4x4.Identity;

        if (!Matrix4x4.Invert(hmd, out var currentHmdInverse)) return false;

        if (!_active)
        {
            // first input is set up as the anchor, don't calibrate on invalid prev values
            _active = true;
            _targetInverse = targetInverse;
            _prevHmdInverse = currentHmdInverse;
            return false;
        }

        var current = targetInverse;
        var previous = _targetInverse;
        if (!Matrix4x4.Invert(current, out var currentInverse))
            return false;

        // A * M = M * B
        // A: current, B: previous
        // current^-1 * previous: relative motion since last update
        // currentHmd * previousHmd^-1: relative motion of the HMD since last update
        // Thus: current^-1 * previous * M = M * (currentHmd * previousHmd^-1)
        // This is true iff M is correct and represents a constant in physical space - minimizing the error means finding M
        var a = currentInverse * previous;
        var b = hmd * _prevHmdInverse;

        // Nearly motionless pairs carry almost no orientation or arc
        // information; they only dilute the solve with noise.
        if (Utilities.RotationAngle(b) < MinimumRotation) return false;

        if (Pairs.Count < MaxSamples * calibrateUpTo)
        {
            AddRotationConstraint(a, b);
            Pairs.Enqueue((a, b));
            _targetInverse = targetInverse;
            _prevHmdInverse = currentHmdInverse;
        }

        if (!SolveRotation(out var rotation) || !SolveSimilarity(rotation, out var translation, out var scale)) return false;

        result = Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation);
        result.Translation = translation;
        return true;
    }

    // Quaternion form of the rotation constraint. Matrix4x4 uses row vectors:
    // Matrix(a) * Matrix(x) is Concatenate(a, x), which is x * a in
    // conventional quaternion multiplication. So A * M = M * B reads
    // qM * qA = qB * qM, or qM * qA - qB * qM = 0: a homogeneous linear
    // equation C * qM = 0 in the 4 components of qM.
    static void AddRotationConstraint(Matrix4x4 a, Matrix4x4 b)
    {
        var qa = Quaternion.CreateFromRotationMatrix(a);
        var qb = Quaternion.CreateFromRotationMatrix(b);

        // Linear constraining requires consistent signs, always pick short-arc
        if (qa.W < 0f) qa = Quaternion.Negate(qa);
        if (qb.W < 0f) qb = Quaternion.Negate(qb);

        // If R(q) is a rotation matrix such that R(q) * point = point * q,
        // and L(q) is correspondingly L(q) * point = q * point, then for
        // qM * qA - qB * qM = 0 as C * qM = 0, we get C = R(qA) - L(qB).
        // Accumulating C^T * C into RotationNormal sums the squared residuals
        // of all pairs into one quadratic form in qM.
        var c = _rotationScratch4x4; // new double[4, 4]
        Right(qa, c); // c = R(qa)
        SubtractLeft(qb, c); // c = R(qa) - L(qb)
        for (var row = 0; row < 4; row++)
            for (var col = 0; col < 4; col++)
                for (var k = 0; k < 4; k++)
                    RotationNormal[row, col] += c[k, row] * c[k, col]; // Accumulate C^T * C into RotationNormal
    }

    // Least-squares fit of the mount quaternion. RotationNormal's quadratic
    // form is the total squared mismatch ||C * qM||^2 summed over all pairs;
    // as a sum of squares it is a bowl-shaped error surface over the unit
    // sphere of quaternions. The best-fit unit quaternion is its eigenvector
    // with the smallest eigenvalue - the flattest direction of the bowl
    // (the same eigenvector trick as Horn's quaternion absolute-orientation
    // method).
    //
    // The eigendecomposition is the classical Jacobi method: a symmetric
    // matrix is diagonalized by a sequence of 2D plane rotations, each
    // chosen to zero the largest remaining off-diagonal entry. Off-diagonal
    // entries measure how much the current axes mix pairs of eigenvectors;
    // once they are gone, the diagonal holds the eigenvalues and the product
    // of rotations (vectors) holds the eigenvectors.
    //
    // Degeneracy: the fit is unique (up to the q / -q sign symmetry) only
    // if exactly one eigenvalue is approx. 0. If the second-smallest collapses
    // too, the data admits a whole family of equally good solutions -
    // typically because every sample rotated about essentially one axis,
    // leaving the mount's twist around that axis unconstrained. Motions
    // around at least two independent axes are needed.
    static bool SolveRotation(out Quaternion rotation)
    {
        var a = (double[,])RotationNormal.Clone();
        var vectors = _rotationScratch4x4; Array.Clear(vectors); // new double[4, 4]
        for (var i = 0; i < 4; i++) vectors[i, i] = 1;

        for (var iteration = 0; iteration < 42; iteration++)
        {
            var p = 0;
            var q = 1;
            var largest = 0d;
            for (var row = 0; row < 4; row++)
            {
                for (var col = row + 1; col < 4; col++)
                {
                    var abs = Math.Abs(a[row, col]);
                    if (abs > largest)
                    {
                        largest = abs;
                        p = row;
                        q = col;
                    }
                }
            }
            // No significant off-diagonal left: matrix is diagonal to
            // working precision.
            if (largest < 1e-12) break;

            var angle = 0.5 * Math.Atan2(2d * a[p, q], a[q, q] - a[p, p]);
            var cosine = Math.Cos(angle);
            var sine = Math.Sin(angle);
            for (var k = 0; k < 4; k++)
            {
                var kp = a[k, p]; var kq = a[k, q];
                a[k, p] = cosine * kp - sine * kq;
                a[k, q] = sine * kp + cosine * kq;
            }
            for (var k = 0; k < 4; k++)
            {
                var pk = a[p, k]; var qk = a[q, k];
                a[p, k] = cosine * pk - sine * qk;
                a[q, k] = sine * pk + cosine * qk;
                var vp = vectors[k, p]; var vq = vectors[k, q];
                vectors[k, p] = cosine * vp - sine * vq;
                vectors[k, q] = sine * vp + cosine * vq;
            }
        }

        // The diagonal now holds the eigenvalues, in no particular order.
        // Find smallest, second-smallest and largest with a linear scan.
        var min = 0;
        var second = 1;
        var max = 0;
        for (var i = 0; i < 4; i++)
        {
            if (a[i, i] < a[min, min]) min = i;
            if (a[i, i] > a[max, max]) max = i;
        }
        for (var i = 0; i < 4; i++)
        {
            if (i != min && (second == min || a[i, i] < a[second, second]))
                second = i;
        }

        // Reject if there is no signal at all, or if the second-smallest
        // eigenvalue collapsed: a second near-zero eigenvalue means a family
        // of equally good solutions, not one best fit (see header).
        if (a[max, max] < 1e-9 || a[second, second] < a[max, max] * 1e-4)
        {
            rotation = default;
            return false; // insufficient rotation around independent axes
        }

        rotation = Quaternion.Normalize(new Quaternion((float)vectors[1, min], (float)vectors[2, min], (float)vectors[3, min], (float)vectors[0, min]));
        return true;
    }

    // Stage 2: least squares for translation t and uniform scale with the
    // rotation fixed. Each pair contributes the 3 scalar equations of
    //     t * (Id - Rb) + scale * (a * Rm) = b
    // (see Update). With unknown vector x = (t, scale), accumulating the
    // equation rows' outer products and right-hand sides over all pairs
    // builds the normal equations of a plain linear least-squares problem.
    // x[0..2] is t, x[3] is scale; a non-positive scale would mirror or
    // collapse target space, which is nonsense, so reject it.
    static bool SolveSimilarity(Quaternion rotation, out Vector3 translation, out float scale)
    {
        var rm = Matrix4x4.CreateFromQuaternion(rotation);
        var c = _rotationScratch3x3; // new double[3, 3]
        var normal = _rotationScratch4x4; Array.Clear(normal); // new double[4, 4]
        Span<double> rhs = stackalloc double[4]; rhs.Clear();
        Span<double> u = stackalloc double[3];
        Span<double> bv = stackalloc double[3];
        Span<double> row = stackalloc double[4];
        foreach (var (a, b) in Pairs)
        {
            // Per pair, one scalar equation per output component of
            //     t * (Id - Rb) + scale * (a * Rm) = b
            // with unknown vector x = (tx, ty, tz, scale):
            // - rotatedA: a turned into HMD axes (a * Rm)
            // - c: Id - Rb, the arc-term coefficient of t
            // - bv: b, the HMD-side translation (equation right-hand side)
            // - row: the output equation's coefficients - column `output` of
            //   (Id - Rb) for t, component `output` of a * Rm for scale
            // Each row contributes row * row^T to the normal matrix and
            // row * b to the right-hand side; Solve() then solves the system.
            var rotatedA = Vector3.TransformNormal(a.Translation, rm);
            c[0, 0] = 1d - b.M11; c[0, 1] = -b.M12; c[0, 2] = -b.M13;
            c[1, 0] = -b.M21; c[1, 1] = 1d - b.M22; c[1, 2] = -b.M23;
            c[2, 0] = -b.M31; c[2, 1] = -b.M32; c[2, 2] = 1d - b.M33;
            u[0] = rotatedA.X; u[1] = rotatedA.Y; u[2] = rotatedA.Z;
            bv[0] = b.M41; bv[1] = b.M42; bv[2] = b.M43;
            for (var output = 0; output < 3; output++)
            {
                row[0] = c[0, output]; row[1] = c[1, output];
                row[2] = c[2, output]; row[3] = u[output];
                for (var i = 0; i < 4; i++)
                {
                    rhs[i] += bv[output] * row[i];
                    for (var j = 0; j < 4; j++)
                        normal[i, j] += row[i] * row[j];
                }
            }
        }

        Span<double> x = stackalloc double[4];
        if (!Solve(normal, rhs, ref x) || x[3] <= 0d)
        {
            translation = default;
            scale = 1f;
            return false;
        }
        translation = new Vector3((float)x[0], (float)x[1], (float)x[2]);
        scale = (float)x[3];
        return true;
    }

    // Gauss-Jordan elimination with partial pivoting on the augmented
    // matrix. Pivoting (swap in the largest-magnitude coefficient) avoids
    // dividing by tiny numbers, which would amplify rounding error. A
    // negligible pivot means the accumulated equations do not determine the
    // unknowns (too few or too similar samples); reject.
    static bool Solve(double[,] normal, Span<double> rhs, ref Span<double> x)
    {
        var n = rhs.Length;
        var a = new double[n, n + 1];
        var scale = 0d;
        for (var row = 0; row < n; row++)
            for (var col = 0; col < n; col++)
            {
                a[row, col] = normal[row, col];
                scale = Math.Max(scale, Math.Abs(a[row, col]));
            }
        for (var row = 0; row < n; row++) a[row, n] = rhs[row];

        for (var col = 0; col < n; col++)
        {
            var pivot = col;
            for (var row = col + 1; row < n; row++)
                if (Math.Abs(a[row, col]) > Math.Abs(a[pivot, col])) pivot = row;
            if (scale == 0d || Math.Abs(a[pivot, col]) < scale * 1e-5)
            {
                x.Clear();
                return false;
            }
            for (var k = col; k <= n; k++) (a[col, k], a[pivot, k]) = (a[pivot, k], a[col, k]);
            var divisor = a[col, col];
            for (var k = col; k <= n; k++) a[col, k] /= divisor;
            for (var row = 0; row < n; row++)
            {
                if (row == col) continue;
                var factor = a[row, col];
                for (var k = col; k <= n; k++) a[row, k] -= factor * a[col, k];
            }
        }

        for (var i = 0; i < n; i++) x[i] = a[i, n];
        return true;
    }

    // Conventional quaternion multiplication as 4x4 matrices, component
    // order (w, x, y, z). Right(q, m) fills m so that m * p = p * q
    // (q multiplied on the right); SubtractLeft subtracts the matrix with
    // m * p = q * p (q multiplied on the left).
    static void Right(Quaternion q, double[,] m)
    {
        var w = q.W; var x = q.X; var y = q.Y; var z = q.Z;
        m[0, 0] = w; m[0, 1] = -x; m[0, 2] = -y; m[0, 3] = -z;
        m[1, 0] = x; m[1, 1] = w; m[1, 2] = z; m[1, 3] = -y;
        m[2, 0] = y; m[2, 1] = -z; m[2, 2] = w; m[2, 3] = x;
        m[3, 0] = z; m[3, 1] = y; m[3, 2] = -x; m[3, 3] = w;
    }

    static void SubtractLeft(Quaternion q, double[,] m)
    {
        var w = q.W; var x = q.X; var y = q.Y; var z = q.Z;
        m[0, 0] -= w; m[0, 1] -= -x; m[0, 2] -= -y; m[0, 3] -= -z;
        m[1, 0] -= x; m[1, 1] -= w; m[1, 2] -= -z; m[1, 3] -= y;
        m[2, 0] -= y; m[2, 1] -= z; m[2, 2] -= w; m[2, 3] -= -x;
        m[3, 0] -= z; m[3, 1] -= -y; m[3, 2] -= x; m[3, 3] -= w;
    }
}
