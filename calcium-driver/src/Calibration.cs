using System.Numerics;
using System.Collections.Concurrent;

public static class Calibration
{
    const float MinimumRotation = 0.015f; // Ignore pairs with very small motion
    static readonly double[,] RotationNormal = new double[4, 4];
    static readonly ConcurrentQueue<(Matrix4x4 A, Matrix4x4 B)> Pairs = new();

    static bool _active;
    static Matrix4x4 _targetInverse;
    static Matrix4x4 _hmd;

    internal static Lock CalibrationLock = new();
    internal static bool Active => _active;

    public const int MaxSamples = 384;
    public static int CollectedSampleCount => Pairs.Count;

    internal static void Stop()
    {
        if (_active)
        {
            _active = false;
            _targetInverse = Matrix4x4.Identity;
            _hmd = Matrix4x4.Identity;
            Array.Clear(RotationNormal);
            Pairs.Clear();
        }
    }

    // Algorithm based on Hand-Eye calibration.
    //
    // Matrix chain:
    // - targetInverse: target tracking space -> target-local
    // - M: target-local -> HMD-local (the fixed mount offset we solve)
    // - hmd: HMD-local -> HMD tracking space
    // Since the target is rigidly mounted to the HMD, `targetInverse * M * hmd` is constant.
    //
    // For two samples this becomes A * M = M * B. A pair alone has many valid
    // answers, so replacing M with A * M * inverse(B) only rotates its error.
    // We instead accumulate pairs, solve M's rotation from all orientation
    // constraints, then solve translation and uniform scale with rotation fixed.
    // Write A as (Ra, a), B as (Rb, b), and M as (scale * Rm, t):
    // - a and b are the same head motion as seen by the target and HMD systems.
    // - Rm turns target axes into HMD axes; scale changes target-space distances.
    // - t is the offset from the target tracker origin to the HMD origin.
    // Id is the 3x3 identity rotation. First we turn target-space translation
    // a into HMD axes and units: scale * (a * Rm). The remaining difference to
    // HMD translation b must be the arc traced because the target and HMD have
    // different rotation origins: t * (Id - Rb). It is zero with no HMD
    // rotation. Rotation is solved above from orientations; this solve then
    // finds the translation and scale that make both spaces' motion agree.
    // t * (Id - Rb) + scale * (a * Rm) = b.
    internal static bool Update(Matrix4x4 targetInverse, Matrix4x4 hmd, out Matrix4x4 result)
    {
        result = Matrix4x4.Identity;

        if (!_active)
        {
            // first input is set up as the anchor, don't calibrate on invalid prev values
            _active = true;
            _targetInverse = targetInverse;
            _hmd = hmd;
            return false;
        }

        var current = targetInverse;
        var previous = _targetInverse;
        if (!Matrix4x4.Invert(current, out var currentInverse) || !Matrix4x4.Invert(_hmd, out var previousHmdInverse))
            return false;

        // A * M = M * B
        // A: current, B: previous
        // current^-1 * previous: relative motion since last update
        // currentHmd * previousHmd^-1: relative motion of the HMD since last update
        // Thus: current^-1 * previous * M = M * (currentHmd * previousHmd^-1)
        // This is true iff M is correct and represents a constant in physical space - minimizing the error means finding M
        var a = currentInverse * previous;
        var b = hmd * previousHmdInverse;
        if (RotationAngle(b) < MinimumRotation) return false;
        if (Pairs.Count >= MaxSamples) return false;

        AddRotationConstraint(a, b);
        Pairs.Enqueue((a, b));
        _targetInverse = targetInverse;
        _hmd = hmd;

        if (!SolveRotation(out var rotation) || !SolveSimilarity(rotation, out var translation, out var scale)) return false;

        result = Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation);
        result.Translation = translation;
        return true;
    }

    static float RotationAngle(Matrix4x4 m)
    {
        var q = Quaternion.CreateFromRotationMatrix(m);
        return 2f * MathF.Acos(Math.Clamp(MathF.Abs(q.W), 0f, 1f));
    }

    // Matrix4x4 uses row vectors: Matrix(a) * Matrix(x) is Concatenate(a, x),
    // which is x * a in conventional quaternion multiplication.
    static void AddRotationConstraint(Matrix4x4 a, Matrix4x4 b)
    {
        var qa = Quaternion.CreateFromRotationMatrix(a);
        var qb = Quaternion.CreateFromRotationMatrix(b);
        if (qa.W < 0f) qa = Quaternion.Negate(qa);
        if (qb.W < 0f) qb = Quaternion.Negate(qb);

        // A * M = M * B becomes qM * qA = qB * qM.
        var c = new double[4, 4];
        Right(qa, c);
        SubtractLeft(qb, c);
        for (var row = 0; row < 4; row++)
            for (var col = 0; col < 4; col++)
                for (var k = 0; k < 4; k++)
                    RotationNormal[row, col] += c[k, row] * c[k, col];
    }

    // Each pair says that turning in target axes by Ra must equal turning in
    // HMD axes by Rb after applying the fixed mount rotation. In quaternion
    // form that is a four-number residual C * qM. RotationNormal is the sum
    // of C-transpose * C for every motion pair: it measures total squared
    // mismatch for each possible mount quaternion. The unit quaternion in its
    // smallest-eigenvalue direction has the least mismatch across all pairs.
    static bool SolveRotation(out Quaternion rotation)
    {
        var a = (double[,])RotationNormal.Clone();
        var vectors = new double[4, 4];
        for (var i = 0; i < 4; i++) vectors[i, i] = 1;

        for (var iteration = 0; iteration < 32; iteration++)
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
        if (a[max, max] < 1e-9 || a[second, second] < a[max, max] * 1e-4)
        {
            rotation = default;
            return false; // insufficient rotation around independent axes
        }

        rotation = Quaternion.Normalize(new Quaternion((float)vectors[1, min], (float)vectors[2, min], (float)vectors[3, min], (float)vectors[0, min]));
        return true;
    }

    static bool SolveSimilarity(Quaternion rotation, out Vector3 translation, out float scale)
    {
        var normal = new double[4, 4];
        var rhs = new double[4];
        var rm = Matrix4x4.CreateFromQuaternion(rotation);
        foreach (var (a, b) in Pairs)
        {
            var scaledA = Vector3.TransformNormal(a.Translation, rm);
            var c = new double[3, 3]
            {
                { 1d - b.M11, -b.M12, -b.M13 },
                { -b.M21, 1d - b.M22, -b.M23 },
                { -b.M31, -b.M32, 1d - b.M33 },
            };
            var u = new[] { (double)scaledA.X, scaledA.Y, scaledA.Z };
            var bv = new[] { (double)b.M41, b.M42, b.M43 };
            for (var output = 0; output < 3; output++)
            {
                var row = new[] { c[0, output], c[1, output], c[2, output], u[output] };
                for (var i = 0; i < 4; i++)
                {
                    rhs[i] += bv[output] * row[i];
                    for (var j = 0; j < 4; j++)
                        normal[i, j] += row[i] * row[j];
                }
            }
        }

        if (!Solve(normal, rhs, out var x) || x[3] <= 0d)
        {
            translation = default;
            scale = 1f;
            return false;
        }
        translation = new Vector3((float)x[0], (float)x[1], (float)x[2]);
        scale = (float)x[3];
        return true;
    }

    static bool Solve(double[,] normal, double[] rhs, out double[] x)
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
                x = [];
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

        x = new double[n];
        for (var i = 0; i < n; i++) x[i] = a[i, n];
        return true;
    }

    // Conventional quaternion multiplication matrices, with component order w, x, y, z.
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
