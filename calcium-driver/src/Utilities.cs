using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

public static class Utilities
{
    static readonly Lock _logLock = new();
    static readonly StreamWriter _log = CreateLog();

    public static string GetDataPath()
    {
        var path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Calcium");
        Directory.CreateDirectory(path);
        return path;
    }

    static StreamWriter CreateLog()
    {
        lock (_logLock)
        {
            var stream = new FileStream(Path.Combine(GetDataPath(), "driver_log.txt"),
                FileMode.OpenOrCreate, FileAccess.Write, FileShare.ReadWrite);
            stream.Seek(0, SeekOrigin.Begin);
            stream.SetLength(0); // truncated file
            var ret = new StreamWriter(stream) { AutoFlush = true };
            ret.WriteLine("=== Log Started ===");
            return ret;
        }
    }

    public static void Log(string msg)
    {
        lock (_logLock)
        {
            _log.WriteLine(DateTime.Now.ToString("HH:mm:ss.fff ") + msg);
        }
    }

    public static unsafe IntPtr AllocAscii(string s)
    {
        var p = (byte*)NativeMemory.Alloc((nuint)(s.Length + 1));
        for (int i = 0; i < s.Length; i++) p[i] = (byte)s[i];
        p[s.Length] = 0;
        return (IntPtr)p;
    }

    public static unsafe bool AsciiEquals(byte* p, string expected)
    {
        for (int i = 0; i < expected.Length; i++)
        {
            if (p[i] != (byte)expected[i]) return false;
            if (p[i] == 0) return false;
        }
        return p[expected.Length] == 0;
    }

    // Math Helpers:
    
    public static HmdVector3d_t ToOpenVR(this Vector3 v) => new() { x = v.X, y = v.Y, z = v.Z };
    public static HmdQuaternion_t ToOpenVR(this Quaternion q) => new() { x = q.X, y = q.Y, z = q.Z, w = q.W };
    public static Vector3 ToNumerics(this in HmdVector3d_t v) => new((float)v.x, (float)v.y, (float)v.z);
    public static Quaternion ToNumerics(this in HmdQuaternion_t q) => new((float)q.x, (float)q.y, (float)q.z, (float)q.w);

    // Rotation angle in radians, independent of sign
    public static float RotationAngle(Matrix4x4 m) => RotationAngle(Quaternion.CreateFromRotationMatrix(m));
    public static float RotationAngle(Quaternion q) => 2f * MathF.Acos(Math.Clamp(MathF.Abs(q.W), 0f, 1f));

    public static Matrix4x4 Blend(in Matrix4x4 a, in Matrix4x4 b, float tRot, float tScale, float tTranslate)
    {
        tRot = Math.Clamp(tRot, 0f, 1f);
        tScale = Math.Clamp(tScale, 0f, 1f);
        tTranslate = Math.Clamp(tTranslate, 0f, 1f);

        if (Matrix4x4.Decompose(a, out var aScale, out var aRotation, out var aTranslation) &&
            Matrix4x4.Decompose(b, out var bScale, out var bRotation, out var bTranslation))
        {
            var scale = Vector3.Lerp(aScale, bScale, tScale);
            var rotation = Quaternion.Slerp(aRotation, bRotation, tRot);
            var translation = Vector3.Lerp(aTranslation, bTranslation, tTranslate);
            return Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation);
        }
        return a;
    }

    public static Matrix4x4 GetPoseMatrix(in DriverPose_t pose)
    {
        var raw = Matrix4x4.CreateFromQuaternion(pose.qRotation.ToNumerics());
        raw.Translation = pose.vecPosition.ToNumerics();
        var world = Matrix4x4.CreateFromQuaternion(pose.qWorldFromDriverRotation.ToNumerics());
        world.Translation = pose.vecWorldFromDriverTranslation.ToNumerics();
        return raw * world;
    }

    public static void ApplyWorldTransform(ref DriverPose_t pose, in Matrix4x4 correction)
    {
        Matrix4x4.Decompose(correction, out var correctionScale, out var correctionRotation, out var correctionTranslation);
        var world = Matrix4x4.CreateFromQuaternion(pose.qWorldFromDriverRotation.ToNumerics());
        world.Translation = pose.vecWorldFromDriverTranslation.ToNumerics() * correctionScale.X;

        var rotation = Matrix4x4.CreateFromQuaternion(correctionRotation);
        rotation.Translation = correctionTranslation;
        world *= rotation;

        pose.vecPosition = (pose.vecPosition.ToNumerics() * correctionScale.X).ToOpenVR();
        pose.qWorldFromDriverRotation = Quaternion.CreateFromRotationMatrix(world).ToOpenVR();
        pose.vecWorldFromDriverTranslation = world.Translation.ToOpenVR();
    }

    public static string SerializeMatrix(Matrix4x4 matrix)
    {
        var sb = new StringBuilder();
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                sb.Append(matrix[row, col]);
                sb.Append(',');
            }
        }
        return sb.ToString();
    }

    public static Matrix4x4? DeserializeMatrix(string serialized)
    {
        try
        {
            var values = serialized.Split(',', StringSplitOptions.RemoveEmptyEntries).Select(float.Parse).ToArray();
            if (values.Length != 16) throw new ArgumentException($"Invalid length {values.Length}");
            return new Matrix4x4(
                values[0], values[1], values[2], values[3],
                values[4], values[5], values[6], values[7],
                values[8], values[9], values[10], values[11],
                values[12], values[13], values[14], values[15]
            );
        }
        catch (Exception ex)
        {
            Utilities.Log($"Failed to deserialize matrix: {ex}");
            return null;
        }
    }
}

// Provides atomic value swap while avoiding allocations by using a pool of intermediate values.
// Larger pool has a higher chance to avoid race conditions, but uses more memory.
public class PooledAtomicStrongBox<T>
    where T: struct
{
    private StrongBox<T> _activeBox;
    private List<StrongBox<T>> _pool;
    private long _index;

    public T Value => Volatile.Read(ref _activeBox).Value; // atomic reference read

    public PooledAtomicStrongBox(int poolSize, T initialValue)
    {
        _pool = new List<StrongBox<T>>(poolSize);
        for (int i = 0; i < poolSize; i++)
            _pool.Add(new StrongBox<T>(initialValue));
        _activeBox = _pool[0];
    }

    public void Set(T value)
    {
        var index = Interlocked.Increment(ref _index);
        var newBox = _pool[(int)(index % _pool.Count)];
        newBox.Value = value;
        Interlocked.Exchange(ref _activeBox, newBox);
    }
}