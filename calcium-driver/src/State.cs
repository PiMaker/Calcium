using System.Collections.Concurrent;
using System.Numerics;

public class State
{
    public static State Current { get; } = new State();

    public string ActiveSerialNumber = "";
    public volatile uint ActiveTargetIndex = 0;
    public volatile bool Calibrate = false;

    public volatile int Speed = 100; // default speed value (0-200)
    public float SpeedFactor => Speed / 100f;

    public bool MinimizeOnStartup = false;

    public readonly ConcurrentDictionary<uint, Device> Devices = new();

    public PooledAtomicStrongBox<Matrix4x4> ActiveOffset = new(32, Matrix4x4.Identity); // offset of the rigidly mounted tracker from HMD root
    public PooledAtomicStrongBox<Matrix4x4> ActiveCorrection = new(32, Matrix4x4.Identity); // active world-space correction matrix

    public void BeginCalibration()
    {
        if (Calibrate) return;
        Calibrate = true;
        ActiveOffset.Set(Matrix4x4.Identity);
        ActiveCorrection.Set(Matrix4x4.Identity);
    }

    public void FinishCalibration()
    {
        Calibrate = false;
    }

    public void WriteToDisk()
    {
        var offset = ActiveOffset.Value;
        var path = Path.Combine(Utilities.GetDataPath(), "settings.ini");
        var ini = $"TargetSerialNumber = {Volatile.Read(ref ActiveSerialNumber)}{Environment.NewLine}ActiveOffset = {Utilities.SerializeMatrix(offset)}{Environment.NewLine}MinimizeOnStartup = {MinimizeOnStartup}{Environment.NewLine}Speed = {Speed}";
        File.WriteAllText(path, ini);
        Utilities.Log($"Wrote settings to disk at {path}");
    }

    public void ReadFromDisk()
    {
        var path = Path.Combine(Utilities.GetDataPath(), "settings.ini");
        try
        {
            var ini = File.ReadAllText(path);
            var lines = ini.Split([Environment.NewLine], StringSplitOptions.None);
            foreach (var line in lines)
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                var parts = line.Split("=", 2);
                if (parts.Length != 2) continue;
                var key = parts[0].Trim();
                var value = parts[1].Trim();
                if (key == "TargetSerialNumber") Interlocked.Exchange(ref ActiveSerialNumber, value);
                if (key == "ActiveOffset") ActiveOffset.Set(Utilities.DeserializeMatrix(value).GetValueOrDefault(Matrix4x4.Identity));
                if (key == "MinimizeOnStartup") MinimizeOnStartup = bool.Parse(value);
                if (key == "Speed") Speed = int.Parse(value);
            }
            Utilities.Log($"Read settings from disk from {path}");
        }
        catch (Exception ex)
        {
            Utilities.Log($"Failed to read settings from disk: {ex}");
        }
    }
}

public class Device(uint id, string trackingSpace, string serialNumber, int deviceClass)
{
    public uint ID { get; } = id;

    public readonly Lock PoseGate = new(); // Used externally in PoseHook to prevent re-entrance on the same id
    private readonly Lock _gate = new(); // Used internally for thread-safe access to properties

    public readonly Outliers Outliers = new();
    public PooledAtomicStrongBox<Matrix4x4> LastPose { get; } = new(16, Matrix4x4.Identity);

    public string TrackingSpace
    {
        get { lock (_gate) { return field; } }
        set { lock (_gate) { field = value; } }
    } = trackingSpace;

    public string SerialNumber
    {
        get { lock (_gate) { return field; } }
        set { lock (_gate) { field = value; } }
    } = serialNumber;

    public volatile int DeviceClass = deviceClass;

    // TODO: Add more devices here
    public bool IsKnownHandTracking => SerialNumber.StartsWith("VRLINKQ_Hand");
}