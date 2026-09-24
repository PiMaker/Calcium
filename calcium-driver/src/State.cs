using System.Collections.Concurrent;
using System.Numerics;

public class State
{
    public static State Current { get; } = new State();

    public volatile string ActiveSerialNumber = "";
    public volatile uint ActiveTargetIndex = 0;

    public volatile bool Calibrating = false;
    public event Action OnCalibrationComplete;

    public volatile int Speed = 100; // default speed value (0-200)
    public float SpeedFactor => Speed / 100f;

    public bool MinimizeOnStartup = false;

    public readonly ConcurrentDictionary<uint, Device> Devices = new();

    public PooledAtomicStrongBox<Matrix4x4> ActiveOffset = new(32, Matrix4x4.Identity); // offset of the rigidly mounted tracker from HMD root
    public PooledAtomicStrongBox<Matrix4x4> ActiveCorrection = new(32, Matrix4x4.Identity); // active world-space correction matrix

    public void BeginCalibration()
    {
        if (Calibrating) return;
        Calibrating = true;
        ActiveOffset.Set(Matrix4x4.Identity);
        ActiveCorrection.Set(Matrix4x4.Identity);
    }

    public void FinishCalibration()
    {
        Calibrating = false;
        OnCalibrationComplete?.Invoke();
    }

    public void WriteToDisk()
    {
        var offset = ActiveOffset.Value;
        var path = Path.Combine(Utilities.GetDataPath(), "settings.ini");
        var ini = $"TargetSerialNumber = {ActiveSerialNumber}{Environment.NewLine}ActiveOffset = {Utilities.SerializeMatrix(offset)}{Environment.NewLine}MinimizeOnStartup = {MinimizeOnStartup}{Environment.NewLine}Speed = {Speed}";
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

    public readonly Outliers Outliers = new();
    public PooledAtomicStrongBox<Matrix4x4> LastPose { get; } = new(16, Matrix4x4.Identity);

    public volatile string TrackingSpace = trackingSpace;
    public volatile string SerialNumber = serialNumber;
    public volatile int DeviceClass = deviceClass;

    // TODO: Add more devices here
    public bool IsKnownHandTracking => SerialNumber.StartsWith("VRLINKQ_Hand") || SerialNumber == "HANDL" || SerialNumber == "HANDR";
}