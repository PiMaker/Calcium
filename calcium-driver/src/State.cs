using System.Collections.Concurrent;
using System.Numerics;

public class State
{
    public static State Current { get; } = new State();

    public volatile string ActiveSerialNumber = "";
    private volatile uint _activeTargetIndex = 0;
    public uint ActiveTargetIndex
    {
        get => _activeTargetIndex;
        set
        {
            if (Devices.TryGetValue(value, out var device))
            {
                _activeTargetIndex = value;
                ActiveSerialNumber = device.SerialNumber;
                Utilities.Log($"Active target set to ID={value}, SerialNumber={device.SerialNumber}");
            }
        }
    }

    public event Action OnCalibrationComplete;
    private volatile int _calibrating = 0;
    public bool Calibrating
    {
        get => _calibrating != 0;
        set
        {
            if (value)
            {
                if (Interlocked.CompareExchange(ref _calibrating, 1, 0) == 0)
                {
                    ActiveOffset.Set(Matrix4x4.Identity);
                    ActiveCorrection.Set(Matrix4x4.Identity);
                    Utilities.Log("Calibration started.");
                }
            }
            else
            {
                if (Interlocked.CompareExchange(ref _calibrating, 0, 1) == 1)
                {
                    OnCalibrationProgress?.Invoke(1.0f);
                    OnCalibrationComplete?.Invoke();
                    Utilities.Log("Calibration finished.");
                }
            }
        }
    }
    
    public event Action<float> OnCalibrationProgress;
    public void ReportCalibrationProgress(float progress) => OnCalibrationProgress?.Invoke(progress);

    public volatile int Speed = 100; // default speed value (0-200)
    public float SpeedFactor => Speed / 100f;
    public bool MinimizeOnStartup = false;

    public readonly ConcurrentDictionary<uint, Device> Devices = new();

    public PooledAtomicStrongBox<Matrix4x4> ActiveOffset = new(32, Matrix4x4.Identity); // offset of the rigidly mounted tracker from HMD root
    public PooledAtomicStrongBox<Matrix4x4> ActiveCorrection = new(32, Matrix4x4.Identity); // active world-space correction matrix

    public void InsertDevice(uint id)
    {
        var ct = DeviceProperties.GetContainer(id);
        var trackingSpace = DeviceProperties.GetString(ct, id, OpenVr.PropTrackingSystemName);
        var serialNumber = DeviceProperties.GetString(ct, id, OpenVr.PropSerialNumber);
        var deviceClass = DeviceProperties.GetInt(ct, id, OpenVr.PropDeviceClass);
        if (Devices.TryAdd(id, new Device(id, trackingSpace, serialNumber, deviceClass)))
            Utilities.Log($"Added device: ID={id}, TrackingSpace={trackingSpace}, SerialNumber={serialNumber}, DeviceClass={deviceClass}");
    }

    public void RemoveDevice(uint id)
    {
        if (Devices.TryRemove(id, out _))
            Utilities.Log("Device disconnected: " + id);
    }

    public void WriteToDisk()
    {
        var offset = ActiveOffset.Value;
        var path = Path.Combine(Utilities.GetDataPath(), "settings.ini");
        var ini =
            $"TargetSerialNumber = {ActiveSerialNumber}{Environment.NewLine}" +
            $"ActiveOffset = {Utilities.SerializeMatrix(offset)}{Environment.NewLine}" +
            $"MinimizeOnStartup = {MinimizeOnStartup}{Environment.NewLine}" +
            $"Speed = {Speed}";
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