using System.Runtime.InteropServices;
using MinHook;

public static unsafe class HookInjector
{
    static readonly HookEngine _engine = new();
    internal static OpenVr.TrackedDevicePoseUpdated _poseOriginal = null!;

    // Entry-point: this gets called by OpenVR after our DLL is loaded.
    [UnmanagedCallersOnly(EntryPoint = "HmdDriverFactory")]
    static void* HmdDriverFactory(byte* pInterfaceName, int* pReturnCode)
    {
        Utilities.Log($"Factory entry, requested: {new string((sbyte*)pInterfaceName)}");

        if (Utilities.AsciiEquals(pInterfaceName, OpenVr.ServerTrackedDeviceProviderVersion))
        {
            var provider = Provider.Object;
            return provider;
        }

        if (pReturnCode != null)
            *pReturnCode = OpenVr.InitInterfaceNotFound;

        return null;
    }

    // Fetches IVRServerDriverHost_006 during Init and hooks the returned
    // host's TrackedDevicePoseUpdated for the process lifetime. All driver-host
    // instances share the native function, so we see every driver's pose pushes.
    public static void ArmPoseHook(DriverContext* ctx)
    {
        try
        {
            int err = 0;
            var host = ctx->VTable->GetGenericInterface(
                ctx, (byte*)Utilities.AllocAscii(OpenVr.ServerDriverHost006Version), &err);
            Utilities.Log($"Init fetch {OpenVr.ServerDriverHost006Version} -> {(long)host:x} err={err}");
            if (host == null) return;

            var target = (IntPtr)host->VTable->TrackedDevicePoseUpdated;
            _poseOriginal = _engine.CreateHook<OpenVr.TrackedDevicePoseUpdated>(target, PoseHook.PoseDetour);
            _engine.EnableHook(_poseOriginal);
        }
        catch (Exception e)
        {
            Utilities.Log($"Pose arm failed: {e.Message}");
        }
    }
}

// Unmanaged C++-style provider object: [vtable ptr][7 slots] matching IServerTrackedDeviceProvider.
// We tell the OpenVR runtime that this implementation is our plugin. We mostly just need it to get loaded and stay resident.
public static unsafe class Provider
{
    static void* _object;
    static IntPtr _versions;

    static Frontend _frontend;

    public static void* Object
    {
        get
        {
            if (_object == null) Build();
            return _object;
        }
    }

    static void Build()
    {
        var vtable = (IntPtr*)NativeMemory.Alloc(7 * (nuint)sizeof(IntPtr));
        vtable[0] = (IntPtr)(delegate* unmanaged<void*, void*, int>)&InitImpl;
        vtable[1] = (IntPtr)(delegate* unmanaged<void*, void>)&CleanupImpl;
        vtable[2] = (IntPtr)(delegate* unmanaged<void*, void*>)&GetInterfaceVersionsImpl;
        vtable[3] = (IntPtr)(delegate* unmanaged<void*, void>)&RunFrameImpl;
        vtable[4] = (IntPtr)(delegate* unmanaged<void*, byte>)&ShouldBlockStandbyImpl;
        vtable[5] = (IntPtr)(delegate* unmanaged<void*, void>)&EnterStandbyImpl;
        vtable[6] = (IntPtr)(delegate* unmanaged<void*, void>)&LeaveStandbyImpl;

        var obj = (void**)NativeMemory.Alloc((nuint)sizeof(IntPtr));
        obj[0] = vtable;
        _object = obj;
    }

    [UnmanagedCallersOnly]
    static int InitImpl(void* self, void* context)
    {
        Utilities.Log("Calcium init");
        State.Current.ReadFromDisk();
        var drvContext = (DriverContext*)context;
        HookInjector.ArmPoseHook(drvContext);
        DeviceProperties.Arm(drvContext);
        _frontend = new Frontend();
        return 0;
    }

    [UnmanagedCallersOnly]
    static void CleanupImpl(void* self)
    {
        Utilities.Log("Calcium cleanup");
        _frontend?.Dispose();
        _frontend = null;
    }

    [UnmanagedCallersOnly]
    static void* GetInterfaceVersionsImpl(void* self)
    {
        if (_versions == IntPtr.Zero) _versions = BuildVersions();
        return (void*)_versions;
    }

    [UnmanagedCallersOnly]
    static void RunFrameImpl(void* self) { }

    [UnmanagedCallersOnly]
    static byte ShouldBlockStandbyImpl(void* self) => 0;

    [UnmanagedCallersOnly]
    static void EnterStandbyImpl(void* self) { }

    [UnmanagedCallersOnly]
    static void LeaveStandbyImpl(void* self) { }

    static IntPtr BuildVersions()
    {
        // which interfaces we implement - doesn't matter too much which one, just one that always gets loaded
        var table = (IntPtr*)NativeMemory.Alloc(2 * (nuint)sizeof(IntPtr));
        table[0] = Utilities.AllocAscii(OpenVr.ServerTrackedDeviceProviderVersion);
        table[1] = IntPtr.Zero;
        return (IntPtr)table;
    }
}