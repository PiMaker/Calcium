using System.Runtime.InteropServices;

// Very basic open-coded bindings for the OpenVR driver interface.

[StructLayout(LayoutKind.Sequential)]
public struct HmdQuaternion_t
{
    public double w, x, y, z;
}

[StructLayout(LayoutKind.Sequential)]
public struct HmdVector3d_t
{
    public double x, y, z;
}

// openvr_driver.h DriverPose_t
[StructLayout(LayoutKind.Sequential)]
public struct DriverPose_t
{
    public double poseTimeOffset;
    public HmdQuaternion_t qWorldFromDriverRotation;
    public HmdVector3d_t vecWorldFromDriverTranslation;
    public HmdQuaternion_t qDriverFromHeadRotation;
    public HmdVector3d_t vecDriverFromHeadTranslation;
    public HmdVector3d_t vecPosition;
    public HmdVector3d_t vecVelocity;
    public HmdVector3d_t vecAcceleration;
    public HmdQuaternion_t qRotation;
    public HmdVector3d_t vecAngularVelocity;
    public HmdVector3d_t vecAngularAcceleration;
    public int result;
    public byte poseIsValid;
    public byte willDriftInYaw;
    public byte shouldApplyHeadModel;
    public byte deviceIsConnected;
}

public static class OpenVr
{
    public const int DriverPoseSize = 280;
    public const int InitInterfaceNotFound = 105; // VRInitError_Init_InterfaceNotFound

    public const int TrackingResultUninitialized = 1; // ETrackResult::TrackingResult_Uninitialized
    public const int TrackingResultRunningOk = 200; // ETrackingResult::TrackingResult_Running_OK
    public const int TrackingResultRunningOutOfRange = 201; // ETrackingResult::TrackingResult_Running_OutOfRange

    public const string ServerTrackedDeviceProviderVersion = "IServerTrackedDeviceProvider_004";
    public const string ServerDriverHost006Version = "IVRServerDriverHost_006";
    public const string PropertiesVersion = "IVRProperties_001";

    // ETrackedDeviceProperty values used for device identification
    public const int PropTrackingSystemName = 1000; // Prop_TrackingSystemName_String
    public const int PropModelNumber = 1001; // Prop_ModelNumber_String
    public const int PropSerialNumber = 1002; // Prop_SerialNumber_String
    public const int PropDeviceClass = 1029; // Prop_DeviceClass_Int32, returns an ETrackedDeviceClass

    // ETrackedDeviceClass values
    public const int DeviceClassInvalid = 0;
    public const int DeviceClassHmd = 1;
    public const int DeviceClassController = 2;
    public const int DeviceClassGenericTracker = 3;
    public const int DeviceClassTrackingReference = 4;
    public const int DeviceClassDisplayRedirect = 5;

    public const int TrackedPropSuccess = 0; // ETrackedPropertyError::TrackedProp_Success

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void TrackedDevicePoseUpdated(IntPtr self, uint deviceIndex, IntPtr pose, uint structSize);
}

// Object model of the vrserver-side interfaces. MSVC x64 COM-style layout: an object's first
// member is a pointer to its vtable; vtable fields map 1:1 to declaration-order slots.
// C++ virtual calls additionally pass `this` as the first argument.

[StructLayout(LayoutKind.Sequential)]
public unsafe struct DriverContext
{
    public VTableDriverContext* VTable;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct VTableDriverContext
{
    public delegate* unmanaged<void*, byte*, int*, ServerDriverHost*> GetGenericInterface;
    public void* GetDriverHandle;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct ServerDriverHost
{
    public VTableServerDriverHost* VTable;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct VTableServerDriverHost
{
    public void* TrackedDeviceAdded;

    public delegate* unmanaged<void*, uint, void*, uint, void> TrackedDevicePoseUpdated;
}

// openvr_driver.h IVRProperties_001
[StructLayout(LayoutKind.Sequential)]
public struct PropertyRead_t
{
    public int prop;
    public unsafe void* pvBuffer;
    public uint unBufferSize;
    public uint unTag;
    public uint unRequiredBufferSize;
    public int eError;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct VTableProperties
{
    public delegate* unmanaged<void*, ulong, PropertyRead_t*, uint, int> ReadPropertyBatch;
    public delegate* unmanaged<void*, ulong, void*, uint, int> WritePropertyBatch;
    public delegate* unmanaged<void*, int, byte*> GetPropErrorNameFromEnum;
    public delegate* unmanaged<void*, uint, ulong> TrackedDeviceToPropertyContainer;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct Properties
{
    public VTableProperties* VTable;
}

public static unsafe class DeviceProperties
{
    static Properties* _props;

    public static void Arm(DriverContext* ctx)
    {
        int err = 0;
        _props = (Properties*)ctx->VTable->GetGenericInterface(
            ctx, (byte*)Utilities.AllocAscii(OpenVr.PropertiesVersion), &err);
    }

    public static ulong GetContainer(uint deviceIndex) =>
        _props != null ? _props->VTable->TrackedDeviceToPropertyContainer(_props, deviceIndex) : 0;

    public static string GetString(ulong container, uint deviceIndex, int prop)
    {
        if (_props == null) return "";
        if (container == 0) return "";

        byte* buffer = stackalloc byte[4096];
        buffer[4095] = 0; // makin' sure
        var read = new PropertyRead_t { prop = prop, pvBuffer = buffer, unBufferSize = 4095 };
        var error = _props->VTable->ReadPropertyBatch(_props, container, &read, 1);
        return error == OpenVr.TrackedPropSuccess ? new string((sbyte*)read.pvBuffer) : "";
    }

    public static int GetInt(ulong container, uint deviceIndex, int prop)
    {
        if (_props == null) return 0;
        if (container == 0) return 0;

        int value = 0;
        var read = new PropertyRead_t { prop = prop, pvBuffer = &value, unBufferSize = sizeof(int) };
        var error = _props->VTable->ReadPropertyBatch(_props, container, &read, 1);
        return error == OpenVr.TrackedPropSuccess ? value : 0;
    }
}