using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

// P/Invoke resolution uses the host process (vrserver) DLL search path, which
// does not include our driver directory. Loading the native libraries by full
// path registers them process-wide, so later LoadLibrary-by-name calls find them.
public static partial class LibLoader
{
    static int _moduleAnchor = 0; // any address inside our own DLL image

    const uint GetModuleHandleExFlagFromAddress = 0x4;
    const uint GetModuleHandleExFlagUnchangedRefCount = 0x2;
    static readonly char[] _modulePathBuffer = new char[1024];

    [LibraryImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static partial bool GetModuleHandleExW(uint flags, IntPtr address, out IntPtr module);

    [LibraryImport("kernel32.dll", StringMarshalling = StringMarshalling.Utf16)]
    private static partial uint GetModuleFileNameW(IntPtr module, [Out] char[] buffer, uint size);

    public static unsafe void PreloadNativeDependencies()
    {
        try
        {
            const uint flags = GetModuleHandleExFlagFromAddress | GetModuleHandleExFlagUnchangedRefCount;
            if (!GetModuleHandleExW(flags, (IntPtr)Unsafe.AsPointer(ref _moduleAnchor), out IntPtr module))
                return;
            var len = GetModuleFileNameW(module, _modulePathBuffer, (uint)_modulePathBuffer.Length);
            if (len == 0 || len >= _modulePathBuffer.Length) return;
            var dir = Path.GetDirectoryName(new string(_modulePathBuffer, 0, (int)len));
            if (dir == null) return;

            foreach (var name in new[] { "libSkiaSharp.dll", "libHarfBuzzSharp.dll", "av_libglesv2.dll" })
            {
                var path = Path.Combine(dir, name);
                if (File.Exists(path))
                {
                    if (!NativeLibrary.TryLoad(path, out _))
                        throw new DllNotFoundException($"TryLoad failed: {path}");
                    
                    Utilities.Log($"[LibLoader] Successfully loaded native library: {path}");
                }
            }
        }
        catch (Exception ex)
        {
            Utilities.Log($"[LibLoader] Native dependency preload failed: {ex.Message}");
        }
    }
}