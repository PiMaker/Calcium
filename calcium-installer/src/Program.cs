using System.Diagnostics;
using System.Drawing;
using Win32.SimpleGui;

public static class Program
{
    const string DriverName = "01calcium";

    static Window _form;
    static Label _pathLabel;
    static Button _installButton;
    static Button _uninstallButton;
    static Win32.SimpleGui.Icon _icon;

    [STAThread]
    static void Main()
    {
        Application.EnableHiDPISupportForCurrentProcess();
        Application.EnableVisualStylesForCurrentThread();
        Application.EnableDarkMode();

        _icon = new Win32.SimpleGui.Icon(Icon.Data);
        using var font = new Font("Segoe UI", 10f);
        using var bigFont = new Font("Segoe UI", 14f);
        var darkColor = Color.FromArgb(64, 64, 64);

        _form = new Window("Calcium Installer", 440, 260, font, _icon)
        {
            CanMaximize = false,
            CanResize = false,
            Background = darkColor,
        };

        var versionLabel = new Label("v" + CalciumVersion.Version, Alignment.Right)
        {
            X = 348, Y = 230, Width = 80,
            Foreground = Color.FromArgb(160, 160, 160),
            Background = darkColor,
        };

        _pathLabel = new Label(alignment: Alignment.Center)
        {
            X = 8, Y = 24, Width = _form.Width - 16, Height = 42,
            Foreground = Color.White,
            Background = darkColor,
        };

        _installButton = new Button("Install or Update") { Y = 85, Width = BaseLayout.Fill, Height = 56, Margin = new Margin(46, 0), Font = bigFont };
        _installButton.OnClick += _ => Install();

        _uninstallButton = new Button("Uninstall") { Y = 155, Width = BaseLayout.Fill, Height = 56, Margin = new Margin(46, 0), Font = bigFont };
        _uninstallButton.OnClick += _ => Uninstall();
        
        _form.Children.Add(versionLabel);
        _form.Children.Add(_pathLabel);
        _form.Children.Add(_installButton);
        _form.Children.Add(_uninstallButton);

        Refresh();
        Application.ScheduleTimer(Refresh, 1000);

        _form.Arrange();
        Application.RunEventLoop();
    
        _form.Dispose();
        _icon.Dispose();
    }

    static string SteamVrPath;
    static string DriverDest => Path.Combine(SteamVrPath ?? "", "drivers", DriverName);
    static bool Detected => SteamVrPath != null && Directory.Exists(DriverDest);

    static void Refresh()
    {
        SteamVrPath = FindSteamVrPath();
        var valid = SteamVrPath != null && File.Exists(Path.Combine(SteamVrPath, "bin", "win64", "vrserver.exe"));
        _pathLabel.SetText(SteamVrPath == null
            ? new TextBuffer($"SteamVR not found in registry.")
            : valid
                ? new TextBuffer($"SteamVR driver path:\n{SteamVrPath}")
                : new TextBuffer($"Not a SteamVR folder (missing bin\\win64\\vrserver.exe):\n{SteamVrPath}"));
        _installButton.Disabled = !valid;
        _uninstallButton.Disabled = !valid || !Detected;

        _pathLabel.BringToFront();
    }

    static string FindSteamVrPath()
    {
        var steam = Microsoft.Win32.Registry.CurrentUser.OpenSubKey(@"Software\Valve\Steam")?.GetValue("SteamPath") as string
            ?? Microsoft.Win32.Registry.LocalMachine.OpenSubKey(@"SOFTWARE\WOW6432Node\Valve\Steam")?.GetValue("InstallPath") as string;
        if (steam == null) return null;
        var final = Path.Combine(steam.Replace('/', '\\'), "steamapps", "common", "SteamVR");
        if (final.Length > 0)
            final = char.ToUpper(final[0]) + final[1..];
        return final;
    }

    static void Install()
    {
        if (!GuardSteamVrClosed()) return;
        var dest = DriverDest;
        Directory.CreateDirectory(Path.Combine(dest, "bin", "win64"));
        Directory.CreateDirectory(Path.Combine(dest, "settings"));
        var asm = typeof(Program).Assembly;
        foreach (var name in asm.GetManifestResourceNames())
        {
            if (name == "driver.vrdrivermanifest")
                WriteResource(name, Path.Combine(dest, name));
            else if (name == "default.vrsettings")
                WriteResource(name, Path.Combine(dest, "settings", name));
            else if (name.EndsWith(".dll", StringComparison.OrdinalIgnoreCase))
                WriteResource(name, Path.Combine(dest, "bin", "win64", name));
        }
        Application.MessageBox("Calcium Installer", "Successfully installed!", _icon, MessageBoxIcon.Information, false);
        Refresh();
    }

    static void WriteResource(string name, string path)
    {
        using var src = typeof(Program).Assembly.GetManifestResourceStream(name);
        using var dst = File.Create(path);
        src.CopyTo(dst);
    }

    static void Uninstall()
    {
        if (!GuardSteamVrClosed()) return;
        Directory.Delete(DriverDest, true);
        Application.MessageBox("Calcium Installer", "Successfully uninstalled!", _icon, MessageBoxIcon.Information, false);
        Refresh();
    }

    static bool GuardSteamVrClosed()
    {
        if (Process.GetProcessesByName("vrserver").Length == 0) return true;
        Application.MessageBox("Calcium Installer", "vrserver.exe is running - quit SteamVR first.", _icon, MessageBoxIcon.Error);
        return false;
    }
}
