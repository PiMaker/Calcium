using System.Drawing;
using Win32.SimpleGui;

public class Frontend : IDisposable
{
    // dark theme
    static readonly Color BackgroundColor = Color.FromArgb(255, 48, 48, 48);
    static readonly Color ElementColor = Color.FromArgb(255, 64, 64, 64);
    static readonly Color ForegroundColor = Color.White;

    readonly Thread _thread;
    readonly ManualResetEventSlim _windowCreated = new(false, 0);
    volatile Window _window;
    volatile bool _closeConfirmed; // set by Dispose to bypass the close confirmation

    ListBox _deviceList;
    Label _helpText;
    Button _calibrateButton;
    Checkbox _minimizeOnStartup;
    Label _speedLabel;
    Slider _speedSlider;
    Label _versionLabel;

    readonly TextBuffer _rowBuffer = new(512);
    readonly List<Device> _deviceCache = new();
    DateTimeOffset? _lastSpeedChange;
    float _lastCalibrationProgress;
    bool _playDing;

    public Frontend()
    {
        _thread = new Thread(Run) { IsBackground = true, Name = "Calcium Frontend" };
        _thread.SetApartmentState(ApartmentState.STA); // Win32 UI thread
        _thread.Start();
    }

    void Run()
    {
        try
        {
            Application.EnableHiDPISupportForCurrentProcess();
            Application.EnableVisualStylesForCurrentThread();
            Application.EnableDarkMode();

            using var icon = new Win32.SimpleGui.Icon(Icon.Data);
            using var fontUI = new Font("Segoe UI", 12f);
            using var fontSmall = new Font("Segoe UI", 9f);
            using var fontMono = new Font("Consolas", 15f);

            var window = new Window("Calcium", 720, 442, fontUI, icon)
            {
                CanMaximize = false,
                CanResize = false,
                Background = BackgroundColor,
            };
            window.OnCloseRequest += () =>
                _closeConfirmed || window.MessageBox("Close Calcium",
                    "Closing this window stops tracking correction until you restart SteamVR.",
                    icon, MessageBoxIcon.Question);

            var layout = new VerticalLayout { Width = BaseLayout.Fill, Height = BaseLayout.Fill, Margin = new Margin(8, 8) };
            window.Children.Add(layout);

            _deviceList = new ListBox
            {
                Width = BaseLayout.Fill,
                Height = BaseLayout.Fill,
                Font = fontMono,
                Foreground = ForegroundColor,
                Background = ElementColor,
            };
            _deviceList.OnSelectedIndexChanged += OnSelectedIndexChanged;
            _deviceList.Items.Add(new($"    Disable Space Correction"));
            layout.Children.Add(_deviceList);

            _helpText = new Label
            {
                Width = BaseLayout.Fill,
                Height = 28,
                Foreground = ForegroundColor,
            };
            layout.Children.Add(_helpText);

            var buttons = new HorizontalLayout { Width = BaseLayout.Fill, Height = 40, Spacing = 8 };
            layout.Children.Add(buttons);

            _calibrateButton = new Button("Calibrate") { Width = 150, Height = 40, Disabled = true };
            _calibrateButton.OnClick += _ => Calibrate();
            buttons.Children.Add(_calibrateButton);

            _minimizeOnStartup = new Checkbox()
            {
                Width = 20,
                Height = 40,
                Foreground = ForegroundColor,
                Margin = new Margin(6, 0, -4, 0),
            };
            var minimizeLabel = new Label("Minimize on Startup", centerVertically: true)
            {
                Width = BaseLayout.Fill,
                Height = 37,
                Foreground = ForegroundColor,
            };
            buttons.Children.Add(_minimizeOnStartup);
            buttons.Children.Add(minimizeLabel);
            _minimizeOnStartup.Checked = State.Current.MinimizeOnStartup;
            _minimizeOnStartup.OnCheckedChanged += (_, on) => SetMinimizeOnStartup(on);
            minimizeLabel.OnClick += _ => { _minimizeOnStartup.Checked = !_minimizeOnStartup.Checked; SetMinimizeOnStartup(_minimizeOnStartup.Checked); };

            _versionLabel = new Label("v" + CalciumVersion.Version, Alignment.Right, centerVertically: true)
            {
                Width = BaseLayout.Fill,
                Height = 40,
                Foreground = Color.FromArgb(160, 160, 160),
                Margin = new Margin(horizontal: 8),
            };
            buttons.Children.Add(_versionLabel);

            var speedLayout = new HorizontalLayout() { Width = BaseLayout.Fill, Height = 40, Spacing = 8 };
            _speedLabel = new Label($"Correction Speed ({State.Current.SpeedFactor:P0}):", centerVertically: true)
            {
                Width = 190,
                Height = 29,
                Foreground = ForegroundColor,
            };
            speedLayout.Children.Add(_speedLabel);
            _speedSlider = new Slider()
            {
                Width = BaseLayout.Fill,
                Height = 40,
                Margin = new Margin(0, 8, 0, -8),
            };
            _speedSlider.OnValueChanged += (_, value) => SetSpeed((int)value);
            speedLayout.Children.Add(_speedSlider);
            layout.Children.Add(speedLayout);

            window.Arrange(); // run layout pass once
            _window = window;
            _windowCreated.Set();

            _speedSlider.Min = 1;
            _speedSlider.Max = 200;
            _speedSlider.Value = State.Current.Speed;

            State.Current.OnCalibrationComplete += DispatchDing;
            State.Current.OnCalibrationProgress += ReportCalibrationProgress;

            Application.ScheduleTimer(RefreshDevices, 333);
            RefreshDevices();
            if (State.Current.MinimizeOnStartup)
                window.State = Window.WindowState.Minimized;

            Utilities.Log("Starting frontend event loop");
            Application.RunEventLoop(); // blocks until the window closes
            State.Current.ActiveTrackerSerial = null; // UI closed: disable tracking
            _window = null;
        }
        catch (Exception ex)
        {
            Utilities.Log($"Frontend error: {ex}");
        }
        finally
        {
            _windowCreated.Set(); // release Dispose even if startup failed
        }
    }

    void RefreshDevices()
    {
        try
        {
            if (_playDing)
            {
                Application.PlaySound(MessageBoxIcon.Information);
                _playDing = false;
            }

            _deviceCache.Clear();
            foreach (var dev in State.Current.Devices.Values)
            {
                if (dev.DeviceClass is OpenVr.DeviceClassController or OpenVr.DeviceClassGenericTracker)
                    _deviceCache.Add(dev);
            }
            _deviceCache.Sort(static (a, b) => a.ID.CompareTo(b.ID));

            var i = 1;
            var foundActive = false;
            var activeSerial = State.Current.ActiveTrackerSerial;
            var activeTrackingSpace = State.Current.ActiveTrackingSpace;
            foreach (var dev in _deviceCache)
            {
                _rowBuffer.Clear();
                BuildDeviceRow(dev, _rowBuffer);
                
                if (_deviceList.Items.Count <= i)
                    _deviceList.Items.Add(new($"{_rowBuffer}"));
                else if (!_deviceList.Items[i].Equals(_rowBuffer))
                    _deviceList.Items[i].Set($"{_rowBuffer}");

                // check if we select this entry
                // if none are found but activeSerial is set we will not select any entry until the stored tracked appears
                if (activeSerial == dev.SerialNumber)
                {
                    foundActive = true;
                    _deviceList.SelectedIndex = i;
                }

                i++;
            }

            // not found and not saved, select "Disabled" entry
            if (!foundActive && string.IsNullOrEmpty(activeSerial))
                _deviceList.SelectedIndex = 0;

            for (var j = _deviceList.Items.Count - 1; j >= i; j--)
                _deviceList.Items.Remove(_deviceList.Items[j]);

            // calibration logic
            var calibrating = State.Current.Calibrating;
            var hasCalibration = !State.Current.ActiveOffset.Value.IsIdentity;
            _calibrateButton.Disabled = string.IsNullOrEmpty(activeSerial) || string.IsNullOrEmpty(activeTrackingSpace) || calibrating;

            // set help text based on current app status
            if (!foundActive && !string.IsNullOrEmpty(activeSerial))
            {
                _helpText.SetText($"Waiting on tracker saved by serial number ({activeSerial}). Make sure it's tracking! ⌛");
            }
            else if (string.IsNullOrEmpty(activeSerial))
            {
                _helpText.SetText($"Select the device that you have attached to your headset in the list above. 🔍");
            }
            else if (calibrating)
            {
                _helpText.SetText($"Calibration progress: {_lastCalibrationProgress:P1} Gently move your head along all axes! 🔃");
            }
            else if (!string.IsNullOrEmpty(activeSerial))
            {
                if (!hasCalibration)
                    _helpText.SetText($"Click 'Calibrate' and follow the instructions to perform the one-time setup. ⚙️");
                else
                    _helpText.SetText($"Calibration found for {activeSerial}. Everything should be working! ✔️");
            }
            else
            {
                _helpText.SetText($"Unknown state? ⚠️");
            }

            // write speed to disk after a delay
            if (_lastSpeedChange.HasValue && (DateTimeOffset.UtcNow - _lastSpeedChange.Value).TotalSeconds > 3)
            {
                State.Current.WriteToDisk();
                _lastSpeedChange = null;
            }

            // debug
            _versionLabel.SetText($"v{CalciumVersion.Version}, {GC.GetTotalAllocatedBytes(precise: false)} B");
        }
        catch (Exception ex)
        {
            Utilities.Log($"An error occurred in the frontend loop: {ex.Message}");
        }
    }

    static void BuildDeviceRow(Device d, TextBuffer b)
    {
        if (d.ID >= 10) b.Append($" {d.ID} ");
        else b.Append($"  {d.ID} ");
        AppendField(b, d.SerialNumber, 19);
        AppendField(b, ClassToString(d.DeviceClass, d.IsKnownHandTracking), 7);
        AppendField(b, d.TrackingSpace, 6);
        AppendLastPosition(b, d);

        static void AppendField(TextBuffer b, string text, int width)
        {
            b.Append($"{text}");
            for (var pad = width - text.Length; pad >= 0; pad--)
                b.Append($" ");
        }

        static void AppendLastPosition(TextBuffer b, Device d)
        {
            var v = d.LastPose.Value;
            if (v.IsIdentity)
            {
                b.Append($" N/A");
                return;
            }

            var pos = v.Translation;
            if (pos.X < 0) b.Append($"{pos.X:F1}");
            else b.Append($" {pos.X:F1}");
            if (pos.Y < 0) b.Append($" {pos.Y:F1}");
            else b.Append($"  {pos.Y:F1}");
            if (pos.Z < 0) b.Append($" {pos.Z:F1}");
            else b.Append($"  {pos.Z:F1}");
        }

        static string ClassToString(int deviceClass, bool isHand) => deviceClass switch
        {
            OpenVr.DeviceClassController => isHand ? "Hand" : "Contr.",
            OpenVr.DeviceClassGenericTracker => "Tracker",
            _ => deviceClass.ToString(),
        };
    }

    private void OnSelectedIndexChanged(ListBox _, int index)
    {
        if (index == 0)
        {
            State.Current.ActiveTrackerSerial = null; // disabled
            return;
        }

        if (--index < 0 || index >= _deviceCache.Count) return;
        State.Current.ActiveTrackerSerial = _deviceCache[index].SerialNumber;
    }

    void ReportCalibrationProgress(float progress) => _lastCalibrationProgress = progress;
    void DispatchDing() => _playDing = true;

    static void Calibrate() => State.Current.Calibrating = true;
    static void SetMinimizeOnStartup(bool minimize)
    {
        State.Current.MinimizeOnStartup = minimize;
        State.Current.WriteToDisk();
    }
    void SetSpeed(int speed)
    {
        State.Current.Speed = speed;
        _lastSpeedChange = DateTimeOffset.UtcNow;
        _speedLabel.SetText($"Correction Speed ({State.Current.SpeedFactor:P0}):");
    }

    public void Dispose()
    {
        State.Current.OnCalibrationComplete -= DispatchDing;
        State.Current.OnCalibrationProgress -= ReportCalibrationProgress;

        _windowCreated.Wait();
        _closeConfirmed = true; // driver unload: close without confirmation
        if (_window is Window window)
            window.Dispose(); // thread-safe: posts WM_CLOSE
        _thread?.Join();
    }
}
