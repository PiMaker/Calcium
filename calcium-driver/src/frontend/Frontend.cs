using System.Drawing;
using System.Numerics;
using System.Text;
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
    Button _resetCalibrationButton;
    Checkbox _minimizeOnStartup;
    Label _speedLabel;
    Slider _speedSlider;

    readonly List<Device> _deviceCache = new();
    readonly List<string> _rows = new();
    readonly StringBuilder _row = new();
    readonly Dictionary<uint, Vector3> _lastDevicePositions = new();
    int playedSoundForStage = -1;

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

            var window = new Window("Calcium", 680, 440, fontUI, icon)
            {
                CanMaximize = false,
                CanResize = false,
            };
            window.OnCloseRequest += () =>
                _closeConfirmed || window.MessageBox("Close Calcium",
                    "Closing this window stops tracking correction until you restart SteamVR.",
                    icon, MessageBoxIcon.Question);

            var versionLabel = new Label("v" + CalciumVersion.Version)
            {
                X = 630, Y = 5, Width = 40,
                Foreground = Color.FromArgb(160, 160, 160),
                Background = BackgroundColor,
            };
            window.Children.Add(versionLabel);

            var background = new Panel(0, 0, window.Width, window.Height - 40, BackgroundColor);
            window.Children.Add(background);

            var layout = new VerticalLayout { Width = BaseLayout.Fill, Height = BaseLayout.Fill, Margin = new Margin(8, 8) };
            window.Children.Add(layout);

            layout.Children.Add(new Label("Select Tracker mounted to Headset:")
            {
                Width = BaseLayout.Fill,
                Height = 28,
                Foreground = ForegroundColor,
                Background = BackgroundColor,
            });

            _deviceList = new ListBox
            {
                Width = BaseLayout.Fill,
                Height = BaseLayout.Fill,
                Font = fontMono,
                Foreground = ForegroundColor,
                Background = ElementColor,
            };
            _deviceList.OnSelectedIndexChanged += (_, i) =>
            {
                var index = i - 1; // item 0 is the Disabled row
                State.Current.ActiveTargetIndex = index >= 0 && index < _deviceCache.Count ? _deviceCache[index].ID : 0;
            };
            layout.Children.Add(_deviceList);

            _helpText = new Label
            {
                Width = BaseLayout.Fill,
                Height = 52,
                Foreground = ForegroundColor,
                Background = BackgroundColor,
            };
            layout.Children.Add(_helpText);

            var buttons = new HorizontalLayout { Width = BaseLayout.Fill, Height = 40, Spacing = 8 };
            layout.Children.Add(buttons);

            _calibrateButton = new Button("Calibrate") { Width = 150, Height = 40, Disabled = true };
            _calibrateButton.OnClick += _ => Calibrate();
            buttons.Children.Add(_calibrateButton);

            _resetCalibrationButton = new Button("Reset Calibration") { Width = 190, Height = 40, Disabled = true, Margin = new Margin(8, 0) };
            _resetCalibrationButton.OnClick += _ => ResetCalibration();
            buttons.Children.Add(_resetCalibrationButton);

            _minimizeOnStartup = new Checkbox()
            {
                Width = 20,
                Height = 40,
                Foreground = ForegroundColor,
                Background = BackgroundColor,
                Margin = new Margin(6, 0, -4, 0),
            };
            var minimizeLabel = new Label("Minimize on Startup", centerVertically: true)
            {
                Width = 180,
                Height = 37,
                Foreground = ForegroundColor,
                Background = BackgroundColor,
            };
            buttons.Children.Add(_minimizeOnStartup);
            buttons.Children.Add(minimizeLabel);
            _minimizeOnStartup.Checked = State.Current.MinimizeOnStartup;
            _minimizeOnStartup.OnCheckedChanged += (_, on) => SetMinimizeOnStartup(on);

            var speedLayout = new HorizontalLayout() { Width = BaseLayout.Fill, Height = 40, Spacing = 8, Margin = new Margin(0, 0, 0, 8) };
            _speedLabel = new Label($"Speed ({State.Current.SpeedFactor:P0}):", centerVertically: true)
            {
                Width = 140,
                Height = 40,
                Foreground = ForegroundColor,
                Background = BackgroundColor,
            };
            speedLayout.Children.Add(_speedLabel);
            _speedSlider = new Slider()
            {
                Width = BaseLayout.Fill,
                Height = 40,
                Margin = new Margin(0, 8, 0, 0),
            };
            _speedSlider.OnValueChanged += (_, value) => SetSpeed((int)value);
            speedLayout.Children.Add(_speedSlider);
            layout.Children.Add(speedLayout);

            window.Arrange(); // run layout pass once
            background.SendToBack();
            _window = window;
            _windowCreated.Set();

            _speedSlider.Min = 1;
            _speedSlider.Max = 200;
            _speedSlider.Value = State.Current.Speed;

            Application.ScheduleTimer(RefreshDevices, 250);
            RefreshDevices();
            if (State.Current.MinimizeOnStartup)
                window.State = Window.WindowState.Minimized;

            Utilities.Log("Starting frontend event loop");
            Application.RunEventLoop(); // blocks until the window closes
            State.Current.ActiveTargetIndex = 0; // UI closed: disable tracking
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
            _deviceCache.Clear();
            foreach (var dev in State.Current.Devices.Values)
            {
                if (dev.DeviceClass is OpenVr.DeviceClassController or OpenVr.DeviceClassGenericTracker)
                {
                    _deviceCache.Add(dev);

                    var lp = dev.LastPose.Value;
                    if (!lp.IsIdentity)
                        _lastDevicePositions[dev.ID] = lp.Translation;
                }

                CheckDeviceChanged(dev);
            }
            _deviceCache.Sort(static (a, b) => a.ID.CompareTo(b.ID));

            // set rows in place when the device count is stable, reset the list when it changed;
            // per-tick garbage is limited to the item strings themselves
            _rows.Clear();
            _rows.Add("    Space Correction Disabled");
            foreach (var d in _deviceCache)
                AppendDeviceRow(d);

            var items = _deviceList.Items;
            if (items.Count != _rows.Count)
            {
                items.Clear();
                foreach (var row in _rows) items.Add(row);
            }
            else
            {
                for (var i = 0; i < _rows.Count; i++)
                    if (items[i] != _rows[i])
                        items[i] = _rows[i];
            }
            _deviceList.SelectedIndex = SelectedIndex();

            var foundActive = false;
            foreach (var dev in _deviceCache)
            {
                if (State.Current.ActiveTargetIndex == dev.ID)
                    foundActive = true;
            }

            var calibrationState = State.Current.CalibrateUpTo;

            // calibration logic
            bool waitingOnStepUp = false;
            if (calibrationState > 0)
            {
                var collected = Calibration.CollectedSampleCount;
                if (collected >= Calibration.MaxSamples * calibrationState)
                {
                    if (playedSoundForStage != calibrationState)
                    {
                        Application.PlaySound(MessageBoxIcon.Information);
                        playedSoundForStage = calibrationState;
                    }

                    if (calibrationState < Calibration.CalibrationSteps)
                        waitingOnStepUp = true;
                    else
                        State.Current.FinishCalibration();
                }
            }
            _calibrateButton.Disabled = State.Current.ActiveTargetIndex == 0 || (!waitingOnStepUp && calibrationState > 0);
            _calibrateButton.Text = waitingOnStepUp ? "Continue" : "Calibrate";

            var hasCalibration = !State.Current.ActiveOffset.Value.IsIdentity;
            _resetCalibrationButton.Disabled = !hasCalibration;

            // set help text based on current app status
            var activeSerial = Volatile.Read(ref State.Current.ActiveSerialNumber);
            if (!foundActive && !string.IsNullOrEmpty(activeSerial))
            {
                _helpText.Text = $"A mounted tracker was saved by serial number ({activeSerial}), but isn't available yet. Make sure it's turned on and tracking! ⌛";
            }
            else if (State.Current.ActiveTargetIndex == 0)
            {
                _helpText.Text = "Select the device that you have attached to your headset in the list above. To identify it, try moving your head and watching the last column. 🔍";
            }
            else if (calibrationState > 0)
            {
                if (waitingOnStepUp)
                {
                    _helpText.Text = $"Calibration step {calibrationState} of {Calibration.CalibrationSteps} complete. Move somewhere else in your playspace and press 'Continue'! ⏭️";
                }
                else
                {
                    var samples = Calibration.CollectedSampleCount;
                    var progress = (samples - (Calibration.MaxSamples * (calibrationState - 1)))/(float)(Calibration.MaxSamples * calibrationState);
                    _helpText.Text = $"Calibration step {calibrationState} of {Calibration.CalibrationSteps}: {progress:P2}\nGently move and rotate your head! 🔃";
                }
            }
            else if (State.Current.ActiveTargetIndex != 0)
            {
                if (!hasCalibration)
                {
                    _helpText.Text = "No calibration found. Click 'Calibrate' and follow the instructions to perform the one-time setup. ⚙️";
                }
                else
                {
                    _helpText.Text = $"Calibration found for active device. Everything should be working! ✔️\nBlend factor: {State.Current.LastCorrectionBlend:P2}";
                }
            }
            else
            {
                _helpText.Text = "Unknown state? ⚠️";
            }
        }
        catch (Exception ex)
        {
            Utilities.Log($"An error occurred in the frontend loop: {ex.Message}");
        }
    }

    void AppendDeviceRow(Device d)
    {
        _row.Clear();
        if (d.ID >= 10) _row.Append(' ').Append(d.ID);
        else _row.Append("  ").Append(d.ID);
        _row.Append(' ');
        AppendField(d.TrackingSpace, 11);
        _row.Append(' ');
        AppendField(d.SerialNumber, 19);
        _row.Append(' ');
        AppendField(ClassToString(d.DeviceClass), 7);
        _row.Append(' ');
        AppendLastPosition(d);
        _rows.Add(_row.ToString());

        void AppendField(string text, int width)
        {
            _row.Append(text);
            for (var pad = width - text.Length; pad > 0; pad--)
                _row.Append(' ');
        }

        void AppendLastPosition(Device d)
        {
            if (_lastDevicePositions.TryGetValue(d.ID, out var pos))
            {
                _row.Append(FloatToString(pos.X)).Append(',');
                _row.Append(FloatToString(pos.Y)).Append(',');
                _row.Append(FloatToString(pos.Z));
            }
            else
            {
                _row.Append(" N/A");
            }

            string FloatToString(float value) => value < 0 ? value.ToString("F1") : " " + value.ToString("F1");
        }
    }

    int SelectedIndex()
    {
        var target = State.Current.ActiveTargetIndex;
        if (target == 0) return 0;
        for (var i = 0; i < _deviceCache.Count; i++)
        {
            if (_deviceCache[i].ID == target)
                return i + 1;
        }
        return -1;
    }

    // TODO: Maybe remove? Doesn't seem necessary. But cheap.
    static bool CheckDeviceChanged(Device dev)
    {
        var id = dev.ID;
        var ct = DeviceProperties.GetContainer(id);
        var deviceClass = DeviceProperties.GetInt(ct, id, OpenVr.PropDeviceClass);
        var serialNumber = DeviceProperties.GetString(ct, id, OpenVr.PropSerialNumber);
        var trackingSpace = DeviceProperties.GetString(ct, id, OpenVr.PropTrackingSystemName);
        if (deviceClass != dev.DeviceClass ||
            serialNumber != dev.SerialNumber ||
            trackingSpace != dev.TrackingSpace)
        {
            Utilities.Log("Device changed: " + id);
            State.Current.Devices.TryRemove(id, out _);
            return true;
        }
        return false;
    }

    static string ClassToString(int deviceClass) => deviceClass switch
    {
        OpenVr.DeviceClassController => "Ctrller",
        OpenVr.DeviceClassGenericTracker => "Tracker",
        _ => deviceClass.ToString(),
    };

    void Calibrate()
    {
        var state = State.Current.CalibrateUpTo;
        if (state == 0)
            State.Current.BeginCalibration();
        else
            State.Current.StepUpCalibration();
    }

    void ResetCalibration()
    {
        State.Current.ActiveOffset.Set(Matrix4x4.Identity);
        _resetCalibrationButton.Disabled = true;
    }

    void SetMinimizeOnStartup(bool minimize)
    {
        State.Current.MinimizeOnStartup = minimize;
        State.Current.WriteToDisk();
    }

    void SetSpeed(int speed)
    {
        State.Current.Speed = speed;
        State.Current.WriteToDisk();
        _speedLabel.Text = $"Speed ({State.Current.SpeedFactor:P0}):";
    }

    public void Dispose()
    {
        _windowCreated.Wait();
        _closeConfirmed = true; // driver unload: close without confirmation
        if (_window is Window window)
            window.Dispose(); // thread-safe: posts WM_CLOSE
        _thread?.Join();
    }
}
