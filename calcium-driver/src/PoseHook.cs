using System.Numerics;

public static class PoseHook
{
    const float MaxRotationSpeedCorrecting = 0.75f; // radians per second
    const float MaxRotationSpeedCalibrating = 4f; // radians per second

    static readonly CorrectionFilter _filter = new();

    static readonly Matrix4x4 _targetRemovalOffset = Matrix4x4.CreateTranslation(0, 9002, 0); // way up high to hide it

    public static unsafe void PoseDetour(IntPtr self, uint deviceIndex, IntPtr posePtr, uint structSize)
    {
        try
        {
            if (posePtr != IntPtr.Zero && structSize == OpenVr.DriverPoseSize)
            {
                ref var pose = ref *(DriverPose_t*)posePtr;
                var state = State.Current;
                if (!state.Devices.TryGetValue(deviceIndex, out var selfDevice))
                {
                    if (pose.deviceIsConnected == 0) return;
                    InsertDevice(state, deviceIndex);
                    selfDevice = state.Devices[deviceIndex];

                    // activate after disk restore on launch
                    if (selfDevice.SerialNumber == state.ActiveSerialNumber)
                        state.ActiveTargetIndex = deviceIndex;
                }

                // check for Hand controllers _before_ disconnect handling
                // we expect these to stay connected forever once they show up, but they report as disconnected when not in view of tracking cams
                var selfTrackingSpace = selfDevice.TrackingSpace;
                if (string.IsNullOrEmpty(selfTrackingSpace) || selfDevice.IsKnownHandTracking)
                    return;

                if (pose.deviceIsConnected == 0)
                {
                    Utilities.Log("Device disconnected: " + deviceIndex);
                    state.Devices.TryRemove(deviceIndex, out _);
                    return;
                }

                var poseMatrix = Utilities.GetPoseMatrix(pose);
                var activeTargetIndex = state.ActiveTargetIndex;
                var isActiveTracker = deviceIndex != 0 /* HMD */ && deviceIndex == activeTargetIndex;
                var correctedTrackingSpace = state.Devices.TryGetValue(activeTargetIndex, out var device) ? device.TrackingSpace : null;

                var selfSerialNumber = selfDevice.SerialNumber;
                if (isActiveTracker && state.ActiveSerialNumber != selfSerialNumber)
                {
                    Interlocked.Exchange(ref state.ActiveSerialNumber, selfSerialNumber);
                    state.WriteToDisk();
                }

                if (selfDevice.DeviceClass == OpenVr.DeviceClassTrackingReference)
                {
                    // basestations should still be shifted, but aren't needed for calibration
                    //selfDevice.LastPose.Set(poseMatrix);
                    if (activeTargetIndex != 0 && selfDevice.TrackingSpace == correctedTrackingSpace)
                        Utilities.ApplyWorldTransform(ref pose, state.ActiveCorrection.Value);
                    return;
                }

                lock (selfDevice.PoseGate)
                {
                    var isValid = pose.deviceIsConnected != 0 &&
                                  pose.poseIsValid != 0 &&
                                  pose.result == OpenVr.TrackingResultRunningOk &&
                                  !selfDevice.Outliers.IsOutlierAndStore(poseMatrix,
                                      state.Calibrating ? MaxRotationSpeedCalibrating : MaxRotationSpeedCorrecting * State.Current.SpeedFactor);

                    if (isValid)
                    {
                        // LastPose tracking
                        selfDevice.LastPose.Set(poseMatrix);

                        // handle running correction and calibration
                        if (isActiveTracker)
                            HandleValidActiveTrackerPose(state, poseMatrix);
                    }
                    else if (deviceIndex == 0 /* HMD */)
                    {
                        // ignore outlier/error pose, reset LastPose to indicate for calibration to skip a step
                        selfDevice.LastPose.Set(Matrix4x4.Identity);
                    }
                    else
                    {
                        // TODO: For debugging trackers that die in UI
                        selfDevice.LastPose.Set(poseMatrix);
                    }

                    if (deviceIndex != 0 /* HMD */ && activeTargetIndex != 0 &&
                        selfDevice.TrackingSpace == correctedTrackingSpace)
                    {
                        // apply the latest correction data to the pose
                        Utilities.ApplyWorldTransform(ref pose, state.ActiveCorrection.Value);

                        // hide active target tracker
                        if (isActiveTracker)
                            Utilities.ApplyWorldTransform(ref pose, _targetRemovalOffset);
                    }
                }
            }
        }
        catch (Exception e)
        {
            Utilities.Log($"An exception occurred in PoseDetour: {e}");
        }
        finally
        {
            // always call the original exactly once, even on early-out
            HookInjector._poseOriginal(self, deviceIndex, posePtr, structSize);
        }
    }

    // must hold activeDevice.PoseLock, updates correction matrix
    static void HandleValidActiveTrackerPose(State state, Matrix4x4 poseMatrix)
    {
        // check if we and the HMD have a valid, recent pose
        if (!Matrix4x4.Invert(poseMatrix, out var activeInverse) ||
            !state.Devices.TryGetValue(0, out var hmdDevice))
        {
            return;
        }

        var hmdPose = hmdDevice.LastPose.Value;
        if (hmdPose.IsIdentity) return;

        // calibration logic, if requested
        var resetFilter = false;
        if (state.Calibrating)
        {
            if (Calibration.Update(activeInverse, hmdPose, out var result))
            {
                state.ActiveOffset.Set(result);
                resetFilter = true;
            }

            if (Calibration.CollectedSampleCount >= Calibration.MaxSamples)
            {
                var collected = Calibration.CollectedSampleCount;
                if (collected >= Calibration.MaxSamples)
                {
                    State.Current.FinishCalibration();
                }

                Utilities.Log("Calibration complete!");
                Calibration.Reset();
                state.WriteToDisk();
            }
        }

        // Matrix chain to arrive at the actual playspace correction from the calibrated tracker offset:
        // - pose: from 0,0,0 to current device's position/rotation
        // - activeInverse: from target device's space to world space
        //   -> both the target device and the current device move as if the target device is now at 0,0,0
        // - ActiveOffset: calibrated offset matrix, rigid in physical space
        //   -> we move our combined device thingy to it's offset, as if the hmd was at the origin
        // - hmdPose: from 0,0,0 to HMD's position/rotation
        //   -> we finally move it all into HMD's space
        var correction = activeInverse * state.ActiveOffset.Value * hmdPose;
        var newCorrection = _filter.ApplyFilter(state.ActiveCorrection.Value, correction, resetFilter, state.SpeedFactor);
        state.ActiveCorrection.Set(newCorrection);
    }

    static void InsertDevice(State state, uint id)
    {
        var ct = DeviceProperties.GetContainer(id);
        var trackingSpace = DeviceProperties.GetString(ct, id, OpenVr.PropTrackingSystemName);
        var serialNumber = DeviceProperties.GetString(ct, id, OpenVr.PropSerialNumber);
        var deviceClass = DeviceProperties.GetInt(ct, id, OpenVr.PropDeviceClass);
        state.Devices.TryAdd(id, new Device(id, trackingSpace, serialNumber, deviceClass));
        Utilities.Log($"Added device: ID={id}, TrackingSpace={trackingSpace}, SerialNumber={serialNumber}, DeviceClass={deviceClass}");
    }
}
