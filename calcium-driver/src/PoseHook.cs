using System.Numerics;

public static class PoseHook
{
    const float MaxRotationSpeedCorrecting = 2f; // radians per second
    const float MaxRotationSpeedCalibrating = 8f; // radians per second

    const float MaxAngularVelocity = 2f;
    const float MaxAngularAcceleration = 0.4f;

    const float BlendRotationFactor = 0.01f;
    const float BlendTranslationFactor = 0.1f;
    const float BlendScaleFactor = 0.002f;

    const float TrackingJumpThreshold = 0.2f; // off by more than 20cm - consider tracking jump and correct immediately
    const float TrackingJumpRotThreshold = (float)Math.PI / 2f;

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
                    if (selfDevice.SerialNumber == Volatile.Read(ref state.ActiveSerialNumber))
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
                if (isActiveTracker && Volatile.Read(ref state.ActiveSerialNumber) != selfSerialNumber)
                {
                    Interlocked.Exchange(ref state.ActiveSerialNumber, selfSerialNumber);
                    state.WriteToDisk();
                }

                if (selfDevice.DeviceClass == OpenVr.DeviceClassTrackingReference)
                {
                    // basestations should still be shifted, but aren't needed for calibration
                    selfDevice.LastPose.Set(poseMatrix);
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
                                      state.CalibrateUpTo > 0 ? MaxRotationSpeedCalibrating : MaxRotationSpeedCorrecting);

                    if (isValid)
                    {
                        // LastPose tracking
                        selfDevice.LastPose.Set(poseMatrix);

                        // handle running correction and calibration
                        if (isActiveTracker)
                            HandleValidActiveTrackerPose(state, poseMatrix, ref pose);
                    }
                    else
                    {
                        // ignore outlier/error pose, reset LastPose to indicate for calibration to skip a step
                        // still apply offset below though
                        selfDevice.LastPose.Set(Matrix4x4.Identity);
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

                if (state.CalibrateUpTo == 0 && Calibration.Active)
                {
                    lock (Calibration.CalibrationLock)
                    {
                        Utilities.Log("Stopping calibration collector.");
                        Calibration.Stop();
                        state.WriteToDisk();
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

    static float GetDelta(Matrix4x4 a, Matrix4x4 b)
    {
        if (Matrix4x4.Decompose(a, out _, out var aRotation, out var aTranslation) &&
            Matrix4x4.Decompose(b, out _, out var bRotation, out var bTranslation))
        {
            var deltaTranslation = Vector3.Distance(aTranslation, bTranslation);
            var deltaRotation = Quaternion.Dot(aRotation, bRotation);
            return deltaTranslation * 4f + (1.0f - deltaRotation) * 32f;
        }
        return 0f;
    }

    // must hold activeDevice.PoseLock, updates correction matrix
    static void HandleValidActiveTrackerPose(State state, Matrix4x4 poseMatrix, ref DriverPose_t pose)
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
        var calibrate = state.CalibrateUpTo;
        if (calibrate > 0)
        {
            lock (Calibration.CalibrationLock)
            {
                if (Calibration.Update(activeInverse, hmdPose, out var result, calibrate))
                {
                    Utilities.Log("Calibration result: " + Utilities.SerializeMatrix(result));
                    state.ActiveOffset.Set(result);
                }
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
        BlendIntoCorrection(state, correction, ref pose);
    }

    static void BlendIntoCorrection(State state, Matrix4x4 newCorrection, ref DriverPose_t pose) // -> into state.ActiveCorrection
    {
        var prevCorrection = state.ActiveCorrection.Value;

        var translationDelta = Vector3.Distance(prevCorrection.Translation, newCorrection.Translation);
        var angularDelta = Matrix4x4.Invert(newCorrection, out var inverted) ? Utilities.RotationAngle(prevCorrection * inverted) : 0f;
        if (translationDelta > TrackingJumpThreshold || angularDelta > TrackingJumpRotThreshold)
        {
            Utilities.Log($"Tracking jump detected: {translationDelta}m, {angularDelta:F4}rad");
            state.ActiveCorrection.Set(newCorrection);
            return;
        }

        // Blend into the active pose correction based on rotation velocity and acceleration.
        // Translation is not accounted for, perfectly linear motion without rotation is unlikely.
        // The goal is to avoid considering interpolated or intertially extrapolated poses from the
        // device that may overshoot or have greater deltas due to time-misalignment.
        var angularVelocity = pose.vecAngularVelocity;
        var angularAcceleration = pose.vecAngularAcceleration;
        var maxVelocity = Math.Max(Math.Abs(angularVelocity.x), Math.Max(Math.Abs(angularVelocity.y), Math.Abs(angularVelocity.z)));
        var maxAcceleration = Math.Max(Math.Abs(angularAcceleration.x), Math.Max(Math.Abs(angularAcceleration.y), Math.Abs(angularAcceleration.z)));

        var blendVelocity = 1f - Math.Min(maxVelocity / MaxAngularVelocity, 1f);
        var blendAcceleration = 1f - Math.Min(maxAcceleration / MaxAngularAcceleration, 1f);

        var blend = (float)Math.Min(blendVelocity, blendAcceleration);
        state.LastCorrectionBlend = blend;

        var correction = Utilities.Blend(prevCorrection, newCorrection,
            tRot: blend * BlendRotationFactor,
            tScale: blend * BlendScaleFactor,
            tTranslate: blend * BlendTranslationFactor + translationDelta * BlendTranslationFactor /* linearize somewhat */);

        state.ActiveCorrection.Set(correction);
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
