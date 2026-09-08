using System.Numerics;

public static class PoseHook
{
    const float MaxMotionEstimate = 0.666f;
    const float MotionDecay = 0.993f;

    const float MinBlendFactor = 0.0008f;
    const float BlendDecay = 0.985f;
    static volatile float BlendFactor = 1f;

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

                if (pose.deviceIsConnected == 0)
                {
                    Utilities.Log("Device disconnected: " + deviceIndex);
                    state.Devices.TryRemove(deviceIndex, out _);
                    return;
                }

                var selfTrackingSpace = selfDevice.TrackingSpace;
                if (string.IsNullOrEmpty(selfTrackingSpace) || selfDevice.SerialNumber.StartsWith("VRLINKQ_Hand"))
                    return;

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
                    selfDevice.LastPose = poseMatrix;
                    if (activeTargetIndex != 0 && selfDevice.TrackingSpace == correctedTrackingSpace)
                        Utilities.ApplyWorldTransform(ref pose, state.ActiveCorrection.Value);
                    return;
                }

                lock (selfDevice.PoseGate)
                {
                    var isValid = pose.deviceIsConnected != 0 &&
                                  pose.poseIsValid != 0 &&
                                  pose.result == OpenVr.TrackingResultRunningOk &&
                                  !selfDevice.Outliers.IsOutlierAndStore(poseMatrix);

                    if (isValid)
                    {
                        // motion estimation and LastPose tracking
                        if (selfDevice.LastPose is Matrix4x4 lastPose)
                        {
                            var delta = GetDelta(poseMatrix, lastPose);
                            selfDevice.MotionEstimate = Math.Clamp(delta + selfDevice.MotionEstimate, 0f, 4f);
                        }
                        selfDevice.MotionEstimate *= MotionDecay;
                        selfDevice.LastPose = poseMatrix;

                        // handle running correction and calibration
                        if (isActiveTracker)
                            HandleValidActiveTrackerPose(state, selfDevice, poseMatrix);
                    }
                    else
                    {
                        // ignore outlier/error pose, reset LastPose to indicate for calibration to skip a step
                        // still apply offset below though
                        selfDevice.LastPose = null;
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

                if (!state.Calibrate && Calibration.Active)
                {
                    lock (Calibration.CalibrationLock)
                    {
                        Utilities.Log("Stopping calibration collector.");
                        Calibration.Stop();
                        state.WriteToDisk();
                        BlendFactor = 1f;
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
    static void HandleValidActiveTrackerPose(State state, Device targetDevice, Matrix4x4 poseMatrix)
    {
        // check if we and the HMD have a valid, recent pose
        if (!Matrix4x4.Invert(poseMatrix, out var activeInverse) ||
            !state.Devices.TryGetValue(0, out var hmdDevice))
        {
            return;
        }

        var hmdPoseRaw = hmdDevice.LastPose;
        if (hmdPoseRaw is not Matrix4x4 hmdPose) return;

        // calibration logic, if requested
        var blendReset = false;
        if (state.Calibrate)
        {
            lock (Calibration.CalibrationLock)
            {
                if (Calibration.Update(activeInverse, hmdPose, out var result))
                {
                    Utilities.Log("Calibration result: " + Utilities.SerializeMatrix(result));
                    state.ActiveOffset.Set(result);
                    blendReset = true;
                }
            }
        }

        // only compute offset correction when not moving to avoid head jiggle moving trackers
        if (!blendReset && BlendFactor < 0.1f)
        {
            if (targetDevice.MotionEstimate > MaxMotionEstimate) return;
            if (hmdDevice.MotionEstimate > MaxMotionEstimate) return;
        }

        // offset logic, based on calibrated offset
        var offset = state.ActiveOffset.Value;

        // Matrix chain:
        // - pose: from 0,0,0 to current device's position/rotation
        // - activeInverse: from target device's space to world space
        //   -> both the target device and the current device move as if the target device is now at 0,0,0
        // - offset: calibrated offset matrix, rigid in physical space
        //   -> we move our combined device thingy to it's offset, as if the hmd was at the origin
        // - hmdPose: from 0,0,0 to HMD's position/rotation
        //   -> we finally move it all into HMD's space
        var correction = activeInverse * offset * hmdPose;
        BlendIntoCorrection(state, correction, blendReset);
    }

    static void BlendIntoCorrection(State state, Matrix4x4 newCorrection, bool blendReset) // -> into state.ActiveCorrection
    {
        var prevCorrection = state.ActiveCorrection.Value;
        var blend = blendReset ? 1f : BlendFactor;
        var correction = Utilities.Blend(prevCorrection, newCorrection, blend);
        blend = Math.Max(blend * BlendDecay, MinBlendFactor);
        BlendFactor = blend;
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
