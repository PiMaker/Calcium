using System.Numerics;

public class PoseHandler
{
    const float MaxRotationSpeedCorrecting = 0.75f; // radians per second
    const float MaxRotationSpeedCalibrating = 1.25f; // radians per second

    readonly CorrectionFilter _filter = new();
    readonly Calibration _calibration = new();
    readonly Matrix4x4 _targetRemovalOffset = Matrix4x4.CreateTranslation(0, 9002, 0); // way up high to hide it

    public void HandleIncomingPose(uint deviceIndex, ref DriverPose_t pose)
    {
        var state = State.Current;
        if (!state.Devices.TryGetValue(deviceIndex, out var selfDevice))
        {
            if (pose.deviceIsConnected == 0) return;
            state.InsertDevice(deviceIndex);
            selfDevice = state.Devices[deviceIndex];
        }

        // check for Hand controllers _before_ disconnect handling
        // we expect these to stay connected forever once they show up, but they report as disconnected when not in view of tracking cams
        var selfTrackingSpace = selfDevice.TrackingSpace;
        if (string.IsNullOrEmpty(selfTrackingSpace) || selfDevice.IsKnownHandTracking)
            return;

        if (pose.deviceIsConnected == 0)
        {
            state.RemoveDevice(deviceIndex);
            return;
        }

        var poseMatrix = Utilities.GetPoseMatrix(pose);
        var activeTargetSerial = state.ActiveTrackerSerial;
        var selfSerial = selfDevice.SerialNumber;
        var isActiveTracker = deviceIndex != 0 /* HMD */ && !string.IsNullOrEmpty(selfSerial) && selfSerial == activeTargetSerial;
        if (isActiveTracker) state.ActiveTrackingSpace = selfTrackingSpace;
        var correctedTrackingSpace = state.ActiveTrackingSpace;

        if (selfDevice.DeviceClass == OpenVr.DeviceClassTrackingReference)
        {
            // basestations should still be shifted, but aren't needed for calibration
            selfDevice.LastPose.Set(poseMatrix);
            if (!string.IsNullOrEmpty(activeTargetSerial) && selfDevice.TrackingSpace == correctedTrackingSpace)
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

            if (deviceIndex != 0 /* HMD */ && !string.IsNullOrEmpty(activeTargetSerial) &&
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

    // must hold activeDevice.PoseLock, updates correction matrix
    void HandleValidActiveTrackerPose(State state, Matrix4x4 poseMatrix)
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
            // update actual calibration with new pose data
            if (_calibration.Update(activeInverse, hmdPose, out var result))
            {
                state.ActiveOffset.Set(result);
                resetFilter = true;
            }

            // report progress to UI
            var collected = _calibration.CollectedSampleCount;
            state.ReportCalibrationProgress((float)collected / Calibration.MaxSamples);

            // check if we're finished
            if (collected >= Calibration.MaxSamples)
            {
                State.Current.Calibrating = false;

                var final = state.ActiveOffset.Value;
                if (!final.IsIdentity && Matrix4x4.Decompose(final, out var scale, out var rotation, out var translation))
                {
                    // log some interesting stuff
                    Utilities.Log($"Result - Scale: {scale}, Rotation: {rotation}, Translation: {translation}");
                    Utilities.Log($"Gravity error - {_calibration.CheckGravityAlignment(rotation)}deg RMS");
                }
                else
                {
                    // I don't think this can happen, but if you're here because it just did, congrats
                    Utilities.Log("Calibration complete but final matrix was invalid, try again.");
                    state.ActiveOffset.Set(Matrix4x4.Identity);
                    resetFilter = true;
                }

                _calibration.Reset();
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
}
