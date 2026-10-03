using System.Numerics;
using System.Diagnostics;

public class Outliers
{
    const int HistoryLength = 16;
    const int RecoverySamples = 5;
    const float MaxRotationSpeed = 0.55f;
    const float MaxTranslationSpeed = 0.8f;
    const float SustainedTranslationSpeed = 0.75f;
    const float MinimumStraightness = 0.99825f;

    readonly ContinuousRingBuffer<(Matrix4x4 Matrix, long Time)> _samples = new(HistoryLength);
    int _recoveryCounter = 0;

    public State LastState { get; private set; } = State.Init;

    public enum State
    {
        Init,
        Valid,
        Drift,
        OutlierTranslation,
        OutlierRotation,
        Recovering,
        LostTracking,
        Invalid,
    }

    public State IsOutlier(Matrix4x4 sample, ref DriverPose_t pose, float speed)
    {
        var state = IsOutlierInternal(sample, ref pose, (float)Math.Clamp(speed, 0.65, 1.35));
        LastState = state;
        return state;
    }

    private State IsOutlierInternal(Matrix4x4 sample, ref DriverPose_t pose, float speed)
    {
        var now = Stopwatch.GetTimestamp();
        if (_samples.Count == 0)
        {
            _samples.Enqueue((sample, now));
            return State.Init;
        }

        var translation = Vector3.Distance(pose.vecVelocity.ToNumerics(), Vector3.Zero);
        var angularVelocity = pose.vecAngularVelocity.ToNumerics();
        var maxAngular = Math.Max(Math.Abs(angularVelocity.X), Math.Max(Math.Abs(angularVelocity.Y), Math.Abs(angularVelocity.Z)));

        var outlierTranslation = !float.IsFinite(translation) || (translation > MaxTranslationSpeed * speed);
        var outlierRotation = !float.IsFinite(maxAngular) || (maxAngular > MaxRotationSpeed * speed);
        var outlier = outlierTranslation || outlierRotation;

        // If the driver itself determines the pose to be invalid, we don't enqueue it for drift detection
        var validPose = pose.poseIsValid != 0;
        if (validPose)
            _samples.Enqueue((sample, now));

        var drift = IsSustainedDrift(now);
        var lost = IsLostTrackingIndicator(sample);

        outlier |= drift;
        outlier |= lost;

        // Recovery logic, require a few valid samples before continuing
        if (outlier)
        {
            _recoveryCounter = RecoverySamples;
        }
        else if (_recoveryCounter > 0)
        {
            _recoveryCounter--;
            return State.Recovering;
        }

        // Special result cases
        if (lost)
            return State.LostTracking;
        else if (drift)
            return State.Drift;
        else if (!validPose)
            return State.Invalid;

        // General outlier/valid result
        return outlierTranslation ? State.OutlierTranslation :
            (outlierRotation ? State.OutlierRotation : State.Valid);
    }

    // A Lighthouse-loss IMU drift is typically a fast, nearly straight translation
    // over many callbacks. The path-length ratio rejects it without averaging poses.
    bool IsSustainedDrift(long now)
    {
        if (_samples.Count < HistoryLength) return false;
        var (firstMatrix, firstTime) = _samples[0];
        var seconds = (float)(now - firstTime) / Stopwatch.Frequency;

        var previous = firstMatrix.Translation;
        var pathLength = 0f;
        for (int i = 1; i < _samples.Count; i++)
        {
            var (sampleMatrix, _) = _samples[i];
            pathLength += Vector3.Distance(previous, sampleMatrix.Translation);
            previous = sampleMatrix.Translation;
        }
        var displacement = Vector3.Distance(firstMatrix.Translation, previous);
        return displacement / seconds > SustainedTranslationSpeed &&
               pathLength > 0f && displacement / pathLength > MinimumStraightness;
    }

    static bool IsLostTrackingIndicator(Matrix4x4 sample)
    {
        if (Matrix4x4.Decompose(sample, out var _, out var _, out var position))
        {
            // SteamVR lighthouse devices report 0,0,N (with N arbitrary) when they completely lost tracking
            // we handle this case specially in the caller
            var zeros = 0;
            if (Math.Abs(position.X) < 0.00001f) zeros++;
            if (Math.Abs(position.Y) < 0.00001f) zeros++;
            if (Math.Abs(position.Z) < 0.00001f) zeros++;
            if (zeros >= 2) return true;
        }
        return false;
    }
}