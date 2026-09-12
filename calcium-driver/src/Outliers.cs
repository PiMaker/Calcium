using System.Numerics;
using System.Diagnostics;

public class Outliers
{
    const int HistoryLength = 16;
    const int RecoverySamples = 3;
    const float MaxTranslationSpeed = 4f; // metres per second
    const float MaxRotationSpeed = 1f; // radians per second
    const float MinTranslation = 0.00001f; // perfectly still devices are probably not tracking
    const float SustainedTranslationSpeed = 1f;
    const float MinimumStraightness = 0.97f;
    const float TranslationSlack = 0.01f; // tracker noise and callback jitter
    const float RotationSlack = 0.075f;

    readonly Queue<(Matrix4x4 Matrix, long Time)> _samples = new();
    int _recoveryCounter = 0;

    public bool IsOutlierAndStore(Matrix4x4 sample)
    {
        var now = Stopwatch.GetTimestamp();
        if (_samples.Count == 0)
        {
            Store(sample, now);
            return true;
        }

        var previous = _samples.Last();
        var seconds = (float)(now - previous.Time) / Stopwatch.Frequency;
        var translation = Vector3.Distance(sample.Translation, previous.Matrix.Translation);
        var previousRotation = Quaternion.CreateFromRotationMatrix(previous.Matrix);
        var rotation = Quaternion.CreateFromRotationMatrix(sample);
        var dot = Math.Clamp(MathF.Abs(Quaternion.Dot(previousRotation, rotation)), 0f, 1f);
        var angle = 2f * MathF.Acos(dot);

        var outlier = !float.IsFinite(translation) || !float.IsFinite(angle) ||
            (seconds > 0f && (translation > TranslationSlack + MaxTranslationSpeed * seconds ||
                              angle > RotationSlack + MaxRotationSpeed * seconds ||
                              translation < MinTranslation));
        Store(sample, now);
        outlier |= IsSustainedDrift(now);

        if (outlier)
        {
            _recoveryCounter = RecoverySamples;
        }
        else if (_recoveryCounter > 0)
        {
            _recoveryCounter--;
            outlier = true;
        }

        return outlier;
    }

    void Store(Matrix4x4 sample, long time)
    {
        _samples.Enqueue((sample, time));
        if (_samples.Count > HistoryLength) _samples.Dequeue();
    }

    // A Lighthouse-loss IMU drift is typically a fast, nearly straight translation
    // over many callbacks. The path-length ratio rejects it without averaging poses.
    bool IsSustainedDrift(long now)
    {
        if (_samples.Count < HistoryLength) return false;
        var first = _samples.Peek();
        var seconds = (float)(now - first.Time) / Stopwatch.Frequency;

        var previous = first.Matrix.Translation;
        var pathLength = 0f;
        foreach (var sample in _samples)
        {
            pathLength += Vector3.Distance(previous, sample.Matrix.Translation);
            previous = sample.Matrix.Translation;
        }
        var displacement = Vector3.Distance(first.Matrix.Translation, previous);
        return displacement / seconds > SustainedTranslationSpeed &&
               pathLength > 0f && displacement / pathLength > MinimumStraightness;
    }
}