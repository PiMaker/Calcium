using System.Numerics;
using System.Diagnostics;

public class Outliers
{
    const int HistoryLength = 16;
    const int RecoverySamples = 5;
    const float MaxRotationSpeed = 0.5f; // radians per second
    const float MaxTranslationSpeed = 5f; // metres per second
    const float MinTranslation = 0.00001f; // perfectly still devices are probably not tracking
    const float SustainedTranslationSpeed = 1f;
    const float MinimumStraightness = 0.97f;
    const float TranslationSlack = 0.01f; // tracker noise and callback jitter
    const float RotationSlack = 0.05f;

    readonly ContinuousRingBuffer<(Matrix4x4 Matrix, long Time)> _samples = new(HistoryLength);
    int _recoveryCounter = 0;

    public bool IsOutlierAndStore(Matrix4x4 sample)
    {
        var now = Stopwatch.GetTimestamp();
        if (_samples.Count == 0)
        {
            _samples.Enqueue((sample, now));
            return true;
        }

        var (prevMatrix, prevTime) = _samples[^1];
        var seconds = (float)(now - prevTime) / Stopwatch.Frequency;
        var translation = Vector3.Distance(sample.Translation, prevMatrix.Translation);
        var previousRotation = Quaternion.CreateFromRotationMatrix(prevMatrix);
        var rotation = Quaternion.CreateFromRotationMatrix(sample);
        var dot = Math.Clamp(MathF.Abs(Quaternion.Dot(previousRotation, rotation)), 0f, 1f);
        var angle = 2f * MathF.Acos(dot);

        var outlier = !float.IsFinite(translation) || !float.IsFinite(angle) ||
            (seconds > 0f && (translation > TranslationSlack + MaxTranslationSpeed * seconds ||
                              angle > RotationSlack + MaxRotationSpeed * seconds ||
                              translation < MinTranslation));

        _samples.Enqueue((sample, now));
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
}