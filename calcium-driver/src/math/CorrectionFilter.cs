using System.Numerics;
using System.Diagnostics;

public class CorrectionFilter
{
    const float TranslationSpeed = 0.4f;
    const float RotationSpeed = 0.04f;
    const float ScaleSpeed = 0.0025f;

    const float TrackingJumpThreshold = 0.70f; // off by more than 70cm - consider tracking jump and correct immediately
    const float TrackingJumpRotThreshold = (float)Math.PI / 2f;

    bool _initialized;
    long _lastTime;

    Quaternion _rotation;
    float _scale;

    public void UpdateTime() => Interlocked.Exchange(ref _lastTime, Stopwatch.GetTimestamp());

    public Matrix4x4 ApplyFilter(in Matrix4x4 prevCorrection, Matrix4x4 newCorrection, Vector3 headPosition, bool gravityAligned, bool resetFilter, float speed)
    {
        if (!Matrix4x4.Invert(newCorrection, out var inverted)) return prevCorrection;
        var anchor = Vector3.Transform(headPosition, inverted); // current head position in tracker coordinate space given by new data at 100% application
        if (gravityAligned)
        {
            newCorrection = Utilities.YawOnly(newCorrection);
            newCorrection.Translation = headPosition - Vector3.TransformNormal(anchor, newCorrection);
        }

        var translationDelta = Vector3.Distance(Vector3.Transform(anchor, prevCorrection), headPosition);
        var angularDelta = Matrix4x4.Decompose(prevCorrection * inverted, out _, out var deltaRotation, out _) ? Utilities.RotationAngle(deltaRotation) : 0f;
        if (!_initialized || translationDelta > TrackingJumpThreshold || angularDelta > TrackingJumpRotThreshold || resetFilter)
        {
            Utilities.Log($"Tracking jump detected: {translationDelta}m, {angularDelta:F4}rad");
            Init(newCorrection);
            return newCorrection;
        }

        return ComputeSmoothed(prevCorrection, newCorrection, anchor, headPosition, speed);
    }

    private void Init(in Matrix4x4 initialData)
    {
        if (Matrix4x4.Decompose(initialData, out var scale, out _rotation, out var translation))
        {
            _scale = scale.X;
            _initialized = true;

            Utilities.Log($"Filter initialized with translation: {translation}, rotation: {Utilities.EulerAngles(_rotation)}, scale: {_scale}");
        }
    }

    private Matrix4x4 ComputeSmoothed(in Matrix4x4 prevCorrection, in Matrix4x4 newCorrection, Vector3 anchor, Vector3 headPosition, float speed)
    {
        var now = Stopwatch.GetTimestamp();
        var dt = (now - _lastTime) / (float)Stopwatch.Frequency;

        if (!_initialized)
            return newCorrection;

        if (!Matrix4x4.Decompose(newCorrection, out var newScale, out var newRotation, out _))
            return newCorrection;

        static float RateToAlpha(float rate, float dt) => 1f - MathF.Exp(-rate * dt);

        var headDelta = headPosition - Vector3.Transform(anchor, prevCorrection);
        _rotation = Quaternion.Slerp(_rotation, newRotation, RateToAlpha(speed * RotationSpeed, dt));
        _scale = float.Lerp(_scale, newScale.X, RateToAlpha(speed * ScaleSpeed, dt));

        // Compensate rotation and scale at the head, not the playspace origin.
        var result = Matrix4x4.CreateScale(_scale) * Matrix4x4.CreateFromQuaternion(_rotation);
        result.Translation = prevCorrection.Translation + headDelta * RateToAlpha(speed * TranslationSpeed, dt)
            + Vector3.TransformNormal(anchor, prevCorrection - result);

        return result;
    }
}
