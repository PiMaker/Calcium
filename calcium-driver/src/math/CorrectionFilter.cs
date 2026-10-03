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

    Vector3 _translation;
    Quaternion _rotation;
    float _scale;

    public Matrix4x4 ApplyFilter(in Matrix4x4 prevCorrection, in Matrix4x4 newCorrection, bool resetFilter, float speed)
    {
        var translationDelta = Vector3.Distance(prevCorrection.Translation, newCorrection.Translation);
        var angularDelta = Matrix4x4.Invert(newCorrection, out var inverted) ? Utilities.RotationAngle(prevCorrection * inverted) : 0f;
        if (translationDelta > TrackingJumpThreshold || angularDelta > TrackingJumpRotThreshold || resetFilter)
        {
            Utilities.Log($"Tracking jump detected: {translationDelta}m, {angularDelta:F4}rad");
            Init(newCorrection);
            return newCorrection;
        }

        return ComputeSmoothed(newCorrection, speed);
    }

    private void Init(in Matrix4x4 initialData)
    {
        if (Matrix4x4.Decompose(initialData, out var scale, out _rotation, out _translation))
        {
            _scale = scale.X;
            _initialized = true;

            Utilities.Log($"Filter initialized with translation: {_translation}, rotation: {Utilities.EulerAngles(_rotation)}, scale: {_scale}");
        }

        _lastTime = Stopwatch.GetTimestamp();
    }

    private Matrix4x4 ComputeSmoothed(in Matrix4x4 newData, float speed)
    {
        var now = Stopwatch.GetTimestamp();
        var dt = (now - _lastTime) / (float)Stopwatch.Frequency;

        if (!_initialized)
            return newData;

        _lastTime = now;

        if (!Matrix4x4.Decompose(newData, out var newScale, out var newRotation, out var newTranslation))
            return newData;

        static float RateToAlpha(float rate, float dt) => 1f - MathF.Exp(-rate * dt);

        _translation = Vector3.Lerp(_translation, newTranslation, RateToAlpha(speed * TranslationSpeed, dt));
        _rotation = Quaternion.Slerp(_rotation, newRotation, RateToAlpha(speed * RotationSpeed, dt));
        _scale = float.Lerp(_scale, newScale.X, RateToAlpha(speed * ScaleSpeed, dt));

        var result = Matrix4x4.CreateScale(_scale) * Matrix4x4.CreateFromQuaternion(_rotation);
        result.Translation = _translation;
        return result;
    }
}
