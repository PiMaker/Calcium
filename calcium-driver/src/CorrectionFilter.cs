using System.Numerics;
using System.Diagnostics;

public class CorrectionFilter
{
    const float TranslationSpeed = 0.4f;
    const float RotationSpeed = 0.04f;
    const float ScaleSpeed = 0.0025f;

    bool _initialized;
    long _lastTime;

    Vector3 _translation;
    Quaternion _rotation;
    Vector3 _scale;

    public void Init(in Matrix4x4 initialData)
    {
        if (Matrix4x4.Decompose(initialData, out _scale, out _rotation, out _translation))
            _initialized = true;

        _lastTime = Stopwatch.GetTimestamp();
    }

    public Matrix4x4 ApplyFilter(in Matrix4x4 newData, float speed)
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
        _scale = Vector3.Lerp(_scale, newScale, RateToAlpha(speed * ScaleSpeed, dt));

        return Compose();
    }

    Matrix4x4 Compose()
    {
        var result = Matrix4x4.CreateScale(_scale) * Matrix4x4.CreateFromQuaternion(_rotation);
        result.Translation = _translation;
        return result;
    }
}
