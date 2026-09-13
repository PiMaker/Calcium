
using System.Numerics;

public class CorrectionFilter
{
    public void Init(in Matrix4x4 initialData)
    {
        // TODO
    }

    public Matrix4x4 ApplyFilter(in Matrix4x4 newData, float speed)
    {
        return newData; // TODO
    }

    // TODO: old helper function, delete if not used, or rewrite
    private static Matrix4x4 Blend(in Matrix4x4 a, in Matrix4x4 b, float tRot, float tScale, float tTranslate)
    {
        tRot = Math.Clamp(tRot, 0f, 1f);
        tScale = Math.Clamp(tScale, 0f, 1f);
        tTranslate = Math.Clamp(tTranslate, 0f, 1f);

        if (Matrix4x4.Decompose(a, out var aScale, out var aRotation, out var aTranslation) &&
            Matrix4x4.Decompose(b, out var bScale, out var bRotation, out var bTranslation))
        {
            var scale = Vector3.Lerp(aScale, bScale, tScale);
            var rotation = Quaternion.Slerp(aRotation, bRotation, tRot);
            var translation = Vector3.Lerp(aTranslation, bTranslation, tTranslate);
            return Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation);
        }
        return a;
    }
}