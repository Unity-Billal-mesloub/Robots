namespace Robots;

static class RapidFormatting
{
    const string DefaultExternal = "[9E9,9E9,9E9,9E9,9E9,9E9]";

    internal static string JointTargetValue(
        RobotSystem robotSystem,
        JointTarget target,
        int group,
        bool useDefaultExternalVariable)
    {
        ArgumentNullException.ThrowIfNull(robotSystem);
        ArgumentNullException.ThrowIfNull(target);

        if (robotSystem is not SystemAbb system)
            throw new ArgumentException("RAPID joint targets require an ABB robot system.", nameof(robotSystem));

        ArgumentOutOfRangeException.ThrowIfNegative(group);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(group, system.MechanicalGroups.Count);

        if (system.ValidateTargetAxes(group, target) is string error)
            throw new ArgumentException($"{error}.", nameof(target));

        var mechanicalGroup = system.MechanicalGroups[group];
        double[] joints = target.Joints.Map(mechanicalGroup.RadianToDegree);
        string jointValues = $"[{joints[0]:0.####},{joints[1]:0.####},{joints[2]:0.####},{joints[3]:0.####},{joints[4]:0.####},{joints[5]:0.####}]";
        string external = ExternalTargetValue(mechanicalGroup, target, useDefaultExternalVariable);
        return $"[{jointValues},{external}]";
    }

    internal static string ExternalTargetValue(
        MechanicalGroup mechanicalGroup,
        Target target,
        bool useDefaultExternalVariable)
    {
        if (mechanicalGroup.Externals.Length == 0)
            return useDefaultExternalVariable ? "extj" : DefaultExternal;

        double[] values = mechanicalGroup.RadiansToDegreesExternal(target);
        var externals = new string[6];
        Array.Fill(externals, "9E9");

        if (target.ExternalCustom is null)
        {
            for (int i = 0; i < values.Length; i++)
                externals[i] = $"{values[i]:0.####}";
        }
        else
        {
            for (int i = 0; i < target.ExternalCustom.Length; i++)
            {
                string value = target.ExternalCustom[i];

                if (!string.IsNullOrEmpty(value))
                    externals[i] = value;
            }
        }

        return $"[{string.Join(",", externals)}]";
    }
}
