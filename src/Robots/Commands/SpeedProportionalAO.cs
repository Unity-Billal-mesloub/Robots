namespace Robots.Commands;

/// <summary>
/// Marks a command that changes how the target motion is emitted rather than adding a separate instruction.
/// </summary>
public interface IMotionCommand { }

/// <summary>
/// Keeps an analogue output proportional to actual TCP speed while this command is present on contiguous motions.
/// </summary>
public class SpeedProportionalAO(
    int ao,
    double outputPerTcpSpeed,
    double delay = 0,
    string? name = null) : Command(name), IMotionCommand
{
    public int AO { get; } = CheckAO(ao);
    public double OutputPerTcpSpeed { get; } = CheckPositive(outputPerTcpSpeed, nameof(outputPerTcpSpeed));
    public double Delay { get; } = CheckNonNegative(delay, nameof(delay));

    static int CheckAO(int ao)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(ao);
        return ao;
    }

    public override string ToString() => $"Command (AO {AO} proportional to TCP speed)";
}
