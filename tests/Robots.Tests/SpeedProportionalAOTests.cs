using NUnit.Framework;
using Rhino.Geometry;
using Robots.Commands;

namespace Robots.Tests;

public class SpeedProportionalAOTests
{
    [Test]
    public void AbbEmitsOneSpeedProcessForContiguousLinearMotions()
    {
        SpeedProportionalAO process = new(0, 0.05, 0.1, "Flow");
        Program program = CreateAbbProgram(process);
        string code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(Count(code, "VAR triggdata FlowOn;"), Is.EqualTo(1));
            Assert.That(Count(code, "VAR triggdata FlowOff;"), Is.EqualTo(1));
            Assert.That(Count(code, @"TriggSpeed FlowOn,0\Start,0.1,AO1,0.05\DipLag:=0.1;"), Is.EqualTo(1));
            Assert.That(Count(code, "TriggSpeed FlowOff,0,0.1,AO1,0;"), Is.EqualTo(1));
            Assert.That(Count(code, "TriggL "), Is.EqualTo(3));
            Assert.That(Count(code, @"\T2:=FlowOff"), Is.EqualTo(1));
            Assert.That(Count(code, "SetAO AO1,0;"), Is.EqualTo(3));
            Assert.That(code, Does.Contain("ERROR\nSetAO AO1,0;\nRAISE;"));
            Assert.That(code, Does.Contain("MoveAbsJ [["));
            Assert.That(program.Warnings, Has.None.Contains("Commands on a fly-by target"));
        });
    }

    [Test]
    public void AbbRejectsAFileBoundaryInsideAProcessSequence()
    {
        SpeedProportionalAO process = new(0, 0.05, name: "Flow");
        Program program = CreateAbbProgram(process, multiFileIndices: [0, 3]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.Contains("multi-file boundary cannot split"));
    }

    static Program CreateAbbProgram(
        SpeedProportionalAO process,
        IReadOnlyList<int>? multiFileIndices = null)
    {
        Plane start = Plane.WorldYZ;
        start.Origin = new(300, 200, 610);
        Plane first = start;
        first.Origin = new(300, 100, 610);
        Plane second = first;
        second.Origin = new(300, 0, 610);
        Plane third = second;
        third.Origin = new(300, -100, 610);
        Zone flyBy = new(10);

        Target[] targets =
        [
            new CartesianTarget(start, RobotConfigurations.Wrist, Motions.Joint),
            new CartesianTarget(first, motion: Motions.Linear),
            new CartesianTarget(second, motion: Motions.Linear, command: process),
            new CartesianTarget(third, motion: Motions.Linear, zone: flyBy, command: process),
            new CartesianTarget(first, motion: Motions.Linear, command: process),
            new CartesianTarget(start, motion: Motions.Linear)
        ];

        return new(
            "P",
            TestRobots.AbbIrb120(),
            [TestRobots.Toolpath(targets)],
            multiFileIndices: multiFileIndices);
    }

    static int Count(string source, string value) =>
        source.Split(value, StringSplitOptions.None).Length - 1;
}
