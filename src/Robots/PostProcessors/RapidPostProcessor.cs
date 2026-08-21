using static System.Math;
using Rhino.Geometry;
using Robots.Commands;

namespace Robots;

class RapidPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program) =>
        new PostInstance((SystemAbb)system, program).Code;

    class PostInstance
    {
        readonly SystemAbb _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemAbb system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            for (var group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                var process = RapidSpeedProcess.Create(system, program, group);
                List<List<string>> groupCode = [MainModule(group, process)];

                for (var file = 0; file < program.MultiFileIndices.Count; file++)
                    groupCode.Add(SubModule(file, group, process));

                Code.Add(groupCode);
            }
        }

        List<string> MainModule(int group, RapidSpeedProcess? process)
        {
            List<string> code = [];
            var multiProgram = _program.MultiFileIndices.Count > 1;
            var groupName = _system.MechanicalGroups[group].Name;

            code.Add($"MODULE {_program.Name}_{groupName}");
            if (_system.MechanicalGroups[group].Externals.Length == 0)
                code.Add("VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];");
            code.Add("VAR confdata conf := [0,0,0,0];");

            var attributes = _program.Attributes;

            if (_system.MechanicalGroups.Count > 1)
            {
                code.Add("VAR syncident sync1;");
                code.Add("VAR syncident sync2;");
                var tasks = string.Join(", ", _system.MechanicalGroups.Select(group => $@"[""{group.Name}""]"));
                code.Add($@"TASK PERS tasks all_tasks{{{_system.MechanicalGroups.Count}}} := [{tasks}];");
            }

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
                code.Add(Tool(tool));

            foreach (var frame in attributes.OfType<Frame>().Where(t => !t.UseController))
                code.Add(Frame(frame));

            foreach (var speed in attributes.OfType<Speed>())
                code.Add(Speed(speed));

            foreach (var zone in attributes.OfType<Zone>().Where(z => z.IsFlyBy))
                code.Add(Zone(zone));

            PostProcessorUtil.AddDeclarations(code, _program);

            if (process is not null)
                code.AddRange(process.Declarations);

            code.Add("PROC Main()");
            if (!multiProgram)
                code.Add("ConfL \\Off;");

            if (group == 0)
                PostProcessorUtil.AddInitCommands(code, _program);

            if (process is not null)
                code.Add(process.OffCode);

            if (_system.MechanicalGroups.Count > 1)
                code.Add("SyncMoveOn sync1, all_tasks;");

            if (multiProgram)
            {
                for (var file = 0; file < _program.MultiFileIndices.Count; file++)
                {
                    code.Add($"Load\\Dynamic, \"HOME:/{_program.Name}/{_program.Name}_{groupName}_{file:000}.{_system.ModuleExtension}\";");
                    code.Add($"%\"{_program.Name}_{groupName}_{file:000}:Main\"%;");
                    code.Add($"UnLoad \"HOME:/{_program.Name}/{_program.Name}_{groupName}_{file:000}.{_system.ModuleExtension}\";");
                }

                if (process is not null)
                    code.Add(process.OffCode);

                if (_system.MechanicalGroups.Count > 1)
                    code.Add("SyncMoveOff sync2;");

                AddErrorHandler(code, process);
                code.Add("ENDPROC");
                code.Add("ENDMODULE");
            }

            return code;
        }

        List<string> SubModule(int file, int group, RapidSpeedProcess? process)
        {
            var mechGroup = _system.MechanicalGroups[group];
            var multiProgram = _program.MultiFileIndices.Count > 1;
            var groupName = mechGroup.Name;
            var (start, end) = _program.GetTargetRange(file);
            List<string> code = [];

            if (multiProgram)
            {
                code.Add($"MODULE {_program.Name}_{groupName}_{file:000}");
                code.Add("PROC Main()");
                code.Add("ConfL \\Off;");
            }

            for (var i = start; i < end; i++)
            {
                var programTarget = _program.Targets[i].ProgramTargets[group];
                var target = programTarget.Target;
                var zone = (target.Zone.IsFlyBy ? target.Zone.Name : "fine").NotNull("Zone name cannot be null.");
                var id = _system.MechanicalGroups.Count > 1 ? $@"\ID:={programTarget.Index}" : "";
                var external = RapidFormatting.ExternalTargetValue(mechGroup, target, useDefaultExternalVariable: true);

                AddTargetCommands(code, programTarget, runBefore: true);

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)target;
                    var targetValue = RapidFormatting.JointTargetValue(_system, jointTarget, group, useDefaultExternalVariable: true);
                    code.Add($"MoveAbsJ {targetValue}{id},{target.Speed.Name},{zone},{target.Tool.Name};");
                }
                else
                {
                    var cartesian = (CartesianTarget)target;
                    var plane = cartesian.Plane;
                    var quaternion = plane.ToQuaternion();
                    var pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
                    var orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                var cf1 = (int)Floor(programTarget.Kinematics.Joints[0] / (PI / 2));
                                var cf4 = (int)Floor(programTarget.Kinematics.Joints[3] / (PI / 2));
                                var cf6 = (int)Floor(programTarget.Kinematics.Joints[5] / (PI / 2));

                                if (cf1 < 0) cf1--;
                                if (cf4 < 0) cf4--;
                                if (cf6 < 0) cf6--;

                                var configuration = programTarget.Kinematics.Configuration;
                                var shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                var elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                if (shoulder) elbow = !elbow;
                                var wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                var cfx = 0;
                                if (wrist) cfx += 1;
                                if (elbow) cfx += 2;
                                if (shoulder) cfx += 4;

                                var conf = $"[{cf1},{cf4},{cf6},{cfx}]";
                                var robtarget = $"[{pos},{orient},{conf},{external}]";
                                code.Add($@"MoveJ {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};");
                                break;
                            }

                        case Motions.Linear:
                            {
                                var robtarget = $"[{pos},{orient},conf,{external}]";

                                if (process is not null && process.HasAt(i))
                                {
                                    var starts = !process.HasAt(i - 1);
                                    var stops = !process.HasAt(i + 1);

                                    if (starts)
                                        code.AddRange(process.SetupCode);

                                    code.Add(process.LinearMove(
                                        robtarget,
                                        id,
                                        target.Speed.Name,
                                        zone,
                                        target.Tool.Name,
                                        target.Frame.Name,
                                        stops));
                                }
                                else
                                {
                                    code.Add($@"MoveL {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};");
                                }

                                break;
                            }

                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }

                AddTargetCommands(code, programTarget, runBefore: false);
            }

            if (!multiProgram)
            {
                if (process is not null)
                    code.Add(process.OffCode);

                if (_system.MechanicalGroups.Count > 1)
                    code.Add("SyncMoveOff sync2;");
            }

            AddErrorHandler(code, process);
            code.Add("ENDPROC");
            code.Add("ENDMODULE");
            return code;
        }

        void AddTargetCommands(List<string> code, ProgramTarget target, bool runBefore)
        {
            foreach (var command in target.Commands.Where(command => command.RunBefore == runBefore))
            {
                if (command is IMotionCommand)
                    continue;

                var commandCode = command.Code(_program, target.Target);

                if (!string.IsNullOrWhiteSpace(commandCode))
                    code.Add(commandCode);
            }
        }

        static void AddErrorHandler(List<string> code, RapidSpeedProcess? process)
        {
            if (process is null)
                return;

            code.Add("ERROR");
            code.Add(process.OffCode);
            code.Add("RAISE;");
        }

        static string Tool(Tool tool)
        {
            var tcp = tool.Tcp;
            var quaternion = tcp.ToQuaternion();
            var weight = tool.Weight > 0.001 ? tool.Weight : 0.001;
            var centroid = tool.Centroid;

            if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                centroid = new(0, 0, 0.001);

            var pos = $"[{tcp.OriginX:0.###},{tcp.OriginY:0.###},{tcp.OriginZ:0.###}]";
            var orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
            var loaddata = $"[{weight:0.###},[{centroid.X:0.###},{centroid.Y:0.###},{centroid.Z:0.###}],[1,0,0,0],0,0,0]";
            return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
        }

        string Frame(Frame frame)
        {
            var plane = frame.Plane;
            plane.InverseOrient(ref _system.BasePlane);
            var quaternion = plane.ToQuaternion();
            var pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
            var orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
            var coupledMech = "";
            var coupledBool = frame.IsCoupled ? "FALSE" : "TRUE";

            if (frame.IsCoupled)
            {
                coupledMech = frame.CoupledMechanism == -1
                    ? $"ROB_{frame.CoupledMechanicalGroup + 1}"
                    : $"STN_{frame.CoupledMechanism + 1}";
            }

            return $@"TASK PERS wobjdata {frame.Name}:=[FALSE,{coupledBool},""{coupledMech}"",[{pos},{orient}],[[0,0,0],[1,0,0,0]]];";
        }

        static string Speed(Speed speed)
        {
            var rotation = speed.RotationSpeed.ToDegrees();
            var rotationExternal = speed.RotationExternal.ToDegrees();
            return $"TASK PERS speeddata {speed.Name}:=[{speed.TranslationSpeed:0.###},{rotation:0.###},{speed.TranslationExternal:0.###},{rotationExternal:0.###}];";
        }

        static string Zone(Zone zone)
        {
            var angle = zone.Rotation.ToDegrees();
            var angleExternal = zone.RotationExternal.ToDegrees();
            return $"TASK PERS zonedata {zone.Name}:=[FALSE,{zone.Distance:0.###},{zone.Distance:0.###},{zone.Distance:0.###},{angle:0.###},{zone.Distance:0.###},{angleExternal:0.###}];";
        }
    }
}
