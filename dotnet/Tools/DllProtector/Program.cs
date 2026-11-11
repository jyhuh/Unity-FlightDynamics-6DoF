// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;

namespace FlightDynamics6DoF.Tools.DllProtector
{
    /// <summary>
    /// Pre-compilation source-level symbol obfuscator for private fields and private methods of FlightDynamics6DoF.Core.
    /// Protects proprietary 6-DOF, BEMT, VLM, and GNC internal implementation details when compiling the netstandard2.1 DLL
    /// for Unity Asset Store distribution, while keeping all public classes, structs, methods, and XML docs intact.
    /// </summary>
    public static class Program
    {
        public static int Main(string[] args)
        {
            if (args.Length < 2)
            {
                Console.Error.WriteLine("Usage: DllProtector <inputCoreDir> <outputStagedDir>");
                return 1;
            }

            string inputDir = Path.GetFullPath(args[0]);
            string outputDir = Path.GetFullPath(args[1]);

            if (Directory.Exists(outputDir))
                Directory.Delete(outputDir, true);
            Directory.CreateDirectory(outputDir);

            // Inject TargetFrameworkAttribute for .NET Standard 2.1
            string assemblyInfoPath = Path.Combine(outputDir, "AssemblyAttributes.g.cs");
            File.WriteAllText(assemblyInfoPath,
                "using System.Reflection;\n" +
                "using System.Runtime.Versioning;\n" +
                "[assembly: TargetFramework(\".NETStandard,Version=v2.1\", FrameworkDisplayName = \".NET Standard 2.1\")]\n" +
                "[assembly: AssemblyTitle(\"FlightDynamics6DoF.Core\")]\n" +
                "[assembly: AssemblyProduct(\"FlightDynamics 6-DOF Engineering Suite\")]\n" +
                "[assembly: AssemblyVersion(\"1.0.0.0\")]\n");

            var csFiles = Directory.GetFiles(inputDir, "*.cs", SearchOption.AllDirectories);
            var renameMap = new Dictionary<string, string>(StringComparer.Ordinal);

            var privateMethodRegex = new Regex(@"private\s+(?:static\s+)?(?:readonly\s+)?[A-Za-z0-9_<>,\[\]\?\.]+\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", RegexOptions.Compiled);
            var privateFieldRegex = new Regex(@"private\s+(?:static\s+)?(?:readonly\s+)?[A-Za-z0-9_<>,\[\]\?\.]+\s+(_[A-Za-z0-9_]+)\s*[;=]", RegexOptions.Compiled);

            foreach (string file in csFiles)
            {
                if (file.Contains($"{Path.DirectorySeparatorChar}obj{Path.DirectorySeparatorChar}") ||
                    file.Contains($"{Path.DirectorySeparatorChar}bin{Path.DirectorySeparatorChar}"))
                    continue;

                string src = File.ReadAllText(file);
                foreach (Match m in privateMethodRegex.Matches(src))
                {
                    string id = m.Groups[1].Value;
                    if (!renameMap.ContainsKey(id))
                        renameMap[id] = GenerateObfuscatedIdentifier(id);
                }
                foreach (Match m in privateFieldRegex.Matches(src))
                {
                    string id = m.Groups[1].Value;
                    if (!renameMap.ContainsKey(id))
                        renameMap[id] = GenerateObfuscatedIdentifier(id);
                }
            }

            int fileCount = 0;
            foreach (string file in csFiles)
            {
                if (file.Contains($"{Path.DirectorySeparatorChar}obj{Path.DirectorySeparatorChar}") ||
                    file.Contains($"{Path.DirectorySeparatorChar}bin{Path.DirectorySeparatorChar}"))
                    continue;

                string src = File.ReadAllText(file);
                foreach (var kv in renameMap)
                {
                    src = Regex.Replace(src, $@"\b{Regex.Escape(kv.Key)}\b", kv.Value);
                }

                string outFile = Path.Combine(outputDir, $"Unit_{fileCount++:D2}.cs");
                File.WriteAllText(outFile, "#nullable enable\n" + src);
            }

            Console.WriteLine($"[DllProtector] Obfuscated {renameMap.Count} internal/private symbols across {fileCount} compilation units.");
            return 0;
        }

        private static string GenerateObfuscatedIdentifier(string original)
        {
            using var sha = SHA256.Create();
            byte[] hash = sha.ComputeHash(Encoding.UTF8.GetBytes("FD6DOF_SALT_2026_" + original));
            return "_0x" + BitConverter.ToString(hash, 0, 5).Replace("-", "");
        }
    }
}
