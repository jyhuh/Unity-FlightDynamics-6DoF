// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Shared configuration and disk retention policy for FlightLogs (CSV + HTML reports).
    /// Prevents unbounded memory growth during long flights and caps disk usage in Assets/FlightLogs/.
    /// </summary>
    [Serializable]
    public struct FlightLogRetentionSettings
    {
        public const int DefaultMaxFramesInMemory = 30000;
        public const int DefaultMaxLogFilesOnDisk = 30;
        public const float DefaultMaxTotalLogSizeMb = 200f;
        public const bool DefaultAutoPruneOldLogs = true;

        private const string KeyMaxFrames = "FD6DoF_Log_MaxFramesInMemory";
        private const string KeyMaxFiles = "FD6DoF_Log_MaxFilesOnDisk";
        private const string KeyMaxSizeMb = "FD6DoF_Log_MaxTotalSizeMb";
        private const string KeyAutoPrune = "FD6DoF_Log_AutoPruneOldLogs";

        public int MaxFramesInMemory;
        public int MaxLogFilesOnDisk;
        public float MaxTotalLogSizeMb;
        public bool AutoPruneOldLogs;

        public static FlightLogRetentionSettings Load()
        {
#if UNITY_EDITOR
            return new FlightLogRetentionSettings
            {
                MaxFramesInMemory = Mathf.Clamp(EditorPrefs.GetInt(KeyMaxFrames, DefaultMaxFramesInMemory), 1000, 200000),
                MaxLogFilesOnDisk = Mathf.Clamp(EditorPrefs.GetInt(KeyMaxFiles, DefaultMaxLogFilesOnDisk), 1, 500),
                MaxTotalLogSizeMb = Mathf.Clamp(EditorPrefs.GetFloat(KeyMaxSizeMb, DefaultMaxTotalLogSizeMb), 5f, 4096f),
                AutoPruneOldLogs = EditorPrefs.GetBool(KeyAutoPrune, DefaultAutoPruneOldLogs)
            };
#else
            return new FlightLogRetentionSettings
            {
                MaxFramesInMemory = DefaultMaxFramesInMemory,
                MaxLogFilesOnDisk = DefaultMaxLogFilesOnDisk,
                MaxTotalLogSizeMb = DefaultMaxTotalLogSizeMb,
                AutoPruneOldLogs = DefaultAutoPruneOldLogs
            };
#endif
        }

        public void Save()
        {
            MaxFramesInMemory = Mathf.Clamp(MaxFramesInMemory, 1000, 200000);
            MaxLogFilesOnDisk = Mathf.Clamp(MaxLogFilesOnDisk, 1, 500);
            MaxTotalLogSizeMb = Mathf.Clamp(MaxTotalLogSizeMb, 5f, 4096f);

#if UNITY_EDITOR
            EditorPrefs.SetInt(KeyMaxFrames, MaxFramesInMemory);
            EditorPrefs.SetInt(KeyMaxFiles, MaxLogFilesOnDisk);
            EditorPrefs.SetFloat(KeyMaxSizeMb, MaxTotalLogSizeMb);
            EditorPrefs.SetBool(KeyAutoPrune, AutoPruneOldLogs);
#endif
        }

        public static string GetDefaultLogDirectory()
        {
            return Path.Combine(Application.dataPath, "FlightLogs");
        }

        public static (int csvCount, int totalFileCount, double totalSizeMb) MeasureDirectoryUsage(string logDirectory)
        {
            if (string.IsNullOrEmpty(logDirectory) || !Directory.Exists(logDirectory))
                return (0, 0, 0.0);

            var allFiles = Directory.GetFiles(logDirectory, "*", SearchOption.TopDirectoryOnly)
                .Where(f => !f.EndsWith(".meta", StringComparison.OrdinalIgnoreCase))
                .ToArray();

            int csvCount = allFiles.Count(f => f.EndsWith(".csv", StringComparison.OrdinalIgnoreCase));
            long bytes = 0;
            for (int i = 0; i < allFiles.Length; i++)
            {
                try { bytes += new FileInfo(allFiles[i]).Length; } catch { }
            }

            return (csvCount, allFiles.Length, bytes / (1024.0 * 1024.0));
        }

        /// <summary>
        /// Deletes the oldest CSV/HTML log pairs when file count or total disk size exceeds the configured limits.
        /// Returns the number of deleted primary CSV sessions.
        /// </summary>
        public int ApplyRetentionPolicy(string logDirectory)
        {
            if (!AutoPruneOldLogs || string.IsNullOrEmpty(logDirectory) || !Directory.Exists(logDirectory))
                return 0;

            var csvFiles = Directory.GetFiles(logDirectory, "*.csv", SearchOption.TopDirectoryOnly)
                .Select(p => new FileInfo(p))
                .OrderBy(fi => fi.LastWriteTimeUtc)
                .ToList();

            int deletedSessions = 0;
            double maxBytes = MaxTotalLogSizeMb * 1024.0 * 1024.0;

            long ComputeTotalBytes()
            {
                long sum = 0;
                foreach (string f in Directory.GetFiles(logDirectory, "*", SearchOption.TopDirectoryOnly))
                {
                    if (f.EndsWith(".meta", StringComparison.OrdinalIgnoreCase)) continue;
                    try { sum += new FileInfo(f).Length; } catch { }
                }
                return sum;
            }

            while (csvFiles.Count > 0 && (csvFiles.Count > MaxLogFilesOnDisk || ComputeTotalBytes() > maxBytes))
            {
                var oldest = csvFiles[0];
                csvFiles.RemoveAt(0);
                DeleteLogSessionFiles(oldest.FullName);
                deletedSessions++;
            }

            return deletedSessions;
        }

        private static void DeleteLogSessionFiles(string csvPath)
        {
            try
            {
                if (File.Exists(csvPath)) File.Delete(csvPath);
                if (File.Exists(csvPath + ".meta")) File.Delete(csvPath + ".meta");

                string? dir = Path.GetDirectoryName(csvPath);
                string baseName = Path.GetFileNameWithoutExtension(csvPath);
                if (!string.IsNullOrEmpty(dir))
                {
                    // Matching HTML report naming patterns:
                    // flight_YYYYMMDD_HHMMSS.csv -> flight_report_YYYYMMDD_HHMMSS.html
                    // manual_YYYYMMDD_HHMMSS.csv -> manual_report_YYYYMMDD_HHMMSS.html
                    var candidates = new List<string>
                    {
                        Path.Combine(dir, baseName + ".html"),
                        Path.Combine(dir, baseName.Replace("flight_", "flight_report_") + ".html"),
                        Path.Combine(dir, baseName.Replace("manual_", "manual_report_") + ".html")
                    };

                    foreach (string htmlPath in candidates)
                    {
                        if (File.Exists(htmlPath)) File.Delete(htmlPath);
                        if (File.Exists(htmlPath + ".meta")) File.Delete(htmlPath + ".meta");
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Failed to prune old log '{csvPath}': {ex.Message}");
            }
        }
    }
}
