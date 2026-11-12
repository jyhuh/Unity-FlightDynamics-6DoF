// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.Collections.Generic;
using UnityEngine;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Procedural 3D Mesh Hierarchy Generator for <see cref="FlightDynamicsVehicle"/> and reference test range environment.
    /// Constructs accurate 3D visual representations directly from <see cref="AircraftConfigSpec"/> (Fuselage, Arms, Motors,
    /// Spinning 2-Blade Rotors, Main Wing, Horizontal Elevator, Vertical Fin, Landing Skids, and Helipad Ground Grid).
    /// </summary>
    public static class ProceduralAircraftVisualBuilder
    {
        public static void RebuildVehicleVisuals(FlightDynamicsVehicle vehicle)
        {
            if (vehicle == null) return;

            // Remove old visual root if present
            Transform? oldRoot = vehicle.transform.Find("VisualAirframeRoot");
            if (oldRoot != null)
            {
                if (Application.isPlaying)
                    Object.Destroy(oldRoot.gameObject);
                else
                    Object.DestroyImmediate(oldRoot.gameObject);
            }

            // Ensure root collider exists for Level 0 Rigidbody / scene selection
            BoxCollider? boxCol = vehicle.GetComponent<BoxCollider>();
            if (boxCol == null)
                boxCol = vehicle.gameObject.AddComponent<BoxCollider>();

            var cfg = vehicle.ActiveConfig;
            boxCol.size = new Vector3(
                Mathf.Max(0.35f, (float)cfg.ReferenceSpanMeters),
                0.18f,
                Mathf.Max(0.35f, (float)cfg.Fuselage.Length));
            boxCol.center = new Vector3(0f, 0.02f, 0f);

            GameObject visualRoot = new GameObject("VisualAirframeRoot");
            visualRoot.transform.SetParent(vehicle.transform, false);

            Material darkCarbonMat = CreateColoredMaterial("CarbonDark", new Color(0.16f, 0.18f, 0.21f));
            Material avionicsMat = CreateColoredMaterial("AvionicsBlue", new Color(0.12f, 0.38f, 0.68f));
            Material noseMarkerMat = CreateColoredMaterial("NoseOrange", new Color(0.95f, 0.36f, 0.10f));
            Material wingMat = CreateColoredMaterial("AeroWingWhite", new Color(0.88f, 0.90f, 0.94f));
            Material frontPropMat = CreateColoredMaterial("FrontPropOrange", new Color(1.0f, 0.45f, 0.12f));
            Material rearPropMat = CreateColoredMaterial("RearPropCyan", new Color(0.15f, 0.80f, 0.92f));

            // 1. Central Fuselage / Avionics Pod
            float fusLen = Mathf.Max(0.24f, (float)cfg.Fuselage.Length);
            float fusDia = Mathf.Max(0.10f, (float)cfg.Fuselage.MaxDiameter);
            CreatePart(PrimitiveType.Cube, "FuselagePod", visualRoot.transform,
                Vector3.zero, Quaternion.identity,
                new Vector3(fusDia, fusDia * 0.55f, fusLen * 0.75f), darkCarbonMat);

            CreatePart(PrimitiveType.Cube, "AvionicsCanopy", visualRoot.transform,
                new Vector3(0f, fusDia * 0.32f, 0.02f), Quaternion.identity,
                new Vector3(fusDia * 0.72f, fusDia * 0.28f, fusLen * 0.45f), avionicsMat);

            // Forward (+Z in Unity = North/Forward) nose indicator cone/bar
            CreatePart(PrimitiveType.Cube, "ForwardNoseIndicator", visualRoot.transform,
                new Vector3(0f, fusDia * 0.20f, fusLen * 0.38f), Quaternion.identity,
                new Vector3(fusDia * 0.45f, fusDia * 0.18f, 0.08f), noseMarkerMat);

            // 2. Landing Gear (Tricycle Wheeled Gear for Fixed-Wing / Skids for Multirotor)
            float gearHeight = Mathf.Max(0.06f, (float)cfg.LandingGearHeightMeters);
            bool isWheeled = cfg.Category == VehicleAirframeCategory.FixedWing ||
                             cfg.Category == VehicleAirframeCategory.HybridQuadPlaneVtol;

            if (isWheeled)
            {
                float wheelRadius = 0.025f;
                float wheelAxleY = -gearHeight + wheelRadius;
                float strutTopY = -fusDia * 0.25f;
                float strutMidY = 0.5f * (strutTopY + wheelAxleY);
                float strutLen = Mathf.Abs(strutTopY - wheelAxleY);

                // A. Nose Gear (Front center)
                float noseZ = fusLen * 0.35f;
                CreatePart(PrimitiveType.Cylinder, "NoseStrut", visualRoot.transform,
                    new Vector3(0f, strutMidY, noseZ), Quaternion.identity,
                    new Vector3(0.012f, strutLen * 0.5f, 0.012f), darkCarbonMat);
                CreatePart(PrimitiveType.Cylinder, "NoseWheel", visualRoot.transform,
                    new Vector3(0f, wheelAxleY, noseZ), Quaternion.Euler(0f, 0f, 90f),
                    new Vector3(wheelRadius * 2f, 0.016f, wheelRadius * 2f), darkCarbonMat);

                // B. Main Landing Gear (Left & Right wheels with wide track width)
                float mainTrackX = 0.18f;
                float mainZ = -0.05f;

                // Left Main Gear
                Vector3 mainLTop = new Vector3(0f, strutTopY, mainZ);
                Vector3 mainLBottom = new Vector3(-mainTrackX, wheelAxleY, mainZ);
                Quaternion rotL = Quaternion.FromToRotation(Vector3.up, (mainLTop - mainLBottom).normalized);
                CreatePart(PrimitiveType.Cylinder, "MainStrut_L", visualRoot.transform,
                    0.5f * (mainLTop + mainLBottom), rotL,
                    new Vector3(0.012f, Vector3.Distance(mainLTop, mainLBottom) * 0.5f, 0.012f), darkCarbonMat);
                CreatePart(PrimitiveType.Cylinder, "MainWheel_L", visualRoot.transform,
                    mainLBottom, Quaternion.Euler(0f, 0f, 90f),
                    new Vector3(wheelRadius * 2f, 0.020f, wheelRadius * 2f), darkCarbonMat);

                // Right Main Gear
                Vector3 mainRTop = new Vector3(0f, strutTopY, mainZ);
                Vector3 mainRBottom = new Vector3(mainTrackX, wheelAxleY, mainZ);
                Quaternion rotR = Quaternion.FromToRotation(Vector3.up, (mainRTop - mainRBottom).normalized);
                CreatePart(PrimitiveType.Cylinder, "MainStrut_R", visualRoot.transform,
                    0.5f * (mainRTop + mainRBottom), rotR,
                    new Vector3(0.012f, Vector3.Distance(mainRTop, mainRBottom) * 0.5f, 0.012f), darkCarbonMat);
                CreatePart(PrimitiveType.Cylinder, "MainWheel_R", visualRoot.transform,
                    mainRBottom, Quaternion.Euler(0f, 0f, 90f),
                    new Vector3(wheelRadius * 2f, 0.020f, wheelRadius * 2f), darkCarbonMat);
            }
            else
            {
                // Multicopter Landing Skids resting exactly at -gearHeight
                float skidSpread = Mathf.Max(0.12f, fusDia * 0.9f);
                float skidBottomY = -gearHeight + 0.008f;
                CreatePart(PrimitiveType.Cube, "LandingSkid_L", visualRoot.transform,
                    new Vector3(-skidSpread, skidBottomY, 0f), Quaternion.identity,
                    new Vector3(0.015f, 0.015f, fusLen * 0.65f), darkCarbonMat);
                CreatePart(PrimitiveType.Cube, "LandingSkid_R", visualRoot.transform,
                    new Vector3(skidSpread, skidBottomY, 0f), Quaternion.identity,
                    new Vector3(0.015f, 0.015f, fusLen * 0.65f), darkCarbonMat);
            }

            // 3. 3D Lifting Surfaces (Main Wing, Horizontal Tail, Vertical Fin)
            if (cfg.Wings != null)
            {
                for (int i = 0; i < cfg.Wings.Count; i++)
                {
                    var w = cfg.Wings[i];
                    Vec3d apexU = FrameConverter.FrdToUnityVector(w.ApexPositionFrd);
                    float meanChord = (float)(0.5 * (w.RootChord + w.TipChord));
                    float span = (float)w.Span;
                    float thickness = Mathf.Max(0.012f, meanChord * (float)w.Airfoil.MaxThicknessRatio);

                    if (w.IsVerticalFin)
                    {
                        Vector3 finCenter = new Vector3((float)apexU.X, (float)apexU.Y + span * 0.5f, (float)apexU.Z - meanChord * 0.5f);
                        CreatePart(PrimitiveType.Cube, $"Wing_{w.Name}", visualRoot.transform,
                            finCenter, Quaternion.identity,
                            new Vector3(thickness, span, meanChord), noseMarkerMat);
                    }
                    else
                    {
                        Vector3 wingCenter = new Vector3((float)apexU.X, (float)apexU.Y, (float)apexU.Z - meanChord * 0.45f);
                        CreatePart(PrimitiveType.Cube, $"Wing_{w.Name}", visualRoot.transform,
                            wingCenter, Quaternion.identity,
                            new Vector3(span, thickness, meanChord), wingMat);
                    }
                }
            }

            // 4. Arms, Motor Mounts & Spinning 2-Blade Rotors
            vehicle.VisualRotorTransforms.Clear();
            for (int i = 0; i < cfg.Rotors.Count; i++)
            {
                var r = cfg.Rotors[i];
                Vec3d mountU = FrameConverter.FrdToUnityVector(r.MountPositionFrd);
                Vector3 mountLocal = new Vector3((float)mountU.X, (float)mountU.Y, (float)mountU.Z);

                // Structural arm from fuselage center to motor mount
                Vector3 armMid = mountLocal * 0.5f;
                float armLen = mountLocal.magnitude;
                if (armLen > 0.04f)
                {
                    Quaternion armRot = Quaternion.LookRotation(mountLocal.normalized, Vector3.up);
                    CreatePart(PrimitiveType.Cube, $"Arm_{r.Name}", visualRoot.transform,
                        armMid, armRot, new Vector3(0.022f, 0.018f, armLen), darkCarbonMat);
                }

                // Motor stator housing
                CreatePart(PrimitiveType.Cylinder, $"MotorBell_{r.Name}", visualRoot.transform,
                    mountLocal, Quaternion.identity, new Vector3(0.042f, 0.020f, 0.042f), avionicsMat);

                // Spinning Rotor Hub + Blades pivot
                GameObject spinnerPivot = new GameObject($"RotorSpinner_{r.Name}");
                spinnerPivot.transform.SetParent(visualRoot.transform, false);
                spinnerPivot.transform.localPosition = mountLocal + new Vector3(0f, 0.022f, 0f);

                // Orient rotor disk perpendicular to thrust axis
                Vec3d thrustU = FrameConverter.FrdToUnityVector(r.ThrustAxisFrd).Normalized();
                Vector3 thrustDir = new Vector3((float)thrustU.X, (float)thrustU.Y, (float)thrustU.Z);
                if (thrustDir.sqrMagnitude > 0.01f)
                    spinnerPivot.transform.localRotation = Quaternion.FromToRotation(Vector3.up, thrustDir.normalized);

                Material propMat = r.MountPositionFrd.X >= 0.0 ? frontPropMat : rearPropMat;
                float diameter = (float)(r.Radius * 2.0);
                float meanBladeChord = r.RadialChordsMeters.Length > 2 ? (float)r.RadialChordsMeters[2] : 0.026f;

                CreatePart(PrimitiveType.Cube, "Blade_Main", spinnerPivot.transform,
                    Vector3.zero, Quaternion.Euler(0f, 0f, 8f * r.RotationDirection),
                    new Vector3(diameter, 0.004f, meanBladeChord), propMat);

                CreatePart(PrimitiveType.Cylinder, "PropHubCap", spinnerPivot.transform,
                    new Vector3(0f, 0.005f, 0f), Quaternion.identity,
                    new Vector3(0.024f, 0.008f, 0.024f), darkCarbonMat);

                vehicle.VisualRotorTransforms.Add(spinnerPivot.transform);
            }
        }

        public static GameObject EnsureGroundEnvironment()
        {
            GameObject? envRoot = GameObject.Find("EngineeringGroundEnvironment");
            if (envRoot != null) return envRoot;

            envRoot = new GameObject("EngineeringGroundEnvironment");

            Material groundMat = CreateColoredMaterial("GroundAsphalt", new Color(0.20f, 0.22f, 0.25f));
            Material padMat = CreateColoredMaterial("HelipadMarker", new Color(0.92f, 0.72f, 0.15f));
            Material northMarkerMat = CreateColoredMaterial("NorthAxisMarker", new Color(0.20f, 0.75f, 0.35f));

            GameObject plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.name = "GroundSurface_500m";
            plane.transform.SetParent(envRoot.transform, false);
            plane.transform.localPosition = Vector3.zero;
            plane.transform.localScale = new Vector3(50f, 1f, 50f);
            var planeRenderer = plane.GetComponent<Renderer>();
            if (planeRenderer != null) planeRenderer.sharedMaterial = groundMat;

            // Helipad H-pad at (0, 0, 0)
            CreatePart(PrimitiveType.Cylinder, "HelipadRing", envRoot.transform,
                new Vector3(0f, 0.005f, 0f), Quaternion.identity, new Vector3(2.2f, 0.005f, 2.2f), padMat);

            // Distance reference pylons along North (+Z) and East (+X) every 10m
            for (int d = 10; d <= 50; d += 10)
            {
                CreatePart(PrimitiveType.Cube, $"NorthMarker_{d}m", envRoot.transform,
                    new Vector3(0f, 0.25f, d), Quaternion.identity, new Vector3(0.3f, 0.5f, 0.3f), northMarkerMat);
                CreatePart(PrimitiveType.Cube, $"EastMarker_{d}m", envRoot.transform,
                    new Vector3(d, 0.25f, 0f), Quaternion.identity, new Vector3(0.3f, 0.5f, 0.3f), padMat);
            }

            return envRoot;
        }

        private static GameObject CreatePart(
            PrimitiveType type,
            string name,
            Transform parent,
            Vector3 localPos,
            Quaternion localRot,
            Vector3 localScale,
            Material mat)
        {
            GameObject go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localRotation = localRot;
            go.transform.localScale = localScale;

            Collider? col = go.GetComponent<Collider>();
            if (col != null)
            {
                if (Application.isPlaying)
                    Object.Destroy(col);
                else
                    Object.DestroyImmediate(col);
            }

            Renderer? r = go.GetComponent<Renderer>();
            if (r != null)
                r.sharedMaterial = mat;

            return go;
        }

        private static Material CreateColoredMaterial(string name, Color color)
        {
            Shader? shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");

            var mat = new Material(shader) { name = name };
            mat.color = color;
            if (mat.HasProperty("_BaseColor"))
                mat.SetColor("_BaseColor", color);
            return mat;
        }
    }
}
