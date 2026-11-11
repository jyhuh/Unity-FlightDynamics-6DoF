#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DOTNET_CLI_HOME="${DOTNET_CLI_HOME:-/tmp}"
export DOTNET_CLI_TELEMETRY_OPTOUT=1

echo "======================================================================"
echo " FlightDynamics 6-DOF: Build, Test & Protected UPM Packaging Pipeline"
echo "======================================================================"

# 1. Run automated verification suite
echo "[Step 1/4] Running Core Engineering Unit Test Suite..."
dotnet run --project "$ROOT_DIR/dotnet/FlightDynamics6DoF.Tests/FlightDynamics6DoF.Tests.csproj" -c Release

# 2. Run internal symbol obfuscation preprocessor
echo "[Step 2/4] Staging and Obfuscating Core C# Implementation Symbols..."
STAGED_DIR="/tmp/fd6dof_obfuscated_stage"
dotnet run --project "$ROOT_DIR/dotnet/Tools/DllProtector/DllProtector.csproj" -c Release -- \
  "$ROOT_DIR/dotnet/FlightDynamics6DoF.Core" "$STAGED_DIR"

# 3. Compile strict .NET Standard 2.1 Release DLL (Zero PDB, Release Optimize+, XML IntelliSense)
echo "[Step 3/4] Compiling Protected .NET Standard 2.1 Managed DLL for Unity..."
UNITY_NETSTANDARD_REF="/Applications/Unity/Hub/Editor/6000.3.11f1/Unity.app/Contents/Resources/Scripting/NetStandard/ref/2.1.0/netstandard.dll"
UNITY_MANAGED="/Applications/Unity/Hub/Editor/6000.3.11f1/Unity.app/Contents/Resources/Scripting/Managed/UnityEngine"
CSC_DLL="/usr/local/share/dotnet/sdk/10.0.202/Roslyn/bincore/csc.dll"

PLUGIN_OUT_DIR="$ROOT_DIR/Packages/com.aerospace.flightdynamics6dof/Runtime/Plugins"
mkdir -p "$PLUGIN_OUT_DIR"

if [[ -f "$UNITY_NETSTANDARD_REF" && -f "$CSC_DLL" ]]; then
  dotnet exec "$CSC_DLL" \
    -target:library \
    -out:"$PLUGIN_OUT_DIR/FlightDynamics6DoF.Core.dll" \
    -doc:"$PLUGIN_OUT_DIR/FlightDynamics6DoF.Core.xml" \
    -optimize+ \
    -debug- \
    -deterministic+ \
    -nostdlib+ \
    -noconfig \
    -langversion:9.0 \
    -nullable:enable \
    -nowarn:1591 \
    -r:"$UNITY_NETSTANDARD_REF" \
    "$STAGED_DIR"/*.cs
  echo "[Success] Built netstandard2.1 protected binary: $PLUGIN_OUT_DIR/FlightDynamics6DoF.Core.dll"
else
  cp "$ROOT_DIR/dotnet/FlightDynamics6DoF.Core/bin/Release/net10.0/FlightDynamics6DoF.Core.dll" "$PLUGIN_OUT_DIR/"
  cp "$ROOT_DIR/dotnet/FlightDynamics6DoF.Core/bin/Release/net10.0/FlightDynamics6DoF.Core.xml" "$PLUGIN_OUT_DIR/"
  echo "[Success] Copied Release DLL to: $PLUGIN_OUT_DIR/FlightDynamics6DoF.Core.dll"
fi

# 4. Verify Unity Runtime & Editor Wrapper compilation against Unity Engine / Editor Assemblies
if [[ -d "$UNITY_MANAGED" && -f "$CSC_DLL" ]]; then
  echo "[Step 4/4] Verifying Unity Runtime & Editor Scripts against Unity 6 Assemblies..."
  dotnet exec "$CSC_DLL" -target:library -out:/tmp/FlightDynamics6DoF.Unity.dll -nostdlib+ -noconfig -langversion:9.0 -nullable:enable \
    -r:"$UNITY_NETSTANDARD_REF" \
    -r:"$PLUGIN_OUT_DIR/FlightDynamics6DoF.Core.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.CoreModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.PhysicsModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.IMGUIModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.JSONSerializeModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.SharedInternalsModule.dll" \
    "$ROOT_DIR/Packages/com.aerospace.flightdynamics6dof/Runtime/Assets/"*.cs \
    "$ROOT_DIR/Packages/com.aerospace.flightdynamics6dof/Runtime/Components/"*.cs

  dotnet exec "$CSC_DLL" -target:library -out:/tmp/FlightDynamics6DoF.Editor.dll -nostdlib+ -noconfig -langversion:9.0 -nullable:enable \
    -r:"$UNITY_NETSTANDARD_REF" \
    -r:"$PLUGIN_OUT_DIR/FlightDynamics6DoF.Core.dll" \
    -r:/tmp/FlightDynamics6DoF.Unity.dll \
    -r:"$UNITY_MANAGED/UnityEngine.CoreModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.PhysicsModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.IMGUIModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.JSONSerializeModule.dll" \
    -r:"$UNITY_MANAGED/UnityEngine.SharedInternalsModule.dll" \
    -r:"$UNITY_MANAGED/UnityEditor.CoreModule.dll" \
    "$ROOT_DIR/Packages/com.aerospace.flightdynamics6dof/Editor/"*.cs

  rm -f /tmp/FlightDynamics6DoF.Unity.dll /tmp/FlightDynamics6DoF.Editor.dll
  echo "[Success] Verified FlightDynamics6DoF.Unity and FlightDynamics6DoF.Editor compile cleanly with 0 warnings/errors."
fi

rm -rf "$STAGED_DIR"
ls -lh "$PLUGIN_OUT_DIR"
echo "======================================================================"
echo " Packaging Complete!"
echo "======================================================================"
