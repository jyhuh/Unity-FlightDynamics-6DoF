#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNITY_BIN="/Applications/Unity/Hub/Editor/6000.3.11f1/Unity.app/Contents/MacOS/Unity"
CMD_FILE="$ROOT_DIR/Temp/fd6dof_cli_cmd.txt"
RES_FILE="$ROOT_DIR/Temp/fd6dof_cli_res.txt"
LOCK_FILE="$ROOT_DIR/Temp/UnityLockfile"

CMD="${*:-status}"

if [[ -f "$LOCK_FILE" ]]; then
  echo "[Unity CLI] Detected running Unity Editor instance on $ROOT_DIR."
  echo "[Unity CLI] Sending live Editor command: '$CMD'..."
  rm -f "$RES_FILE"
  echo "$CMD" > "$CMD_FILE"

  # Wait up to 8 seconds for Unity Editor update loop to process command
  for _ in {1..32}; do
    if [[ -f "$RES_FILE" ]]; then
      cat "$RES_FILE"
      echo ""
      rm -f "$RES_FILE"
      exit 0
    fi
    sleep 0.25
  done

  echo "[Unity CLI] Command staged in $CMD_FILE. Focus the Unity Editor window once so it finishes script compilation and executes '$CMD'."
else
  echo "[Unity CLI] No GUI Editor lockfile detected. Running headless Unity -batchmode CLI..."
  "$UNITY_BIN" \
    -batchmode \
    -nographics \
    -quit \
    -projectPath "$ROOT_DIR" \
    -executeMethod FlightDynamics6DoF.Editor.UnityEditorCliAndSceneBootstrap.RunBatchSetupScene \
    -logFile /tmp/unity_batch_cli.log
  grep "\[FlightDynamics6DoF" /tmp/unity_batch_cli.log || tail -n 20 /tmp/unity_batch_cli.log
fi
