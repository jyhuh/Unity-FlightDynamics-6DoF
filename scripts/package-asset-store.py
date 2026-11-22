#!/usr/bin/env python3
"""
Builds the Unity Asset Store package (.unitypackage) and UPM tarball (.tgz)
into /Users/byabya/UnityDrone/build/.
Preserves exact Unity .meta GUIDs so Scene and ScriptableObject references remain 100% intact.
"""
import hashlib
import io
import os
import re
import tarfile
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
UPM_SRC_DIR = ROOT_DIR / "Packages" / "com.aerospace.flightdynamics6dof"
OUT_PKG_DIR = ROOT_DIR / "build"
UNITYPACKAGE_PATH = OUT_PKG_DIR / "FlightDynamics6DoF_AssetStore_v1.0.0.unitypackage"
UPM_TGZ_PATH = OUT_PKG_DIR / "com.aerospace.flightdynamics6dof-1.0.0.tgz"

GUID_RE = re.compile(r"guid:\s*([0-9a-fA-F]{32})")


def deterministic_guid(path_str: str) -> str:
    return hashlib.md5(f"FlightDynamics6DoF::{path_str}".encode("utf-8")).hexdigest()


def make_folder_meta(guid: str) -> bytes:
    content = (
        "fileFormatVersion: 2\n"
        f"guid: {guid}\n"
        "folderAsset: yes\n"
        "DefaultImporter:\n"
        "  externalObjects: {}\n"
        "  userData: \n"
        "  assetBundleName: \n"
        "  assetBundleVariant: \n"
    )
    return content.encode("utf-8")


def make_default_file_meta(guid: str) -> bytes:
    content = (
        "fileFormatVersion: 2\n"
        f"guid: {guid}\n"
        "DefaultImporter:\n"
        "  externalObjects: {}\n"
        "  userData: \n"
        "  assetBundleName: \n"
        "  assetBundleVariant: \n"
    )
    return content.encode("utf-8")


def extract_or_create_meta(src_file: Path, logical_path: str) -> tuple[str, bytes]:
    meta_path = Path(str(src_file) + ".meta")
    if meta_path.exists():
        meta_bytes = meta_path.read_bytes()
        m = GUID_RE.search(meta_bytes.decode("utf-8", errors="ignore"))
        if m:
            return m.group(1).lower(), meta_bytes
    guid = deterministic_guid(logical_path)
    return guid, make_default_file_meta(guid)


def add_tar_bytes(tar: tarfile.TarFile, arcname: str, data: bytes) -> None:
    info = tarfile.TarInfo(name=arcname)
    info.size = len(data)
    info.mtime = int(time.time())
    info.mode = 0o644
    tar.addfile(info, io.BytesIO(data))


def build_unitypackage() -> list[tuple[str, str, int]]:
    OUT_PKG_DIR.mkdir(parents=True, exist_ok=True)
    file_mappings: list[tuple[Path, str]] = []

    for root, dirs, files in os.walk(UPM_SRC_DIR):
        dirs[:] = [d for d in sorted(dirs) if not d.startswith(".")]
        for fname in sorted(files):
            if fname.startswith(".") or fname.endswith(".meta"):
                continue
            src_path = Path(root) / fname
            rel_path = src_path.relative_to(UPM_SRC_DIR).as_posix()
            dst_logical = f"Assets/FlightDynamics6DoF/{rel_path}"
            file_mappings.append((src_path, dst_logical))

    demo_scene = ROOT_DIR / "Assets" / "Scenes" / "FlightDynamics6DoF_EngineeringDemo.unity"
    if demo_scene.exists():
        file_mappings.append(
            (demo_scene, "Assets/FlightDynamics6DoF/Demo/Scenes/FlightDynamics6DoF_EngineeringDemo.unity")
        )

    demo_profile = ROOT_DIR / "Assets" / "EngineeringAircraftProfile.asset"
    if demo_profile.exists():
        file_mappings.append(
            (demo_profile, "Assets/FlightDynamics6DoF/Demo/Profiles/EngineeringAircraftProfile.asset")
        )

    folder_paths: set[str] = set()
    for _, dst_logical in file_mappings:
        parts = dst_logical.split("/")
        for i in range(2, len(parts)):
            folder_paths.add("/".join(parts[:i]))

    entries_summary: list[tuple[str, str, int]] = []

    with tarfile.open(UNITYPACKAGE_PATH, "w:gz") as tar:
        for folder in sorted(folder_paths):
            f_guid = deterministic_guid(f"folder::{folder}")
            add_tar_bytes(tar, f"{f_guid}/pathname", folder.encode("utf-8"))
            add_tar_bytes(tar, f"{f_guid}/asset.meta", make_folder_meta(f_guid))

        for src_path, dst_logical in file_mappings:
            guid, meta_bytes = extract_or_create_meta(src_path, dst_logical)
            asset_bytes = src_path.read_bytes()
            add_tar_bytes(tar, f"{guid}/pathname", dst_logical.encode("utf-8"))
            add_tar_bytes(tar, f"{guid}/asset.meta", meta_bytes)
            add_tar_bytes(tar, f"{guid}/asset", asset_bytes)
            entries_summary.append((guid, dst_logical, len(asset_bytes)))

    return entries_summary


def build_upm_tgz() -> None:
    OUT_PKG_DIR.mkdir(parents=True, exist_ok=True)
    with tarfile.open(UPM_TGZ_PATH, "w:gz") as tar:
        for root, dirs, files in os.walk(UPM_SRC_DIR):
            dirs[:] = [d for d in sorted(dirs) if not d.startswith(".")]
            for fname in sorted(files):
                if fname.startswith("."):
                    continue
                src_path = Path(root) / fname
                rel_path = src_path.relative_to(UPM_SRC_DIR).as_posix()
                add_tar_bytes(tar, f"package/{rel_path}", src_path.read_bytes())


if __name__ == "__main__":
    entries = build_unitypackage()
    build_upm_tgz()
    print(f"[Success] Built Asset Store package: {UNITYPACKAGE_PATH} ({UNITYPACKAGE_PATH.stat().st_size:,} bytes, {len(entries)} files)")
    print(f"[Success] Built UPM tarball package: {UPM_TGZ_PATH} ({UPM_TGZ_PATH.stat().st_size:,} bytes)")
