"""Patch Gazebo Fuel 'Depot' model to add mesh collisions.

This repository's `worlds/warehouse.sdf` includes the Fuel model:
  https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot

Some versions of the Depot model ship with only a ground plane collision.
That makes the building (walls/roof) non-collidable. This script patches the
cached Fuel model.sdf in-place by adding mesh-based collision elements.

Usage:
  patch_depot_collision
  patch_depot_collision --dry-run
  patch_depot_collision --path /home/user/.gz/fuel/.../models/depot/6/model.sdf
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as ET


def _find_latest_depot_model_sdf() -> Optional[Path]:
    """Find the latest cached Fuel Depot model.sdf under ~/.gz/fuel."""
    base = Path.home() / ".gz" / "fuel"
    if not base.exists():
        return None

    # Known layouts:
    # ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/depot/<ver>/model.sdf
    # ~/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/depot/<ver>/model.sdf
    candidates = list(base.glob("**/models/depot/*/model.sdf"))
    if not candidates:
        return None

    def version_key(p: Path) -> int:
        try:
            return int(p.parent.name)
        except ValueError:
            return -1

    candidates.sort(key=version_key, reverse=True)
    return candidates[0]


def _has_mesh_collision(link: ET.Element, mesh_uri: str) -> bool:
    for coll in link.findall("collision"):
        geom = coll.find("geometry")
        if geom is None:
            continue
        mesh = geom.find("mesh")
        if mesh is None:
            continue
        uri = mesh.findtext("uri")
        if (uri or "").strip() == mesh_uri:
            return True
    return False


def _add_mesh_collision(
    link: ET.Element,
    *,
    name: str,
    mesh_uri: str,
    scale_xyz: str,
) -> None:
    coll = ET.Element("collision", {"name": name})
    geom = ET.SubElement(coll, "geometry")
    mesh = ET.SubElement(geom, "mesh")
    scale = ET.SubElement(mesh, "scale")
    scale.text = scale_xyz
    uri = ET.SubElement(mesh, "uri")
    uri.text = mesh_uri

    # Put collision near the top (after any existing collisions, before visuals).
    visuals = link.findall("visual")
    if visuals:
        idx = list(link).index(visuals[0])
        link.insert(idx, coll)
    else:
        link.append(coll)


def patch_depot_model_sdf(path: Path, *, dry_run: bool) -> bool:
    """Patch model.sdf at `path`. Returns True if modifications were made."""
    tree = ET.parse(path)
    root = tree.getroot()

    model = root.find("model")
    if model is None:
        raise RuntimeError("Invalid SDF: <model> not found")

    main_link = None
    for link in model.findall("link"):
        if link.get("name") == "main":
            main_link = link
            break
    if main_link is None:
        raise RuntimeError("Depot model: <link name='main'> not found")

    changed = False

    # Depot building mesh collision (walls/roof/floor etc)
    if not _has_mesh_collision(main_link, "meshes/Depot.dae"):
        _add_mesh_collision(
            main_link,
            name="depot_mesh_collision",
            mesh_uri="meshes/Depot.dae",
            scale_xyz="0.6 0.6 0.6",
        )
        changed = True

    # Crates mesh collision (optional, but usually desirable)
    if not _has_mesh_collision(main_link, "meshes/Crates.dae"):
        _add_mesh_collision(
            main_link,
            name="crates_mesh_collision",
            mesh_uri="meshes/Crates.dae",
            scale_xyz="0.6 0.6 0.6",
        )
        changed = True

    if not changed:
        return False

    if dry_run:
        return True

    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy2(path, backup)

    # Pretty-print for readability (Python 3.9+).
    try:
        ET.indent(tree, space="  ", level=0)
    except Exception:
        # If indent isn't available, still write valid XML.
        pass
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Patch cached Gazebo Fuel 'Depot' model.sdf to add mesh collisions."
    )
    parser.add_argument(
        "--path",
        type=Path,
        default=None,
        help="Explicit path to Depot model.sdf (otherwise auto-detect from ~/.gz/fuel).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't write files; only report whether changes would be made.",
    )
    parser.add_argument(
        "--make-static",
        action="store_true",
        help="Set <static>true</static> on the Depot model (often needed for mesh collisions to be reliable).",
    )
    parser.add_argument(
        "--format-only",
        action="store_true",
        help="Rewrite (pretty-print) model.sdf even if no structural changes are needed.",
    )
    args = parser.parse_args()

    target = args.path or _find_latest_depot_model_sdf()
    if target is None or not target.exists():
        raise SystemExit(
            "DepotのFuelキャッシュが見つかりませんでした。先に world で Depot を一度 include してダウンロードしてください。"
        )

    # Optional: make model static (environment collision).
    if not args.dry_run and args.make_static:
        tree = ET.parse(target)
        root = tree.getroot()
        model = root.find("model")
        static_el = None
        if model is not None:
            static_el = model.find("static")
        if static_el is None and model is not None:
            static_el = ET.SubElement(model, "static")
        if static_el is not None:
            static_el.text = "true"
            backup = target.with_suffix(target.suffix + ".bak_static")
            shutil.copy2(target, backup)
            try:
                ET.indent(tree, space="  ", level=0)
            except Exception:
                pass
            tree.write(target, encoding="utf-8", xml_declaration=True)

    changed = patch_depot_model_sdf(target, dry_run=args.dry_run)
    if args.dry_run:
        print(f"[dry-run] target={target} changed={changed}")
        return

    if changed:
        print(f"Patched: {target} (backup: {target}.bak)")
    else:
        print(f"No change needed: {target}")

    if args.format_only and not args.dry_run:
        tree = ET.parse(target)
        try:
            ET.indent(tree, space="  ", level=0)
        except Exception:
            pass
        tree.write(target, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()


