"""Lesson 5: generate a labelled pallet-detection dataset with Replicator.

    C:\\isaacsim\\python.bat scripts\\03_generate_dataset.py --frames 40

Runs headless by default -- there is nothing to watch, and the GPU is better
spent rendering. Output lands in ``_output/sdg_dataset``:

    rgb/                      the image a detector trains on
    bounding_box_2d_tight/    .npy boxes + a _labels.json mapping id -> class
    semantic_segmentation/    per-pixel masks
    distance_to_image_plane/  depth in metres
    camera_params/            intrinsics and pose per frame

Note this script never touches ROS 2. Data generation is an offline, render-
bound job; the ROS 2 bridge is an online, physics-bound one. Keeping them in
separate processes is how these pipelines are run in practice.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:  # pragma: no cover
    pass

from warehouse_amr import app                          # noqa: E402
from warehouse_amr.config import describe, load_config  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="Replicator synthetic data generation")
    p.add_argument("--frames", type=int, default=None, help="override SDG_NUM_FRAMES")
    p.add_argument("--gui", action="store_true", help="show the window while rendering")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = load_config()
    if args.frames is not None:
        cfg.sdg_num_frames = args.frames
    print(describe(cfg))

    simulation_app = app.launch(cfg, headless=not args.gui)
    app.enable_required_extensions(with_ros2=False)
    simulation_app.update()

    import omni.usd  # noqa: E402

    from warehouse_amr.scene.warehouse import build_warehouse  # noqa: E402
    from warehouse_amr.sdg import build_dataset, summarize_dataset  # noqa: E402

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    print("[sdg] building scene ...")
    handles = build_warehouse(stage, cfg)
    simulation_app.update()

    if not handles.pallets:
        print("[sdg] WARNING: no pallets in the scene -- every label will be empty.")
        print("[sdg] check USD_ASSETS_ROOT in .env points at your content packs.")

    out_dir = build_dataset(cfg)
    summarize_dataset(out_dir)

    app.shutdown(simulation_app, 0)   # does not return


if __name__ == "__main__":
    # main() ends in app.shutdown(), which exits the process itself; this is
    # only reached if that ever changes.
    raise SystemExit(main())
