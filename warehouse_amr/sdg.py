"""Lesson 5: synthetic data generation with Replicator.

The industrial motivation: before an AMR can pick up a pallet it has to *see*
one, and training that detector needs thousands of labelled images of pallets
under every lighting condition and viewing angle a warehouse throws at it.
Photographing and hand-labelling those is weeks of work. Rendering them is an
afternoon -- and the labels come out perfect, for free, because the simulator
already knows where every pallet is.

The three ideas that make Replicator different from "just take screenshots":

* **Semantics.** ``add_labels`` (Isaac Sim 5.x; it replaced the older
  ``SemanticsAPI``) tags a prim as a ``pallet``. Annotators then derive
  bounding boxes and segmentation masks from the scene graph, not from
  image processing, so they are exact.
* **Randomizers.** Registered once inside a ``rep.trigger.on_frame()`` block
  and re-rolled every captured frame. This is domain randomisation: vary
  lighting, pose and clutter widely enough that reality looks like just
  another sample from the training distribution.
* **The orchestrator.** ``rep.orchestrator.step()`` advances one *capture*,
  which is not the same as one physics step -- it re-rolls randomisers,
  renders subframes until the image settles, and hands the result to writers.
"""

from __future__ import annotations

import omni.replicator.core as rep  # type: ignore

from .scene.warehouse import PALLET_LABEL


def _pallet_prims():
    """Select every prim labelled as a pallet, by semantics rather than path."""
    return rep.get.prims(semantics=[("class", PALLET_LABEL)])


def setup_randomizers(camera) -> None:
    """Register the per-frame randomisations.

    Everything inside ``on_frame`` is a *description* evaluated at capture
    time, not code that runs now -- the first Replicator gotcha.
    """
    pallets = _pallet_prims()

    with rep.trigger.on_frame():
        # --- lighting -------------------------------------------------------
        # Warehouses run anything from dim sodium to bright LED, and shadow
        # direction is the single biggest nuisance variable for a detector.
        rep.create.light(
            light_type="Sphere",
            temperature=rep.distribution.normal(5500, 900),
            intensity=rep.distribution.normal(35000, 12000),
            position=rep.distribution.uniform((-9, -5, 3), (9, 5, 6)),
            scale=rep.distribution.uniform(0.5, 1.5),
            count=3,
        )

        # --- pallet placement ----------------------------------------------
        # Re-scattering the pallets each frame gives varied layouts, occlusion
        # and inter-object overlap for free.
        with pallets:
            rep.modify.pose(
                position=rep.distribution.uniform((-8.0, -4.0, 0.06), (8.0, 4.0, 0.06)),
                rotation=rep.distribution.uniform((0, 0, -180), (0, 0, 180)),
            )

        # --- camera ---------------------------------------------------------
        # Roughly eye-to-forklift height, always aimed at the pallets, so the
        # subject is in frame while everything around it changes. The y range
        # stops short of the racks at +/-4.4 m: park the camera inside a rack
        # and the frame is one solid wall of shelving, which is a wasted
        # sample rather than a useful hard one.
        with camera:
            rep.modify.pose(
                position=rep.distribution.uniform((-8.5, -3.0, 0.8), (8.5, 3.0, 3.0)),
                look_at=pallets,
            )


def build_dataset(cfg) -> str:
    """Render ``cfg.sdg_num_frames`` labelled frames. Returns the output dir."""
    out_dir = str(cfg.sdg_output_dir)

    # Capture-on-play would fire the writers whenever the timeline runs; we
    # want captures only when we explicitly ask for them.
    rep.orchestrator.set_capture_on_play(False)

    camera = rep.create.camera(name="SdgCamera", focal_length=20.0)
    render_product = rep.create.render_product(camera, cfg.sdg_resolution, name="SdgView")

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=out_dir,
        rgb=True,
        bounding_box_2d_tight=True,     # the detector's training target
        semantic_segmentation=True,     # per-pixel masks, useful for grasping
        distance_to_image_plane=True,   # depth, for 3D reasoning
        camera_params=True,             # intrinsics/extrinsics per frame
        colorize_semantic_segmentation=True,
        semantic_types=["class"],
    )
    writer.attach([render_product])

    setup_randomizers(camera)

    print("[sdg] writing %d frames to %s" % (cfg.sdg_num_frames, out_dir))
    for i in range(cfg.sdg_num_frames):
        # rt_subframes renders each capture several times before saving, which
        # gives textures and denoising time to converge. Too low and the first
        # frames come out blurry or with unloaded materials.
        rep.orchestrator.step(rt_subframes=cfg.sdg_subframes)
        if (i + 1) % 10 == 0:
            print("[sdg]   %d/%d" % (i + 1, cfg.sdg_num_frames))

    # Writing is asynchronous; without this the process can exit mid-flush.
    rep.orchestrator.wait_until_complete()

    writer.detach()
    render_product.destroy()
    print("[sdg] done -> %s" % out_dir)
    return out_dir


def summarize_dataset(out_dir: str) -> None:
    """Print what actually landed on disk, so the lesson ends with evidence."""
    from pathlib import Path

    root = Path(out_dir)
    if not root.exists():
        print("[sdg] nothing written to %s" % out_dir)
        return
    by_kind = {}
    for p in root.rglob("*"):
        if p.is_file():
            kind = p.parent.name if p.parent != root else p.suffix
            by_kind[kind] = by_kind.get(kind, 0) + 1
    print("\n[sdg] dataset contents (%s):" % root)
    for kind in sorted(by_kind):
        print("   %-34s %d files" % (kind, by_kind[kind]))
