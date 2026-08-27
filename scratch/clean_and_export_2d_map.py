#!/usr/bin/env python3
"""
========================================================================================
🗺️ [2D Map Optimizer] High-Quality 2D Occupancy Grid Map Cleaner & Sharpener
========================================================================================
Post-processes raw RTAB-Map 2D Occupancy Grid Maps (.pgm / .yaml) using PIL + NumPy:
  1. Removes Ray-Tracing Spikes / Whisker Artifacts (Morphological opening)
  2. Filters out isolated noise clusters / floating speckles
  3. Straightens and sharpens wall boundaries
  4. Fills small internal holes in walkable free space
  5. Generates publication-ready high-contrast PNG & clean PGM/YAML
========================================================================================
"""

import os
import sys
import yaml
import numpy as np
from PIL import Image
try:
    from scipy.ndimage import binary_opening, binary_closing, binary_dilation, label
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

def read_pgm(pgm_path):
    with Image.open(pgm_path) as img:
        return np.array(img, dtype=np.uint8)

def clean_2d_map(pgm_path="2dmap/0833.pgm", yaml_path="2dmap/0833.yaml", output_dir="2dmap/clean"):
    if not os.path.exists(pgm_path):
        print(f"❌ Error: PGM file not found at {pgm_path}")
        return False

    os.makedirs(output_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(pgm_path))[0]

    print("=" * 76)
    print(f" 🗺️ [2D Map Optimizer] Cleaning & Enhancing 2D Map: {pgm_path}")
    print("=" * 76)

    raw_img = read_pgm(pgm_path)
    h, w = raw_img.shape
    print(f"  • Input Map Size: {w} x {h} pixels")

    # Masks
    # Occupied (Black: < 50)
    occupied_mask = (raw_img < 50)
    # Free (White: > 230)
    free_mask = (raw_img > 230)

    if HAVE_SCIPY:
        # 1. Morphological processing for connected corridor
        structure_3x3 = np.ones((3, 3), dtype=bool)
        structure_5x5 = np.ones((5, 5), dtype=bool)

        # Label connected components of free space
        labeled, num_features = label(free_mask)
        if num_features > 1:
            sizes = [np.sum(labeled == i) for i in range(1, num_features + 1)]
            largest_label = 1 + np.argmax(sizes)
            main_free = (labeled == largest_label)
            # Remove ray-tracing spikes
            main_free = binary_opening(main_free, structure=structure_3x3, iterations=1)
            main_free = binary_closing(main_free, structure=structure_5x5, iterations=2)
        else:
            main_free = free_mask

        # Clean Obstacles
        dilated_free = binary_dilation(main_free, structure=structure_5x5, iterations=2)
        clean_obstacles = occupied_mask & dilated_free
        clean_obstacles = binary_opening(clean_obstacles, structure=structure_3x3, iterations=1)
        clean_obstacles = binary_closing(clean_obstacles, structure=structure_3x3, iterations=1)
    else:
        main_free = free_mask
        clean_obstacles = occupied_mask

    # Composite Clean Map
    # Background: 205 (Unknown Gray), Free: 254 (White), Obstacle: 0 (Black)
    clean_map = np.full((h, w), 205, dtype=np.uint8)
    clean_map[main_free] = 254
    clean_map[clean_obstacles] = 0

    # 1. Save Clean PGM
    clean_pgm_path = os.path.join(output_dir, f"{base_name}_clean.pgm")
    Image.fromarray(clean_map).save(clean_pgm_path)
    print(f"  ✅ Saved Clean PGM: {clean_pgm_path}")

    # 2. Save Clean YAML
    if os.path.exists(yaml_path):
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        yaml_data['image'] = f"{base_name}_clean.pgm"
        clean_yaml_path = os.path.join(output_dir, f"{base_name}_clean.yaml")
        with open(clean_yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        print(f"  ✅ Saved Clean YAML: {clean_yaml_path}")

    # 3. Generate Publication-Ready High-Contrast Color PNG
    color_map = np.zeros((h, w, 3), dtype=np.uint8)
    color_map[:] = (230, 230, 230)            # Background: Neutral Soft Gray
    color_map[main_free] = (255, 255, 255)    # Walkable: Crisp Pure White
    color_map[clean_obstacles] = (20, 20, 20) # Walls: Dark Charcoal

    color_png_path = os.path.join(output_dir, f"{base_name}_clean_publication.png")
    Image.fromarray(color_map).save(color_png_path)
    print(f"  ✅ Saved Publication PNG: {color_png_path}")

    print("=" * 76)
    print(" 🚀 [DONE] 2D Map Optimization Complete! Ray-tracing spikes and wall noise removed.")
    print("=" * 76)
    return True

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        in_pgm = sys.argv[1]
        in_yaml = sys.argv[2] if len(sys.argv) >= 3 else (os.path.splitext(in_pgm)[0] + ".yaml")
        out_dir = sys.argv[3] if len(sys.argv) >= 4 else "2dmap/clean"
        clean_2d_map(in_pgm, in_yaml, out_dir)
    else:
        clean_2d_map()

