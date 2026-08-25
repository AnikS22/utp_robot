#!/usr/bin/env python3
"""Conservative occupancy-image cleanup for VISUAL REVIEW, never automatic Nav2 replacement.

Pose errors cannot be repaired in a raster. This removes tiny occupied specks and closes one-cell
gaps only. Unknown cells remain unknown; the script never invents free space.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import cv2, numpy as np

def main():
    ap=argparse.ArgumentParser();ap.add_argument("map_pgm",type=Path);ap.add_argument("--min-component",type=int,default=4)
    a=ap.parse_args();src=cv2.imread(str(a.map_pgm),cv2.IMREAD_GRAYSCALE)
    if src is None:raise SystemExit(f"cannot read {a.map_pgm}")
    occupied=(src<65).astype(np.uint8)
    n,lab,stats,_=cv2.connectedComponentsWithStats(occupied,8)
    kept=np.zeros_like(occupied)
    for i in range(1,n):
        if stats[i,cv2.CC_STAT_AREA]>=a.min_component:kept[lab==i]=1
    closed=cv2.morphologyEx(kept,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8),iterations=1)
    out=src.copy();out[occupied.astype(bool)]=254;out[closed.astype(bool)]=0
    stem=a.map_pgm.with_suffix("");dst=Path(str(stem)+"_denoise_visual_only.pgm")
    cv2.imwrite(str(dst),out);cv2.imwrite(str(dst.with_suffix('.png')),out)
    print(dst);print(f"occupied pixels {occupied.sum()} -> {closed.sum()}; visual review only")
if __name__=="__main__":main()
