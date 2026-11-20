#!/usr/bin/env bash
set -euo pipefail
root=/home/robotx/box_size_training
font=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
ffmpeg -y -hide_banner -loglevel error \
  -ss 46 -i "$root/source_videos/WhatsApp Video 2026-11-05 at 11.19.23 PM.mp4" \
  -ss 47 -i "$root/source_videos/WhatsApp Video 2026-11-05 at 11.31.38 PM.mp4" \
  -ss 68 -i "$root/source_videos/WhatsApp Video 2026-11-05 at 11.33.52 PM.mp4" \
  -filter_complex "[0:v]fps=30,scale=640:360,pad=640:720:0:180:black,drawtext=fontfile=$font:text=SMALL:x=20:y=20:fontsize=38:fontcolor=white[v0];[1:v]fps=30,scale=402:720,pad=640:720:119:0:black,drawtext=fontfile=$font:text=MEDIUM:x=20:y=20:fontsize=38:fontcolor=white[v1];[2:v]fps=30,scale=402:720,pad=640:720:119:0:black,drawtext=fontfile=$font:text=LARGE:x=20:y=20:fontsize=38:fontcolor=white[v2];[v0][v1][v2]hstack=inputs=3[v]" \
  -map '[v]' -t 7.5 -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  -movflags +faststart "$root/three_boxes_holdout_composite.mp4"
ls -lh "$root/three_boxes_holdout_composite.mp4"
