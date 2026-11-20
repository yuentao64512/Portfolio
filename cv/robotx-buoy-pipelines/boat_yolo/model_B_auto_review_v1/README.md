# Model B auto-label review package

This package is a conservative first pass over the 249 buoy crops.

- 143 images were visually identified as numbered buoys.
- 115 images received automatic boxes.
- 186 signs received paired boxes: one `diamond` plus one `digit_*` box.
- A/B/C/D letter buoys and obvious false crops were left empty.
- 28 numbered images could not be boxed reliably and remain for manual review.

## Contents

- `labels/`: YOLO text labels. Every accepted sign is represented by two lines
  with identical geometry: class 1 (`diamond`) and the corresponding digit
  class.
- `xanylabeling_json/`: the same labels in X-AnyLabeling JSON format.
- `previews/`: every source crop with accepted boxes drawn in green.
- `manifest.csv`: accepted and missed candidates, including image index and
  review status.
- `summary.json`: counts and class names.

## Class IDs

0 `circle`, 1 `diamond`, 2 `digit_0`, 3 `digit_1`, 4 `digit_2`,
5 `digit_3`, 6 `digit_4`, 7 `digit_5`, 8 `digit_6`, 9 `digit_7`,
10 `digit_8`, 11 `digit_9`.

All generated labels are marked review-required. Review the green previews and
the 28 `mapped_number_but_no_box_review_required` rows before copying labels
into the final Model B training dataset.
