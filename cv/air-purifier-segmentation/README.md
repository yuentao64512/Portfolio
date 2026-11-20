# One-class air-purifier segmentation

This project collects visually diverse air-purifier images, creates foreground
segmentation masks using open-vocabulary YOLOE, and trains `yolo11n-seg` with a
single `air_purifier` class.

The completed dataset has 778 positive images plus 139 hard-negative images
(fans, humidifiers, heaters, speakers, bins, computer towers, and empty rooms).
The negatives teach the model that a visually similar appliance is not
automatically an air purifier. The stable copy of the final checkpoint is:

`air_purifier_yolo11n_seg.pt`

Downloaded images are kept locally with their source URLs in
`raw/sources.jsonl`. Search-result licensing is not automatically known. Review
rights before redistributing the images or using them commercially.

## Pipeline

```bash
./run_pipeline.sh
```

Individual stages:

```bash
/home/robotx/yolo11env/bin/python collect_images.py
/home/robotx/yolo11env/bin/python auto_annotate.py
/home/robotx/yolo11env/bin/python collect_images.py --queries negative_queries.txt --output raw_negative
/home/robotx/yolo11env/bin/python add_negatives.py
/home/robotx/yolo11env/bin/python train_yolo11_seg.py
```

Always inspect `dataset/review/` before relying on an automatically generated
dataset. Remove bad image/label pairs and retrain if masks contain phones,
furniture, packaging, or only part of a purifier.

Run inference after training:

```bash
/home/robotx/yolo11env/bin/python predict.py image.jpg
```

Use `--cutouts` to save each detected purifier as a transparent PNG, or use
`--show` to open the annotated output in a window:

```bash
/home/robotx/yolo11env/bin/python predict.py image.jpg --cutouts
```

Inference defaults to confidence `0.80`. On this small held-out split it gave
98.1% mask precision, 70.3% mask recall, and no detections in the 10 negative
test images. Lower `--confidence` if missed purifiers matter more than false
alarms; raise it if false alarms are more costly.

The automatic labels should still be manually reviewed before production or
commercial use. "Background ignored" here means that the class is trained from
object masks instead of treating the whole rectangular image as the purifier.
