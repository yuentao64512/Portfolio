# Docking CV - AGX Orin

Native AGX deployment of the one-class dock detector.

## Models

- models/dock_partial_best.pt: portable PyTorch source model.
- models/dock_partial_best.onnx: ONNX intermediate used for the native build.
- models/dock_partial_best.engine: AGX-native TensorRT 10.3 FP16 engine, fixed batch 1 and 640x640 input.

TensorRT plans are hardware/runtime specific. Rebuild the engine on the target Jetson after TensorRT, CUDA, JetPack, input-shape, or model changes.

## Run

Camera 0:

    cd ~/docking_cv
    ./run_docking.sh --source 0 --conf 0.5 --show

Video with saved annotations:

    cd ~/docking_cv
    ./run_docking.sh --source /path/to/video.mp4 --conf 0.5 --save --name dock_video

Headless stream:

    cd ~/docking_cv
    ./run_docking.sh --source rtsp://host/stream --conf 0.5

## Verified

The TensorRT engine loaded successfully on this AGX and detected a held-out edge-clipped dock at 0.978 confidence. Warmed-up median model inference was 11.12 ms for TensorRT FP16 versus 29.47 ms for PyTorch on the same 640x640 image.
