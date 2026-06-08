#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO


def fourcc(code: str) -> int:
    padded = (code + "    ")[:4]
    return cv2.VideoWriter_fourcc(*padded)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run YOLO on Gemini335 RGB frames.")
    parser.add_argument("--device", default="/dev/video6")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="YUYV")
    parser.add_argument("--model", required=True)
    parser.add_argument("--task", default="segment")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument(
        "--classes",
        default="",
        help="Optional comma-separated class names or ids, e.g. trash or 0.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=12.0,
        help="Run duration in seconds. Use 0 or a negative value for live mode.",
    )
    parser.add_argument("--every", type=int, default=3, help="Run inference every N captured frames.")
    parser.add_argument("--device-id", default="0", help="Ultralytics device, e.g. 0 or cpu.")
    parser.add_argument("--output-dir", default="yolo_workspace/runs/gemini_yolo")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--ollama-model", default="", help="Optional VLM model for one final-frame review.")
    parser.add_argument(
        "--ollama-prompt",
        default=(
            "请用一句话描述图中是否有可拾取垃圾，并列出最可能的物体类别。"
            "只输出简短中文结论。"
        ),
    )
    return parser.parse_args()


def open_capture(args: argparse.Namespace) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera device: {args.device}")
    cap.set(cv2.CAP_PROP_FOURCC, fourcc(args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    return cap


def summarize_detections(result) -> list[str]:
    names = result.names
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    summary: list[str] = []
    for cls, conf, xyxy in zip(boxes.cls, boxes.conf, boxes.xyxy):
        x1, y1, x2, y2 = [float(v) for v in xyxy]
        label = names.get(int(cls), str(int(cls)))
        summary.append(f"{label}:{float(conf):.2f}@[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
    return summary


def resolve_classes(model: YOLO, spec: str) -> list[int] | None:
    tokens = [token.strip() for token in spec.split(",") if token.strip()]
    if not tokens:
        return None

    names = getattr(model, "names", {}) or {}
    name_to_id = {str(name).lower(): int(idx) for idx, name in names.items()}
    class_ids: list[int] = []
    for token in tokens:
        if token.isdigit():
            class_ids.append(int(token))
            continue
        key = token.lower()
        if key not in name_to_id:
            valid = ", ".join(str(v) for v in names.values())
            raise ValueError(f"unknown class name {token!r}; valid names include: {valid}")
        class_ids.append(name_to_id[key])
    return class_ids


def ask_ollama(model: str, prompt: str, image_path: Path) -> str:
    payload = {
        "model": model,
        "prompt": prompt,
        "images": [base64.b64encode(image_path.read_bytes()).decode("ascii")],
        "stream": False,
    }
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        "http://127.0.0.1:11434/api/generate",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=90) as response:
        body = json.loads(response.read().decode("utf-8"))
    return str(body.get("response", "")).strip()


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_dir)
    run_dir = output_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    model_path = Path(args.model)
    model = YOLO(str(model_path), task=args.task)
    class_ids = resolve_classes(model, args.classes)
    cap = open_capture(args)

    print(f"camera={args.device} {args.width}x{args.height}@{args.fps:g} fourcc={args.fourcc}")
    print(f"model={model_path}")
    if class_ids is not None:
        print(f"classes={args.classes} ids={class_ids}")
    print(f"run_dir={run_dir}")

    window_name = "Arachne Gemini YOLO"
    if args.show:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    frame_count = 0
    infer_count = 0
    last_annotated: Path | None = None
    last_raw: Path | None = None
    started = time.perf_counter()
    inference_ms: list[float] = []

    try:
        while args.duration <= 0.0 or time.perf_counter() - started < args.duration:
            try:
                ok, frame = cap.read()
            except cv2.error as exc:
                print(f"camera_read_error={exc}")
                time.sleep(0.05)
                continue
            if not ok or frame is None:
                print("camera_read_failed")
                time.sleep(0.05)
                continue
            frame_count += 1
            if frame_count % max(args.every, 1) != 0:
                continue

            t0 = time.perf_counter()
            predict_kwargs = {
                "imgsz": args.imgsz,
                "conf": args.conf,
                "device": args.device_id,
                "verbose": False,
            }
            if class_ids is not None:
                predict_kwargs["classes"] = class_ids
            result = model.predict(frame, **predict_kwargs)[0]
            dt_ms = (time.perf_counter() - t0) * 1000.0
            inference_ms.append(dt_ms)
            infer_count += 1

            detections = summarize_detections(result)
            annotated = result.plot()
            cv2.putText(
                annotated,
                f"{dt_ms:.1f} ms  infer #{infer_count}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            last_raw = run_dir / "latest_raw.jpg"
            last_annotated = run_dir / "latest_annotated.jpg"
            cv2.imwrite(str(last_raw), frame)
            cv2.imwrite(str(last_annotated), annotated)

            if detections:
                print(f"infer={infer_count} {dt_ms:.1f}ms detections: " + ", ".join(detections))
            else:
                print(f"infer={infer_count} {dt_ms:.1f}ms detections: none")

            if args.show:
                cv2.imshow(window_name, annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    print("window_stop_requested")
                    break
                try:
                    if cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) < 0:
                        print("window_closed")
                        break
                except cv2.error:
                    print("window_closed")
                    break
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = max(time.perf_counter() - started, 1e-6)
    capture_fps = frame_count / elapsed
    infer_fps = infer_count / elapsed
    avg_ms = sum(inference_ms) / len(inference_ms) if inference_ms else 0.0
    print(f"frames={frame_count} capture_fps={capture_fps:.1f}")
    print(f"inferences={infer_count} infer_fps={infer_fps:.1f} avg_infer_ms={avg_ms:.1f}")
    if last_annotated is not None:
        print(f"latest_annotated={last_annotated}")

    if args.ollama_model and last_annotated is not None:
        print(f"ollama_model={args.ollama_model}")
        try:
            print("ollama_response=" + ask_ollama(args.ollama_model, args.ollama_prompt, last_annotated))
        except Exception as exc:
            print(f"ollama_error={type(exc).__name__}: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
