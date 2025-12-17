---
title: "Ollama LLM + Piper TTS Voice Pipeline"
date: "2025-11-10"
summary: "Working setup for Piper TTS, ROS2 TTS publisher, and Ollama (Phi-3) to run a full voice pipeline on the Isaac ROS dev container."
writer: "Farhin Mansuri"
keywords: "LLM, TTS, Piper, ROS2, Ollama, Jetson"
---

This guide captures the working setup used to integrate:

- **Piper TTS**
- **ROS2 TTS Publisher Node**
- **OllamaResponder LLM Node (Phi-3 model)**
- End-to-end voice flow: `/llm_prompt` → LLM → `/speak_text` → Piper → audio

All commands run inside the Isaac ROS development container unless noted.

---

## 1) Confirm container

Expected container name:

```
isaac_ros_dev-aarch64-custom-container
```

Check running containers:

```bash
docker ps
```

Look for `isaac_ros_dev-aarch64-custom-container`.

---

## 2) Install Piper TTS (inside container)

Enter the container shell:

```bash
docker exec -it isaac_ros_dev-aarch64-custom-container bash
```

Create directories:

```bash
mkdir -p /opt/piper
mkdir -p /models/piper
```

Install Piper ARM64 binary:

```bash
cd /opt
wget https://github.com/rhasspy/piper/releases/download/v0.0.2/piper_arm64.tar.gz
tar -xzf piper_arm64.tar.gz
```

Verify:

```bash
ls /opt/piper
# expect: espeak-ng-data, libespeak-*.so, libonnxruntime.so.1.14.1, piper
```

Test CLI:

```bash
/opt/piper/piper --help
```

---

## 3) Install Piper voice model (Lessac)

```bash
cd /models/piper
pwd  # expect /models/piper

wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json

ls -l  # expect the .onnx and .json
```

---

## 4) Test Piper manually (inside container)

```bash
echo "This is a test of Piper inside the container" | /opt/piper/piper --model /models/piper/en_US-lessac-medium.onnx --output_file /tmp/tts.wav
aplay /tmp/tts.wav
```

You should hear synthesized speech.

---

## 5) Update ROS2 TTS publisher (`tts_publisher.py`)

Path:

```
/workspaces/isaac_ros-dev/src/speech_processor/speech_processor/tts_publisher.py
```

Ensure the node publishes to your expected topic (e.g., `/speak_text`).

---

## 6) Add executables to `setup.py`

File:

```
/workspaces/isaac_ros-dev/src/speech_processor/setup.py
```

Ensure:

```python
entry_points={
    "console_scripts": [
        "tts_publisher = speech_processor.tts_publisher:main",
        "llm_responder = speech_processor.llm_responder:main",
    ],
}
```

Then rebuild/install your ROS package.

---

## 7) Piper TTS node (CLI quick test)

```bash
printf "Hello Anteaters!" | /opt/piper/piper --model /models/piper/en_US-lessac-medium.onnx --output_file /tmp/tts.wav && aplay /tmp/tts.wav
```

---

## 8) Full voice pipeline validation

1. Ensure Piper model is downloaded and working (steps above).
2. Ensure `tts_publisher` and `llm_responder` are installed via `setup.py`.
3. Launch the nodes (example):

```bash
ros2 run speech_processor tts_publisher
ros2 run speech_processor llm_responder
```

4. Publish a prompt to the LLM topic (adjust topic/name as used in your nodes):

```bash
ros2 topic pub --once /llm_prompt std_msgs/msg/String "data: 'Say hello from the rover'"
```

5. Verify the chain:
   - LLM responds (Phi-3 via Ollama)
   - `tts_publisher` receives text on `/speak_text`
   - Piper generates audio and `aplay` plays it

---

## Notes & expectations

- Container name: `isaac_ros_dev-aarch64-custom-container`
- Model: `en_US-lessac-medium.onnx`
- Executables exposed via `setup.py`: `tts_publisher`, `llm_responder`
- End-to-end path: `/llm_prompt` → LLM → `/speak_text` → Piper → audio

