---
title: "Speech Pipeline Overview"
date: "2025-10-21"
summary: "Natural voice interaction system for the autonomous rover enabling escort commands and small talk through wake word detection, speech recognition, and intent classification."
writer: "Selina Shrestha"
keywords: "Speech Recognition, NLP, ROS2, Wake Word, Intent Classification"
---

## **Overview**

This pipeline enables natural voice interaction with the autonomous rover, allowing users to issue escort commands (e.g., "Take me to the science library") as well as engage in small talk. The system continuously listens for a wake word, records voice commands upon detection, transcribes speech to text, classifies intent, and executes the corresponding navigation or dialogue response.

![Speech Pipeline Flowchart](/docs/speech-pipeline/flowchart.png)

_Figure: Speech Pipeline Flowchart_

### Key Components

| Component         | Function                             | Tools/Models              |
| ----------------- | ------------------------------------ | ------------------------- |
| **PyAudio**       | Audio capture and hardware interface | Python audio library      |
| **openWakeWord**  | Lightweight wake-word detection      | `hey_jarvis_v0.1.tflite`  |
| **ASR Module**    | Speech-to-text conversion            | Google speech recognition |
| **DistilBERT**    | Intent classification                | Fine-tuned NLP model      |
| **Sentence-BERT** | Location semantic matching           | `all-MiniLM-L6-v2`        |
| **phi-LLM**       | Dialogue generation                  | Lightweight LLM           |
| **Nav2**          | Navigation control                   | ROS 2 navigation stack    |

## ROS2 Package: speech_processor

This package contains the runtime nodes that implement the speech pipeline: wake-word + ASR, intent classification, and destination matching.

### 1. Node: `wake_audio_processor`

```bash
ros2 run speech_processor wake_audio_processor
```

**Purpose:** Always-on wake-word listener that, on a hit, records a 5 sec audio, transcribes it to text, and publishes the transcript.

**What it does:**

1. Listens for wake word "Hey Jarvis" (WAKE state).
2. Once detected, activates speech recognition for 5 seconds (ASR state). Publishes user's speech input to /speech_text.
3. After that, goes back into listening for wake word (WAKE state).

**Parameters:**

- `device_index` (int, default `0`) – Mic input device.
- `threshold` (float, default `0.35`) – Wake sensitivity.
- `debounce_s` (float, default `1.5`) – Minimum time between wake hits.
- `gain` (float, default `10.0`) – Software gain (clipped safe).
- `log_scores` (bool, default `false`) – Log wake scores.
- `asr_seconds` (int, default `5`) – Post-wake record window.
- `asr_preroll_ms` (int, default `0`) – Optional preroll discard.

**Topics:**

- **Publishes**
  - `/wake_word` — `std_msgs/Empty` (event: wake detected)
  - `/speech_text` — `std_msgs/String` (lower-cased transcript)
  - `/wake_asr/state` — `std_msgs/String` (`"WAKE"` or `"ASR"`)
- **Subscribes**
  - _(none)_

### 2. Node: `intent_classifier`

```bash
ros2 run speech_processor intent_classifier
```

**Purpose:** Classifies the transcribed text into intents using a fine-tuned **DistilBERT** model.

**What it does:**

- Loads tokenizer/model from a local path.
- For every transcript, predicts an intent label (e.g., `Escort`, `SmallTalk`, `Stop`).
- **Current behavior:** forwards only **Escort** intents downstream (others are logged; hook points for small-talk/stop handlers).

**Topics:**

- **Subscribes**
  - `/speech_text` — `std_msgs/String` (ASR transcript)
- **Publishes**
  - `/escort_command` — `std_msgs/String` (original text when intent == `Escort`)

### 3. Node: `location_matcher`

```bash
ros2 run speech_processor location_matcher
```

**Purpose:** Maps free-form escort commands to the **closest known UCI location name** using semantic similarity.

**What it does:**

- Loads **Sentence-BERT** (`all-MiniLM-L6-v2`).
- Encodes a fixed list of campus locations once.
- On each escort command, encodes the phrase, computes cosine similarity, selects the best match, and publishes the matched location name.
- _(Downstream component—outside this node—maps the location string to `(x, y)` and publishes a Nav2 goal.)_

**Topics:**

- **Subscribes**
  - `/escort_command` — `std_msgs/String` (user's escort phrase)
- **Publishes**
  - `/matched_location` — `std_msgs/String` (canonical location name)

### Sample logs:

`wake_audio_processor`

![Wake Audio Processor Log](/docs/speech-pipeline/image.png)

`intent_classifier`

![Intent Classifier Log](/docs/speech-pipeline/image-1.png)

`location_matcher`

![Location Matcher Log](/docs/speech-pipeline/image-2.png)
