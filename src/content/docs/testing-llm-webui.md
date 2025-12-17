---
title: "Testing LLM Models Using Web UI"
date: "2025-10-30"
summary: "How to run and tune text-generation-webui on Jetson, including container setup, model loading, and recommended parameters."
writer: "Farhin Mansuri"
keywords: "LLM, Jetson, text-generation-webui, Ollama, llama.cpp"
---

## Overview

This guide walks through running LLM models on Jetson using the text-generation-webui container, plus quick Ollama usage without the web UI.

## Quick: Ollama container only (no Web UI)

1. Start the Ollama container (host network):

```bash
jetson-containers run --name ollama dustynv/ollama:r35.4.1
```

2. Run a model inside the container:

```bash
ollama run phi3
```

3. Call the Ollama server API from another container or host:

```bash
curl http://localhost:11434/api/generate -d '{
  "model": "phi",
  "prompt": "Say hello from Jetson",
  "stream": false
}'
```

## Part 1 — Playground for LLMs (text-generation-webui)

### System

- Hardware: Jetson AGX Xavier  
- JetPack / L4T: 5.1.1 (R35.4.1)  
- CUDA: 11.4

### Steps

1) Check JetPack / L4T version

```bash
head -n 1 /etc/nv_tegra_release
# → # R35 (release), REVISION: 4.1
```

2) Clone Jetson Containers repository

```bash
git clone https://github.com/dusty-nv/jetson-containers
cd jetson-containers
```

3) Install Jetson Containers helper scripts

```bash
bash install.sh
```

4) Verify CUDA and environment

```bash
nvcc --version
# → Cuda 11.4 V11.4.315
```

5) List available Jetson Containers

```bash
jetson-containers list | head
# → L4T 35.4.1 / JetPack 5.1.2 / CUDA 11.4
```

6) Run the text-generation-webui container

```bash
jetson-containers run $(autotag text-generation-webui)
```

7) Access the interface

Wait for:

```
Running on local URL: http://0.0.0.0:7860
```

Then open:

```
http://<jetson-ip>:7860
```

![text-generation-webui UI](../../public/docs/testing-llm-webui/image.png)

### Configure the model

1) Use the correct model loader  
   - Loader: `llama.cpp`

2) Use the recommended model  
   - Model: `llama-2-7b-chat.Q4_K_M.gguf` (7B, 4-bit — good for Orin Nano / Xavier)

3) Apply GGUF parameters  
   - Load the model once, then set advanced params:  
     - `n_gpu_layers = 128` (or lower, e.g., 40 if VRAM < 16GB)  
     - `n_gqa` is only for 70B models (not needed for 7B)

4) Load again  
   - Click **Reload/Load** and wait for “Model loaded successfully”.

5) Instruction template  
   - `Parameters → Instruction Template → Llama-v2`, then **Submit**.

## Summary

- Loader: `llama.cpp`  
- Model: `llama-2-7b-chat.Q4_K_M.gguf`  
- n_gpu_layers: 128 (or auto)  
- Instruction Template: `Llama-v2`

