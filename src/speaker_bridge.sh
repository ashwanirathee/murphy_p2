#!/bin/bash

CONTAINER="r1_ros"
WORKDIR="/home/ubuntu/r1"

TTS_ENGINE="${TTS_ENGINE:-espeak}"

PIPER_BIN="${PIPER_BIN:-piper}"
PIPER_MODEL="${PIPER_MODEL:-$HOME/piper-voices/en_US-lessac-medium.onnx}"

RAW_OUT="/tmp/r1_tts_raw.wav"
PADDED_OUT="/tmp/r1_tts_padded.wav"

LEADING_SILENCE="${LEADING_SILENCE:-0.45}"
TTS_GAIN_DB="${TTS_GAIN_DB:--15}"

cleanup() {
    echo
    echo "[INFO] Stopping speaker bridge..."

    # Kill any ros2 topic echo created inside the container
    docker exec "$CONTAINER" bash -lc \
        "pkill -f 'ros2 topic echo /audio/speech_text' || true" 2>/dev/null

    # Kill local playback/TTS if still running
    pkill -P $$ 2>/dev/null || true

    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

play_wav() {
    local wav="$1"

    sox "$wav" "$PADDED_OUT" pad "$LEADING_SILENCE" 0.2 gain "$TTS_GAIN_DB"

    aplay -q "$PADDED_OUT" 2>/dev/null || afplay "$PADDED_OUT"
}

speak_espeak() {
    local text="$1"

    espeak-ng -v en-us -s 130 -p 35 -a 120 \
        -w "$RAW_OUT" "$text"

    play_wav "$RAW_OUT"
}

speak_piper() {
    local text="$1"

    if [ ! -f "$PIPER_MODEL" ]; then
        echo "[ERROR] Piper model not found: $PIPER_MODEL"
        return 1
    fi

    echo "$text" | "$PIPER_BIN" \
        --model "$PIPER_MODEL" \
        --output_file "$RAW_OUT" >/dev/null 2>&1

    play_wav "$RAW_OUT"
}

speak() {
    local text="$1"

    case "$TTS_ENGINE" in
        espeak)
            speak_espeak "$text"
            ;;
        piper)
            speak_piper "$text"
            ;;
        *)
            echo "[ERROR] Unknown TTS_ENGINE: $TTS_ENGINE"
            echo "Use: TTS_ENGINE=espeak or TTS_ENGINE=piper"
            ;;
    esac
}

echo "[INFO] Using TTS engine: $TTS_ENGINE"
echo "[INFO] Gain: $TTS_GAIN_DB dB"
echo "[INFO] Press Ctrl+C to stop."

# Kill stale listeners before starting
docker exec "$CONTAINER" bash -lc \
    "pkill -f 'ros2 topic echo /audio/speech_text' || true" 2>/dev/null

docker exec -i "$CONTAINER" bash -lc \
"cd $WORKDIR && source install/setup.bash && ros2 topic echo /audio/speech_text --field data" \
| while read -r line; do
    if [ -n "$line" ] && [ "$line" != "---" ]; then
        echo "[SPEAK][$TTS_ENGINE] $line"
        speak "$line"
    fi
done