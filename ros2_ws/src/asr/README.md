# Exodapt Perception ASR Package

### How to use

```bash
export AZURE_SPEECH_KEY=YOUR_KEY
export AZURE_SPEECH_REGION=YOUR_REGION
ros2 launch asr azure_asr_launch.xml \
  azure_speech_key:=$AZURE_SPEECH_KEY \
  azure_speech_region:=$AZURE_SPEECH_REGION
```