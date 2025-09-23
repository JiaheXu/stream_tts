import soundfile as sf
import numpy as np
from scipy.signal import resample_poly
import sounddevice as sd

input_file = "output.wav"      # your 22.05 kHz input
output_file = "output_48k.wav" # will be 48 kHz

# Step 1: Read file as float32 in [-1, 1]
data, sr = sf.read(input_file, dtype="float32")
print(f"Read: {sr} Hz, shape={data.shape}")

# Step 2: Resample (exact ratio: 48000 / 22050 = 320 / 147)
resampled = resample_poly(data, up=320, down=147).astype(np.float32)

# Step 3: Write back as PCM_16 at 48 kHz
sf.write(output_file, resampled, 48000, subtype="PCM_16")
print(f"✅ Wrote {output_file} at 48 kHz")

# Step 4: Playback check
sd.play(resampled, 48000)
sd.wait()