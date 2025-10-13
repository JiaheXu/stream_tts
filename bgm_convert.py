import os
from pydub import AudioSegment

def convert_all_to_mono_wav(input_dir="./bgm", output_dir="./bgm_wav"):
    os.makedirs(output_dir, exist_ok=True)

    for filename in os.listdir(input_dir):
        if filename.lower().endswith((".mp3", ".m4a", ".wav")):
            input_path = os.path.join(input_dir, filename)
            base, _ = os.path.splitext(filename)
            output_path = os.path.join(output_dir, base + ".wav")

            try:
                # Load based on extension
                if filename.lower().endswith(".mp3"):
                    audio = AudioSegment.from_mp3(input_path)
                elif filename.lower().endswith(".m4a"):
                    audio = AudioSegment.from_file(input_path, format="m4a")
                else:
                    audio = AudioSegment.from_wav(input_path)

                # Convert to mono
                audio = audio.set_channels(1)

                # Export
                audio.export(output_path, format="wav")
                print(f"✅ Converted to mono: {filename} → {base}.wav")
            except Exception as e:
                print(f"❌ Failed: {filename}, Error: {e}")


if __name__ == "__main__":
    convert_all_to_mono_wav()

