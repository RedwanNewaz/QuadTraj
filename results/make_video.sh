#!/usr/bin/env bash

#!/bin/bash

# Directory containing subfolders with image sequences
BASE_DIR=$1

# Check if BASE_DIR is provided
if [ -z "$BASE_DIR" ]; then
  echo "Usage: $0 <base_directory>"
  exit 1
fi

# Loop over each subdirectory in the base directory
for DIR in "$BASE_DIR"/*/; do
  # Check if DIR is a directory
  if [ -d "$DIR" ]; then
    # Extract the directory name
    DIR_NAME=$(basename "$DIR")

    # Define the output video file name
    OUTPUT_VIDEO="${BASE_DIR}/${DIR_NAME}.mp4"

    # Use ffmpeg to create a video from the image sequence in the current directory
    ffmpeg -framerate 24 -i "${DIR}%04d.png" -c:v libx264 -r 30 -pix_fmt yuv420p "$OUTPUT_VIDEO"

    echo "Generated video: $OUTPUT_VIDEO"
  fi
done
