#!/bin/bash
set -e

# Get the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Path to the custom YAML file
CUSTOM_YAML="${SCRIPT_DIR}/custom_deps.yaml"

# Check if the custom YAML file exists
if [ ! -f "$CUSTOM_YAML" ]; then
    echo "Custom dependencies file not found: $CUSTOM_YAML"
    exit 1
fi

# Create a temporary directory for the custom source list
TEMP_DIR=$(mktemp -d)
CUSTOM_SOURCE_LIST="${TEMP_DIR}/99-custom-deps.list"

# Create the custom source list
echo "yaml file://${CUSTOM_YAML}" > "$CUSTOM_SOURCE_LIST"

# Add the custom source to rosdep
sudo cp "$CUSTOM_SOURCE_LIST" /etc/ros/rosdep/sources.list.d/

# Update rosdep database
if ! rosdep update --include-eol-distros; then
    echo "Failed to update rosdep database"
    sudo rm /etc/ros/rosdep/sources.list.d/99-custom-deps.list
    rm -rf "$TEMP_DIR"
    exit 1
fi

# Install dependencies using the custom source
if ! rosdep install --as-root pip:false --from-paths "${SCRIPT_DIR}" --ignore-src -r -y --rosdistro=humble; then
    echo "Failed to install dependencies"
    sudo rm /etc/ros/rosdep/sources.list.d/99-custom-deps.list
    rm -rf "$TEMP_DIR"
    exit 1
fi

# Clean up
sudo rm /etc/ros/rosdep/sources.list.d/99-custom-deps.list
rm -rf "$TEMP_DIR"

echo "Custom dependencies have been installed successfully."
