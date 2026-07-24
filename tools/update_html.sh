#!/bin/bash
# Convert logs_page.html to logs_page.h and stats_html.html to stats_html.c
# Usage: ./tools/update_html.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Converting test HTML to C source..."
python3 "$SCRIPT_DIR/convert_track_html.py"

if [ $? -eq 0 ]; then
    echo "✓ Stats HTML successfully converted to C source file"
else
    echo "✗ Error converting test HTML file"
    exit 1
fi