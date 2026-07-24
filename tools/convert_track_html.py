#!/usr/bin/env python3
"""
Convert track-and-trace.html to track-and-trace.c
Usage: python3 convert_track_html.py
"""

import os
import sys

def convert_html():
    # Paths relative to project root
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    html_file = os.path.join(project_root, "src", "track-and-trace.html")
    c_file = os.path.join(project_root, "src", "track-and-trace.c")
    
    # Check if HTML file exists
    if not os.path.exists(html_file):
        print(f"Error: {html_file} not found")
        return False
    
    # Read HTML content
    try:
        with open(html_file, 'r', encoding='utf-8') as f:
            html_content = f.read()
    except Exception as e:
        print(f"Error reading HTML file: {e}")
        return False
    
    # Generate C file content

    c_content = f"""const char track_and_trace_html_content[] = R"rawliteral(
{html_content})rawliteral";
"""    
    # Write C file
    try:
        with open(c_file, 'w', encoding='utf-8') as f:
            f.write(c_content)
        print(f"Successfully converted {html_file} -> {c_file}")
        return True
    except Exception as e:
        print(f"Error writing C file: {e}")
        return False

if __name__ == "__main__":
    success = convert_html()
    sys.exit(0 if success else 1)