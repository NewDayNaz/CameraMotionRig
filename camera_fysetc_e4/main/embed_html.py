#!/usr/bin/env python3
"""
Convert HTML file to C header for embedding
Extracts velocity limits from stepper_limits.h and injects them into HTML
"""
import sys
import os
import re

def escape_c_string(s):
    """Escape string for C string literal"""
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    s = s.replace('\n', '\\n')
    s = s.replace('\r', '\\r')
    s = s.replace('\t', '\\t')
    return s

def parse_stepper_limits(limits_file):
    """Parse velocity limits from stepper_limits.h"""
    limits = {}
    if not os.path.exists(limits_file):
        print(f"Warning: {limits_file} not found, using default values")
        return {'MAX_PAN_VELOCITY': 500.0, 'MAX_TILT_VELOCITY': 500.0, 'MAX_ZOOM_VELOCITY': 100.0}
    
    with open(limits_file, 'r', encoding='utf-8') as f:
        content = f.read()
        
    # Extract #define values (handles both integer and float)
    pattern = r'#define\s+(\w+)\s+([0-9]+\.?[0-9]*f?)'
    matches = re.findall(pattern, content)
    
    for name, value in matches:
        # Remove 'f' suffix if present and convert to float
        value_clean = value.rstrip('fF')
        try:
            limits[name] = float(value_clean)
        except ValueError:
            pass
    
    return limits

def inject_limits(html_content, limits):
    """Replace placeholders in HTML with actual limit values"""
    replacements = {
        '{{MAX_PAN_VELOCITY}}': str(int(limits.get('MAX_PAN_VELOCITY', 500.0))),
        '{{MAX_ZOOM_VELOCITY}}': str(int(limits.get('MAX_ZOOM_VELOCITY', 100.0))),
    }
    
    for placeholder, value in replacements.items():
        html_content = html_content.replace(placeholder, value)
    
    return html_content

def html_to_header(html_file, header_file, limits_file=None):
    """Convert HTML file to C header"""
    # Find limits file relative to HTML file directory
    if limits_file is None:
        html_dir = os.path.dirname(os.path.abspath(html_file))
        limits_file = os.path.join(html_dir, 'stepper_limits.h')
    
    # Parse velocity limits
    limits = parse_stepper_limits(limits_file)
    
    # Read HTML file
    with open(html_file, 'r', encoding='utf-8') as f:
        html_content = f.read()
    
    # Inject limit values into HTML
    html_content = inject_limits(html_content, limits)
    
    # Escape for C string
    escaped = escape_c_string(html_content)
    
    # Generate header file
    header_content = f"""/**
 * @file web_ui_html.h
 * @brief Auto-generated header containing embedded HTML/JS
 * @note This file is auto-generated from web_ui.html - DO NOT EDIT
 * @note Velocity limits are injected from stepper_limits.h
 */

#ifndef WEB_UI_HTML_H
#define WEB_UI_HTML_H

static const char html_page[] = "{escaped}";

#endif // WEB_UI_HTML_H
"""
    
    with open(header_file, 'w', encoding='utf-8') as f:
        f.write(header_content)
    
    print(f"Generated {header_file} from {html_file}")
    print(f"  Injected limits: PAN={limits.get('MAX_PAN_VELOCITY', '?')}, ZOOM={limits.get('MAX_ZOOM_VELOCITY', '?')}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: embed_html.py <input.html> <output.h>")
        sys.exit(1)
    
    html_file = sys.argv[1]
    header_file = sys.argv[2]
    
    if not os.path.exists(html_file):
        print(f"Error: {html_file} not found")
        sys.exit(1)
    
    html_to_header(html_file, header_file)

