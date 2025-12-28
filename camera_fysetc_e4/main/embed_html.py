#!/usr/bin/env python3
"""
Convert HTML file to C header for embedding
"""
import sys
import os

def escape_c_string(s):
    """Escape string for C string literal"""
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    s = s.replace('\n', '\\n')
    s = s.replace('\r', '\\r')
    s = s.replace('\t', '\\t')
    return s

def html_to_header(html_file, header_file):
    """Convert HTML file to C header"""
    with open(html_file, 'r', encoding='utf-8') as f:
        html_content = f.read()
    
    # Escape for C string
    escaped = escape_c_string(html_content)
    
    # Generate header file
    header_content = f"""/**
 * @file web_ui_html.h
 * @brief Auto-generated header containing embedded HTML/JS
 * @note This file is auto-generated from web_ui.html - DO NOT EDIT
 */

#ifndef WEB_UI_HTML_H
#define WEB_UI_HTML_H

static const char html_page[] = "{escaped}";

#endif // WEB_UI_HTML_H
"""
    
    with open(header_file, 'w', encoding='utf-8') as f:
        f.write(header_content)
    
    print(f"Generated {header_file} from {html_file}")

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

