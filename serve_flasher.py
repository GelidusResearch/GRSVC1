#!/usr/bin/env python3
"""
Simple HTTP server for serving ESP Web Tools files with proper CORS headers.
Usage: python serve_flasher.py [port]
Default port: 8000
"""

import http.server
import socketserver
import sys
import os
from urllib.parse import urlparse

class CORSHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.send_header('Cross-Origin-Embedder-Policy', 'require-corp')
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def guess_type(self, path):
        mimetype = super().guess_type(path)
        if path.endswith('.bin'):
            return 'application/octet-stream'
        elif path.endswith('.json'):
            return 'application/json'
        return mimetype

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    
    # Change to src directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.join(script_dir, 'src')
    
    if os.path.exists(src_dir):
        os.chdir(src_dir)
        print(f"Serving from: {src_dir}")
    else:
        print(f"Warning: 'src' directory not found, serving from: {os.getcwd()}")
    
    with socketserver.TCPServer(("", port), CORSHTTPRequestHandler) as httpd:
        print(f"🚀 ESP Web Tools Server")
        print(f"📂 Directory: {os.getcwd()}")
        print(f"🌐 Server: http://localhost:{port}")
        print(f"⚡ ESP Web Tools: http://localhost:{port}/usb-web-installer.html")
        print(f"🛑 Press Ctrl+C to stop")
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print(f"\n🛑 Server stopped")

if __name__ == "__main__":
    main()
